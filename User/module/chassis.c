#include "module/chassis.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

// 控制周期上限取自底盘重构方案，超过 100 ms 时禁止追赶计算
#define CHASSIS_MAX_PERIOD_S (0.1F)

// 跟随反馈时效沿用云台电机反馈上限 90 ms，转速换算系数把 rpm 转成 rad/s
#define CHASSIS_FOLLOW_TIMEOUT_US (90000U)
#define CHASSIS_RPM_TO_RAD_S (M_2PI / 60.0F)

_Static_assert(CHASSIS_WHEEL_COUNT == CAN_DEVICES_CHASSIS_MOTOR_COUNT, "底盘与 CAN 电机数量必须一致");

/*
 * 单一底盘实例的私有跟随状态，由底盘任务独占：
 * - cycle_config：最近一次通过校验的持久配置快照，跟随 PID 的参数指针指向本副本
 * - follow_pid：跟随角度环的积分与微分历史
 * - yaw_feedback_valid：本周期反馈入口产生的时效、角度与连续性校验结果
 */
typedef struct
{
    Chassis_Config_t cycle_config;
    KPID_t follow_pid;
    bool yaw_feedback_valid;
} Chassis_Internal_t;

static Chassis_Internal_t chassis_internal;

static bool Chassis_IsConfigValid(const Chassis_Config_t *config, float frequency_hz);
static void Chassis_ResetWheel(Chassis_Wheel_t *wheel);
static void Chassis_PrimeWheel(Chassis_Wheel_t *wheel, float speed_limit_rpm);
static bool Chassis_ApplyPidConfig(Chassis_t *chassis);
static void Chassis_RampMove(MoveVector_t *ramped, const MoveVector_t *target, float scale_rpm,
                             float accel_rpm_s, float period_s);
static void Chassis_ControlWheel(Chassis_Wheel_t *wheel, const Chassis_Config_t *config, bool run,
                                 float period_s, bool wheel_ramp);
static void Chassis_RunWheel(Chassis_Wheel_t *wheel, const Chassis_Config_t *config, float period_s,
                             bool wheel_ramp);

/**
 * @brief 复制配置并初始化运动学、跟随 PID、四轮 PID 与滤波器
 *
 * @param[out] chassis 待初始化的底盘实例，初始化后不得整体复制
 * @param[in] config 默认配置，允许为 NULL，空指针时初始化失败
 * @param[in] frequency_hz 控制采样频率，单位 Hz，须为有限正值
 * @return 成功返回 0，失败返回 -1 并保持未初始化和零输出
 */
int8_t Chassis_Init(Chassis_t *chassis, const Chassis_Config_t *config, float frequency_hz)
{
    if (chassis == NULL)
    {
        return -1;
    }

    // 清空旧状态并标记缺失反馈，保证所有初始化失败路径输出零电流
    memset(chassis, 0, sizeof(*chassis));
    memset(&chassis_internal, 0, sizeof(chassis_internal));
    Chassis_UpdateFeedback(chassis, NULL);
    Chassis_UpdateGimbalFeedback(chassis, NULL, 0U, 0.0F, false);
    if (config == NULL)
    {
        return -1;
    }
    chassis->config = *config;
    chassis->config_valid = Chassis_IsConfigValid(config, frequency_hz);
    if (!chassis->config_valid)
    {
        return -1;
    }

    // 跟随参数保存在持久快照中，避免控制器引用周期栈上的临时配置
    chassis_internal.cycle_config = *config;
    if (PID_Init(&chassis_internal.follow_pid, KPID_MODE_SET_D, frequency_hz,
                 &chassis_internal.cycle_config.follow_pid) != 0)
    {
        chassis->config_valid = false;
        return -1;
    }

    // 用零向量验证布局支持四轮输出，避免二轮布局被错误用于底盘
    const MoveVector_t zero_move =
    {
        0,
    };
    float zero_speed[CHASSIS_WHEEL_COUNT];
    if (Mixer_Init(&chassis->mixer, config->layout) != MIXER_OK ||
        Mixer_Apply(&chassis->mixer, &zero_move, zero_speed, CHASSIS_WHEEL_COUNT, 0.0F) != MIXER_OK)
    {
        chassis->config_valid = false;
        return -1;
    }

    // 四轮引用同一份参数，各自持有独立积分、微分及两级滤波状态
    chassis->applied_wheel_speed_pid = config->wheel_speed_pid;
    for (uint32_t i = 0U; i < CHASSIS_WHEEL_COUNT; i++)
    {
        Chassis_Wheel_t *wheel = &chassis->wheel[i];
        if (PID_Init(&wheel->speed_pid, KPID_MODE_CALC_D, frequency_hz, &chassis->applied_wheel_speed_pid) != 0)
        {
            return -1;
        }
        LowPassFilter2p_Init(&wheel->speed_filter, frequency_hz, config->speed_filter_cutoff_hz);
        LowPassFilter2p_Init(&wheel->current_filter, frequency_hz, config->current_filter_cutoff_hz);
    }
    chassis->initialized = true;
    return 0;
}

/**
 * @brief 从 CAN 快照更新四轮实测转速
 *
 * @param[in,out] chassis 底盘实例
 * @param[in] can 本周期 CAN 快照，允许为 NULL，缺失或无效反馈写入 NaN
 * @return 无返回值
 */
void Chassis_UpdateFeedback(Chassis_t *chassis, const CANDevices_Snapshot_t *can)
{
    if (chassis == NULL)
    {
        return;
    }

    // 仅接纳本周期更新成功且在线的有限反馈，不沿用历史转速
    for (uint32_t i = 0U; i < CHASSIS_WHEEL_COUNT; i++)
    {
        chassis->wheel[i].measured_speed_rpm =
            can != NULL && can->feedback_update_status[i] == CAN_DEVICES_OK && can->motor_online[i] &&
            isfinite(can->actual_speed_rpm[i]) ? can->actual_speed_rpm[i] : NAN;
    }
}

/**
 * @brief 校验云台 yaw 反馈并更新车头相对角度与相对角速度，后续控制只使用本次结果
 *
 * @param[in,out] chassis 底盘实例
 * @param[in] feedback 最近一次 yaw 电机反馈，未取得时传 NULL
 * @param[in] now_us 读取反馈后的单调时间，单位微秒
 * @param[in] zero_deg 已标定的方向修正后编码器零点，单位度，须为有限值
 * @param[in] reversed 云台编码器方向是否反向
 * @return 无返回值
 */
void Chassis_UpdateGimbalFeedback(Chassis_t *chassis, const MOTOR_RM_Feedback_t *feedback,
                                  uint64_t now_us, float zero_deg, bool reversed)
{
    if (chassis == NULL)
    {
        return;
    }

    // 最新帧可能尚未到达或已经超时，帧龄用低位无符号差处理时间戳回绕
    const uint32_t age_us = feedback != NULL ? (uint32_t)now_us - feedback->last_update_time : UINT32_MAX;
    if (feedback == NULL || age_us > CHASSIS_FOLLOW_TIMEOUT_US || !feedback->angle_valid ||
        feedback->angle_lost_count != 0U || !isfinite(zero_deg) || !isfinite(feedback->rotor_speed) ||
        !(feedback->rotor_single_angle >= 0.0F && feedback->rotor_single_angle < M_2PI))
    {
        chassis_internal.yaw_feedback_valid = false;
        chassis->follow_relative_angle_deg = NAN;
        chassis->follow_relative_speed_rad_s = NAN;
        return;
    }

    // 最短机械角用于跟随与坐标转换，电机转速换算为方向修正后的角速度
    const float angle = reversed ? -feedback->rotor_single_angle : feedback->rotor_single_angle;
    const float relative = remainderf(angle - zero_deg * M_DEG2RAD_MULT, M_2PI);
    chassis->follow_relative_angle_deg = relative * M_RAD2DEG_MULT;
    const float speed_rad_s = feedback->rotor_speed * CHASSIS_RPM_TO_RAD_S;
    chassis->follow_relative_speed_rad_s = reversed ? -speed_rad_s : speed_rad_s;
    chassis_internal.yaw_feedback_valid = true;
}

/**
 * @brief 推进底盘模式和每轮状态机并计算电流
 *
 * @param[in,out] chassis 底盘实例
 * @param[in] command 归一化控制命令，允许为 NULL，非法命令按待机处理
 * @param[in] period_s 实测控制周期，单位 s，须在 (0, 0.1] 内
 * @return 无返回值
 */
void Chassis_Control(Chassis_t *chassis, const Chassis_Command_t *command, float period_s)
{
    if (chassis == NULL)
    {
        return;
    }

    // 依据初始化的采样周期校验在线配置，未初始化的实例始终处于安全输出
    const float frequency_hz = chassis->initialized ? 1.0F / chassis->wheel[0].speed_pid.dt_min : 0.0F;
    chassis->config_valid = Chassis_IsConfigValid(&chassis->config, frequency_hz);
    bool follow_config_changed = false;
    if (chassis->initialized && chassis->config_valid)
    {
        follow_config_changed = Chassis_ApplyPidConfig(chassis);
    }

    // 默认整组待机，只有合法命令和有效周期才能请求运动学计算
    Chassis_Mode_t next_mode = CHASSIS_MODE_RELAX;
    float requested_speed[CHASSIS_WHEEL_COUNT] =
    {
        0,
    };
    if (chassis->initialized && chassis->config_valid && command != NULL && isfinite(period_s) &&
        period_s > 0.0F && period_s <= CHASSIS_MAX_PERIOD_S && isfinite(command->wheel_speed_scale_rpm) &&
        command->wheel_speed_scale_rpm >= 0.0F)
    {
        // 跟随与自旋依赖云台机械角，反馈超时或失真时退回待机
        switch (command->mode)
        {
            case CHASSIS_MODE_INDEPENDENT:
                next_mode = CHASSIS_MODE_INDEPENDENT;
                break;

            case CHASSIS_MODE_FOLLOW_GIMBAL:
            case CHASSIS_MODE_SPIN:
                next_mode = chassis_internal.yaw_feedback_valid ? command->mode : CHASSIS_MODE_RELAX;
                break;

            case CHASSIS_MODE_RELAX:
            default:
                break;
        }
    }

    // 跟随输出只由本周期 PID 产生，进入跟随前的历史已在初始化或上次退出时清除
    const bool follow_requested = next_mode == CHASSIS_MODE_FOLLOW_GIMBAL;
    chassis->follow_wz = 0.0F;
    if (next_mode == CHASSIS_MODE_INDEPENDENT)
    {
        // 独立模式沿用原有语义：车体坐标平移，单轮目标再由轮速斜坡和换向保护约束
        const float scale_rpm = fminf(command->wheel_speed_scale_rpm, chassis->config.max_wheel_speed_rpm);
        if (Mixer_Apply(&chassis->mixer, &command->move, requested_speed, CHASSIS_WHEEL_COUNT,
                        scale_rpm) != MIXER_OK)
        {
            next_mode = CHASSIS_MODE_RELAX;
        }
    }
    else if (next_mode == CHASSIS_MODE_FOLLOW_GIMBAL || next_mode == CHASSIS_MODE_SPIN)
    {
        // 指令斜坡先限制云台坐标平移与自旋分量的变化率，旋转和混控放在斜坡之后
        const float scale_rpm = fminf(command->wheel_speed_scale_rpm, chassis->config.max_wheel_speed_rpm);
        Chassis_RampMove(&chassis->ramped_move, &command->move, scale_rpm, chassis->config.wheel_accel_rpm_s,
                         period_s);
        MoveVector_t move = chassis->ramped_move;

        // 平移按云台相对车头的角度旋转到车体坐标，轮速随旋转的变化不再受斜坡限制
        const float relative_rad = chassis->follow_relative_angle_deg * M_DEG2RAD_MULT;
        const float cosine = cosf(relative_rad);
        const float sine = sinf(relative_rad);
        move.vx = cosine * chassis->ramped_move.vx - sine * chassis->ramped_move.vy;
        move.vy = sine * chassis->ramped_move.vx + cosine * chassis->ramped_move.vy;

        // 跟随输出直接叠加在斜坡后的自旋分量上，闭环输出不进入斜坡
        if (follow_requested)
        {
            // 父树实测正轮速使云台相对角增加，正角差应产生负转向请求，此处沿用该符号约定
            // 运行中改参先清除旧历史，退出跟随时由后续统一清理
            if (follow_config_changed)
            {
                PID_Reset(&chassis_internal.follow_pid);
            }
            const float follow_output = PID_Calc(&chassis_internal.follow_pid, 0.0F, relative_rad,
                                                 chassis->follow_relative_speed_rad_s, period_s);
            move.wz = chassis->ramped_move.wz + follow_output;
            chassis->follow_wz = follow_output;
        }

        // 混控入口检查运动分量并生成本周期轮速，失败时取消本周期运行请求
        if (Mixer_Apply(&chassis->mixer, &move, requested_speed, CHASSIS_WHEEL_COUNT, scale_rpm) != MIXER_OK)
        {
            next_mode = CHASSIS_MODE_RELAX;
        }
    }

    // 斜坡状态只在跟随与自旋之间保持，待机、独立模式和混控失败时集中清零
    if (next_mode != CHASSIS_MODE_FOLLOW_GIMBAL && next_mode != CHASSIS_MODE_SPIN)
    {
        ResetMoveVector(&chassis->ramped_move);
    }

    // 退出跟随或本周期跟随计算被混控拒绝时清理历史，持续待机不重复复位
    if (next_mode != CHASSIS_MODE_FOLLOW_GIMBAL &&
        (chassis->mode == CHASSIS_MODE_FOLLOW_GIMBAL || follow_requested))
    {
        PID_Reset(&chassis_internal.follow_pid);
        chassis->follow_wz = 0.0F;
    }

    // 模式切换统一清除动态历史，随后由单轮状态机决定是否从实测转速起步
    const bool wheel_ramp = next_mode == CHASSIS_MODE_INDEPENDENT;
    for (uint32_t i = 0U; i < CHASSIS_WHEEL_COUNT; i++)
    {
        Chassis_Wheel_t *wheel = &chassis->wheel[i];
        if (chassis->mode != next_mode)
        {
            Chassis_ResetWheel(wheel);
            wheel->state = CHASSIS_WHEEL_RELAX;
        }
        wheel->requested_speed_rpm = next_mode != CHASSIS_MODE_RELAX ? requested_speed[i] : 0.0F;
        Chassis_ControlWheel(wheel, &chassis->config, next_mode != CHASSIS_MODE_RELAX, period_s, wheel_ramp);
    }
    chassis->mode = next_mode;
}

/**
 * @brief 导出四路电流，保留调用方设置的命令序号
 *
 * @param[in] chassis 底盘实例，允许为 NULL，未初始化时输出全零
 * @param[out] output CAN 电流命令，仅写 current_a[0~3]
 * @return 无返回值
 */
void Chassis_DumpOutput(const Chassis_t *chassis, CANDevices_Command_t *output)
{
    if (output == NULL)
    {
        return;
    }

    // 最后一道输出检查保证未初始化、非运行及非法电流不会进入通信任务
    for (uint32_t i = 0U; i < CHASSIS_WHEEL_COUNT; i++)
    {
        output->current_a[i] = 0.0F;
        if (chassis != NULL && chassis->initialized && chassis->config_valid &&
            chassis->mode != CHASSIS_MODE_RELAX && chassis->wheel[i].state == CHASSIS_WHEEL_RUNNING &&
            isfinite(chassis->wheel[i].output_current_a) && isfinite(chassis->config.max_current_a) &&
            chassis->config.max_current_a > 0.0F)
        {
            output->current_a[i] = AbsClip(chassis->wheel[i].output_current_a, chassis->config.max_current_a);
        }
    }
}

/**
 * @brief 校验底盘限幅、斜坡、滤波、轮速 PID 和跟随 PID 参数
 *
 * @param[in] config 待校验的非空配置
 * @param[in] frequency_hz 初始化时的采样频率，单位 Hz
 * @return 所有数值有限且范围合法时返回 true
 */
static bool Chassis_IsConfigValid(const Chassis_Config_t *config, float frequency_hz)
{
    const KPID_Params_t *pid = &config->wheel_speed_pid;
    const KPID_Params_t *follow = &config->follow_pid;
    const float nyquist_hz = frequency_hz * 0.5F;
    return isfinite(frequency_hz) && frequency_hz > 0.0F && isfinite(1.0F / frequency_hz) &&
           isfinite(config->max_wheel_speed_rpm) && config->max_wheel_speed_rpm > 0.0F &&
           isfinite(config->wheel_accel_rpm_s) && config->wheel_accel_rpm_s > 0.0F &&
           isfinite(config->reverse_release_speed_rpm) && config->reverse_release_speed_rpm >= 0.0F &&
           isfinite(config->max_current_a) && config->max_current_a > 0.0F &&
           isfinite(config->speed_filter_cutoff_hz) && config->speed_filter_cutoff_hz < nyquist_hz &&
           isfinite(config->current_filter_cutoff_hz) && config->current_filter_cutoff_hz < nyquist_hz &&
           isfinite(pid->k) && pid->k > 0.0F && isfinite(pid->p) && pid->p >= 0.0F &&
           isfinite(pid->i) && pid->i >= 0.0F && isfinite(pid->d) && pid->d >= 0.0F &&
           isfinite(pid->i_limit) && pid->i_limit >= 0.0F && isfinite(pid->out_limit) && pid->out_limit >= 0.0F &&
           isfinite(pid->d_cutoff_freq) && pid->d_cutoff_freq < nyquist_hz &&
           isfinite(pid->range) && pid->range >= 0.0F &&
           isfinite(follow->k) && follow->k > 0.0F && isfinite(follow->p) && follow->p >= 0.0F &&
           isfinite(follow->i) && follow->i >= 0.0F && isfinite(follow->d) && follow->d >= 0.0F &&
           isfinite(follow->i_limit) && follow->i_limit >= 0.0F &&
           isfinite(follow->out_limit) && follow->out_limit > 0.000001F && follow->out_limit <= 1.0F &&
           isfinite(follow->d_cutoff_freq) && follow->d_cutoff_freq < nyquist_hz && follow->range == M_2PI;
}

/**
 * @brief 清除单轮速度环与两级滤波动态历史并输出零电流
 *
 * @param[in,out] wheel 待复位的单轮状态
 * @return 无返回值
 */
static void Chassis_ResetWheel(Chassis_Wheel_t *wheel)
{
    if (wheel->speed_pid.param != NULL)
    {
        PID_Reset(&wheel->speed_pid);
    }
    if (wheel->speed_filter.b0 > 0.0F)
    {
        LowPassFilter2p_Reset(&wheel->speed_filter, 0.0F);
    }
    if (wheel->current_filter.b0 > 0.0F)
    {
        LowPassFilter2p_Reset(&wheel->current_filter, 0.0F);
    }
    wheel->filtered_speed_rpm = 0.0F;
    wheel->target_speed_rpm = 0.0F;
    wheel->output_current_a = 0.0F;
}

/**
 * @brief 用实测转速预置恢复运行时的反馈、微分和目标斜坡
 *
 * @param[in,out] wheel 反馈有效的单轮状态
 * @param[in] speed_limit_rpm 输出轴转速绝对值上限，单位 rpm
 * @return 无返回值
 */
static void Chassis_PrimeWheel(Chassis_Wheel_t *wheel, float speed_limit_rpm)
{
    // 预置速度与电流滤波器，使恢复运行时不沿用停机前历史
    PID_Reset(&wheel->speed_pid);
    wheel->filtered_speed_rpm = LowPassFilter2p_Reset(&wheel->speed_filter, wheel->measured_speed_rpm);
    LowPassFilter2p_Reset(&wheel->current_filter, 0.0F);
    wheel->target_speed_rpm = AbsClip(wheel->filtered_speed_rpm, speed_limit_rpm);

    // 同步微分上一反馈，避免恢复使能时产生一次性电流冲击
    const float scaled_feedback = wheel->speed_pid.param->k * wheel->filtered_speed_rpm;
    LowPassFilter2p_Reset(&wheel->speed_pid.dfilter, scaled_feedback);
    wheel->speed_pid.last.k_fb = scaled_feedback;
}

/**
 * @brief 采纳持久配置快照并同步四轮参数，返回跟随参数是否改变
 *
 * @param[in,out] chassis 配置合法且初始化成功的底盘实例
 * @return 跟随 PID 参数改变时返回 true，否则返回 false
 */
static bool Chassis_ApplyPidConfig(Chassis_t *chassis)
{
    // 跟随控制器长期引用持久快照，更新前检测变更，滤波参数只在改变时同步
    const KPID_Params_t *follow = &chassis->config.follow_pid;
    const KPID_Params_t *applied = &chassis_internal.cycle_config.follow_pid;
    const bool follow_changed = follow->k != applied->k || follow->p != applied->p ||
        follow->i != applied->i || follow->d != applied->d || follow->i_limit != applied->i_limit ||
        follow->out_limit != applied->out_limit || follow->d_cutoff_freq != applied->d_cutoff_freq;
    const bool cutoff_changed = follow->d_cutoff_freq != applied->d_cutoff_freq;
    chassis_internal.cycle_config = chassis->config;
    if (cutoff_changed)
    {
        LowPassFilter2p_Init(&chassis_internal.follow_pid.dfilter,
                             1.0F / chassis_internal.follow_pid.dt_min, follow->d_cutoff_freq);
    }

    // 四轮参数继续使用公共副本，只有轮速参数变更才清除轮速积分
    const KPID_Params_t *next = &chassis->config.wheel_speed_pid;
    const KPID_Params_t *old = &chassis->applied_wheel_speed_pid;
    if (next->k == old->k && next->p == old->p && next->i == old->i && next->d == old->d &&
        next->i_limit == old->i_limit && next->out_limit == old->out_limit &&
        next->d_cutoff_freq == old->d_cutoff_freq && next->range == old->range)
    {
        return follow_changed;
    }

    // 只在参数变更时复制一次；截止频率变化时同步重建各轮微分滤波系数
    const bool filter_changed = next->d_cutoff_freq != old->d_cutoff_freq;
    const bool gain_changed = next->k != old->k;
    chassis->applied_wheel_speed_pid = *next;
    for (uint32_t i = 0U; i < CHASSIS_WHEEL_COUNT; i++)
    {
        KPID_t *pid = &chassis->wheel[i].speed_pid;
        PID_ResetIntegral(pid);
        if (filter_changed || gain_changed)
        {
            LowPassFilter2p_Init(&pid->dfilter, 1.0F / pid->dt_min, next->d_cutoff_freq);
            const float scaled_feedback = next->k * chassis->wheel[i].filtered_speed_rpm;
            LowPassFilter2p_Reset(&pid->dfilter, scaled_feedback);
            pid->last.k_fb = scaled_feedback;
        }
    }
    return follow_changed;
}

/**
 * @brief 按时限速率把运动向量逼近目标，平移按二维向量整体限步
 *
 * @param[in,out] ramped 持久斜坡状态
 * @param[in] target 本周期目标运动向量
 * @param[in] scale_rpm 归一化轮速尺度，单位 rpm
 * @param[in] accel_rpm_s 纯平移时等效的单轮转速变化率，单位 rpm/s
 * @param[in] period_s 实测控制周期，单位 s
 * @return 无返回值
 */
static void Chassis_RampMove(MoveVector_t *ramped, const MoveVector_t *target, float scale_rpm,
                             float accel_rpm_s, float period_s)
{
    // 尺度为零时混控输出恒为零，直接跟随目标，避免除零
    if (scale_rpm == 0.0F)
    {
        *ramped = *target;
        return;
    }

    // 归一化步长由配置换算：纯前后或纯左右推杆时单轮转速变化率等于 accel_rpm_s
    const float step = accel_rpm_s / scale_rpm * period_s;

    // 平移按二维向量整体限步，避免两个分量到位时间不同导致起步过程中方向偏转
    const float dx = target->vx - ramped->vx;
    const float dy = target->vy - ramped->vy;
    const float distance = hypotf(dx, dy);
    if (distance > step)
    {
        ramped->vx += dx * step / distance;
        ramped->vy += dy * step / distance;
    }
    else
    {
        ramped->vx = target->vx;
        ramped->vy = target->vy;
    }
    ramped->wz = RampTo(ramped->wz, target->wz, step);
}

/**
 * @brief 集中处理单轮待机、运行、离线与恢复转换
 *
 * @param[in,out] wheel 当前单轮状态
 * @param[in] config 已校验的底盘配置，待机时允许配置非法
 * @param[in] run 本周期是否请求运行
 * @param[in] period_s 实测控制周期，运行时须在 (0, 0.1] s 内
 * @param[in] wheel_ramp 为 true 时启用单轮转速斜坡和换向保护，仅独立模式使用
 * @return 无返回值
 */
static void Chassis_ControlWheel(Chassis_Wheel_t *wheel, const Chassis_Config_t *config, bool run,
                                 float period_s, bool wheel_ramp)
{
    Chassis_WheelState_t next_state = CHASSIS_WHEEL_RELAX;
    switch (wheel->state)
    {
        case CHASSIS_WHEEL_RELAX:
        case CHASSIS_WHEEL_RUNNING:
        case CHASSIS_WHEEL_OFFLINE:
            if (run)
            {
                next_state = isfinite(wheel->measured_speed_rpm) ? CHASSIS_WHEEL_RUNNING : CHASSIS_WHEEL_OFFLINE;
            }
            break;

        default:
            // 非法状态本周期强制待机，下周期才允许重新预置运行
            break;
    }

    // 入口动作只在恢复运行时预置，非运行状态持续清除历史并维持零输出
    if (next_state == CHASSIS_WHEEL_RUNNING && wheel->state != CHASSIS_WHEEL_RUNNING)
    {
        Chassis_PrimeWheel(wheel, config->max_wheel_speed_rpm);
    }
    wheel->state = next_state;
    switch (wheel->state)
    {
        case CHASSIS_WHEEL_RUNNING:
            Chassis_RunWheel(wheel, config, period_s, wheel_ramp);
            break;

        case CHASSIS_WHEEL_OFFLINE:
        case CHASSIS_WHEEL_RELAX:
        default:
            Chassis_ResetWheel(wheel);
            break;
    }
}

/**
 * @brief 执行单轮限幅、独立模式斜坡、换向保护和速度闭环
 *
 * @param[in,out] wheel 运行中的单轮状态，反馈须有限
 * @param[in] config 已校验的底盘配置
 * @param[in] period_s 实测控制周期，单位 s
 * @param[in] wheel_ramp 为 true 时启用单轮转速斜坡和换向保护，仅独立模式使用
 * @return 无返回值，计算结果非法时复位本轮并转入待机
 */
static void Chassis_RunWheel(Chassis_Wheel_t *wheel, const Chassis_Config_t *config, float period_s,
                             bool wheel_ramp)
{
    float target = AbsClip(wheel->requested_speed_rpm, config->max_wheel_speed_rpm);

    // 目标反转先清除旧方向积分，跟随和自旋需要允许目标随云台角度换向
    if ((wheel->target_speed_rpm > 0.0F && target < 0.0F) ||
        (wheel->target_speed_rpm < 0.0F && target > 0.0F))
    {
        PID_ResetIntegral(&wheel->speed_pid);
    }

    // 独立模式按实测周期推进斜坡并等待实测转速过零，跟随和自旋直接采用解算目标
    if (!wheel_ramp)
    {
        wheel->target_speed_rpm = target;
    }
    else
    {
        if ((wheel->measured_speed_rpm > config->reverse_release_speed_rpm && target < 0.0F) ||
            (wheel->measured_speed_rpm < -config->reverse_release_speed_rpm && target > 0.0F))
        {
            target = 0.0F;
        }
        wheel->target_speed_rpm = RampTo(wheel->target_speed_rpm, target, config->wheel_accel_rpm_s * period_s);
    }

    // 依次计算反馈滤波、速度 PID 和电流滤波
    wheel->filtered_speed_rpm = LowPassFilter2p_Apply(&wheel->speed_filter, wheel->measured_speed_rpm);
    const float current = PID_Calc(&wheel->speed_pid, wheel->target_speed_rpm, wheel->filtered_speed_rpm,
                                   0.0F, period_s);
    wheel->output_current_a = LowPassFilter2p_Apply(&wheel->current_filter, current);

    // 非有限计算只隔离本轮，下一周期重新预置后才恢复闭环
    if (!isfinite(wheel->target_speed_rpm) || !isfinite(wheel->filtered_speed_rpm) ||
        !isfinite(wheel->output_current_a))
    {
        Chassis_ResetWheel(wheel);
        wheel->state = CHASSIS_WHEEL_RELAX;
        return;
    }
    wheel->output_current_a = AbsClip(wheel->output_current_a, config->max_current_a);
}
