#include "module/chassis.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

// 控制周期上限取自底盘重构方案，超过 100 ms 时禁止追赶计算
#define CHASSIS_MAX_PERIOD_S (0.1F)

/*
 * 独立轮速观测的初始时效约束：帧龄不超过 20 毫秒，四轮源时间跨度不超过 4 毫秒
 * 时间来自设备解码时刻，不代表电机内部采样时刻，实际总线延迟仍需录波核对
 */
#define CHASSIS_MOTION_MAX_AGE_US (20000U)
#define CHASSIS_MOTION_MAX_SKEW_US (4000U)

_Static_assert(CHASSIS_WHEEL_COUNT == CAN_DEVICES_CHASSIS_MOTOR_COUNT, "底盘与 CAN 电机数量必须一致");

static bool Chassis_IsConfigValid(const Chassis_Config_t *config, float frequency_hz);
static void Chassis_ResetWheel(Chassis_Wheel_t *wheel);
static void Chassis_PrimeWheel(Chassis_Wheel_t *wheel, float speed_limit_rpm);
static void Chassis_ApplyPidConfig(Chassis_t *chassis, const Chassis_Config_t *config);
static void Chassis_ControlWheel(Chassis_Wheel_t *wheel, const Chassis_Config_t *config, bool run, float period_s);
static void Chassis_RunWheel(Chassis_Wheel_t *wheel, const Chassis_Config_t *config, float period_s);

/**
 * @brief 复制配置并初始化运动学、四轮 PID 与滤波器
 *
 * @param[out] chassis 待初始化的底盘实例，调用方保证非空，初始化后不得整体复制
 * @param[in] config 默认配置，调用方保证非空
 * @param[in] frequency_hz 控制采样频率，单位 Hz，须为有限正值
 * @return 成功返回 0，失败返回 -1 并保持未初始化和零输出
 */
int8_t Chassis_Init(Chassis_t *chassis, const Chassis_Config_t *config, float frequency_hz)
{
    // 清空旧状态并标记缺失反馈，保证所有初始化失败路径输出零电流
    memset(chassis, 0, sizeof(*chassis));
    Chassis_UpdateFeedback(chassis, NULL);
    chassis->config = *config;
    chassis->config_valid = Chassis_IsConfigValid(config, frequency_hz);
    if (!chassis->config_valid)
    {
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
 * @param[in] can 本周期 CAN 快照，无新消息时传 NULL，缺失或无效反馈写入 NaN
 * @return 无返回值
 */
void Chassis_UpdateFeedback(Chassis_t *chassis, const CANDevices_Snapshot_t *can)
{
    // 设备整数协议换算保证数值有限，此处只接纳本周期更新成功且在线的反馈
    for (uint32_t i = 0U; i < CHASSIS_WHEEL_COUNT; i++)
    {
        chassis->wheel[i].measured_speed_rpm =
            can != NULL && can->feedback_update_status[i] == CAN_DEVICES_OK && can->motor_online[i] ?
            can->actual_speed_rpm[i] : NAN;
    }
}

/**
 * @brief 推进底盘模式和每轮状态机并计算电流
 *
 * @param[in,out] chassis 底盘实例
 * @param[in] command 归一化控制命令，无命令时传 NULL，非法命令按待机处理
 * @param[in] period_s 实测控制周期，单位 s，须在 (0, 0.1] 内
 * @return 无返回值
 */
void Chassis_Control(Chassis_t *chassis, const Chassis_Command_t *command, float period_s)
{
    // 初始化失败的实例没有可复位控制器，输出由既有待机状态统一阻断
    if (!chassis->initialized)
    {
        return;
    }

    // EmberProbe 可在任意时刻写入 config，本周期只校验并使用这一份快照
    const Chassis_Config_t snapshot = *(const volatile Chassis_Config_t *)&chassis->config;
    const float frequency_hz = 1.0F / chassis->wheel[0].speed_pid.dt_min;
    chassis->config_valid = Chassis_IsConfigValid(&snapshot, frequency_hz);
    if (chassis->config_valid)
    {
        Chassis_ApplyPidConfig(chassis, &snapshot);
    }

    // 默认整组待机，只有合法独立命令和有效周期才能请求运动学计算
    Chassis_Mode_t next_mode = CHASSIS_MODE_RELAX;
    float requested_speed[CHASSIS_WHEEL_COUNT] =
    {
        0,
    };
    if (chassis->config_valid && command != NULL &&
        command->mode == CHASSIS_MODE_INDEPENDENT && isfinite(period_s) &&
        period_s > 0.0F && period_s <= CHASSIS_MAX_PERIOD_S)
    {
        const float scale_rpm = fminf(command->wheel_speed_scale_rpm, snapshot.max_wheel_speed_rpm);
        if (isfinite(command->wheel_speed_scale_rpm) && command->wheel_speed_scale_rpm >= 0.0F &&
            Mixer_Apply(&chassis->mixer, &command->move, requested_speed, CHASSIS_WHEEL_COUNT, scale_rpm) == MIXER_OK)
        {
            next_mode = CHASSIS_MODE_INDEPENDENT;
        }
    }

    // 模式切换统一清除动态历史，随后由单轮状态机决定是否从实测转速起步
    for (uint32_t i = 0U; i < CHASSIS_WHEEL_COUNT; i++)
    {
        Chassis_Wheel_t *wheel = &chassis->wheel[i];
        if (chassis->mode != next_mode)
        {
            wheel->state = CHASSIS_WHEEL_RELAX;
        }
        wheel->requested_speed_rpm = next_mode == CHASSIS_MODE_INDEPENDENT ? requested_speed[i] : 0.0F;
        Chassis_ControlWheel(wheel, &snapshot, next_mode == CHASSIS_MODE_INDEPENDENT, period_s);
    }
    chassis->mode = next_mode;
}

/**
 * @brief 导出四路电流，保留调用方设置的命令序号
 *
 * @param[in] chassis 底盘实例，调用方保证非空，未初始化时输出全零
 * @param[out] output CAN 电流命令，调用方保证非空，仅写 current_a[0~3]
 * @return 无返回值
 */
void Chassis_DumpOutput(const Chassis_t *chassis, CANDevices_Command_t *output)
{
    // 数值与限幅已在计算出口处理，最终只按运行状态决定是否向通信任务输出
    for (uint32_t i = 0U; i < CHASSIS_WHEEL_COUNT; i++)
    {
        output->current_a[i] = chassis->mode == CHASSIS_MODE_INDEPENDENT &&
            chassis->wheel[i].state == CHASSIS_WHEEL_RUNNING ? chassis->wheel[i].output_current_a : 0.0F;
    }
}

/**
 * @brief 从四轮源反馈生成旋转观测，不使用云台同源数据校正轮速
 *
 * @param[in] chassis 已完成本周期控制的底盘实例，调用方保证非空
 * @param[in] can 本周期独立反馈快照，可为空
 * @param[in] mode 有效命令的旋转意图，未知值输出无效观测
 * @param[in] rad_s_per_rpm 平均轮速到车体转速的有限正标定系数
 * @param[in] now_us 读取反馈后的当前时间，单位微秒
 * @param[out] feedback 输出完整观测，调用方保证非空，失败时有效标志为假
 * @return 无返回值
 */
void Chassis_DumpGimbalFeedback(const Chassis_t *chassis, const CANDevices_Snapshot_t *can,
                                Gimbal_ChassisMotion_t mode, float rad_s_per_rpm, uint64_t now_us,
                                Gimbal_ChassisFeedback_t *feedback)
{
    // 每周期先撤销有效性，未标定、来源缺失和不支持的布局不得沿用旧观测
    *feedback = (Gimbal_ChassisFeedback_t)
    {
        .mode = mode,
    };
    if (can == NULL || !chassis->initialized || !chassis->config_valid ||
        !can->initialized || chassis->mixer.mode != MIXER_OMNICROSS ||
        (unsigned)mode >= GIMBAL_CHASSIS_MOTION_UNKNOWN ||
        !isfinite(rad_s_per_rpm) || rad_s_per_rpm <= 0.0f)
    {
        return;
    }

    // 逐轮检查独立源时间与反馈状态，低位时间戳允许跨越微秒计数器回绕
    uint64_t oldest_us = now_us;
    uint64_t newest_us = 0U;
    float measured_sum_rpm = 0.0f;
    float requested_sum_rpm = 0.0f;
    for (uint32_t i = 0U; i < CHASSIS_WHEEL_COUNT; i++)
    {
        const uint32_t age_us = (uint32_t)now_us - can->feedback_time_us[i];
        const Chassis_Wheel_t *wheel = &chassis->wheel[i];
        if (!can->motor_online[i] || can->feedback_update_status[i] != DEVICE_OK ||
            age_us > CHASSIS_MOTION_MAX_AGE_US || now_us < age_us)
        {
            return;
        }
        const uint64_t source_us = now_us - age_us;
        oldest_us = source_us < oldest_us ? source_us : oldest_us;
        newest_us = source_us > newest_us ? source_us : newest_us;
        measured_sum_rpm += can->actual_speed_rpm[i];
        requested_sum_rpm += wheel->state == CHASSIS_WHEEL_RUNNING ? wheel->target_speed_rpm : 0.0f;
    }

    // 对称十字全向布局的平移项在四轮求和后抵消，目标使用限幅及斜坡后的实际值
    feedback->measured_speed_rad_s = 0.25f * measured_sum_rpm * rad_s_per_rpm;
    feedback->requested_speed_rad_s = 0.25f * requested_sum_rpm * rad_s_per_rpm;
    feedback->oldest_sample_time_us = oldest_us;
    feedback->newest_sample_time_us = newest_us;
    feedback->valid = newest_us - oldest_us <= CHASSIS_MOTION_MAX_SKEW_US &&
        isfinite(feedback->measured_speed_rad_s) && isfinite(feedback->requested_speed_rad_s);
}

/**
 * @brief 校验底盘限幅、斜坡、滤波和公共速度 PID 参数
 *
 * @param[in] config 待校验的非空配置
 * @param[in] frequency_hz 初始化时的采样频率，单位 Hz
 * @return 所有数值有限且范围合法时返回 true
 */
static bool Chassis_IsConfigValid(const Chassis_Config_t *config, float frequency_hz)
{
    const KPID_Params_t *pid = &config->wheel_speed_pid;
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
           isfinite(pid->range) && pid->range >= 0.0F;
}

/**
 * @brief 清除单轮速度环与两级滤波动态历史并输出零电流
 *
 * @param[in,out] wheel 待复位的单轮状态
 * @return 无返回值
 */
static void Chassis_ResetWheel(Chassis_Wheel_t *wheel)
{
    PID_Reset(&wheel->speed_pid);
    LowPassFilter2p_Reset(&wheel->speed_filter, 0.0F);
    LowPassFilter2p_Reset(&wheel->current_filter, 0.0F);
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
 * @brief 同步公共 PID 参数并清空四轮旧积分
 *
 * @param[in,out] chassis 配置合法且初始化成功的底盘实例
 * @param[in] config 本周期已校验的配置快照
 * @return 无返回值
 */
static void Chassis_ApplyPidConfig(Chassis_t *chassis, const Chassis_Config_t *config)
{
    const KPID_Params_t *next = &config->wheel_speed_pid;
    const KPID_Params_t *old = &chassis->applied_wheel_speed_pid;
    if (next->k == old->k && next->p == old->p && next->i == old->i && next->d == old->d &&
        next->i_limit == old->i_limit && next->out_limit == old->out_limit &&
        next->d_cutoff_freq == old->d_cutoff_freq && next->range == old->range)
    {
        return;
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
}

/**
 * @brief 集中处理单轮待机、运行、离线与恢复转换
 *
 * @param[in,out] wheel 当前单轮状态
 * @param[in] config 已校验的底盘配置，待机时允许配置非法
 * @param[in] run 本周期是否请求运行
 * @param[in] period_s 实测控制周期，运行时须在 (0, 0.1] s 内
 * @return 无返回值
 */
static void Chassis_ControlWheel(Chassis_Wheel_t *wheel, const Chassis_Config_t *config, bool run, float period_s)
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
            Chassis_RunWheel(wheel, config, period_s);
            break;

        case CHASSIS_WHEEL_OFFLINE:
        case CHASSIS_WHEEL_RELAX:
        default:
            Chassis_ResetWheel(wheel);
            break;
    }
}

/**
 * @brief 执行单轮限幅、换向保护、斜坡和速度闭环
 *
 * @param[in,out] wheel 运行中的单轮状态，反馈须有限
 * @param[in] config 已校验的底盘配置
 * @param[in] period_s 实测控制周期，单位 s
 * @return 无返回值，计算结果非法时复位本轮并转入待机
 */
static void Chassis_RunWheel(Chassis_Wheel_t *wheel, const Chassis_Config_t *config, float period_s)
{
    float target = wheel->requested_speed_rpm;

    // 目标反转先清除旧方向积分，再等待实测转速接近零后放行反向目标
    if ((wheel->target_speed_rpm > 0.0F && target < 0.0F) ||
        (wheel->target_speed_rpm < 0.0F && target > 0.0F))
    {
        PID_ResetIntegral(&wheel->speed_pid);
    }
    if ((wheel->measured_speed_rpm > config->reverse_release_speed_rpm && target < 0.0F) ||
        (wheel->measured_speed_rpm < -config->reverse_release_speed_rpm && target > 0.0F))
    {
        target = 0.0F;
    }

    // 按实测周期推进斜坡，然后依次计算反馈滤波、速度 PID 和电流滤波
    wheel->target_speed_rpm = RampTo(wheel->target_speed_rpm, target, config->wheel_accel_rpm_s * period_s);
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
