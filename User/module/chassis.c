/*
 * 底盘控制模块
 *
 * 消费 CAN 四轮反馈、云台 yaw 反馈与底盘命令，经斜坡、运动学混控和轮速闭环输出四路电流
 */

/* Includes ----------------------------------------------------------------- */
#include "module/chassis.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

/* Private define ----------------------------------------------------------- */
/* 控制周期上限取自底盘重构方案，超过 100 ms 时禁止追赶计算 */
#define CHASSIS_MAX_PERIOD_S (0.1F) /* 控制周期上限，单位：s */

/* 跟随反馈时效沿用云台电机反馈上限 90 毫秒；电机转速由 rpm 换算为 rad/s，每分钟 60 秒、每圈 2π 弧度 */
#define CHASSIS_FOLLOW_TIMEOUT_US (90000U)   /* 跟随反馈帧龄上限，单位：us */
#define CHASSIS_RPM_TO_RAD_S (M_2PI / 60.0F) /* rpm 转 rad/s 系数 */

_Static_assert(CHASSIS_WHEEL_COUNT == CAN_DEVICES_CHASSIS_MOTOR_COUNT, "底盘与 CAN 电机数量必须一致");

/* 四轮前馈初值取自 2026-11-10 架空正向 30 rpm 实测，单位：A/rpm */
/* 依次为左前 0.0137、左后 0.0231、右后 0.0097、右前 0.0092，调试器写入后由控制入口逐周期采纳 */
/* 适用范围及零值关闭语义见头文件 */
volatile float chassis_speed_feedforward_gain_a_per_rpm[CHASSIS_WHEEL_COUNT] =
{
    0.0137F, 0.0231F, 0.0097F, 0.0092F,
};

/* Private typedef ---------------------------------------------------------- */
/* 单一底盘实例的私有跟随控制状态，由底盘任务独占 */
typedef struct
{
    Chassis_Config_t cycle_config; /* 最近一次在控制入口通过校验的持久配置快照 */
    KPID_t follow_pid;             /* 跟随角度 PID 的积分、微分及上一输出历史 */
    bool yaw_feedback_valid;       /* 本周期反馈入口产生的时效、角度及连续性校验结果 */
} Chassis_Internal_t;

/* Private variables -------------------------------------------------------- */
static Chassis_Internal_t chassis_internal; /* 唯一实例的私有跟随状态 */

/* Private function prototypes ---------------------------------------------- */
static bool Chassis_IsConfigValid(const Chassis_Config_t *config, float frequency_hz);
static void Chassis_ResetWheel(Chassis_Wheel_t *wheel);
static void Chassis_PrimeWheel(Chassis_Wheel_t *wheel);
static bool Chassis_ApplyPidConfig(Chassis_t *chassis, const Chassis_Config_t *config);
static void Chassis_RampMove(MoveVector_t *ramped, const MoveVector_t *target, float scale_rpm,
                             float accel_rpm_s, float period_s);
static void Chassis_ControlWheel(Chassis_Wheel_t *wheel, const Chassis_Config_t *config, bool run,
                                 float period_s, float feedforward_gain);
static void Chassis_RunWheel(Chassis_Wheel_t *wheel, const Chassis_Config_t *config, float period_s,
                             float feedforward_gain);

/* Exported functions ------------------------------------------------------- */
/**
 * @brief 复制配置并初始化运动学、跟随 PID、四轮 PID 与滤波器
 *
 * @param chassis 待初始化的底盘实例，初始化后不得整体复制
 * @param config 默认配置
 * @param frequency_hz 控制采样频率，单位：Hz
 * @return int8_t 成功返回 0，失败返回 -1 并保持未初始化和零输出
 */
int8_t Chassis_Init(Chassis_t *chassis, const Chassis_Config_t *config, float frequency_hz)
{
    /* 清空旧状态并标记缺失反馈，保证所有初始化失败路径输出零电流 */
    memset(chassis, 0, sizeof(*chassis));
    memset(&chassis_internal, 0, sizeof(chassis_internal));
    Chassis_UpdateFeedback(chassis, NULL);
    Chassis_UpdateGimbalFeedback(chassis, NULL, 0U, 0.0F, false);
    chassis->config = *config;
    chassis->config_valid = Chassis_IsConfigValid(config, frequency_hz);
    if (!chassis->config_valid)
    {
        return -1;
    }

    /* 跟随参数保存在持久快照中，避免控制器引用周期栈上的临时配置 */
    chassis_internal.cycle_config = *config;
    if (PID_Init(&chassis_internal.follow_pid, KPID_MODE_SET_D, frequency_hz,
                 &chassis_internal.cycle_config.follow_pid) != 0)
    {
        return -1;
    }

    /* 用零向量验证布局支持四轮输出，避免二轮布局被错误用于底盘 */
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

    /* 四轮引用同一份参数，各自持有独立积分、微分及两级滤波状态 */
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
 * @param chassis 底盘实例
 * @param can 本周期 CAN 快照，无新消息时为 NULL，缺失或无效反馈写入 NaN
 * @return 无返回值
 */
void Chassis_UpdateFeedback(Chassis_t *chassis, const CANDevices_Snapshot_t *can)
{
    /* 设备整数协议换算保证数值有限，此处只接纳本周期更新成功且在线的反馈 */
    for (uint32_t i = 0U; i < CHASSIS_WHEEL_COUNT; i++)
    {
        chassis->wheel[i].measured_speed_rpm =
            can != NULL && can->feedback_update_status[i] == CAN_DEVICES_OK && can->motor_online[i] ?
            can->actual_speed_rpm[i] : NAN;
    }
}

/**
 * @brief 校验云台机械反馈并更新相对车头角度及相对角速度
 *
 * @param chassis 底盘实例
 * @param feedback 最近 yaw 反馈，没有缓存时为 NULL
 * @param now_us 单调微秒时间
 * @param zero_deg 已标定的方向修正后编码器零点，单位：度
 * @param reversed 编码器方向修正开关
 * @return 无返回值
 */
void Chassis_UpdateGimbalFeedback(Chassis_t *chassis, const MOTOR_RM_Feedback_t *feedback,
                                   uint64_t now_us, float zero_deg, bool reversed)
{
    /* 最新帧可能尚未到达或已经超时，帧龄用低位无符号差处理时间戳回绕 */
    const uint32_t age_us = feedback != NULL ? (uint32_t)now_us - feedback->last_update_time : UINT32_MAX;
    if (feedback == NULL || age_us > CHASSIS_FOLLOW_TIMEOUT_US ||
        !feedback->angle_valid || feedback->angle_lost_count != 0U ||
        !(feedback->rotor_single_angle >= 0.0F && feedback->rotor_single_angle < M_2PI))
    {
        chassis_internal.yaw_feedback_valid = false;
        chassis->follow_relative_angle_deg = NAN;
        chassis->follow_relative_speed_rad_s = NAN;
        return;
    }

    /* 最短机械角用于跟随与坐标转换，整数协议转速换算为直接微分输入 */
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
 * @param chassis 底盘实例
 * @param command 归一化控制命令，无命令时为 NULL，非法命令按待机处理
 * @param period_s 实测控制周期，单位：s
 * @return 无返回值
 */
void Chassis_Control(Chassis_t *chassis, const Chassis_Command_t *command, float period_s)
{
    /* 初始化失败的实例没有可复位控制器，输出由既有待机状态统一阻断 */
    if (!chassis->initialized)
    {
        return;
    }

    /* 配置与前馈系数可被调试器异步改写，本周期只校验并使用入口快照 */
    const Chassis_Config_t snapshot = *(const volatile Chassis_Config_t *)&chassis->config;
    float feedforward_gain[CHASSIS_WHEEL_COUNT];
    bool feedforward_valid = true;
    for (uint32_t i = 0U; i < CHASSIS_WHEEL_COUNT; i++)
    {
        feedforward_gain[i] = chassis_speed_feedforward_gain_a_per_rpm[i];
        feedforward_valid = isfinite(feedforward_gain[i]) && feedforward_gain[i] >= 0.0F && feedforward_valid;
    }
    const float frequency_hz = 1.0F / chassis->wheel[0].speed_pid.dt_min;
    chassis->config_valid = Chassis_IsConfigValid(&snapshot, frequency_hz) && feedforward_valid;
    bool follow_config_changed = false;
    if (chassis->config_valid)
    {
        follow_config_changed = Chassis_ApplyPidConfig(chassis, &snapshot);
    }

    /* 默认整组待机，只有合法运动命令、有效周期和合法轮速尺度才能请求运动学计算 */
    Chassis_Mode_t next_mode = CHASSIS_MODE_RELAX;
    float requested_speed[CHASSIS_WHEEL_COUNT] =
    {
        0,
    };
    if (chassis->config_valid && command != NULL && isfinite(period_s) &&
        period_s > 0.0F && period_s <= CHASSIS_MAX_PERIOD_S &&
        isfinite(command->wheel_speed_scale_rpm) && command->wheel_speed_scale_rpm >= 0.0F)
    {
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

    /* 跟随输出只由本周期 PID 产生，进入跟随前的历史已在初始化或上次退出时清除 */
    const bool follow_requested = next_mode == CHASSIS_MODE_FOLLOW_GIMBAL;
    chassis->follow_wz = 0.0F;
    MoveVector_t move =
    {
        0,
    };
    if (next_mode == CHASSIS_MODE_FOLLOW_GIMBAL || next_mode == CHASSIS_MODE_SPIN)
    {
        /* 指令斜坡先限制云台坐标平移与自旋分量的变化率，旋转和混控放在斜坡之后 */
        const float scale_rpm = fminf(command->wheel_speed_scale_rpm, snapshot.max_wheel_speed_rpm);
        Chassis_RampMove(&chassis->ramped_move, &command->move, scale_rpm, snapshot.wheel_accel_rpm_s, period_s);
        move = chassis->ramped_move;

        /* 平移按云台相对车头的角度旋转到车体坐标，轮速随旋转的变化不再受斜坡限制 */
        const float relative_rad = chassis->follow_relative_angle_deg * M_DEG2RAD_MULT;
        const float cosine = cosf(relative_rad);
        const float sine = sinf(relative_rad);
        move.vx = cosine * chassis->ramped_move.vx - sine * chassis->ramped_move.vy;
        move.vy = sine * chassis->ramped_move.vx + cosine * chassis->ramped_move.vy;

        /* 跟随输出直接叠加在斜坡后的自旋分量上，闭环输出不进入斜坡 */
        if (follow_requested)
        {
            /* 运行中改参先清除旧历史，退出跟随时由后续统一清理，避免重复复位 */
            if (follow_config_changed)
            {
                PID_Reset(&chassis_internal.follow_pid);
            }
            const float follow_output = PID_Calc(&chassis_internal.follow_pid, 0.0F, relative_rad,
                                                 chassis->follow_relative_speed_rad_s, period_s);
            move.wz = chassis->ramped_move.wz + follow_output;
            chassis->follow_wz = follow_output;
        }

        /* 混控入口检查运动分量并生成本周期轮速，失败时取消本周期运行请求 */
        if (Mixer_Apply(&chassis->mixer, &move, requested_speed, CHASSIS_WHEEL_COUNT, scale_rpm) != MIXER_OK)
        {
            next_mode = CHASSIS_MODE_RELAX;
        }
    }

    /* 斜坡状态只在跟随与自旋之间保持，待机、独立模式和混控失败时集中清零 */
    if (next_mode != CHASSIS_MODE_FOLLOW_GIMBAL && next_mode != CHASSIS_MODE_SPIN)
    {
        ResetMoveVector(&chassis->ramped_move);
    }

    /* 退出跟随或本周期跟随计算被混控拒绝时清理历史，持续待机不重复复位 */
    if (next_mode != CHASSIS_MODE_FOLLOW_GIMBAL &&
        (chassis->mode == CHASSIS_MODE_FOLLOW_GIMBAL || follow_requested))
    {
        PID_Reset(&chassis_internal.follow_pid);
        chassis->follow_wz = 0.0F;
    }

    /* 模式切换统一清除动态历史，随后由单轮状态机决定是否从实测转速起步 */
    for (uint32_t i = 0U; i < CHASSIS_WHEEL_COUNT; i++)
    {
        Chassis_Wheel_t *wheel = &chassis->wheel[i];
        if (chassis->mode != next_mode)
        {
            wheel->state = CHASSIS_WHEEL_RELAX;
        }
        wheel->requested_speed_rpm = next_mode != CHASSIS_MODE_RELAX ? requested_speed[i] : 0.0F;
        Chassis_ControlWheel(wheel, &chassis_internal.cycle_config, next_mode != CHASSIS_MODE_RELAX,
                               period_s, feedforward_gain[i]);
    }
    chassis->mode = next_mode;
}

/**
 * @brief 导出四路电流，保留调用方设置的命令序号
 *
 * @param chassis 底盘实例，未初始化时输出全零
 * @param output CAN 电流命令，仅写 current_a[0~3]
 * @return 无返回值
 */
void Chassis_DumpOutput(const Chassis_t *chassis, CANDevices_Command_t *output)
{
    /* 数值与限幅已在计算出口处理，最终只按运行状态决定是否向通信任务输出 */
    for (uint32_t i = 0U; i < CHASSIS_WHEEL_COUNT; i++)
    {
        output->current_a[i] = (chassis->mode == CHASSIS_MODE_INDEPENDENT ||
            chassis->mode == CHASSIS_MODE_FOLLOW_GIMBAL || chassis->mode == CHASSIS_MODE_SPIN) &&
            chassis->wheel[i].state == CHASSIS_WHEEL_RUNNING ? chassis->wheel[i].output_current_a : 0.0F;
    }
}

/**
 * @brief 查询本周期是否至少一轮取得有效在线反馈，接口说明见头文件
 *
 * @param chassis 已完成本周期反馈更新的底盘实例
 * @return bool 任一轮实测转速有效时返回 true
 */
bool Chassis_HasOnlineWheel(const Chassis_t *chassis)
{
    /* 反馈入口已把离线、更新失败及缺少快照的轮速写为非数值，此处不重复检查 CAN 状态 */
    for (uint32_t i = 0U; i < CHASSIS_WHEEL_COUNT; i++)
    {
        if (isfinite(chassis->wheel[i].measured_speed_rpm))
        {
            return true;
        }
    }
    return false;
}

/* Private functions -------------------------------------------------------- */
/**
 * @brief 在初始化及周期入口集中校验底盘配置、公共速度 PID 和跟随 PID
 *
 * @param config 待校验配置
 * @param frequency_hz 初始化时的采样频率，单位：Hz
 * @return bool 所有数值有限且范围合法时返回 true
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
 * @param wheel 待复位的单轮状态
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
 * @brief 用实测转速预置恢复运行时的反馈与微分历史
 *
 * @param wheel 反馈有效的单轮状态
 * @return 无返回值
 */
static void Chassis_PrimeWheel(Chassis_Wheel_t *wheel)
{
    /* 预置速度与电流滤波器，使恢复运行时不沿用停机前历史 */
    PID_Reset(&wheel->speed_pid);
    wheel->filtered_speed_rpm = LowPassFilter2p_Reset(&wheel->speed_filter, wheel->measured_speed_rpm);
    LowPassFilter2p_Reset(&wheel->current_filter, 0.0F);

    /* 同步微分上一反馈，避免恢复使能时产生一次性电流冲击 */
    const float scaled_feedback = wheel->speed_pid.param->k * wheel->filtered_speed_rpm;
    LowPassFilter2p_Reset(&wheel->speed_pid.dfilter, scaled_feedback);
    wheel->speed_pid.last.k_fb = scaled_feedback;
}

/**
 * @brief 采纳持久配置快照并同步四轮参数，返回跟随参数变更供控制入口处理历史
 *
 * @param chassis 配置合法且初始化成功的底盘实例
 * @param config 本周期已校验的配置快照
 * @return bool 跟随参数改变时返回 true
 */
static bool Chassis_ApplyPidConfig(Chassis_t *chassis, const Chassis_Config_t *config)
{
    /* 跟随控制器长期引用持久快照，更新前检测变更，滤波参数只在改变时同步 */
    const KPID_Params_t *follow = &config->follow_pid;
    const KPID_Params_t *applied = &chassis_internal.cycle_config.follow_pid;
    const bool follow_changed = follow->k != applied->k || follow->p != applied->p ||
        follow->i != applied->i || follow->d != applied->d || follow->i_limit != applied->i_limit ||
        follow->out_limit != applied->out_limit || follow->d_cutoff_freq != applied->d_cutoff_freq;
    const bool cutoff_changed = follow->d_cutoff_freq != applied->d_cutoff_freq;
    chassis_internal.cycle_config = *config;
    if (cutoff_changed)
    {
        LowPassFilter2p_Init(&chassis_internal.follow_pid.dfilter,
                              1.0F / chassis_internal.follow_pid.dt_min, follow->d_cutoff_freq);
    }

    /* 四轮参数继续使用公共副本，只有轮速参数变更才清除轮速积分 */
    const KPID_Params_t *next = &config->wheel_speed_pid;
    const KPID_Params_t *old = &chassis->applied_wheel_speed_pid;
    if (next->k == old->k && next->p == old->p && next->i == old->i && next->d == old->d &&
        next->i_limit == old->i_limit && next->out_limit == old->out_limit &&
        next->d_cutoff_freq == old->d_cutoff_freq && next->range == old->range)
    {
        return follow_changed;
    }

    /* 只在参数变更时复制一次；截止频率变化时同步重建各轮微分滤波系数 */
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
 * @param ramped 持久斜坡状态
 * @param target 本周期目标运动向量
 * @param scale_rpm 归一化轮速尺度，单位：rpm
 * @param accel_rpm_s 纯平移时等效的单轮转速变化率，单位：rpm/s
 * @param period_s 实测控制周期，单位：s
 * @return 无返回值
 */
static void Chassis_RampMove(MoveVector_t *ramped, const MoveVector_t *target, float scale_rpm,
                             float accel_rpm_s, float period_s)
{
    /* 尺度为零时混控输出恒为零，直接跟随目标，避免除零 */
    if (scale_rpm == 0.0F)
    {
        *ramped = *target;
        return;
    }

    /* 归一化步长由配置换算：纯前后或纯左右推杆时单轮转速变化率等于 accel_rpm_s */
    const float step = accel_rpm_s / scale_rpm * period_s;

    /* 平移按二维向量整体限步，避免两个分量到位时间不同导致起步过程中方向偏转 */
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
 * @param wheel 当前单轮状态
 * @param config 已校验的底盘配置，待机时允许配置非法
 * @param run 本周期是否请求运行
 * @param period_s 实测控制周期，单位：s
 * @param feedforward_gain 本轮周期快照中的速度前馈系数，单位：A/rpm
 * @return 无返回值
 */
static void Chassis_ControlWheel(Chassis_Wheel_t *wheel, const Chassis_Config_t *config, bool run,
                                 float period_s, float feedforward_gain)
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
            /* 非法状态本周期强制待机，下周期才允许重新预置运行 */
            break;
    }

    /* 入口动作只在恢复运行时预置，非运行状态持续清除历史并维持零输出 */
    if (next_state == CHASSIS_WHEEL_RUNNING && wheel->state != CHASSIS_WHEEL_RUNNING)
    {
        Chassis_PrimeWheel(wheel);
    }
    wheel->state = next_state;
    switch (wheel->state)
    {
        case CHASSIS_WHEEL_RUNNING:
            Chassis_RunWheel(wheel, config, period_s, feedforward_gain);
            break;

        case CHASSIS_WHEEL_OFFLINE:
        case CHASSIS_WHEEL_RELAX:
        default:
            Chassis_ResetWheel(wheel);
            break;
    }
}

/**
 * @brief 执行单轮目标方向切换、速度闭环和前馈电流合成
 *
 * @param wheel 运行中的单轮状态，反馈须有限
 * @param config 已校验的底盘配置
 * @param period_s 实测控制周期，单位：s
 * @param feedforward_gain 本周期已校验的有限非负系数，单位：A/rpm
 * @return 无返回值，计算结果非法时复位本轮并转入待机
 */
static void Chassis_RunWheel(Chassis_Wheel_t *wheel, const Chassis_Config_t *config, float period_s,
                             float feedforward_gain)
{
    /* 目标变号时清除旧方向积分，目标变化率已由上层指令斜坡限制 */
    const float target = wheel->requested_speed_rpm;
    if ((wheel->target_speed_rpm > 0.0F && target < 0.0F) ||
        (wheel->target_speed_rpm < 0.0F && target > 0.0F))
    {
        PID_ResetIntegral(&wheel->speed_pid);
    }

    /* 轮速层不再二次斜坡，目标直接进入速度环和前馈 */
    wheel->target_speed_rpm = target;
    wheel->filtered_speed_rpm = LowPassFilter2p_Apply(&wheel->speed_filter, wheel->measured_speed_rpm);
    const float feedforward_a = feedforward_gain * wheel->target_speed_rpm;

    /* 速度环只修正反馈误差，合成电流统一经过现有滤波、计算出口检查和最终限流 */
    const float current = PID_Calc(&wheel->speed_pid, wheel->target_speed_rpm, wheel->filtered_speed_rpm,
                                   0.0F, period_s) + feedforward_a;
    wheel->output_current_a = LowPassFilter2p_Apply(&wheel->current_filter, current);

    /* 非有限计算只隔离本轮，下一周期重新预置后才恢复闭环 */
    if (!isfinite(wheel->filtered_speed_rpm) || !isfinite(wheel->output_current_a))
    {
        Chassis_ResetWheel(wheel);
        wheel->state = CHASSIS_WHEEL_RELAX;
        return;
    }
    wheel->output_current_a = AbsClip(wheel->output_current_a, config->max_current_a);
}
