/*
 * 云台控制模块
 *
 * 消费双路电机反馈与惯性快照，经角度外环、速度内环、机械限位与前馈补偿输出两轴电流
 */

/* Includes ----------------------------------------------------------------- */
#include "module/gimbal.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

#include "component/filter.h"
#include "component/user_math.h"

/* Private define ----------------------------------------------------------- */
/* 周期上限 0.1 秒、PID 增益上限 1000；rpm 按每分钟 60 秒、每圈 2π 换算 */
/* 惯性角速度上限 40 rad/s，俯仰奇异点与相邻姿态跳变量各保留 1 度余量，编码器单圈范围在反馈入口校验 */
#define GIMBAL_MAX_PERIOD_S (0.1f)           /* 控制周期上限，单位：s */
#define GIMBAL_MAX_GAIN (1000.0f)            /* PID 增益上限 */
#define GIMBAL_RPM_TO_RAD_S (0.10471975512f) /* rpm 转 rad/s 系数 */
#define GIMBAL_IMU_MAX_RAD_S (40.0f)         /* 惯性角速度上限，单位：rad/s */
#define GIMBAL_IMU_MAX_PITCH_DEG (89.0f)     /* 俯仰角绝对上限，单位：度 */
#define GIMBAL_IMU_JUMP_MARGIN_DEG (1.0f)    /* 相邻帧跳变余量，单位：度 */

/* 惯性角速度滤波参数：小陀螺时结构共振主峰 13.5 Hz、次峰 11.0/14.1/19.0 Hz，均落在云台控制环带宽之外 */
/* 宽带阻覆盖约 9.5~17.5 Hz，低通进一步下压 19 Hz 与更高频段的残余振动 */
#define GIMBAL_RATE_LPF_HZ (20.0f)           /* 大地系角速度低通截止频率，单位：Hz */
#define GIMBAL_RATE_NOTCH_HZ (13.5f)         /* 结构共振带阻中心频率，单位：Hz */
#define GIMBAL_RATE_NOTCH_BW_HZ (8.0f)       /* 结构共振带阻带宽，单位：Hz */
/* yaw 单独取值：激励辨识显示带阻与 20 Hz 低通在 6 Hz 附近滞后约 46 度，速度环增益裕度只剩约 1 dB */
/* 小陀螺扰动随电机相对转速移动，不是固定频率共振，yaw 关闭带阻并把低通提高到 40 Hz 换回相位 */
#define GIMBAL_YAW_RATE_LPF_HZ (40.0f)       /* yaw 大地系角速度低通截止频率，单位：Hz */
#define GIMBAL_YAW_RATE_NOTCH_HZ (0.0f)      /* yaw 带阻中心频率，单位：Hz，0 表示直通 */

/* 摩擦前馈只在准静态生效：目标角速度大时线性淡出，避免大机动里被同向推力推过头 */
#define GIMBAL_FRICTION_FADE_FULL_RAD_S (0.20f) /* 目标角速度低于该值时前馈全量，单位：rad/s */
#define GIMBAL_FRICTION_FADE_ZERO_RAD_S (0.60f) /* 目标角速度高于该值时前馈完全淡出，单位：rad/s */

#define GIMBAL_FULL_TURN_DEG (360.0f)        /* 整圈角度，单位：度 */
#define GIMBAL_AXIS_COUNT (2U)               /* 云台轴数 */

/* 机械边界减速增益，单位：1/s，允许的朝边界速度 = 增益 × 到边界的距离，边界处连续降为零 */
/* 取值为估计起点，增益越大越晚减速，越小越早减速，需按实测的冲限位量整定 */
#define GIMBAL_LIMIT_SPEED_GAIN (20.0f) /* 机械边界减速增益，单位：1/s */

/* yaw 撤驱保护阈值，均为待实物确认的起点 */
/* 底盘离线时编码器与惯性转速差超过 1 rad/s 且上部惯性转速低于 0.3 rad/s，持续 0.15 s 判为悬空带转 */
/* 四个底盘电机全部离线持续 1 s 才启用悬空带转检测 */
#define GIMBAL_YAW_DRAG_RAD_S (1.0f)           /* 编码器与惯性转速差阈值，单位：rad/s */
#define GIMBAL_YAW_DRAG_IMU_STILL_RAD_S (0.3f) /* 上部惯性转速静置阈值，单位：rad/s */
#define GIMBAL_YAW_DRAG_TIME_S (0.15f)         /* 悬空带转判定时间，单位：s */
#define GIMBAL_CHASSIS_OFFLINE_TIME_S (1.0f)   /* 底盘全离线启用检测时间，单位：s */

/* Private typedef ---------------------------------------------------------- */
/* 云台机械角展开器的私有历史 */
typedef struct
{
    bool has_reference;       /* 已建立连续角参考时为 true */
    float last_wrapped_angle; /* 上一帧单圈角度，单位：度 */
    float continuous_angle;   /* 展开后的连续机械角，单位：度 */
} Gimbal_AngleUnwrap_t;

/* 单轴目标设定值，内部捕获后实际使用的姿态目标与速度前馈 */
typedef struct
{
    float target_angle_deg;   /* 姿态目标，单位：度 */
    float target_speed_rad_s; /* 速度目标或前馈，单位：rad/s */
} Gimbal_AxisSetpoint_t;

/* 单轴前馈种类：yaw 只叠加摩擦补偿，pitch 只叠加重力补偿，两者按轴身份固定 */
typedef enum
{
    GIMBAL_FEEDFORWARD_NONE = 0,      /* 不叠加任何前馈 */
    GIMBAL_FEEDFORWARD_YAW_FRICTION,  /* 按限幅后的闭环目标速度叠加二次摩擦模型 */
    GIMBAL_FEEDFORWARD_PITCH_GRAVITY, /* 按大地系俯仰角叠加重力前馈 */
} Gimbal_Feedforward_t;

/* 单轴私有控制状态 */
typedef struct
{
    Gimbal_AxisSetpoint_t setpoint;  /* 捕获后的姿态目标与速度前馈 */
    uint64_t feedback_time_us;       /* 最近有效反馈时刻，单位：us */
    float applied_zero_encoder_deg;  /* 当前机械角零点，单位：度 */
    bool applied_reversed;           /* 当前已应用的电机方向配置 */
    KPID_Params_t applied_angle_pid; /* 角度环已安装参数 */
    KPID_Params_t applied_speed_pid; /* 速度环已安装参数 */
    Gimbal_AngleUnwrap_t unwrap;     /* yaw 连续机械角展开状态，pitch 不使用 */
    KPID_t angle_pid;                /* 角度外环运行状态 */
    KPID_t speed_pid;                /* 速度内环运行状态 */
} Gimbal_AxisInternal_t;

/* 单一云台实例的模块私有状态 */
typedef struct
{
    Gimbal_Config_t control_config;   /* 最近一次通过 Control 入口校验的配置快照 */
    Gimbal_ImuFeedback_t imu;         /* 最近一帧有效或等待校验的惯性反馈缓存 */
    LowPassFilter2p_t yaw_rate_lpf;   /* yaw 大地系角速度低通 */
    LowPassFilter2p_t pitch_rate_lpf; /* pitch 大地系角速度低通 */
    NotchFilter_t yaw_rate_notch;     /* yaw 结构共振带阻 */
    NotchFilter_t pitch_rate_notch;   /* pitch 结构共振带阻 */
    Gimbal_AxisInternal_t yaw;        /* yaw 轴私有控制器与反馈历史 */
    Gimbal_AxisInternal_t pitch;      /* pitch 轴私有控制器与反馈历史 */
    float chassis_offline_s;          /* 底盘持续离线时间，单位：s */
    float yaw_drag_s;                 /* yaw 满足悬空带转条件的累计时间，单位：s */
} Gimbal_Internal_t;

/* Private variables -------------------------------------------------------- */
static Gimbal_Internal_t gimbal_internal; /* 唯一实例的私有运行状态 */

/* Private function --------------------------------------------------------- */
/**
 * @brief 将有限角度环绕到单圈区间 [0, period)
 *
 * @param angle 待环绕角度，单位：度
 * @param period 环绕周期，与 angle 单位一致
 * @return float 环绕到 [0, period) 的角度，单位：度
 */
static float Gimbal_WrapAngle(float angle, float period)
{
    float wrapped = fmodf(angle, period);
    if (wrapped < 0.0f)
    {
        wrapped += period;
    }
    /* 负的极小角度加整圈后可能舍入为整圈，统一归到零点 */
    return wrapped >= period ? 0.0f : wrapped;
}

/**
 * @brief 丢弃上一帧参考，下一次更新重新以单圈角度建立连续坐标
 *
 * @param unwrap 单圈角度展开状态
 * @return 无返回值
 */
static void Gimbal_ResetAngleUnwrap(Gimbal_AngleUnwrap_t *unwrap)
{
    unwrap->has_reference = false;
    unwrap->last_wrapped_angle = 0.0f;
    unwrap->continuous_angle = 0.0f;
}

/**
 * @brief 清空单轴机械反馈并断开连续角参考，待机、失联和故障路径共用
 *
 * @param axis 单轴公开状态
 * @param internal 单轴私有状态
 * @return 无返回值
 */
static void Gimbal_ClearFeedback(Gimbal_Axis_t *axis, Gimbal_AxisInternal_t *internal)
{
    axis->encoder_deg = NAN;
    axis->mechanical_angle_deg = NAN;
    axis->motor_speed_rad_s = NAN;
    internal->feedback_time_us = UINT64_MAX;
    Gimbal_ResetAngleUnwrap(&internal->unwrap);
}

/**
 * @brief 停止单轴输出并锁存下一状态，非零输出状态一律按故障锁存
 *
 * @param axis 单轴公开状态
 * @param internal 单轴私有状态
 * @param next_state 请求进入的下一状态
 * @return 无返回值
 */
static void Gimbal_StopAxis(Gimbal_Axis_t *axis, Gimbal_AxisInternal_t *internal, Gimbal_AxisState_t next_state)
{
    PID_Reset(&internal->angle_pid);
    PID_Reset(&internal->speed_pid);
    axis->target_angle_deg = NAN;
    axis->target_speed_rad_s = 0.0f;
    axis->output_current_a = 0.0f;
    const bool output_allowed = next_state == GIMBAL_AXIS_RELAX || next_state == GIMBAL_AXIS_OFFLINE ||
                                next_state == GIMBAL_AXIS_TRIPPED;
    axis->state = output_allowed ? next_state : GIMBAL_AXIS_FAULT;
}

/**
 * @brief 校验单个 PID 增益
 *
 * @param gain 待校验增益
 * @return bool 有限、非负且不超过配置上限时返回 true
 */
static bool Gimbal_IsGainValid(float gain)
{
    return isfinite(gain) && gain >= 0.0f && gain <= GIMBAL_MAX_GAIN;
}

/**
 * @brief 校验单个 PID 参数
 *
 * @param pid 待校验的 PID 参数
 * @param allow_angle_wrap 仅 pitch 角度环为 true，允许 range 取整圈 2π
 * @return bool 全部字段合法时返回 true
 */
static bool Gimbal_IsPidConfigValid(const KPID_Params_t *pid, bool allow_angle_wrap)
{
    /* 零值与负值关闭环绕，仅获得许可的角度环接受 2π 周期 */
    const bool range_valid = isfinite(pid->range) && (pid->range <= 0.0f || (allow_angle_wrap && pid->range == M_2PI));
    return Gimbal_IsGainValid(pid->k) && Gimbal_IsGainValid(pid->p) && Gimbal_IsGainValid(pid->i) &&
           Gimbal_IsGainValid(pid->d) && isfinite(pid->i_limit) && pid->i_limit >= 0.0f && isfinite(pid->out_limit) &&
           pid->out_limit > 0.0f && isfinite(pid->d_cutoff_freq) && range_valid;
}

/**
 * @brief 校验单轴机械参数、零点范围及两环 PID
 *
 * @param config 待校验的单轴配置
 * @param allow_angle_wrap 仅 pitch 轴为 true
 * @return bool 全部字段合法时返回 true
 */
static bool Gimbal_IsAxisConfigValid(const Gimbal_AxisConfig_t *config, bool allow_angle_wrap)
{
    return isfinite(config->zero_encoder_deg) && config->zero_encoder_deg >= 0.0f &&
           config->zero_encoder_deg < GIMBAL_FULL_TURN_DEG && isfinite(config->min_angle_deg) &&
           isfinite(config->max_angle_deg) && config->min_angle_deg <= config->max_angle_deg &&
           isfinite(config->max_speed_rad_s) && config->max_speed_rad_s > 0.0f &&
           isfinite(config->max_current_a) && config->max_current_a > 0.0f &&
           Gimbal_IsPidConfigValid(&config->angle_pid, allow_angle_wrap) &&
           Gimbal_IsPidConfigValid(&config->speed_pid, false);
}

/**
 * @brief 求二次摩擦模型电流幅值
 *
 * @param config 本周期配置快照
 * @param speed_rad_s 非负机械速度，单位：rad/s
 * @return float 摩擦前馈电流幅值，单位：A
 */
static float Gimbal_FrictionMagnitude(const Gimbal_Config_t *config, float speed_rad_s)
{
    return (config->yaw_friction_quadratic_a_per_rad_s2 * speed_rad_s +
            config->yaw_friction_linear_a_per_rad_s) * speed_rad_s + config->yaw_friction_current_a;
}

/**
 * @brief 校验摩擦参数及二次曲线的端点与区间极值
 *
 * @param config 本周期配置快照
 * @return bool 参数合法时返回 true，上限为零表示关闭补偿
 */
static bool Gimbal_IsFrictionConfigValid(const Gimbal_Config_t *config)
{
    /* 在线写入可能产生非有限系数或错误尺度，集中拒绝后再允许控制链读取 */
    const float max_current_a = config->yaw.max_current_a;
    const bool params_valid = isfinite(config->yaw_friction_current_a) && config->yaw_friction_current_a >= 0.0f &&
        config->yaw_friction_current_a <= max_current_a && isfinite(config->yaw_friction_smoothing_rad_s) &&
        config->yaw_friction_smoothing_rad_s > 0.0f && isfinite(config->yaw_friction_linear_a_per_rad_s) &&
        isfinite(config->yaw_friction_quadratic_a_per_rad_s2) && isfinite(config->yaw_friction_max_speed_rad_s) &&
        config->yaw_friction_max_speed_rad_s >= 0.0f;
    if (!params_valid)
    {
        return false;
    }
    /* 上限为零明确关闭补偿，兼容原配置中缺省为零的新增字段 */
    if (config->yaw_friction_max_speed_rad_s == 0.0f)
    {
        return true;
    }
    if (config->yaw_friction_max_speed_rad_s < config->yaw_friction_smoothing_rad_s)
    {
        return false;
    }

    /* 二次曲线的极值只可能位于端点或顶点，过渡段是零点与首端点的线性连接 */
    const float start_a = Gimbal_FrictionMagnitude(config, config->yaw_friction_smoothing_rad_s);
    const float end_a = Gimbal_FrictionMagnitude(config, config->yaw_friction_max_speed_rad_s);
    if (!isfinite(start_a) || start_a < 0.0f || start_a > max_current_a || !isfinite(end_a) || end_a < 0.0f ||
        end_a > max_current_a)
    {
        return false;
    }
    if (config->yaw_friction_quadratic_a_per_rad_s2 == 0.0f)
    {
        return true;
    }
    const float vertex_rad_s = (-0.5f * config->yaw_friction_linear_a_per_rad_s) /
                               config->yaw_friction_quadratic_a_per_rad_s2;
    if (vertex_rad_s <= config->yaw_friction_smoothing_rad_s || vertex_rad_s >= config->yaw_friction_max_speed_rad_s)
    {
        return true;
    }
    const float vertex_a = Gimbal_FrictionMagnitude(config, vertex_rad_s);
    return isfinite(vertex_a) && vertex_a >= 0.0f && vertex_a <= max_current_a;
}

/**
 * @brief 按目标角速度计算摩擦前馈淡出系数
 *
 * @param target_rad_s 限幅后的目标速度，单位：rad/s
 * @return float 前馈系数，目标速度低于全量阈值时为 1，高于淡出阈值时为 0
 */
static float Gimbal_FrictionFade(float target_rad_s)
{
    const float span_rad_s = GIMBAL_FRICTION_FADE_ZERO_RAD_S - GIMBAL_FRICTION_FADE_FULL_RAD_S;
    const float fade = (GIMBAL_FRICTION_FADE_ZERO_RAD_S - fabsf(target_rad_s)) / span_rad_s;
    return fminf(fmaxf(fade, 0.0f), 1.0f);
}

/**
 * @brief 用限幅后的闭环目标速度计算连续且有界的 yaw 摩擦前馈电流
 *
 * @param config 本周期配置快照
 * @param target_rad_s 限幅后的目标速度，单位：rad/s
 * @return float 摩擦前馈电流，单位：A
 */
static float Gimbal_CalcYawFriction(const Gimbal_Config_t *config, float target_rad_s)
{
    /* 零速附近线性连接，保证静止为零且换向连续 */
    const float width_rad_s = config->yaw_friction_smoothing_rad_s;
    if (fabsf(target_rad_s) < width_rad_s)
    {
        return Gimbal_FrictionMagnitude(config, width_rad_s) * (target_rad_s / width_rad_s);
    }
    /* 超出实测范围后固定使用边界模型值，避免高阶外推放大电流 */
    const float speed_rad_s = fminf(fabsf(target_rad_s), config->yaw_friction_max_speed_rad_s);
    return copysignf(Gimbal_FrictionMagnitude(config, speed_rad_s), target_rad_s);
}

/**
 * @brief 校验 pitch 重力前馈参数
 *
 * @param config 本周期配置快照
 * @return bool 幅值与偏置同向叠加后仍落回限幅内时返回 true
 */
static bool Gimbal_IsPitchGravityConfigValid(const Gimbal_Config_t *config)
{
    /* 在线写入可能产生非有限值或超出电流尺度的系数，集中拒绝后再允许控制链读取 */
    const float max_current_a = config->pitch.max_current_a;
    const bool params_valid = isfinite(config->pitch_gravity_current_a) && config->pitch_gravity_current_a >= 0.0f &&
        config->pitch_gravity_current_a <= max_current_a && isfinite(config->pitch_gravity_angle_deg) &&
        fabsf(config->pitch_gravity_angle_deg) <= 180.0f && isfinite(config->pitch_gravity_bias_a) &&
        fabsf(config->pitch_gravity_bias_a) <= max_current_a;
    if (!params_valid)
    {
        return false;
    }
    /* 幅值与常值项同向叠加即前馈请求的上界，须仍落在该轴最终限幅内 */
    return config->pitch_gravity_current_a + fabsf(config->pitch_gravity_bias_a) <= max_current_a;
}

/**
 * @brief 由大地系俯仰角计算 pitch 重力前馈电流
 *
 * @param config 本周期配置快照
 * @param angle_deg 大地系俯仰角，单位：度
 * @return float 重力前馈电流，单位：A
 */
static float Gimbal_CalcPitchGravity(const Gimbal_Config_t *config, float angle_deg)
{
    /* 重力力矩只随大地系俯仰角变化，编码器机械角含底座姿态偏差，不能替代 */
    const float phase_rad = angle_deg * M_DEG2RAD_MULT - config->pitch_gravity_angle_deg * M_DEG2RAD_MULT;
    return config->pitch_gravity_current_a * cosf(phase_rad) + config->pitch_gravity_bias_a;
}

/**
 * @brief 校验整体配置与实测控制频率
 *
 * @param config 待校验的配置
 * @param frequency_hz 控制频率，单位：Hz
 * @return bool 全部参数合法时返回 true
 */
static bool Gimbal_IsConfigValid(const Gimbal_Config_t *config, float frequency_hz)
{
    return isfinite(frequency_hz) && frequency_hz >= 10.0f && frequency_hz <= 2000.0f &&
        isfinite(config->feedback_timeout_s) && config->feedback_timeout_s >= 1.0e-6f &&
        config->feedback_timeout_s <= 1.0f && isfinite(config->imu_timeout_s) &&
        config->imu_timeout_s >= 1.0e-6f && config->imu_timeout_s <= 0.1f &&
        Gimbal_IsAxisConfigValid(&config->yaw, false) && Gimbal_IsAxisConfigValid(&config->pitch, true) &&
        Gimbal_IsFrictionConfigValid(config) && Gimbal_IsPitchGravityConfigValid(config) &&
        config->yaw.angle_pid.d_cutoff_freq < frequency_hz * 0.5f &&
        config->yaw.speed_pid.d_cutoff_freq < frequency_hz * 0.5f &&
        config->pitch.angle_pid.d_cutoff_freq < frequency_hz * 0.5f &&
        config->pitch.speed_pid.d_cutoff_freq < frequency_hz * 0.5f;
}

/**
 * @brief 判断配置中的 PID 参数与已安装副本是否不同
 *
 * @param config 配置中的 PID 参数
 * @param applied 已安装的 PID 参数副本
 * @return bool 任一字段不同时返回 true
 */
static bool Gimbal_IsPidChanged(const KPID_Params_t *config, const KPID_Params_t *applied)
{
    return config->k != applied->k || config->p != applied->p || config->i != applied->i ||
           config->d != applied->d || config->i_limit != applied->i_limit || config->out_limit != applied->out_limit ||
           config->d_cutoff_freq != applied->d_cutoff_freq || config->range != applied->range;
}

/**
 * @brief 初始化单轴 PID 并安装参数副本
 *
 * @param axis 单轴公开状态
 * @param internal 单轴私有状态
 * @param config 单轴配置
 * @param frequency_hz 控制频率，单位：Hz
 * @return int8_t 成功返回 0，失败返回 -1
 */
static int8_t Gimbal_InitAxis(Gimbal_Axis_t *axis, Gimbal_AxisInternal_t *internal, const Gimbal_AxisConfig_t *config,
                              float frequency_hz)
{
    /* PID 指向模块私有的持久参数副本，调试器修改配置后由每周期比较决定何时同步 */
    internal->applied_zero_encoder_deg = config->zero_encoder_deg;
    internal->applied_reversed = config->reversed;
    internal->applied_angle_pid = config->angle_pid;
    internal->applied_speed_pid = config->speed_pid;
    if (PID_Init(&internal->angle_pid, KPID_MODE_SET_D, frequency_hz, &internal->applied_angle_pid) != 0 ||
        PID_Init(&internal->speed_pid, KPID_MODE_CALC_D, frequency_hz, &internal->applied_speed_pid) != 0)
    {
        return -1;
    }
    axis->state = GIMBAL_AXIS_RELAX;
    return 0;
}

/**
 * @brief 同步单环 PID 参数副本，滤波尺度或速度环增益变化时重建微分滤波并清空积分
 *
 * @param pid 运行中的 PID
 * @param applied 已安装的 PID 参数副本
 * @param config 本周期配置中的 PID 参数
 * @param feedback_speed_rad_s 当前反馈速度，单位：rad/s
 * @param speed_loop 速度环为 true，角度环为 false
 * @return 无返回值
 */
static void Gimbal_SyncPid(KPID_t *pid, KPID_Params_t *applied, const KPID_Params_t *config, float feedback_speed_rad_s,
                           bool speed_loop)
{
    if (!Gimbal_IsPidChanged(config, applied))
    {
        return;
    }
    /* 微分截止频率变化或速度环增益变化时重建滤波器，避免沿用旧尺度 */
    if (config->d_cutoff_freq != applied->d_cutoff_freq || (speed_loop && config->k != applied->k))
    {
        LowPassFilter2p_Init(&pid->dfilter, 1.0f / pid->dt_min, config->d_cutoff_freq);
        if (speed_loop)
        {
            pid->last.k_fb = LowPassFilter2p_Reset(&pid->dfilter, config->k * feedback_speed_rad_s);
        }
    }
    *applied = *config;
    PID_ResetIntegral(pid);
}

/**
 * @brief 采纳单轴配置变更，零点或方向变化时锁存故障并同步两环参数
 *
 * @param axis 单轴公开状态
 * @param internal 单轴私有状态
 * @param config 本周期配置中的单轴配置
 * @return 无返回值
 */
static void Gimbal_ApplyAxisConfig(Gimbal_Axis_t *axis, Gimbal_AxisInternal_t *internal,
                                   const Gimbal_AxisConfig_t *config)
{
    /* 零点或方向变化后机械坐标失效，运行中锁存故障，下一帧重新建立机械角 */
    if (config->zero_encoder_deg != internal->applied_zero_encoder_deg ||
        config->reversed != internal->applied_reversed)
    {
        internal->applied_zero_encoder_deg = config->zero_encoder_deg;
        internal->applied_reversed = config->reversed;
        Gimbal_ClearFeedback(axis, internal);
        if (axis->state == GIMBAL_AXIS_RUNNING || axis->state == GIMBAL_AXIS_OFFLINE)
        {
            Gimbal_StopAxis(axis, internal, GIMBAL_AXIS_FAULT);
        }
    }
    /* 参数副本与配置不同时同步并清空积分，比例和微分项不受历史影响 */
    Gimbal_SyncPid(&internal->angle_pid, &internal->applied_angle_pid, &config->angle_pid, axis->speed_rad_s, false);
    Gimbal_SyncPid(&internal->speed_pid, &internal->applied_speed_pid, &config->speed_pid, axis->speed_rad_s, true);
}

/**
 * @brief 进入闭环前清空两环历史，速度环按当前反馈预置微分项，避免入口处的微分冲击
 *
 * @param axis 单轴公开状态
 * @param internal 单轴私有状态
 * @return 无返回值
 */
static void Gimbal_EnterRunning(Gimbal_Axis_t *axis, Gimbal_AxisInternal_t *internal)
{
    PID_Reset(&internal->angle_pid);
    PID_Reset(&internal->speed_pid);
    internal->speed_pid.last.k_fb = LowPassFilter2p_Reset(&internal->speed_pid.dfilter,
                                                          internal->applied_speed_pid.k * axis->speed_rad_s);
    axis->state = GIMBAL_AXIS_RUNNING;
}

/**
 * @brief 解析公开命令为轴内设定值，非法速度只在本入口判定
 *
 * @param axis 单轴公开状态
 * @param internal 单轴私有状态
 * @param command 本轴公开命令
 * @param mode 本周期云台模式
 * @param period_s 实测控制周期，单位：s
 * @return bool 速度与姿态目标均合法时返回 true
 */
static bool Gimbal_ResolveSetpoint(Gimbal_Axis_t *axis, Gimbal_AxisInternal_t *internal,
                                   const Gimbal_AxisCommand_t *command, Gimbal_Mode_t mode, float period_s)
{
    internal->setpoint.target_speed_rad_s = command->rate_rad_s;
    if (!isfinite(command->rate_rad_s))
    {
        return false;
    }
    if (mode != GIMBAL_MODE_ANGLE)
    {
        return true;
    }
    switch (command->source)
    {
        case GIMBAL_TARGET_INCREMENT:
        /* 非运行状态先解析本次增量，随后由起控捕获覆盖，避免沿用故障前的非法目标 */
            internal->setpoint.target_angle_deg = command->rate_rad_s * M_RAD2DEG_MULT * period_s;
            if (axis->state == GIMBAL_AXIS_RUNNING && isfinite(axis->target_angle_deg))
            {
                internal->setpoint.target_angle_deg += axis->target_angle_deg;
            }
            return isfinite(internal->setpoint.target_angle_deg);
        case GIMBAL_TARGET_ABSOLUTE:
            internal->setpoint.target_angle_deg = command->angle_deg;
            return isfinite(internal->setpoint.target_angle_deg);
        default:
            internal->setpoint.target_angle_deg = internal->setpoint.target_speed_rad_s = NAN;
            return false;
    }
}

/**
 * @brief 计算单轴闭环电流，含机械限位、前馈叠加与积分抗饱和
 *
 * @param axis 单轴公开状态
 * @param internal 单轴私有状态
 * @param config 本周期配置快照中的单轴配置
 * @param mode 本周期云台模式
 * @param command 轴内设定值
 * @param period_s 实测控制周期，单位：s
 * @param feedforward 本轴允许叠加的前馈种类
 * @return bool 计算链及积分有限时返回 true，异常时由调用方转入故障状态
 */
static bool Gimbal_RunAxis(Gimbal_Axis_t *axis, Gimbal_AxisInternal_t *internal, const Gimbal_AxisConfig_t *config,
                           Gimbal_Mode_t mode, const Gimbal_AxisSetpoint_t *command, float period_s,
                           Gimbal_Feedforward_t feedforward)
{
    /* 调用方已保证反馈就绪、命令合法及周期有效，本层只执行控制计算 */
    const float max_speed_rad_s = config->max_speed_rad_s;
    const float command_speed_rad_s = AbsClip(command->target_speed_rad_s, max_speed_rad_s);
    float target_speed_rad_s = command_speed_rad_s;
    const float previous_angle_integral = internal->angle_pid.i;
    bool mechanically_limited = false;
    if (mode == GIMBAL_MODE_ANGLE)
    {
        /* SPEED 模式直接采用命令角速度，ANGLE 模式将其作为前馈与角度环修正量合成 */
        float target_angle_deg = command->target_angle_deg;
        if (config->mechanical_limit_enabled)
        {
            /* 用目标姿态增量预测机械角，底座倾斜时不能直接夹住世界姿态目标 */
            const float min_target_deg = axis->angle_deg + config->min_angle_deg - axis->mechanical_angle_deg;
            const float max_target_deg = axis->angle_deg + config->max_angle_deg - axis->mechanical_angle_deg;
            Clip(&target_angle_deg, min_target_deg, max_target_deg);
            mechanically_limited = target_angle_deg != command->target_angle_deg;
        }
        axis->target_angle_deg = target_angle_deg;
        /* 角度环输入为弧度，微分反馈为大地系欧拉角速度，当前外环微分增益为零 */
        const float correction_rad_s = PID_Calc(&internal->angle_pid, target_angle_deg * M_DEG2RAD_MULT,
                                                axis->angle_deg * M_DEG2RAD_MULT, axis->speed_rad_s, period_s);
        const float combined_rad_s = correction_rad_s + command_speed_rad_s;
        target_speed_rad_s = AbsClip(combined_rad_s, max_speed_rad_s);
        /* 合成速度饱和且误差继续推向饱和方向时撤销本周期积分，保留反向退饱和能力 */
        if ((combined_rad_s > max_speed_rad_s && target_angle_deg > axis->angle_deg) ||
            (combined_rad_s < -max_speed_rad_s && target_angle_deg < axis->angle_deg))
        {
            internal->angle_pid.i = previous_angle_integral;
        }
    }
    else
    {
        axis->target_angle_deg = NAN;
    }
    /* 速度模式及前馈同样受机械边界约束，朝边界的速度随距离线性减小，越界时只允许零速或向行程内部运动 */
    if (config->mechanical_limit_enabled)
    {
        const float scale = M_DEG2RAD_MULT * GIMBAL_LIMIT_SPEED_GAIN;
        const float min_speed = fminf(0.0f, (config->min_angle_deg - axis->mechanical_angle_deg) * scale);
        const float max_speed = fmaxf(0.0f, (config->max_angle_deg - axis->mechanical_angle_deg) * scale);
        const float requested_speed = target_speed_rad_s;
        Clip(&target_speed_rad_s, min_speed, max_speed);
        mechanically_limited = mechanically_limited || requested_speed != target_speed_rad_s;
    }
    /* 速度环输出电流，非有限结果视为计算异常 */
    float current_a = PID_Calc(&internal->speed_pid, target_speed_rad_s, axis->speed_rad_s, 0.0f, period_s);
    /* 各轴只叠加本轴允许的前馈，参数只取本周期已采纳的整体配置快照 */
    const Gimbal_Config_t *const control_config = &gimbal_internal.control_config;
    /* 摩擦方向只取限幅与机械约束后的闭环目标速度，编码器差分速度量化粗且静止时跳变 */
    /* 参与方向判断会在零速附近让补偿电流逐周期翻转并反过来激励电机 */
    /* 大机动时目标速度远大于实测速度，前馈推力会与速度环反向制动叠加，因此按目标速度大小淡出 */
    if (feedforward == GIMBAL_FEEDFORWARD_YAW_FRICTION && control_config->yaw_friction_max_speed_rad_s != 0.0f)
    {
        const float fade = Gimbal_FrictionFade(target_speed_rad_s);
        current_a += fade * Gimbal_CalcYawFriction(control_config, target_speed_rad_s);
    }
    /* 重力前馈与运动方向无关，静止时同样出力，用于承担恒定偏置并给速度环留出动态余量 */
    if (feedforward == GIMBAL_FEEDFORWARD_PITCH_GRAVITY && control_config->pitch_gravity_enabled)
    {
        current_a += Gimbal_CalcPitchGravity(control_config, axis->angle_deg);
    }
    if (!isfinite(target_speed_rad_s))
    {
        return false;
    }
    axis->target_speed_rad_s = target_speed_rad_s;
    axis->output_current_a = AbsClip(current_a, config->max_current_a);
    /* 被边界限制时角度环积分保持原值，速度环积分清零，避免靠限位时残留积分持续顶向行程外侧 */
    /* 边界附近电流只由连续的速度夹限决定，机械角在限位两侧读数跳动也不会产生电流突变 */
    if (mechanically_limited)
    {
        internal->angle_pid.i = previous_angle_integral;
        PID_ResetIntegral(&internal->speed_pid);
    }
    /* 在合成计算链出口检查候选电流，防止限幅掩盖非有限结果，输出仍由状态闸门放行 */
    return isfinite(current_a) && isfinite(internal->speed_pid.i);
}

/**
 * @brief 根据运行许可和反馈推进单轴状态机，运行时计算闭环及该轴允许的补偿
 *
 * @param axis 单轴公开状态
 * @param internal 单轴私有状态
 * @param config 本周期配置快照中的单轴配置
 * @param mode 本周期云台模式
 * @param mode_changed 本周期模式与上一周期不同时为 true
 * @param command 轴内设定值
 * @param period_s 实测控制周期，单位：s
 * @param imu_ready 惯性反馈就绪时为 true
 * @param feedforward 本轴允许叠加的前馈种类，随轴身份固定
 * @return 无返回值
 */
static void Gimbal_ControlAxis(Gimbal_Axis_t *axis, Gimbal_AxisInternal_t *internal, const Gimbal_AxisConfig_t *config,
                               Gimbal_Mode_t mode, bool mode_changed, const Gimbal_AxisSetpoint_t *command,
                               float period_s, bool imu_ready, Gimbal_Feedforward_t feedforward)
{
    /* 参数已由 Control 入口同步，此处仅依据运行请求和明确反馈状态推进状态机 */
    const bool run_requested = config->enabled && (mode == GIMBAL_MODE_SPEED || mode == GIMBAL_MODE_ANGLE);
    const bool feedback_ready = imu_ready && internal->feedback_time_us != UINT64_MAX;

    switch (axis->state)
    {
        /* 待机与失联恢复共用起控路径，失联期间仍校验共用反馈和控制输入 */
        case GIMBAL_AXIS_RELAX:
        case GIMBAL_AXIS_OFFLINE:
            if (axis->state == GIMBAL_AXIS_OFFLINE && run_requested && !imu_ready)
            {
                Gimbal_StopAxis(axis, internal, GIMBAL_AXIS_FAULT);
                break;
            }
            if (!run_requested || !feedback_ready)
            {
                Gimbal_StopAxis(axis, internal, run_requested && axis->state == GIMBAL_AXIS_OFFLINE ?
                    GIMBAL_AXIS_OFFLINE : GIMBAL_AXIS_RELAX);
                break;
            }
            Gimbal_EnterRunning(axis, internal);
            if (!Gimbal_RunAxis(axis, internal, config, mode, command, period_s, feedforward))
            {
                Gimbal_StopAxis(axis, internal, GIMBAL_AXIS_FAULT);
            }
            break;
        case GIMBAL_AXIS_RUNNING:
            if (!run_requested)
            {
                Gimbal_StopAxis(axis, internal, GIMBAL_AXIS_RELAX);
                break;
            }
            if (mode_changed)
            {
                Gimbal_EnterRunning(axis, internal);
            }
            if (!feedback_ready || !Gimbal_RunAxis(axis, internal, config, mode, command, period_s, feedforward))
            {
                Gimbal_StopAxis(axis, internal, GIMBAL_AXIS_FAULT);
            }
            break;
        case GIMBAL_AXIS_FAULT:
        case GIMBAL_AXIS_TRIPPED:
            Gimbal_StopAxis(axis, internal, mode == GIMBAL_MODE_RELAX ? GIMBAL_AXIS_RELAX : axis->state);
            break;
        default:
            Gimbal_StopAxis(axis, internal, GIMBAL_AXIS_FAULT);
            break;
    }
}

/**
 * @brief 用相邻两帧最短路径展开单圈角度，无法唯一判断转动方向时丢弃参考
 *
 * @param unwrap 单圈角度展开状态
 * @param wrapped_angle 本帧单圈角度，单位：度
 * @param period 单圈周期，单位：度
 * @param continuous_angle 输出的连续角度，单位：度
 * @return bool 能唯一判断转动方向时返回 true
 */
static bool Gimbal_UpdateAngleUnwrap(Gimbal_AngleUnwrap_t *unwrap, float wrapped_angle, float period,
                                     float *continuous_angle)
{
    /* 首帧直接以单圈角度建立连续坐标 */
    if (!unwrap->has_reference)
    {
        unwrap->has_reference = true;
        unwrap->last_wrapped_angle = wrapped_angle;
        unwrap->continuous_angle = wrapped_angle;
        *continuous_angle = wrapped_angle;
        return true;
    }

    /* 将相邻帧差值折算到半圈以内，跨越编码器零点时取最短路径 */
    const float half_period = period * 0.5f;
    float delta = wrapped_angle - unwrap->last_wrapped_angle;
    if (delta > half_period)
    {
        delta -= period;
    }
    else if (delta < -half_period)
    {
        delta += period;
    }

    /* 恰为半圈时无法判断转动方向，丢弃参考避免累计错误圈数 */
    const float next_angle = unwrap->continuous_angle + delta;
    if (fabsf(delta) >= half_period || !isfinite(next_angle))
    {
        Gimbal_ResetAngleUnwrap(unwrap);
        return false;
    }

    unwrap->last_wrapped_angle = wrapped_angle;
    unwrap->continuous_angle = next_angle;
    *continuous_angle = next_angle;
    return true;
}

/**
 * @brief 反馈失效时锁存保护状态，待机继续等待，已有故障不得降级为失联
 *
 * @param axis 单轴公开状态
 * @param internal 单轴私有状态
 * @param next_state 请求进入的下一状态
 * @return 无返回值
 */
static void Gimbal_RejectAxisFeedback(Gimbal_Axis_t *axis, Gimbal_AxisInternal_t *internal,
                                      Gimbal_AxisState_t next_state)
{
    if (axis->state == GIMBAL_AXIS_RUNNING || axis->state == GIMBAL_AXIS_OFFLINE)
    {
        Gimbal_StopAxis(axis, internal, next_state);
    }
    else if (axis->state != GIMBAL_AXIS_RELAX && axis->state != GIMBAL_AXIS_FAULT && axis->state != GIMBAL_AXIS_TRIPPED)
    {
        Gimbal_StopAxis(axis, internal, GIMBAL_AXIS_FAULT);
    }
    Gimbal_ClearFeedback(axis, internal);
}

/**
 * @brief 校验一帧电机反馈并更新机械角、转速和时效
 *
 * @param axis 单轴公开状态
 * @param internal 单轴私有状态
 * @param config 本周期配置快照中的单轴配置
 * @param motor 本周期电机反馈，无新帧时为 NULL
 * @param now_us 当前单调时刻，单位：us
 * @param timeout_us 反馈允许的最大帧龄，单位：us
 * @param signed_angle 为 true 时按 pitch 有符号行程映射
 * @return 无返回值
 */
static void Gimbal_UpdateAxisFeedback(Gimbal_Axis_t *axis, Gimbal_AxisInternal_t *internal,
                                      const Gimbal_AxisConfig_t *config, const MOTOR_RM_Feedback_t *motor,
                                      uint64_t now_us, uint64_t timeout_us, bool signed_angle)
{
    if (motor == NULL)
    {
        /* 无新帧时按旧帧龄判定失联，时间回退单独按故障处理 */
        const bool time_backward = internal->feedback_time_us != UINT64_MAX && now_us < internal->feedback_time_us;
        if (internal->feedback_time_us == UINT64_MAX || time_backward ||
            now_us - internal->feedback_time_us > timeout_us)
        {
            Gimbal_RejectAxisFeedback(axis, internal, time_backward ? GIMBAL_AXIS_FAULT : GIMBAL_AXIS_OFFLINE);
        }
        return;
    }

    const uint32_t age_us = (uint32_t)now_us - motor->last_update_time;
    const uint64_t frame_time_us = now_us - age_us;
    const float encoder_rad = motor->rotor_single_angle;
    const float speed_rpm = motor->rotor_speed;
    /* 编码器一旦出现丢帧，输出轴机械角可能整圈偏移，云台在重新对零前保持该轴故障 */
    const bool frame_valid = motor->angle_valid && motor->angle_lost_count == 0U && now_us >= age_us &&
                             age_us <= timeout_us && encoder_rad >= 0.0f && encoder_rad < M_2PI;
    const bool continuous = internal->feedback_time_us != UINT64_MAX && frame_time_us > internal->feedback_time_us &&
                            frame_time_us - internal->feedback_time_us <= timeout_us;
    const bool has_reference = signed_angle ? isfinite(axis->mechanical_angle_deg) : internal->unwrap.has_reference;
    if (!frame_valid || (has_reference && frame_time_us <= internal->feedback_time_us))
    {
        Gimbal_RejectAxisFeedback(axis, internal, GIMBAL_AXIS_FAULT);
        return;
    }
    if (has_reference && !continuous)
    {
        Gimbal_RejectAxisFeedback(axis, internal, GIMBAL_AXIS_OFFLINE);
        return;
    }

    /* 反装轴先翻转角度和转速，使模块内统一以角度增大方向为正 */
    const float raw_encoder_deg = Gimbal_WrapAngle(encoder_rad * M_RAD2DEG_MULT, GIMBAL_FULL_TURN_DEG);
    const float encoder_deg = config->reversed ?
        Gimbal_WrapAngle(GIMBAL_FULL_TURN_DEG - raw_encoder_deg, GIMBAL_FULL_TURN_DEG) : raw_encoder_deg;
    const float speed_rad_s = speed_rpm * GIMBAL_RPM_TO_RAD_S;
    axis->encoder_deg = encoder_deg;
    axis->motor_speed_rad_s = config->reversed ? -speed_rad_s : speed_rad_s;
    internal->feedback_time_us = frame_time_us;

    /* pitch 行程围绕水平零点，直接映射到 [-180, 180)，负角上电和失联恢复均不累计圈数 */
    if (signed_angle)
    {
        const float half_turn_deg = GIMBAL_FULL_TURN_DEG * 0.5f;
        axis->mechanical_angle_deg = Gimbal_WrapAngle(encoder_deg - internal->applied_zero_encoder_deg + half_turn_deg,
                                                      GIMBAL_FULL_TURN_DEG) - half_turn_deg;
        return;
    }

    /* yaw 保留原有连续角展开，跨编码器零点时按相邻帧最短路径累计 */
    const float relative_deg = Gimbal_WrapAngle(encoder_deg - internal->applied_zero_encoder_deg, GIMBAL_FULL_TURN_DEG);
    float angle_deg = NAN;
    if (!Gimbal_UpdateAngleUnwrap(&internal->unwrap, relative_deg, GIMBAL_FULL_TURN_DEG, &angle_deg))
    {
        Gimbal_RejectAxisFeedback(axis, internal, GIMBAL_AXIS_FAULT);
        return;
    }
    axis->mechanical_angle_deg = angle_deg;
}

/**
 * @brief 作废惯性缓存并清空两轴姿态与角速度
 *
 * @param gimbal 云台实例
 * @param internal 模块私有状态
 * @return 无返回值
 */
static void Gimbal_InvalidateImu(Gimbal_t *gimbal, Gimbal_Internal_t *internal)
{
    internal->imu.valid = false;
    gimbal->yaw.angle_deg = gimbal->pitch.angle_deg = NAN;
    gimbal->yaw.speed_rad_s = gimbal->pitch.speed_rad_s = NAN;

    /* 启动期间允许继续等待，运行后的共用反馈异常必须锁存至人工待机 */
    if (gimbal->imu_state != GIMBAL_IMU_WAITING)
    {
        gimbal->imu_state = GIMBAL_IMU_FAULT;
    }
}

/**
 * @brief 校验惯性快照并更新两轴姿态角、欧拉角速度及共用反馈状态
 *
 * @param gimbal 云台实例
 * @param internal 模块私有状态
 * @param imu 本周期惯性快照，无新帧时为 NULL，此时只检查缓存帧龄
 * @param now_us 当前单调时刻，单位：us
 * @return 无返回值
 */
static void Gimbal_UpdateImu(Gimbal_t *gimbal, Gimbal_Internal_t *internal, const Gimbal_ImuFeedback_t *imu,
                             uint64_t now_us)
{
    const uint64_t timeout_us = (uint64_t)(internal->control_config.imu_timeout_s * 1.0e6f);
    const Gimbal_ImuFeedback_t *frame = imu != NULL ? imu : &internal->imu;
    bool valid = frame->valid && frame->sample_time_us <= now_us && now_us - frame->sample_time_us <= timeout_us;

    for (uint32_t i = 0U; i < 3U; ++i)
    {
        valid = valid && isfinite(frame->angle_deg[i]) && isfinite(frame->gyro_rad_s[i]) &&
                fabsf(frame->gyro_rad_s[i]) <= GIMBAL_IMU_MAX_RAD_S;
    }
    valid = valid && fabsf(frame->angle_deg[0]) <= 180.0f && fabsf(frame->angle_deg[1]) < GIMBAL_IMU_MAX_PITCH_DEG;

    /* 已有参考帧时检查相邻帧连续性和跳变，跨零点的 yaw 差值先折算到 ±180 度 */
    if (imu != NULL && internal->imu.valid)
    {
        const uint64_t previous_us = internal->imu.sample_time_us;
        const bool continuous = frame->sample_time_us > previous_us &&
                                frame->sample_time_us - previous_us <= timeout_us;
        valid = valid && continuous;
        const float dt_s = continuous ? (float)(frame->sample_time_us - previous_us) * 1.0e-6f : 0.0f;
        for (uint32_t i = 0U; i < 3U; ++i)
        {
            float delta_deg = frame->angle_deg[i] - internal->imu.angle_deg[i];
            if (i == 0U)
            {
                delta_deg = Gimbal_WrapAngle(delta_deg + 180.0f, GIMBAL_FULL_TURN_DEG) - 180.0f;
            }
            const float max_delta_deg = GIMBAL_IMU_MAX_RAD_S * M_RAD2DEG_MULT * dt_s + GIMBAL_IMU_JUMP_MARGIN_DEG;
            valid = valid && fabsf(delta_deg) <= max_delta_deg;
        }
    }
    if (!valid)
    {
        Gimbal_InvalidateImu(gimbal, internal);
        return;
    }

    if (imu != NULL)
    {
        internal->imu = *imu;
        /* 启动后收到有效首帧即就绪，故障只能由 Control 回到待机解除 */
        gimbal->imu_state = gimbal->imu_state == GIMBAL_IMU_WAITING ? GIMBAL_IMU_READY : gimbal->imu_state;
    }

    const bool ready = gimbal->imu_state == GIMBAL_IMU_READY;

    /* 未就绪期间不外发姿态与角速度，滤波器同步回到零状态，恢复后不继承旧延迟单元 */
    if (!ready)
    {
        LowPassFilter2p_Reset(&internal->yaw_rate_lpf, 0.0f);
        LowPassFilter2p_Reset(&internal->pitch_rate_lpf, 0.0f);
        NotchFilter_Reset(&internal->yaw_rate_notch, 0.0f);
        NotchFilter_Reset(&internal->pitch_rate_notch, 0.0f);
        gimbal->yaw.angle_deg = gimbal->pitch.angle_deg = NAN;
        gimbal->yaw.speed_rad_s = gimbal->pitch.speed_rad_s = NAN;
        return;
    }

    gimbal->yaw.angle_deg = internal->imu.angle_deg[2];
    gimbal->pitch.angle_deg = internal->imu.angle_deg[1];

    /* 机体系陀螺仪按 ZYX 欧拉运动学换算为大地系角速度，pitch 接近奇异点时由上游限幅拒绝 */
    const float roll_rad = internal->imu.angle_deg[0] * M_DEG2RAD_MULT;
    const float sin_roll = sinf(roll_rad);
    const float cos_roll = cosf(roll_rad);
    const float gyro_y = internal->imu.gyro_rad_s[1];
    const float gyro_z = internal->imu.gyro_rad_s[2];
    const float yaw_rate = (gyro_y * sin_roll + gyro_z * cos_roll) / cosf(internal->imu.angle_deg[1] * M_DEG2RAD_MULT);
    const float pitch_rate = gyro_y * cos_roll - gyro_z * sin_roll;
    /* 结构共振落在控制环带宽之外，先带阻再低通，避免振动被速度环原样透传 */
    gimbal->yaw.speed_rad_s = LowPassFilter2p_Apply(&internal->yaw_rate_lpf,
                                                    NotchFilter_Apply(&internal->yaw_rate_notch, yaw_rate));
    gimbal->pitch.speed_rad_s = LowPassFilter2p_Apply(&internal->pitch_rate_lpf,
                                                      NotchFilter_Apply(&internal->pitch_rate_notch, pitch_rate));
}

/* Exported functions ------------------------------------------------------- */
/**
 * @brief 复制配置并初始化两轴 PID 与模块私有状态
 *
 * @param gimbal 云台实例
 * @param config 默认配置
 * @param frequency_hz 控制频率，单位：Hz
 * @return int8_t 成功返回 0，失败返回 -1 并保持两轴零输出
 */
int8_t Gimbal_Init(Gimbal_t *gimbal, const Gimbal_Config_t *config, float frequency_hz)
{
    /* 先复制配置并清除公开观测与模块私有历史，任一失败路径都保持两轴零输出 */
    const Gimbal_Config_t saved_config = *config;
    memset(&gimbal_internal, 0, sizeof(gimbal_internal));
    memset(gimbal, 0, sizeof(*gimbal));
    gimbal->config = saved_config;
    gimbal->mode = GIMBAL_MODE_RELAX;
    gimbal->yaw.state = gimbal->pitch.state = GIMBAL_AXIS_FAULT;
    Gimbal_ClearFeedback(&gimbal->yaw, &gimbal_internal.yaw);
    Gimbal_ClearFeedback(&gimbal->pitch, &gimbal_internal.pitch);
    gimbal->yaw.angle_deg = gimbal->pitch.angle_deg = NAN;
    gimbal->yaw.speed_rad_s = gimbal->pitch.speed_rad_s = NAN;
    gimbal->config_valid = Gimbal_IsConfigValid(&saved_config, frequency_hz);
    if (!gimbal->config_valid)
    {
        return -1;
    }

    /* 两轴共享模块私有的已校验快照，PID 参数指向私有持久副本 */
    gimbal_internal.control_config = saved_config;
    /* 惯性角速度滤波按控制频率一次算出系数，延迟单元已由上面的 memset 清零 */
    LowPassFilter2p_Init(&gimbal_internal.yaw_rate_lpf, frequency_hz, GIMBAL_YAW_RATE_LPF_HZ);
    LowPassFilter2p_Init(&gimbal_internal.pitch_rate_lpf, frequency_hz, GIMBAL_RATE_LPF_HZ);
    NotchFilter_Init(&gimbal_internal.yaw_rate_notch, frequency_hz, GIMBAL_YAW_RATE_NOTCH_HZ, GIMBAL_RATE_NOTCH_BW_HZ);
    NotchFilter_Init(&gimbal_internal.pitch_rate_notch, frequency_hz, GIMBAL_RATE_NOTCH_HZ,
                     GIMBAL_RATE_NOTCH_BW_HZ);
    if (Gimbal_InitAxis(&gimbal->yaw, &gimbal_internal.yaw, &saved_config.yaw, frequency_hz) != 0 ||
        Gimbal_InitAxis(&gimbal->pitch, &gimbal_internal.pitch, &saved_config.pitch, frequency_hz) != 0)
    {
        return -1;
    }
    return 0;
}

/**
 * @brief 接纳两轴电机反馈与共享惯性快照
 *
 * @param gimbal 云台实例
 * @param yaw_motor 本周期 yaw 电机反馈，无新帧时为 NULL
 * @param pitch_motor 本周期 pitch 电机反馈，无新帧时为 NULL
 * @param imu 本周期惯性快照，无新帧时为 NULL
 * @param now_us 当前单调时刻，单位：us
 * @return 无返回值
 */
void Gimbal_UpdateFeedback(Gimbal_t *gimbal, const MOTOR_RM_Feedback_t *yaw_motor,
                           const MOTOR_RM_Feedback_t *pitch_motor, const Gimbal_ImuFeedback_t *imu, uint64_t now_us)
{
    /* 反馈阶段使用最近一次通过集中校验的配置，新的在线调参留给随后 Control 采纳 */
    const uint64_t timeout_us = (uint64_t)(gimbal_internal.control_config.feedback_timeout_s * 1.0e6f);
    Gimbal_UpdateAxisFeedback(&gimbal->yaw, &gimbal_internal.yaw, &gimbal_internal.control_config.yaw, yaw_motor,
                              now_us, timeout_us, false);
    Gimbal_UpdateAxisFeedback(&gimbal->pitch, &gimbal_internal.pitch, &gimbal_internal.control_config.pitch,
                              pitch_motor, now_us, timeout_us, true);
    Gimbal_UpdateImu(gimbal, &gimbal_internal, imu, now_us);
}

/**
 * @brief 在入口校验配置快照与周期，解析命令并推进两轴状态机
 *
 * @param gimbal 云台实例
 * @param command 归一化云台命令，无命令时为 NULL，按主动待机处理
 * @param chassis_state 底盘状态，用于悬空带转检测
 * @param period_s 实测控制周期，单位：s
 * @return 无返回值
 */
void Gimbal_Control(Gimbal_t *gimbal, const Gimbal_Command_t *command, Gimbal_ChassisState_t chassis_state,
                    float period_s)
{
    /* 在线调参只在 Control 入口复制并校验，失败时不采纳快照中的任何字段 */
    const Gimbal_Config_t snapshot = *(const volatile Gimbal_Config_t *)&gimbal->config;
    Gimbal_Axis_t *const axes[GIMBAL_AXIS_COUNT] = { &gimbal->yaw, &gimbal->pitch };
    Gimbal_AxisInternal_t *const internals[GIMBAL_AXIS_COUNT] = { &gimbal_internal.yaw, &gimbal_internal.pitch };
    const Gimbal_AxisConfig_t *const configs[GIMBAL_AXIS_COUNT] = { &snapshot.yaw, &snapshot.pitch };
    const Gimbal_Feedforward_t feedforwards[GIMBAL_AXIS_COUNT] =
    {
        GIMBAL_FEEDFORWARD_YAW_FRICTION,
        GIMBAL_FEEDFORWARD_PITCH_GRAVITY,
    };
    gimbal->config_valid = Gimbal_IsConfigValid(&snapshot, 1.0f / gimbal_internal.yaw.speed_pid.dt_min);
    if (!gimbal->config_valid || !isfinite(period_s) || period_s <= 0.0f || period_s > GIMBAL_MAX_PERIOD_S)
    {
        for (uint32_t i = 0U; i < GIMBAL_AXIS_COUNT; ++i)
        {
            Gimbal_StopAxis(axes[i], internals[i], GIMBAL_AXIS_FAULT);
        }
        return;
    }
    gimbal_internal.control_config = snapshot;
    for (uint32_t i = 0U; i < GIMBAL_AXIS_COUNT; ++i)
    {
        Gimbal_ApplyAxisConfig(axes[i], internals[i], configs[i]);
    }

    /* 无命令表示主动待机，模式非法时两轴转故障 */
    const Gimbal_Command_t relax_command = { .mode = GIMBAL_MODE_RELAX };
    const Gimbal_Command_t *const active_command = command != NULL ? command : &relax_command;
    const Gimbal_Mode_t mode = active_command->mode;
    const bool mode_changed = mode != gimbal->mode;
    if ((unsigned)mode > GIMBAL_MODE_ANGLE)
    {
        for (uint32_t i = 0U; i < GIMBAL_AXIS_COUNT; ++i)
        {
            Gimbal_StopAxis(axes[i], internals[i], GIMBAL_AXIS_FAULT);
        }
        return;
    }

    /* 公开命令先解析为轴内设定值，姿态捕获及保护的覆盖优先于增量 */
    const bool valid[GIMBAL_AXIS_COUNT] =
    {
        Gimbal_ResolveSetpoint(axes[0], internals[0], &active_command->yaw, mode, period_s),
        Gimbal_ResolveSetpoint(axes[1], internals[1], &active_command->pitch, mode, period_s),
    };

    /* 共用源故障只在人工回待机后解除，下一轮等待新的有效快照 */
    if (mode == GIMBAL_MODE_RELAX && gimbal->imu_state == GIMBAL_IMU_FAULT)
    {
        gimbal->imu_state = GIMBAL_IMU_WAITING;
        gimbal_internal.imu.valid = false;
    }

    /* 待机起控或失联恢复时捕获当前姿态，避免追赶停止前的旧目标 */
    if (mode == GIMBAL_MODE_ANGLE && gimbal->imu_state == GIMBAL_IMU_READY)
    {
        for (uint32_t i = 0U; i < GIMBAL_AXIS_COUNT; ++i)
        {
            if (valid[i] && internals[i]->feedback_time_us != UINT64_MAX &&
                (axes[i]->state == GIMBAL_AXIS_RELAX || axes[i]->state == GIMBAL_AXIS_OFFLINE || mode_changed))
            {
                internals[i]->setpoint.target_angle_deg = axes[i]->angle_deg;
                internals[i]->setpoint.target_speed_rad_s = 0.0f;
            }
        }
    }

    /* 两轴独立推进状态机，非法目标只使对应轴进入故障 */
    const bool imu_ready = gimbal->imu_state == GIMBAL_IMU_READY;
    for (uint32_t i = 0U; i < GIMBAL_AXIS_COUNT; ++i)
    {
        if (valid[i])
        {
            Gimbal_ControlAxis(axes[i], internals[i], configs[i], mode, mode_changed, &internals[i]->setpoint,
                               period_s, imu_ready, feedforwards[i]);
        }
        else
        {
            Gimbal_StopAxis(axes[i], internals[i], GIMBAL_AXIS_FAULT);
        }
    }

    /* 小陀螺由底盘在线与已采纳的自旋命令共同确认，在线未自旋不混同为小陀螺 */
    if (chassis_state == GIMBAL_CHASSIS_SPINNING || chassis_state == GIMBAL_CHASSIS_ONLINE)
    {
        gimbal_internal.chassis_offline_s = 0.0f;
    }
    else
    {
        gimbal_internal.chassis_offline_s = fminf(GIMBAL_CHASSIS_OFFLINE_TIME_S,
            gimbal_internal.chassis_offline_s + period_s);
    }

    /* yaw 闭环后仅检查悬空带转，运行状态保证两种转速有限，撤驱在本周期输出闸门前生效 */
    const Gimbal_Axis_t *const yaw = &gimbal->yaw;
    const bool running = yaw->state == GIMBAL_AXIS_RUNNING;
    const bool dragged = running && gimbal_internal.chassis_offline_s >= GIMBAL_CHASSIS_OFFLINE_TIME_S &&
        fabsf(yaw->motor_speed_rad_s - yaw->speed_rad_s) > GIMBAL_YAW_DRAG_RAD_S &&
        fabsf(yaw->speed_rad_s) < GIMBAL_YAW_DRAG_IMU_STILL_RAD_S;
    gimbal_internal.yaw_drag_s = dragged ? gimbal_internal.yaw_drag_s + period_s : 0.0f;
    if (gimbal_internal.yaw_drag_s >= GIMBAL_YAW_DRAG_TIME_S)
    {
        Gimbal_StopAxis(&gimbal->yaw, &gimbal_internal.yaw, GIMBAL_AXIS_TRIPPED);
    }
    gimbal->mode = mode;
}

/**
 * @brief 按轴状态导出本周期两轴电流或零
 *
 * @param gimbal 云台实例
 * @param output 两轴电流，单位：A
 * @return 无返回值
 */
void Gimbal_DumpOutput(const Gimbal_t *gimbal, Gimbal_Output_t *output)
{
    /* 唯一输出闸门按轴状态放行，只使用本周期已采纳方向，不读取 EmberProbe 可写配置 */
    const float yaw_current_a = gimbal->yaw.state == GIMBAL_AXIS_RUNNING ? gimbal->yaw.output_current_a : 0.0f;
    const float pitch_current_a = gimbal->pitch.state == GIMBAL_AXIS_RUNNING ? gimbal->pitch.output_current_a : 0.0f;
    output->yaw_current_a = gimbal_internal.yaw.applied_reversed ? -yaw_current_a : yaw_current_a;
    output->pitch_current_a = gimbal_internal.pitch.applied_reversed ? -pitch_current_a : pitch_current_a;
}
