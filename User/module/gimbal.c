#include "module/gimbal.h"
#include <math.h>
#include <stddef.h>
#include <string.h>
#include "component/user_math.h"

/*
 * 参数沿用合并前值：周期上限 0.1 秒、PID 增益上限 1000；rpm 按每分钟 60 秒、每圈 2π 换算，完整一圈为 360 度
 * 电机编码器单圈范围在反馈入口校验；惯性角速度上限 40 rad/s，俯仰奇异点与相邻姿态跳变量各保留 1 度余量
 */
#define GIMBAL_MAX_PERIOD_S (0.1f)
#define GIMBAL_MAX_GAIN (1000.0f)
#define GIMBAL_RPM_TO_RAD_S (0.10471975512f)
#define GIMBAL_IMU_MAX_RAD_S (40.0f)
#define GIMBAL_IMU_MAX_PITCH_DEG (89.0f)
#define GIMBAL_IMU_JUMP_MARGIN_DEG (1.0f)
#define GIMBAL_FULL_TURN_DEG (360.0f)

/*
 * yaw 撤驱保护阈值，均为待实物确认的起点：
 * - DRAG_RAD_S、DRAG_IMU_STILL_RAD_S、DRAG_TIME_S：底盘离线时编码器与惯性转速差超过 1 rad/s
 *   且上部惯性转速低于 0.3 rad/s，持续 0.15 s 判为悬空带转，沿用原低速检测阈值
 * - CHASSIS_OFFLINE_TIME_S：四个底盘电机全部离线持续 1 s 才启用悬空带转检测，覆盖小陀螺掉线后的惯性滑转
 */
#define GIMBAL_YAW_DRAG_RAD_S (1.0f)
#define GIMBAL_YAW_DRAG_IMU_STILL_RAD_S (0.3f)
#define GIMBAL_YAW_DRAG_TIME_S (0.15f)
#define GIMBAL_CHASSIS_OFFLINE_TIME_S (1.0f)

/*
 * 云台机械角展开器的私有历史：
 * - has_reference：已建立连续角参考时为 true
 * - last_wrapped_angle：上一帧单圈角度，单位度，范围 [0, 360)
 * - continuous_angle：展开后的连续机械角，单位度
 */
typedef struct
{
    bool has_reference;
    float last_wrapped_angle;
    float continuous_angle;
} Gimbal_AngleUnwrap_t;

/*
 * 单轴目标设定值：
 * - target_angle_deg：内部捕获后实际使用的姿态目标，单位度
 * - target_speed_rad_s：内部捕获后实际使用的速度目标或前馈，单位 rad/s
 */
typedef struct
{
    float target_angle_deg;
    float target_speed_rad_s;
} Gimbal_AxisSetpoint_t;

/*
 * 单轴私有控制状态：
 * - setpoint：捕获后的姿态目标与速度前馈，单位分别为度和 rad/s
 * - feedback_time_us：最近有效电机反馈时刻，单位微秒，无反馈时为 UINT64_MAX
 * - applied_zero_encoder_deg：当前机械角零点，单位度，范围 [0, 360)
 * - applied_reversed：当前已应用的电机方向配置
 * - applied_angle_pid、applied_speed_pid：PID 当前使用的参数副本
 * - unwrap：yaw 连续机械角展开状态，pitch 不使用
 * - angle_pid、speed_pid：角度外环和速度内环的运行状态
 */
typedef struct
{
    Gimbal_AxisSetpoint_t setpoint;
    uint64_t feedback_time_us;
    float applied_zero_encoder_deg;
    bool applied_reversed;
    KPID_Params_t applied_angle_pid;
    KPID_Params_t applied_speed_pid;
    Gimbal_AngleUnwrap_t unwrap;
    KPID_t angle_pid;
    KPID_t speed_pid;
} Gimbal_AxisInternal_t;

/*
 * 单一云台实例的模块私有状态：
 * - control_config：最近一次通过 Control 入口校验的配置快照
 * - imu：最近一帧有效或等待校验的惯性反馈缓存
 * - yaw、pitch：两轴私有控制器和反馈历史
 * - chassis_offline_s：底盘持续离线时间，单位秒
 * - yaw_drag_s：yaw 满足悬空带转条件的累计时间，单位秒
 */
typedef struct
{
    Gimbal_Config_t control_config;
    Gimbal_ImuFeedback_t imu;
    Gimbal_AxisInternal_t yaw;
    Gimbal_AxisInternal_t pitch;
    float chassis_offline_s;
    float yaw_drag_s;
} Gimbal_Internal_t;

static Gimbal_Internal_t gimbal_internal;

static void Gimbal_RejectAxisFeedback(Gimbal_Axis_t *axis, Gimbal_AxisInternal_t *internal,
                                      Gimbal_AxisState_t next_state);
static void Gimbal_InvalidateImu(Gimbal_t *gimbal, Gimbal_Internal_t *internal);
static void Gimbal_UpdateImu(Gimbal_t *gimbal, Gimbal_Internal_t *internal,
                             const Gimbal_ImuFeedback_t *imu, uint64_t now_us);
static void Gimbal_UpdateAxisFeedback(Gimbal_Axis_t *axis, Gimbal_AxisInternal_t *internal,
                                      const Gimbal_AxisConfig_t *config,
                                      const MOTOR_RM_Feedback_t *motor, uint64_t now_us, uint64_t timeout_us,
                                      bool signed_angle);

static void Gimbal_ClearFeedback(Gimbal_Axis_t *axis, Gimbal_AxisInternal_t *internal);
static void Gimbal_StopAxis(Gimbal_Axis_t *axis, Gimbal_AxisInternal_t *internal,
                            Gimbal_AxisState_t next_state);
static bool Gimbal_IsPidConfigValid(const KPID_Params_t *pid);
static bool Gimbal_IsAxisConfigValid(const Gimbal_AxisConfig_t *config);
static bool Gimbal_IsConfigValid(const Gimbal_Config_t *config, float frequency_hz);
static bool Gimbal_IsPidChanged(const KPID_Params_t *config, const KPID_Params_t *applied);
static int8_t Gimbal_InitAxis(Gimbal_Axis_t *axis, Gimbal_AxisInternal_t *internal,
                              const Gimbal_AxisConfig_t *config, float frequency_hz);
static void Gimbal_ApplyAxisConfig(Gimbal_Axis_t *axis, Gimbal_AxisInternal_t *internal,
                                   const Gimbal_AxisConfig_t *config);
static void Gimbal_EnterRunning(Gimbal_Axis_t *axis, Gimbal_AxisInternal_t *internal);
static bool Gimbal_RunAxis(Gimbal_Axis_t *axis, Gimbal_AxisInternal_t *internal,
                           const Gimbal_AxisConfig_t *config, Gimbal_Mode_t mode,
                           const Gimbal_AxisSetpoint_t *command, float period_s);
static void Gimbal_ControlAxis(Gimbal_Axis_t *axis, Gimbal_AxisInternal_t *internal,
                               const Gimbal_AxisConfig_t *config, Gimbal_Mode_t mode,
                               bool mode_changed, const Gimbal_AxisSetpoint_t *command, float period_s,
                               bool imu_ready);

/**
 * @brief 将有限角度环绕到单圈区间
 *
 * @param[in] angle 有限角度
 * @param[in] period 单圈周期，须为有限正数，例如 360 或 2π
 * @return 单圈角度，范围 [0, period)
 */
static float Gimbal_WrapAngle(float angle, float period)
{
    float wrapped = fmodf(angle, period);
    if (wrapped < 0.0f)
    {
        wrapped += period;
    }
    // 负的极小角度加整圈后可能舍入为整圈，统一归到零点
    return wrapped >= period ? 0.0f : wrapped;
}

/**
 * @brief 丢弃上一帧参考，下一次更新重新以单圈角度建立连续坐标
 *
 * @param[out] unwrap 展开器状态，调用方保证非空
 * @return 无返回值
 */
static void Gimbal_ResetAngleUnwrap(Gimbal_AngleUnwrap_t *unwrap)
{
    unwrap->has_reference = false;
    unwrap->last_wrapped_angle = 0.0f;
    unwrap->continuous_angle = 0.0f;
}

static bool Gimbal_UpdateAngleUnwrap(Gimbal_AngleUnwrap_t *unwrap, float wrapped_angle, float period,
                                     float *continuous_angle)
{
    // 首帧直接以单圈角度建立连续坐标
    if (!unwrap->has_reference)
    {
        unwrap->has_reference = true;
        unwrap->last_wrapped_angle = wrapped_angle;
        unwrap->continuous_angle = wrapped_angle;
        *continuous_angle = wrapped_angle;
        return true;
    }

    // 将相邻帧差值折算到半圈以内，跨越编码器零点时取最短路径
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

    // 恰为半圈时无法判断转动方向，丢弃参考避免累计错误圈数
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

int8_t Gimbal_Init(Gimbal_t *gimbal, const Gimbal_Config_t *config, float frequency_hz)
{
    // 先复制配置并清除公开观测与模块私有历史，任一失败路径都保持两轴零输出
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

    // 两轴共享模块私有的已校验快照，PID 参数指向私有持久副本
    gimbal_internal.control_config = saved_config;
    if (Gimbal_InitAxis(&gimbal->yaw, &gimbal_internal.yaw, &saved_config.yaw, frequency_hz) != 0 ||
        Gimbal_InitAxis(&gimbal->pitch, &gimbal_internal.pitch, &saved_config.pitch, frequency_hz) != 0)
    {
        return -1;
    }
    return 0;
}

static bool Gimbal_ResolveSetpoint(Gimbal_Axis_t *axis, Gimbal_AxisInternal_t *internal,
                                   const Gimbal_AxisCommand_t *command, Gimbal_Mode_t mode, float period_s)
{
    // 外部命令只在此入口检查，非法速度不能被起控时的姿态捕获掩盖
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
            // 非运行状态先解析本次增量，随后由起控捕获覆盖，避免沿用故障前的非法目标
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

void Gimbal_Control(Gimbal_t *gimbal, const Gimbal_Command_t *command,
                    Gimbal_ChassisState_t chassis_state, float period_s)
{
    // 在线调参只在 Control 入口复制并校验，失败时不采纳快照中的任何字段
    const Gimbal_Config_t snapshot = *(const volatile Gimbal_Config_t *)&gimbal->config;
    gimbal->config_valid = Gimbal_IsConfigValid(&snapshot, 1.0f / gimbal_internal.yaw.speed_pid.dt_min);
    if (!gimbal->config_valid || !isfinite(period_s) || period_s <= 0.0f || period_s > GIMBAL_MAX_PERIOD_S)
    {
        Gimbal_StopAxis(&gimbal->yaw, &gimbal_internal.yaw, GIMBAL_AXIS_FAULT);
        Gimbal_StopAxis(&gimbal->pitch, &gimbal_internal.pitch, GIMBAL_AXIS_FAULT);
        return;
    }
    gimbal_internal.control_config = snapshot;
    Gimbal_ApplyAxisConfig(&gimbal->yaw, &gimbal_internal.yaw, &snapshot.yaw);
    Gimbal_ApplyAxisConfig(&gimbal->pitch, &gimbal_internal.pitch, &snapshot.pitch);

    // 无命令表示主动待机，输出由两轴状态统一阻断
    Gimbal_Command_t relax_command =
    {
        .mode = GIMBAL_MODE_RELAX,
    };
    const Gimbal_Command_t *active_command = command != NULL ? command : &relax_command;
    const Gimbal_Mode_t mode = active_command->mode;
    const bool mode_changed = mode != gimbal->mode;
    if ((unsigned)mode > GIMBAL_MODE_ANGLE)
    {
        Gimbal_StopAxis(&gimbal->yaw, &gimbal_internal.yaw, GIMBAL_AXIS_FAULT);
        Gimbal_StopAxis(&gimbal->pitch, &gimbal_internal.pitch, GIMBAL_AXIS_FAULT);
        return;
    }

    // 公开命令先解析为轴内设定值，姿态捕获及保护的覆盖优先于增量
    const bool yaw_valid = Gimbal_ResolveSetpoint(&gimbal->yaw, &gimbal_internal.yaw,
                                                  &active_command->yaw, mode, period_s);
    const bool pitch_valid = Gimbal_ResolveSetpoint(&gimbal->pitch, &gimbal_internal.pitch,
                                                    &active_command->pitch, mode, period_s);

    // 共用源故障只在人工回待机后解除，下一轮等待新的有效快照
    if (mode == GIMBAL_MODE_RELAX && gimbal->imu_state == GIMBAL_IMU_FAULT)
    {
        gimbal->imu_state = GIMBAL_IMU_WAITING;
        gimbal_internal.imu.valid = false;
    }

    // 待机起控或失联恢复时捕获当前姿态，避免追赶停止前的旧目标
    if (mode == GIMBAL_MODE_ANGLE && gimbal->imu_state == GIMBAL_IMU_READY)
    {
        if (yaw_valid &&
            (gimbal->yaw.state == GIMBAL_AXIS_RELAX || gimbal->yaw.state == GIMBAL_AXIS_OFFLINE || mode_changed) &&
            gimbal_internal.yaw.feedback_time_us != UINT64_MAX)
        {
            gimbal_internal.yaw.setpoint.target_angle_deg = gimbal->yaw.angle_deg;
            gimbal_internal.yaw.setpoint.target_speed_rad_s = 0.0f;
        }
        if (pitch_valid &&
            (gimbal->pitch.state == GIMBAL_AXIS_RELAX || gimbal->pitch.state == GIMBAL_AXIS_OFFLINE || mode_changed) &&
            gimbal_internal.pitch.feedback_time_us != UINT64_MAX)
        {
            gimbal_internal.pitch.setpoint.target_angle_deg = gimbal->pitch.angle_deg;
            gimbal_internal.pitch.setpoint.target_speed_rad_s = 0.0f;
        }
    }

    // 两轴独立推进状态机，非法目标只使对应轴进入故障
    const bool imu_ready = gimbal->imu_state == GIMBAL_IMU_READY;
    if (yaw_valid)
    {
        Gimbal_ControlAxis(&gimbal->yaw, &gimbal_internal.yaw, &snapshot.yaw, mode, mode_changed,
                           &gimbal_internal.yaw.setpoint, period_s, imu_ready);
    }
    else
    {
        Gimbal_StopAxis(&gimbal->yaw, &gimbal_internal.yaw, GIMBAL_AXIS_FAULT);
    }
    if (pitch_valid)
    {
        Gimbal_ControlAxis(&gimbal->pitch, &gimbal_internal.pitch, &snapshot.pitch, mode, mode_changed,
                           &gimbal_internal.pitch.setpoint, period_s, imu_ready);
    }
    else
    {
        Gimbal_StopAxis(&gimbal->pitch, &gimbal_internal.pitch, GIMBAL_AXIS_FAULT);
    }

    // 小陀螺由底盘在线与已采纳的自旋命令共同确认，在线未自旋不混同为小陀螺
    switch (chassis_state)
    {
        case GIMBAL_CHASSIS_SPINNING:
        case GIMBAL_CHASSIS_ONLINE:
            gimbal_internal.chassis_offline_s = 0.0f;
            break;

        case GIMBAL_CHASSIS_OFFLINE:
        default:
            gimbal_internal.chassis_offline_s = fminf(GIMBAL_CHASSIS_OFFLINE_TIME_S,
                gimbal_internal.chassis_offline_s + period_s);
            break;
    }

    // yaw 闭环后仅检查悬空带转，运行状态保证两种转速有限，撤驱在本周期输出闸门前生效
    const Gimbal_Axis_t *yaw = &gimbal->yaw;
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

void Gimbal_DumpOutput(const Gimbal_t *gimbal, Gimbal_Output_t *output)
{
    // 唯一输出闸门按轴状态放行，只使用本周期已采纳方向，不读取 EmberProbe 可写配置
    const float yaw_current_a = gimbal->yaw.state == GIMBAL_AXIS_RUNNING ? gimbal->yaw.output_current_a : 0.0f;
    const float pitch_current_a = gimbal->pitch.state == GIMBAL_AXIS_RUNNING ? gimbal->pitch.output_current_a : 0.0f;
    output->yaw_current_a = gimbal_internal.yaw.applied_reversed ? -yaw_current_a : yaw_current_a;
    output->pitch_current_a = gimbal_internal.pitch.applied_reversed ? -pitch_current_a : pitch_current_a;
}

static bool Gimbal_IsPidConfigValid(const KPID_Params_t *pid)
{
    const bool gains_valid = isfinite(pid->k) && isfinite(pid->p) && isfinite(pid->i) && isfinite(pid->d) &&
                             pid->k >= 0.0f && pid->p >= 0.0f && pid->i >= 0.0f && pid->d >= 0.0f &&
                             pid->k <= GIMBAL_MAX_GAIN && pid->p <= GIMBAL_MAX_GAIN &&
                             pid->i <= GIMBAL_MAX_GAIN && pid->d <= GIMBAL_MAX_GAIN;
    // 输出限幅为零时底层 PID 永不累计积分，连续线性角度要求不启用环绕
    return gains_valid && isfinite(pid->i_limit) && pid->i_limit >= 0.0f && isfinite(pid->out_limit) &&
           pid->out_limit > 0.0f && isfinite(pid->d_cutoff_freq) && pid->range == 0.0f;
}

static bool Gimbal_IsAxisConfigValid(const Gimbal_AxisConfig_t *config)
{
    return isfinite(config->zero_encoder_deg) && config->zero_encoder_deg >= 0.0f &&
           config->zero_encoder_deg < GIMBAL_FULL_TURN_DEG && isfinite(config->min_angle_deg) &&
           isfinite(config->max_angle_deg) && config->min_angle_deg <= config->max_angle_deg &&
           isfinite(config->max_speed_rad_s) && config->max_speed_rad_s > 0.0f &&
           isfinite(config->max_current_a) && config->max_current_a > 0.0f &&
           Gimbal_IsPidConfigValid(&config->angle_pid) && Gimbal_IsPidConfigValid(&config->speed_pid);
}

static bool Gimbal_IsConfigValid(const Gimbal_Config_t *config, float frequency_hz)
{
    return isfinite(frequency_hz) && frequency_hz >= 10.0f && frequency_hz <= 2000.0f &&
        isfinite(config->feedback_timeout_s) && config->feedback_timeout_s >= 1.0e-6f &&
        config->feedback_timeout_s <= 1.0f && isfinite(config->imu_timeout_s) &&
        config->imu_timeout_s >= 1.0e-6f && config->imu_timeout_s <= 0.1f &&
        Gimbal_IsAxisConfigValid(&config->yaw) && Gimbal_IsAxisConfigValid(&config->pitch) &&
        config->yaw.angle_pid.d_cutoff_freq < frequency_hz * 0.5f &&
        config->yaw.speed_pid.d_cutoff_freq < frequency_hz * 0.5f &&
        config->pitch.angle_pid.d_cutoff_freq < frequency_hz * 0.5f &&
        config->pitch.speed_pid.d_cutoff_freq < frequency_hz * 0.5f;
}

static bool Gimbal_IsPidChanged(const KPID_Params_t *config, const KPID_Params_t *applied)
{
    return config->k != applied->k || config->p != applied->p || config->i != applied->i ||
           config->d != applied->d || config->i_limit != applied->i_limit || config->out_limit != applied->out_limit ||
           config->d_cutoff_freq != applied->d_cutoff_freq;
}

static int8_t Gimbal_InitAxis(Gimbal_Axis_t *axis, Gimbal_AxisInternal_t *internal,
                              const Gimbal_AxisConfig_t *config, float frequency_hz)
{
    // PID 指向模块私有的持久参数副本，调试器修改配置后由每周期比较决定何时同步
    internal->applied_zero_encoder_deg = config->zero_encoder_deg;
    internal->applied_reversed = config->reversed;
    internal->applied_angle_pid = config->angle_pid;
    internal->applied_speed_pid = config->speed_pid;

    // 内环采用大地系欧拉角速度，外环保留当前零微分配置
    if (PID_Init(&internal->angle_pid, KPID_MODE_SET_D, frequency_hz, &internal->applied_angle_pid) != 0 ||
        PID_Init(&internal->speed_pid, KPID_MODE_CALC_D, frequency_hz, &internal->applied_speed_pid) != 0)
    {
        return -1;
    }
    axis->state = GIMBAL_AXIS_RELAX;
    return 0;
}

static void Gimbal_ApplyAxisConfig(Gimbal_Axis_t *axis, Gimbal_AxisInternal_t *internal,
                                   const Gimbal_AxisConfig_t *config)
{
    // 零点或方向变化后机械坐标失效，运行中锁存故障，下一帧重新建立机械角
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

    // 参数副本与配置不同时同步并清空积分，比例和微分项不受历史影响
    if (Gimbal_IsPidChanged(&config->angle_pid, &internal->applied_angle_pid))
    {
        if (config->angle_pid.d_cutoff_freq != internal->applied_angle_pid.d_cutoff_freq)
        {
            LowPassFilter2p_Init(&internal->angle_pid.dfilter, 1.0f / internal->angle_pid.dt_min,
                                 config->angle_pid.d_cutoff_freq);
        }
        internal->applied_angle_pid = config->angle_pid;
        PID_ResetIntegral(&internal->angle_pid);
    }
    if (Gimbal_IsPidChanged(&config->speed_pid, &internal->applied_speed_pid))
    {
        if (config->speed_pid.d_cutoff_freq != internal->applied_speed_pid.d_cutoff_freq ||
            config->speed_pid.k != internal->applied_speed_pid.k)
        {
            LowPassFilter2p_Init(&internal->speed_pid.dfilter, 1.0f / internal->speed_pid.dt_min,
                                 config->speed_pid.d_cutoff_freq);
            const float scaled_speed = config->speed_pid.k * axis->speed_rad_s;
            internal->speed_pid.last.k_fb = LowPassFilter2p_Reset(&internal->speed_pid.dfilter, scaled_speed);
        }
        internal->applied_speed_pid = config->speed_pid;
        PID_ResetIntegral(&internal->speed_pid);
    }
}

static void Gimbal_EnterRunning(Gimbal_Axis_t *axis, Gimbal_AxisInternal_t *internal)
{
    PID_Reset(&internal->angle_pid);
    PID_Reset(&internal->speed_pid);

    // 速度环微分作用于反馈，预置上一反馈可避免入口处的微分冲击
    const float scaled_speed = internal->applied_speed_pid.k * axis->speed_rad_s;
    internal->speed_pid.last.k_fb = LowPassFilter2p_Reset(&internal->speed_pid.dfilter, scaled_speed);
    axis->state = GIMBAL_AXIS_RUNNING;
}

static bool Gimbal_RunAxis(Gimbal_Axis_t *axis, Gimbal_AxisInternal_t *internal,
                           const Gimbal_AxisConfig_t *config, Gimbal_Mode_t mode,
                           const Gimbal_AxisSetpoint_t *command, float period_s)
{
    // 调用方已保证反馈就绪、命令合法及周期有效，本层只执行控制计算
    const bool angle_needed = mode == GIMBAL_MODE_ANGLE;
    // SPEED 模式直接采用命令角速度，ANGLE 模式将其作为前馈与角度环修正量合成
    const float max_speed_rad_s = config->max_speed_rad_s;
    const float command_speed_rad_s = AbsClip(command->target_speed_rad_s, max_speed_rad_s);
    float target_speed_rad_s = command_speed_rad_s;
    const float previous_angle_integral = internal->angle_pid.i;
    bool mechanically_limited = false;
    if (angle_needed)
    {
        float target_angle_deg = command->target_angle_deg;
        if (config->mechanical_limit_enabled)
        {
            // 用目标姿态增量预测机械角，底座倾斜时不能直接夹住世界姿态目标
            const float min_target_deg = axis->angle_deg + config->min_angle_deg - axis->mechanical_angle_deg;
            const float max_target_deg = axis->angle_deg + config->max_angle_deg - axis->mechanical_angle_deg;
            Clip(&target_angle_deg, min_target_deg, max_target_deg);
            mechanically_limited = target_angle_deg != command->target_angle_deg;
        }
        axis->target_angle_deg = target_angle_deg;
        // 角度环输入为弧度，微分反馈为大地系欧拉角速度，当前外环微分增益为零
        const float correction_rad_s = PID_Calc(&internal->angle_pid, target_angle_deg * M_DEG2RAD_MULT,
                                                axis->angle_deg * M_DEG2RAD_MULT, axis->speed_rad_s, period_s);
        const float combined_rad_s = correction_rad_s + command_speed_rad_s;
        target_speed_rad_s = AbsClip(combined_rad_s, max_speed_rad_s);
        // 合成速度饱和且误差继续推向饱和方向时撤销本周期积分，保留反向退饱和能力
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
    // 速度模式及前馈同样受机械边界约束，越界时只允许零速或向行程内部运动
    if (config->mechanical_limit_enabled)
    {
        const float scale = M_DEG2RAD_MULT / period_s;
        const float min_speed = fminf(0.0f, (config->min_angle_deg - axis->mechanical_angle_deg) * scale);
        const float max_speed = fmaxf(0.0f, (config->max_angle_deg - axis->mechanical_angle_deg) * scale);
        const float requested_speed = target_speed_rad_s;
        Clip(&target_speed_rad_s, min_speed, max_speed);
        mechanically_limited = mechanically_limited || requested_speed != target_speed_rad_s;
    }
    // 速度环输出电流，非有限结果视为计算异常
    const float previous_speed_integral = internal->speed_pid.i;
    const float current_a = PID_Calc(&internal->speed_pid, target_speed_rad_s, axis->speed_rad_s, 0.0f, period_s);
    if (!isfinite(target_speed_rad_s))
    {
        return false;
    }
    axis->target_speed_rad_s = target_speed_rad_s;
    axis->output_current_a = AbsClip(current_a, config->max_current_a);
    // 边界处阻断指向行程外侧的电流，避免底座运动或积分残留绕过目标限幅
    if (config->mechanical_limit_enabled &&
        ((axis->mechanical_angle_deg >= config->max_angle_deg && axis->output_current_a > 0.0f) ||
         (axis->mechanical_angle_deg <= config->min_angle_deg && axis->output_current_a < 0.0f)))
    {
        axis->output_current_a = 0.0f;
        internal->speed_pid.i = previous_speed_integral;
        mechanically_limited = true;
    }
    if (mechanically_limited)
    {
        internal->angle_pid.i = previous_angle_integral;
    }
    return isfinite(axis->output_current_a) && isfinite(internal->speed_pid.i);
}

static void Gimbal_ControlAxis(Gimbal_Axis_t *axis, Gimbal_AxisInternal_t *internal,
                               const Gimbal_AxisConfig_t *config, Gimbal_Mode_t mode,
                               bool mode_changed, const Gimbal_AxisSetpoint_t *command, float period_s,
                               bool imu_ready)
{
    // 参数已由 Control 入口同步，此处仅依据运行请求和明确反馈状态推进状态机
    const bool run_requested = config->enabled && (mode == GIMBAL_MODE_SPEED || mode == GIMBAL_MODE_ANGLE);
    const bool feedback_ready = imu_ready && internal->feedback_time_us != UINT64_MAX;

    switch (axis->state)
    {
        // 待机与失联恢复共用起控路径，失联期间仍校验共用反馈和控制输入
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
            if (!Gimbal_RunAxis(axis, internal, config, mode, command, period_s))
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
            if (!feedback_ready || !Gimbal_RunAxis(axis, internal, config, mode, command, period_s))
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

void Gimbal_UpdateFeedback(Gimbal_t *gimbal, const MOTOR_RM_Feedback_t *yaw_motor,
                           const MOTOR_RM_Feedback_t *pitch_motor, const Gimbal_ImuFeedback_t *imu, uint64_t now_us)
{
    // 反馈阶段使用最近一次通过集中校验的配置，新的在线调参留给随后 Control 采纳
    const uint64_t timeout_us = (uint64_t)(gimbal_internal.control_config.feedback_timeout_s * 1.0e6f);
    Gimbal_UpdateAxisFeedback(&gimbal->yaw, &gimbal_internal.yaw, &gimbal_internal.control_config.yaw,
                              yaw_motor, now_us, timeout_us, false);
    Gimbal_UpdateAxisFeedback(&gimbal->pitch, &gimbal_internal.pitch, &gimbal_internal.control_config.pitch,
                              pitch_motor, now_us, timeout_us, true);
    Gimbal_UpdateImu(gimbal, &gimbal_internal, imu, now_us);
}

static void Gimbal_ClearFeedback(Gimbal_Axis_t *axis, Gimbal_AxisInternal_t *internal)
{
    axis->encoder_deg = NAN;
    axis->mechanical_angle_deg = NAN;
    axis->motor_speed_rad_s = NAN;
    internal->feedback_time_us = UINT64_MAX;
    Gimbal_ResetAngleUnwrap(&internal->unwrap);
}

static void Gimbal_RejectAxisFeedback(Gimbal_Axis_t *axis, Gimbal_AxisInternal_t *internal,
                                      Gimbal_AxisState_t next_state)
{
    // 反馈事件只改变已运行轴的保护状态，待机继续等待，已有故障不得降级为失联
    switch (axis->state)
    {
        case GIMBAL_AXIS_RUNNING:
        case GIMBAL_AXIS_OFFLINE:
            Gimbal_StopAxis(axis, internal, next_state);
            break;
        case GIMBAL_AXIS_RELAX:
        case GIMBAL_AXIS_FAULT:
        case GIMBAL_AXIS_TRIPPED:
            break;
        default:
            Gimbal_StopAxis(axis, internal, GIMBAL_AXIS_FAULT);
            break;
    }
    Gimbal_ClearFeedback(axis, internal);
}

static void Gimbal_InvalidateImu(Gimbal_t *gimbal, Gimbal_Internal_t *internal)
{
    internal->imu.valid = false;
    gimbal->yaw.angle_deg = gimbal->pitch.angle_deg = NAN;
    gimbal->yaw.speed_rad_s = gimbal->pitch.speed_rad_s = NAN;

    // 启动期间允许继续等待，运行后的共用反馈异常必须锁存至人工待机
    switch (gimbal->imu_state)
    {
        case GIMBAL_IMU_WAITING:
            break;

        case GIMBAL_IMU_READY:
        case GIMBAL_IMU_FAULT:
        default:
            gimbal->imu_state = GIMBAL_IMU_FAULT;
            break;
    }
}

static void Gimbal_UpdateImu(Gimbal_t *gimbal, Gimbal_Internal_t *internal,
                             const Gimbal_ImuFeedback_t *imu, uint64_t now_us)
{
    const float timeout_s = internal->control_config.imu_timeout_s;
    const uint64_t timeout_us = (uint64_t)(timeout_s * 1.0e6f);
    const Gimbal_ImuFeedback_t *frame = imu != NULL ? imu : &internal->imu;
    bool valid = frame->valid && frame->sample_time_us <= now_us &&
                 now_us - frame->sample_time_us <= timeout_us;

    for (uint32_t i = 0U; i < 3U; ++i)
    {
        valid = valid && isfinite(frame->angle_deg[i]) && isfinite(frame->gyro_rad_s[i]) &&
                fabsf(frame->gyro_rad_s[i]) <= GIMBAL_IMU_MAX_RAD_S;
    }
    valid = valid && fabsf(frame->angle_deg[0]) <= 180.0f &&
            fabsf(frame->angle_deg[1]) < GIMBAL_IMU_MAX_PITCH_DEG;

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
        switch (gimbal->imu_state)
        {
            case GIMBAL_IMU_WAITING:
                gimbal->imu_state = GIMBAL_IMU_READY;
                break;

            case GIMBAL_IMU_READY:
            case GIMBAL_IMU_FAULT:
                break;

            default:
                Gimbal_InvalidateImu(gimbal, internal);
                break;
        }
    }

    const bool ready = gimbal->imu_state == GIMBAL_IMU_READY;
    gimbal->yaw.angle_deg = ready ? internal->imu.angle_deg[2] : NAN;
    gimbal->pitch.angle_deg = ready ? internal->imu.angle_deg[1] : NAN;

    const float roll_rad = internal->imu.angle_deg[0] * M_DEG2RAD_MULT;
    const float sin_roll = sinf(roll_rad);
    const float cos_roll = cosf(roll_rad);
    const float gyro_y = internal->imu.gyro_rad_s[1];
    const float gyro_z = internal->imu.gyro_rad_s[2];
    const float yaw_rate = (gyro_y * sin_roll + gyro_z * cos_roll) / cosf(internal->imu.angle_deg[1] * M_DEG2RAD_MULT);
    gimbal->yaw.speed_rad_s = ready ? yaw_rate : NAN;
    gimbal->pitch.speed_rad_s = ready ? gyro_y * cos_roll - gyro_z * sin_roll : NAN;
}

static void Gimbal_UpdateAxisFeedback(Gimbal_Axis_t *axis, Gimbal_AxisInternal_t *internal,
                                      const Gimbal_AxisConfig_t *config, const MOTOR_RM_Feedback_t *motor,
                                      uint64_t now_us, uint64_t timeout_us, bool signed_angle)
{
    if (motor == NULL)
    {
        if (internal->feedback_time_us == UINT64_MAX || now_us < internal->feedback_time_us ||
            now_us - internal->feedback_time_us > timeout_us)
        {
            Gimbal_RejectAxisFeedback(axis, internal, now_us < internal->feedback_time_us &&
                internal->feedback_time_us != UINT64_MAX ? GIMBAL_AXIS_FAULT : GIMBAL_AXIS_OFFLINE);
        }
        return;
    }

    const uint32_t age_us = (uint32_t)now_us - motor->last_update_time;
    const uint64_t frame_time_us = now_us >= age_us ? now_us - age_us : UINT64_MAX;
    const float encoder_rad = motor->rotor_single_angle;
    const float speed_rpm = motor->rotor_speed;
    // 编码器一旦出现丢帧，输出轴机械角可能整圈偏移，云台在重新对零前保持该轴故障
    const bool frame_valid = motor->angle_valid && motor->angle_lost_count == 0U &&
                             frame_time_us != UINT64_MAX && age_us <= timeout_us && encoder_rad >= 0.0f &&
                             encoder_rad < M_2PI;
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

    // 反装轴先翻转角度和转速，使模块内统一以角度增大方向为正
    const float raw_encoder_deg = Gimbal_WrapAngle(encoder_rad * M_RAD2DEG_MULT, GIMBAL_FULL_TURN_DEG);
    const float encoder_deg = config->reversed ?
        Gimbal_WrapAngle(GIMBAL_FULL_TURN_DEG - raw_encoder_deg, GIMBAL_FULL_TURN_DEG) : raw_encoder_deg;
    const float speed_rad_s = speed_rpm * GIMBAL_RPM_TO_RAD_S;
    axis->encoder_deg = encoder_deg;
    axis->motor_speed_rad_s = config->reversed ? -speed_rad_s : speed_rad_s;
    internal->feedback_time_us = frame_time_us;

    // pitch 行程围绕水平零点，直接映射到 [-180, 180)，负角上电和失联恢复均不累计圈数
    if (signed_angle)
    {
        const float half_turn_deg = GIMBAL_FULL_TURN_DEG * 0.5f;
        axis->mechanical_angle_deg = Gimbal_WrapAngle(encoder_deg - internal->applied_zero_encoder_deg + half_turn_deg,
                                                     GIMBAL_FULL_TURN_DEG) - half_turn_deg;
        return;
    }

    // yaw 保留原有连续角展开，跨编码器零点时按相邻帧最短路径累计
    const float relative_deg = Gimbal_WrapAngle(encoder_deg - internal->applied_zero_encoder_deg,
                                                 GIMBAL_FULL_TURN_DEG);
    float angle_deg = NAN;
    if (!Gimbal_UpdateAngleUnwrap(&internal->unwrap, relative_deg, GIMBAL_FULL_TURN_DEG, &angle_deg))
    {
        Gimbal_RejectAxisFeedback(axis, internal, GIMBAL_AXIS_FAULT);
        return;
    }
    axis->mechanical_angle_deg = angle_deg;
}

static void Gimbal_StopAxis(Gimbal_Axis_t *axis, Gimbal_AxisInternal_t *internal,
                            Gimbal_AxisState_t next_state)
{
    PID_Reset(&internal->angle_pid);
    PID_Reset(&internal->speed_pid);
    axis->target_angle_deg = NAN;
    axis->target_speed_rad_s = 0.0f;
    axis->output_current_a = 0.0f;
    axis->state = next_state == GIMBAL_AXIS_RELAX || next_state == GIMBAL_AXIS_OFFLINE ||
        next_state == GIMBAL_AXIS_TRIPPED ? next_state : GIMBAL_AXIS_FAULT;
}
