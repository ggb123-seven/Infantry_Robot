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
 * - SPIN_RAD_S、SPIN_TIME_S：编码器相对转速超过 15 rad/s 持续 0.05 s 判为疯转
 *   遥控命令上限 6 rad/s、当前小陀螺约 1 rad/s、GM6020 空载约 33 rad/s，阈值位于两者之间
 * - DRAG_RAD_S、DRAG_IMU_STILL_RAD_S、DRAG_TIME_S：底盘离线时编码器与惯性转速差超过 1 rad/s
 *   且上部惯性转速低于 0.3 rad/s，持续 0.15 s 判为悬空带转，沿用原低速检测阈值
 * - CHASSIS_OFFLINE_TIME_S：四个底盘电机全部离线持续 1 s 才启用悬空带转检测，覆盖小陀螺掉线后的惯性滑转
 */
#define GIMBAL_YAW_SPIN_RAD_S (15.0f)
#define GIMBAL_YAW_SPIN_TIME_S (0.05f)
#define GIMBAL_YAW_DRAG_RAD_S (1.0f)
#define GIMBAL_YAW_DRAG_IMU_STILL_RAD_S (0.3f)
#define GIMBAL_YAW_DRAG_TIME_S (0.15f)
#define GIMBAL_CHASSIS_OFFLINE_TIME_S (1.0f)

static void Gimbal_RejectAxisFeedback(Gimbal_Axis_t *axis, Gimbal_AxisState_t next_state);
static void Gimbal_InvalidateImu(Gimbal_t *gimbal);
static void Gimbal_UpdateImu(Gimbal_t *gimbal, const Gimbal_ImuFeedback_t *imu, uint64_t now_us);
static void Gimbal_UpdateAxisFeedback(Gimbal_Axis_t *axis, const Gimbal_AxisConfig_t *config,
                                      const MOTOR_RM_Feedback_t *motor, uint64_t now_us, uint64_t timeout_us,
                                      bool signed_angle);

static void Gimbal_ClearFeedback(Gimbal_Axis_t *axis);
static void Gimbal_StopAxis(Gimbal_Axis_t *axis, Gimbal_AxisState_t next_state);
static bool Gimbal_IsPidConfigValid(const KPID_Params_t *pid);
static bool Gimbal_IsAxisConfigValid(const Gimbal_AxisConfig_t *config);
static bool Gimbal_IsConfigValid(const Gimbal_Config_t *config, float frequency_hz);
static bool Gimbal_IsPidChanged(const KPID_Params_t *config, const KPID_Params_t *applied);
static int8_t Gimbal_InitAxis(Gimbal_Axis_t *axis, const Gimbal_AxisConfig_t *config, float frequency_hz);
static void Gimbal_ApplyAxisConfig(Gimbal_Axis_t *axis, const Gimbal_AxisConfig_t *config);
static void Gimbal_EnterRunning(Gimbal_Axis_t *axis);
static bool Gimbal_RunAxis(Gimbal_Axis_t *axis, const Gimbal_AxisConfig_t *config, Gimbal_Mode_t mode,
                           const Gimbal_AxisSetpoint_t *command, float period_s);
static void Gimbal_ControlAxis(Gimbal_Axis_t *axis, const Gimbal_AxisConfig_t *config, Gimbal_Mode_t mode,
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
static void Gimbal_ResetAngleUnwrap(AngleUnwrap_t *unwrap)
{
    unwrap->has_reference = false;
    unwrap->last_wrapped_angle = 0.0f;
    unwrap->continuous_angle = 0.0f;
}

/**
 * @brief 按相邻两帧最短路径展开单圈角度
 *
 * @param[in,out] unwrap 展开器状态，调用方保证非空
 * @param[in] wrapped_angle 本帧单圈角度，调用方保证已经过反馈入口校验及单圈环绕
 * @param[in] period 单圈周期，调用方保证为有限正数
 * @param[out] continuous_angle 展开后的连续角度，调用方保证非空
 * @return 成功返回 true；半圈歧义或结果溢出时返回 false 并清除参考
 */
static bool Gimbal_UpdateAngleUnwrap(AngleUnwrap_t *unwrap, float wrapped_angle, float period, float *continuous_angle)
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
    // 先复制配置并清除全部运行数据，任一失败路径都保持两轴零输出
    const Gimbal_Config_t saved_config = *config;
    memset(gimbal, 0, sizeof(*gimbal));
    gimbal->config = saved_config;
    gimbal->mode = GIMBAL_MODE_RELAX;
    gimbal->yaw.state = gimbal->pitch.state = GIMBAL_AXIS_FAULT;
    Gimbal_ClearFeedback(&gimbal->yaw);
    Gimbal_ClearFeedback(&gimbal->pitch);
    gimbal->yaw.angle_deg = gimbal->pitch.angle_deg = NAN;
    gimbal->yaw.speed_rad_s = gimbal->pitch.speed_rad_s = NAN;
    gimbal->yaw.base_speed_rad_s = gimbal->pitch.base_speed_rad_s = NAN;
    gimbal->config_valid = Gimbal_IsConfigValid(&saved_config, frequency_hz);
    if (!gimbal->config_valid)
    {
        return -1;
    }

    // 两轴共享已校验快照，PID 参数继续指向各轴已有持久副本
    gimbal->control_config = saved_config;
    if (Gimbal_InitAxis(&gimbal->yaw, &saved_config.yaw, frequency_hz) != 0 ||
        Gimbal_InitAxis(&gimbal->pitch, &saved_config.pitch, frequency_hz) != 0)
    {
        return -1;
    }
    return 0;
}

/**
 * @brief 将只读单轴命令解析到持久设定值，增量以上周期实际限幅目标为基准
 *
 * @param[in,out] axis 单轴状态及模块持有的设定值
 * @param[in] command 只读单轴命令
 * @param[in] mode 当前控制模式
 * @param[in] period_s 控制周期，单位秒
 * @return 目标来源合法时为 true，非法来源置非数值并返回 false
 */
static bool Gimbal_ResolveSetpoint(Gimbal_Axis_t *axis, const Gimbal_AxisCommand_t *command,
                                   Gimbal_Mode_t mode, float period_s)
{
    // 外部命令只在此入口检查，非法速度不能被起控时的姿态捕获掩盖
    axis->setpoint.target_speed_rad_s = command->rate_rad_s;
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
            axis->setpoint.target_angle_deg = command->rate_rad_s * M_RAD2DEG_MULT * period_s;
            if (axis->state == GIMBAL_AXIS_RUNNING && isfinite(axis->target_angle_deg))
            {
                axis->setpoint.target_angle_deg += axis->target_angle_deg;
            }
            return isfinite(axis->setpoint.target_angle_deg);
        case GIMBAL_TARGET_ABSOLUTE:
            axis->setpoint.target_angle_deg = command->angle_deg;
            return isfinite(axis->setpoint.target_angle_deg);
        default:
            axis->setpoint.target_angle_deg = axis->setpoint.target_speed_rad_s = NAN;
            return false;
    }
}

void Gimbal_Control(Gimbal_t *gimbal, const Gimbal_Command_t *command, bool chassis_online, float period_s)
{
    // 在线调参只在 Control 入口复制并校验，失败时不采纳快照中的任何字段
    const Gimbal_Config_t snapshot = *(const volatile Gimbal_Config_t *)&gimbal->config;
    gimbal->config_valid = Gimbal_IsConfigValid(&snapshot, 1.0f / gimbal->yaw.speed_pid.dt_min);
    if (!gimbal->config_valid || !isfinite(period_s) || period_s <= 0.0f || period_s > GIMBAL_MAX_PERIOD_S)
    {
        Gimbal_StopAxis(&gimbal->yaw, GIMBAL_AXIS_FAULT);
        Gimbal_StopAxis(&gimbal->pitch, GIMBAL_AXIS_FAULT);
        return;
    }
    gimbal->control_config = snapshot;
    Gimbal_ApplyAxisConfig(&gimbal->yaw, &snapshot.yaw);
    Gimbal_ApplyAxisConfig(&gimbal->pitch, &snapshot.pitch);

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
        Gimbal_StopAxis(&gimbal->yaw, GIMBAL_AXIS_FAULT);
        Gimbal_StopAxis(&gimbal->pitch, GIMBAL_AXIS_FAULT);
        return;
    }

    // 公开命令先解析为轴内设定值，姿态捕获及保护的覆盖优先于增量
    const bool yaw_valid = Gimbal_ResolveSetpoint(&gimbal->yaw, &active_command->yaw, mode, period_s);
    const bool pitch_valid = Gimbal_ResolveSetpoint(&gimbal->pitch, &active_command->pitch, mode, period_s);

    // 共用源故障只在人工回待机后解除，下一轮等待新的有效快照
    if (mode == GIMBAL_MODE_RELAX && gimbal->imu_state == GIMBAL_IMU_FAULT)
    {
        gimbal->imu_state = GIMBAL_IMU_WAITING;
        gimbal->imu.valid = false;
    }

    // 待机起控或失联恢复时捕获当前姿态，避免追赶停止前的旧目标
    if (mode == GIMBAL_MODE_ANGLE && gimbal->imu_state == GIMBAL_IMU_READY)
    {
        if (yaw_valid &&
            (gimbal->yaw.state == GIMBAL_AXIS_RELAX || gimbal->yaw.state == GIMBAL_AXIS_OFFLINE || mode_changed) &&
            gimbal->yaw.feedback_time_us != UINT64_MAX)
        {
            gimbal->yaw.setpoint.target_angle_deg = gimbal->yaw.angle_deg;
            gimbal->yaw.setpoint.target_speed_rad_s = 0.0f;
        }
        if (pitch_valid &&
            (gimbal->pitch.state == GIMBAL_AXIS_RELAX || gimbal->pitch.state == GIMBAL_AXIS_OFFLINE || mode_changed) &&
            gimbal->pitch.feedback_time_us != UINT64_MAX)
        {
            gimbal->pitch.setpoint.target_angle_deg = gimbal->pitch.angle_deg;
            gimbal->pitch.setpoint.target_speed_rad_s = 0.0f;
        }
    }

    // 两轴独立推进状态机，非法目标只使对应轴进入故障
    const bool imu_ready = gimbal->imu_state == GIMBAL_IMU_READY;
    if (yaw_valid)
    {
        Gimbal_ControlAxis(&gimbal->yaw, &snapshot.yaw, mode, mode_changed, &gimbal->yaw.setpoint, period_s, imu_ready);
    }
    else
    {
        Gimbal_StopAxis(&gimbal->yaw, GIMBAL_AXIS_FAULT);
    }
    if (pitch_valid)
    {
        Gimbal_ControlAxis(&gimbal->pitch, &snapshot.pitch, mode, mode_changed, &gimbal->pitch.setpoint,
                           period_s, imu_ready);
    }
    else
    {
        Gimbal_StopAxis(&gimbal->pitch, GIMBAL_AXIS_FAULT);
    }

    // 任一底盘电机在线即可能主动旋转，四个全部离线持续确认后底座转速才视为零
    gimbal->chassis_offline_s = chassis_online ? 0.0f :
        fminf(GIMBAL_CHASSIS_OFFLINE_TIME_S, gimbal->chassis_offline_s + period_s);

    // yaw 闭环后检查疯转与悬空带转，运行状态保证两种转速有限，撤驱在本周期输出闸门前生效
    const Gimbal_Axis_t *yaw = &gimbal->yaw;
    const bool running = yaw->state == GIMBAL_AXIS_RUNNING;
    const bool spinning = running && fabsf(yaw->motor_speed_rad_s) > GIMBAL_YAW_SPIN_RAD_S;
    const bool dragged = running && gimbal->chassis_offline_s >= GIMBAL_CHASSIS_OFFLINE_TIME_S &&
        fabsf(yaw->motor_speed_rad_s - yaw->speed_rad_s) > GIMBAL_YAW_DRAG_RAD_S &&
        fabsf(yaw->speed_rad_s) < GIMBAL_YAW_DRAG_IMU_STILL_RAD_S;
    gimbal->yaw_spin_s = spinning ? gimbal->yaw_spin_s + period_s : 0.0f;
    gimbal->yaw_drag_s = dragged ? gimbal->yaw_drag_s + period_s : 0.0f;
    if (gimbal->yaw_spin_s >= GIMBAL_YAW_SPIN_TIME_S || gimbal->yaw_drag_s >= GIMBAL_YAW_DRAG_TIME_S)
    {
        Gimbal_StopAxis(&gimbal->yaw, GIMBAL_AXIS_TRIPPED);
    }
    gimbal->mode = mode;
}

void Gimbal_DumpOutput(const Gimbal_t *gimbal, Gimbal_Output_t *output)
{
    // 唯一输出闸门按轴状态放行，只使用本周期已采纳方向，不读取 EmberProbe 可写配置
    const float yaw_current_a = gimbal->yaw.state == GIMBAL_AXIS_RUNNING ? gimbal->yaw.output_current_a : 0.0f;
    const float pitch_current_a = gimbal->pitch.state == GIMBAL_AXIS_RUNNING ? gimbal->pitch.output_current_a : 0.0f;
    output->yaw_current_a = gimbal->yaw.applied_reversed ? -yaw_current_a : yaw_current_a;
    output->pitch_current_a = gimbal->pitch.applied_reversed ? -pitch_current_a : pitch_current_a;
}

/**
 * @brief 校验单个 PID 参数是否可安全参与计算
 *
 * @param[in] pid PID 参数
 * @return 增益、限幅和截止频率均有限且在合法范围内时返回 true
 */
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

/**
 * @brief 校验单轴配置是否可安全参与控制
 *
 * @param[in] config 单轴配置
 * @return 零点、角度范围、限幅和两环参数均合法时返回 true
 */
static bool Gimbal_IsAxisConfigValid(const Gimbal_AxisConfig_t *config)
{
    return isfinite(config->zero_encoder_deg) && config->zero_encoder_deg >= 0.0f &&
           config->zero_encoder_deg < GIMBAL_FULL_TURN_DEG && isfinite(config->min_angle_deg) &&
           isfinite(config->max_angle_deg) && config->min_angle_deg <= config->max_angle_deg &&
           isfinite(config->max_speed_rad_s) && config->max_speed_rad_s > 0.0f &&
           isfinite(config->max_current_a) && config->max_current_a > 0.0f &&
           Gimbal_IsPidConfigValid(&config->angle_pid) && Gimbal_IsPidConfigValid(&config->speed_pid);
}

/**
 * @brief 集中校验完整云台配置，供初始化与每周期 Control 入口调用
 *
 * @param[in] config 待校验快照，调用方保证非空
 * @param[in] frequency_hz 控制采样频率，单位 Hz
 * @return 两轴参数及可表示的反馈超时均合法时返回 true
 */
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

/**
 * @brief 判断配置中的 PID 参数是否与实际采用的副本不同
 *
 * @param[in] config 配置中的 PID 参数
 * @param[in] applied 实际采用的 PID 参数副本
 * @return 影响输出或积分的任一字段不同时返回 true
 */
static bool Gimbal_IsPidChanged(const KPID_Params_t *config, const KPID_Params_t *applied)
{
    return config->k != applied->k || config->p != applied->p || config->i != applied->i ||
           config->d != applied->d || config->i_limit != applied->i_limit || config->out_limit != applied->out_limit ||
           config->d_cutoff_freq != applied->d_cutoff_freq;
}

/**
 * @brief 使用已校验的单轴配置初始化串级 PID
 *
 * @param[out] axis 单轴运行数据
 * @param[in] config 单轴配置
 * @param[in] frequency_hz 控制频率，单位 Hz
 * @return 成功返回 0，PID 初始化失败返回 -1
 */
static int8_t Gimbal_InitAxis(Gimbal_Axis_t *axis, const Gimbal_AxisConfig_t *config, float frequency_hz)
{
    // PID 指向实例内的参数副本，调试器修改配置后由每周期比较决定何时同步
    axis->applied_zero_encoder_deg = config->zero_encoder_deg;
    axis->applied_reversed = config->reversed;
    axis->applied_angle_pid = config->angle_pid;
    axis->applied_speed_pid = config->speed_pid;

    // 内环采用大地系欧拉角速度，外环保留当前零微分配置
    if (PID_Init(&axis->angle_pid, KPID_MODE_SET_D, frequency_hz, &axis->applied_angle_pid) != 0 ||
        PID_Init(&axis->speed_pid, KPID_MODE_CALC_D, frequency_hz, &axis->applied_speed_pid) != 0)
    {
        return -1;
    }
    axis->state = GIMBAL_AXIS_RELAX;
    return 0;
}

/**
 * @brief 同步单轴配置，零点或方向改变使运行轴故障，PID 参数改变清空对应积分
 *
 * @param[in,out] axis 单轴运行数据
 * @param[in] config 单轴配置
 * @return 无返回值，调用方保证配置已经通过本周期集中校验
 */
static void Gimbal_ApplyAxisConfig(Gimbal_Axis_t *axis, const Gimbal_AxisConfig_t *config)
{
    // 零点或方向变化后机械坐标失效，运行中锁存故障，下一帧重新建立机械角
    if (config->zero_encoder_deg != axis->applied_zero_encoder_deg || config->reversed != axis->applied_reversed)
    {
        axis->applied_zero_encoder_deg = config->zero_encoder_deg;
        axis->applied_reversed = config->reversed;
        Gimbal_ClearFeedback(axis);
        if (axis->state == GIMBAL_AXIS_RUNNING || axis->state == GIMBAL_AXIS_OFFLINE)
        {
            Gimbal_StopAxis(axis, GIMBAL_AXIS_FAULT);
        }
    }

    // 参数副本与配置不同时同步并清空积分，比例和微分项不受历史影响
    if (Gimbal_IsPidChanged(&config->angle_pid, &axis->applied_angle_pid))
    {
        if (config->angle_pid.d_cutoff_freq != axis->applied_angle_pid.d_cutoff_freq)
        {
            LowPassFilter2p_Init(&axis->angle_pid.dfilter, 1.0f / axis->angle_pid.dt_min,
                                 config->angle_pid.d_cutoff_freq);
        }
        axis->applied_angle_pid = config->angle_pid;
        PID_ResetIntegral(&axis->angle_pid);
    }
    if (Gimbal_IsPidChanged(&config->speed_pid, &axis->applied_speed_pid))
    {
        if (config->speed_pid.d_cutoff_freq != axis->applied_speed_pid.d_cutoff_freq ||
            config->speed_pid.k != axis->applied_speed_pid.k)
        {
            LowPassFilter2p_Init(&axis->speed_pid.dfilter, 1.0f / axis->speed_pid.dt_min,
                                 config->speed_pid.d_cutoff_freq);
            const float scaled_speed = config->speed_pid.k * axis->speed_rad_s;
            axis->speed_pid.last.k_fb = LowPassFilter2p_Reset(&axis->speed_pid.dfilter, scaled_speed);
        }
        axis->applied_speed_pid = config->speed_pid;
        PID_ResetIntegral(&axis->speed_pid);
    }
}

/**
 * @brief 进入运行或切换模式时清空两环历史，并以当前转速预置速度环微分
 *
 * @param[in,out] axis 单轴运行数据，转速须为有限值
 * @return 无返回值
 */
static void Gimbal_EnterRunning(Gimbal_Axis_t *axis)
{
    PID_Reset(&axis->angle_pid);
    PID_Reset(&axis->speed_pid);

    // 速度环微分作用于反馈，预置上一反馈可避免入口处的微分冲击
    const float scaled_speed = axis->applied_speed_pid.k * axis->speed_rad_s;
    axis->speed_pid.last.k_fb = LowPassFilter2p_Reset(&axis->speed_pid.dfilter, scaled_speed);
    axis->state = GIMBAL_AXIS_RUNNING;
}

/**
 * @brief 执行单轴一个周期的闭环计算
 *
 * @param[in,out] axis 单轴运行数据，须处于 RUNNING
 * @param[in] config 单轴配置，已通过校验
 * @param[in] mode 控制模式，只能为 SPEED 或 ANGLE
 * @param[in] command 单轴命令
 * @param[in] period_s 实际控制间隔，单位 s
 * @return 计算成功返回 true；反馈失联、命令或周期非法、计算结果异常时返回 false
 */
static bool Gimbal_RunAxis(Gimbal_Axis_t *axis, const Gimbal_AxisConfig_t *config, Gimbal_Mode_t mode,
                           const Gimbal_AxisSetpoint_t *command, float period_s)
{
    // 调用方已保证反馈就绪、命令合法及周期有效，本层只执行控制计算
    const bool angle_needed = mode == GIMBAL_MODE_ANGLE;
    // SPEED 模式直接采用命令角速度，ANGLE 模式将其作为前馈与角度环修正量合成
    const float max_speed_rad_s = config->max_speed_rad_s;
    const float command_speed_rad_s = AbsClip(command->target_speed_rad_s, max_speed_rad_s);
    float target_speed_rad_s = command_speed_rad_s;
    const float previous_angle_integral = axis->angle_pid.i;
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
        const float correction_rad_s = PID_Calc(&axis->angle_pid, target_angle_deg * M_DEG2RAD_MULT,
                                                axis->angle_deg * M_DEG2RAD_MULT, axis->speed_rad_s, period_s);
        const float combined_rad_s = correction_rad_s + command_speed_rad_s;
        target_speed_rad_s = AbsClip(combined_rad_s, max_speed_rad_s);
        // 合成速度饱和且误差继续推向饱和方向时撤销本周期积分，保留反向退饱和能力
        if ((combined_rad_s > max_speed_rad_s && target_angle_deg > axis->angle_deg) ||
            (combined_rad_s < -max_speed_rad_s && target_angle_deg < axis->angle_deg))
        {
            axis->angle_pid.i = previous_angle_integral;
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
    const float previous_speed_integral = axis->speed_pid.i;
    const float current_a = PID_Calc(&axis->speed_pid, target_speed_rad_s, axis->speed_rad_s, 0.0f, period_s);
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
        axis->speed_pid.i = previous_speed_integral;
        mechanically_limited = true;
    }
    if (mechanically_limited)
    {
        axis->angle_pid.i = previous_angle_integral;
    }
    return isfinite(axis->output_current_a) && isfinite(axis->speed_pid.i);
}

/**
 * @brief 单轴状态机，集中决定起控、失联恢复、停止和故障锁存
 *
 * @param[in,out] axis 单轴运行数据
 * @param[in] config 单轴配置
 * @param[in] mode 本周期控制模式
 * @param[in] mode_changed 本周期模式与上一周期不同时为 true
 * @param[in] command 单轴命令
 * @param[in] period_s 实际控制间隔，单位 s
 * @param[in] imu_ready 共用惯性源本周期已经通过反馈边界校验
 * @return 无返回值
 */
static void Gimbal_ControlAxis(Gimbal_Axis_t *axis, const Gimbal_AxisConfig_t *config, Gimbal_Mode_t mode,
                               bool mode_changed, const Gimbal_AxisSetpoint_t *command, float period_s,
                               bool imu_ready)
{
    // 参数已由 Control 入口同步，此处仅依据运行请求和明确反馈状态推进状态机
    const bool run_requested = config->enabled && (mode == GIMBAL_MODE_SPEED || mode == GIMBAL_MODE_ANGLE);
    const bool feedback_ready = imu_ready && axis->feedback_time_us != UINT64_MAX;

    switch (axis->state)
    {
        // 待机与失联恢复共用起控路径，失联期间仍校验共用反馈和控制输入
        case GIMBAL_AXIS_RELAX:
        case GIMBAL_AXIS_OFFLINE:
            if (axis->state == GIMBAL_AXIS_OFFLINE && run_requested && !imu_ready)
            {
                Gimbal_StopAxis(axis, GIMBAL_AXIS_FAULT);
                break;
            }
            if (!run_requested || !feedback_ready)
            {
                Gimbal_StopAxis(axis, run_requested && axis->state == GIMBAL_AXIS_OFFLINE ?
                    GIMBAL_AXIS_OFFLINE : GIMBAL_AXIS_RELAX);
                break;
            }
            Gimbal_EnterRunning(axis);
            if (!Gimbal_RunAxis(axis, config, mode, command, period_s))
            {
                Gimbal_StopAxis(axis, GIMBAL_AXIS_FAULT);
            }
            break;

        // 运行中撤销请求正常停机，配置或反馈异常锁存故障，切换模式时清空两环历史
        case GIMBAL_AXIS_RUNNING:
            if (!run_requested)
            {
                Gimbal_StopAxis(axis, GIMBAL_AXIS_RELAX);
                break;
            }
            if (mode_changed)
            {
                Gimbal_EnterRunning(axis);
            }
            if (!feedback_ready || !Gimbal_RunAxis(axis, config, mode, command, period_s))
            {
                Gimbal_StopAxis(axis, GIMBAL_AXIS_FAULT);
            }
            break;

        // 故障与保护撤驱均锁存至命令回到 RELAX，避免反馈恢复后在未知位置自动重新出力
        case GIMBAL_AXIS_FAULT:
        case GIMBAL_AXIS_TRIPPED:
            Gimbal_StopAxis(axis, mode == GIMBAL_MODE_RELAX ? GIMBAL_AXIS_RELAX : axis->state);
            break;

        default:
            Gimbal_StopAxis(axis, GIMBAL_AXIS_FAULT);
            break;
    }
}

void Gimbal_UpdateFeedback(Gimbal_t *gimbal, const MOTOR_RM_Feedback_t *yaw_motor,
                           const MOTOR_RM_Feedback_t *pitch_motor, const Gimbal_ImuFeedback_t *imu, uint64_t now_us)
{
    // 反馈阶段使用最近一次通过集中校验的配置，新的在线调参留给随后 Control 采纳
    const uint64_t timeout_us = (uint64_t)(gimbal->control_config.feedback_timeout_s * 1.0e6f);
    Gimbal_UpdateAxisFeedback(&gimbal->yaw, &gimbal->control_config.yaw, yaw_motor, now_us, timeout_us, false);
    Gimbal_UpdateAxisFeedback(&gimbal->pitch, &gimbal->control_config.pitch, pitch_motor, now_us, timeout_us, true);

    // 惯性快照独立校验时效和连续性，控制量只由该快照更新
    Gimbal_UpdateImu(gimbal, imu, now_us);

    // 两侧反馈更新后估计底座大地系角速度，底座水平时与关节共轴，任一侧为非数值时结果同为非数值
    gimbal->yaw.base_speed_rad_s = gimbal->yaw.speed_rad_s - gimbal->yaw.motor_speed_rad_s;
    gimbal->pitch.base_speed_rad_s = gimbal->pitch.speed_rad_s - gimbal->pitch.motor_speed_rad_s;
}

/**
 * @brief 撤销机械反馈并断开连续角参考
 * @param[out] axis 单轴运行数据
 * @return 无返回值
 */
static void Gimbal_ClearFeedback(Gimbal_Axis_t *axis)
{
    axis->encoder_deg = NAN;
    axis->mechanical_angle_deg = NAN;
    axis->motor_speed_rad_s = NAN;
    axis->base_speed_rad_s = NAN;
    axis->feedback_time_us = UINT64_MAX;
    Gimbal_ResetAngleUnwrap(&axis->unwrap);
}

/**
 * @brief 拒绝机械反馈并集中处理失联等待与不可自动恢复的反馈故障
 *
 * @param[in,out] axis 单轴运行数据
 * @param[in] next_state 超时传 OFFLINE，其他反馈异常传 FAULT
 * @return 无返回值
 */
static void Gimbal_RejectAxisFeedback(Gimbal_Axis_t *axis, Gimbal_AxisState_t next_state)
{
    // 反馈事件只改变已运行轴的保护状态，待机继续等待，已有故障不得降级为失联
    switch (axis->state)
    {
        case GIMBAL_AXIS_RUNNING:
        case GIMBAL_AXIS_OFFLINE:
            Gimbal_StopAxis(axis, next_state);
            break;
        case GIMBAL_AXIS_RELAX:
        case GIMBAL_AXIS_FAULT:
        case GIMBAL_AXIS_TRIPPED:
            break;
        default:
            Gimbal_StopAxis(axis, GIMBAL_AXIS_FAULT);
            break;
    }
    Gimbal_ClearFeedback(axis);
}

/**
 * @brief 撤销惯性控制量并根据当前阶段进入等待或故障
 *
 * @param[in,out] gimbal 云台实例
 * @return 无返回值
 */
static void Gimbal_InvalidateImu(Gimbal_t *gimbal)
{
    gimbal->imu.valid = false;
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

/**
 * @brief 校验同帧惯性反馈并推进有效就绪与故障状态机
 *
 * @param[in,out] gimbal 云台实例
 * @param[in] imu 新快照，无新消息时为 NULL
 * @param[in] now_us 当前单调时刻，单位微秒
 * @return 无返回值
 */
static void Gimbal_UpdateImu(Gimbal_t *gimbal, const Gimbal_ImuFeedback_t *imu, uint64_t now_us)
{
    const float timeout_s = gimbal->control_config.imu_timeout_s;
    const uint64_t timeout_us = (uint64_t)(timeout_s * 1.0e6f);
    const Gimbal_ImuFeedback_t *frame = imu != NULL ? imu : &gimbal->imu;
    bool valid = frame->valid && frame->sample_time_us <= now_us &&
                 now_us - frame->sample_time_us <= timeout_us;

    // 所有轴共同校验，任何非有限数据都不能进入任一轴的闭环
    for (uint32_t i = 0U; i < 3U; ++i)
    {
        valid = valid && isfinite(frame->angle_deg[i]) && isfinite(frame->gyro_rad_s[i]) &&
                fabsf(frame->gyro_rad_s[i]) <= GIMBAL_IMU_MAX_RAD_S;
    }
    valid = valid && fabsf(frame->angle_deg[0]) <= 180.0f &&
            fabsf(frame->angle_deg[1]) < GIMBAL_IMU_MAX_PITCH_DEG;

    // 新帧必须严格前进，重复帧、倒退及断流后突然恢复均不得掩盖坐标断开
    if (imu != NULL && gimbal->imu.valid)
    {
        const uint64_t previous_us = gimbal->imu.sample_time_us;
        const bool continuous = frame->sample_time_us > previous_us &&
                                frame->sample_time_us - previous_us <= timeout_us;
        valid = valid && continuous;
        const float dt_s = continuous ? (float)(frame->sample_time_us - previous_us) * 1.0e-6f : 0.0f;
        for (uint32_t i = 0U; i < 3U; ++i)
        {
            float delta_deg = frame->angle_deg[i] - gimbal->imu.angle_deg[i];
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
        Gimbal_InvalidateImu(gimbal);
        return;
    }

    // 有效新快照直接就绪，运动不阻止使能，故障状态仍须先由命令解除
    if (imu != NULL)
    {
        gimbal->imu = *imu;
        switch (gimbal->imu_state)
        {
            case GIMBAL_IMU_WAITING:
                gimbal->imu_state = GIMBAL_IMU_READY;
                break;

            case GIMBAL_IMU_READY:
            case GIMBAL_IMU_FAULT:
                break;

            default:
                Gimbal_InvalidateImu(gimbal);
                break;
        }
    }

    // 当前 INS 已完成安装修正，角度直接取 Pitch 与累计 Yaw，不叠加电机方向
    const bool ready = gimbal->imu_state == GIMBAL_IMU_READY;
    gimbal->yaw.angle_deg = ready ? gimbal->imu.angle_deg[2] : NAN;
    gimbal->pitch.angle_deg = ready ? gimbal->imu.angle_deg[1] : NAN;

    // 机体系角速度按 ZYX 欧拉运动学换算为大地系欧拉角速度，有效性校验已保证 |Pitch| < 89°
    const float roll_rad = gimbal->imu.angle_deg[0] * M_DEG2RAD_MULT;
    const float sin_roll = sinf(roll_rad);
    const float cos_roll = cosf(roll_rad);
    const float gyro_y = gimbal->imu.gyro_rad_s[1];
    const float gyro_z = gimbal->imu.gyro_rad_s[2];
    const float yaw_rate = (gyro_y * sin_roll + gyro_z * cos_roll) / cosf(gimbal->imu.angle_deg[1] * M_DEG2RAD_MULT);
    gimbal->yaw.speed_rad_s = ready ? yaw_rate : NAN;
    gimbal->pitch.speed_rad_s = ready ? gyro_y * cos_roll - gyro_z * sin_roll : NAN;
}

/**
 * @brief 换算单轴反馈，超时进入失联等待，非法帧、时间倒退或展开歧义锁存故障
 *
 * @param[in,out] axis 单轴运行数据
 * @param[in] config 单轴配置
 * @param[in] motor 新反馈帧，本周期没有新帧时为 NULL
 * @param[in] now_us 当前单调时刻，单位微秒
 * @param[in] timeout_us 已校验的反馈最大间隔，单位微秒
 * @param[in] signed_angle 为 true 时使用零点两侧的有符号单圈角，否则展开连续角
 * @return 无返回值
 */
static void Gimbal_UpdateAxisFeedback(Gimbal_Axis_t *axis, const Gimbal_AxisConfig_t *config,
                                      const MOTOR_RM_Feedback_t *motor, uint64_t now_us, uint64_t timeout_us,
                                      bool signed_angle)
{
    // 没有新帧时只检查旧帧时效，超时后清除反馈
    if (motor == NULL)
    {
        if (axis->feedback_time_us == UINT64_MAX || now_us < axis->feedback_time_us ||
            now_us - axis->feedback_time_us > timeout_us)
        {
            Gimbal_RejectAxisFeedback(axis, now_us < axis->feedback_time_us &&
                axis->feedback_time_us != UINT64_MAX ? GIMBAL_AXIS_FAULT : GIMBAL_AXIS_OFFLINE);
        }
        return;
    }

    // 驱动只保存时间戳低 32 位，按与当前时刻的差值恢复完整微秒时刻
    const uint32_t age_us = (uint32_t)now_us - motor->last_update_time;
    const uint64_t frame_time_us = now_us >= age_us ? now_us - age_us : UINT64_MAX;
    const float encoder_rad = motor->rotor_single_angle;
    const float speed_rpm = motor->rotor_speed;
    // 设备 angle_valid 只保证连续角跟踪，不检查原始编码器是否超过 8191，故在此拒绝越界帧
    const bool frame_valid = motor->angle_valid && frame_time_us != UINT64_MAX && age_us <= timeout_us &&
                             encoder_rad >= 0.0f && encoder_rad < M_2PI;
    // 两种角度都拒绝旧帧和超时间隔，pitch 不依赖连续角展开器也保留时序校验
    const bool continuous = axis->feedback_time_us != UINT64_MAX && frame_time_us > axis->feedback_time_us &&
                            frame_time_us - axis->feedback_time_us <= timeout_us;
    const bool has_reference = signed_angle ? isfinite(axis->mechanical_angle_deg) : axis->unwrap.has_reference;
    if (!frame_valid || (has_reference && frame_time_us <= axis->feedback_time_us))
    {
        Gimbal_RejectAxisFeedback(axis, GIMBAL_AXIS_FAULT);
        return;
    }
    if (has_reference && !continuous)
    {
        Gimbal_RejectAxisFeedback(axis, GIMBAL_AXIS_OFFLINE);
        return;
    }

    // 反装轴先翻转角度和转速，使模块内统一以角度增大方向为正
    const float raw_encoder_deg = Gimbal_WrapAngle(encoder_rad * M_RAD2DEG_MULT, GIMBAL_FULL_TURN_DEG);
    const float encoder_deg = config->reversed ?
        Gimbal_WrapAngle(GIMBAL_FULL_TURN_DEG - raw_encoder_deg, GIMBAL_FULL_TURN_DEG) : raw_encoder_deg;
    const float speed_rad_s = speed_rpm * GIMBAL_RPM_TO_RAD_S;
    axis->encoder_deg = encoder_deg;
    axis->motor_speed_rad_s = config->reversed ? -speed_rad_s : speed_rad_s;
    axis->feedback_time_us = frame_time_us;

    // pitch 行程围绕水平零点，直接映射到 [-180, 180)，负角上电和失联恢复均不累计圈数
    if (signed_angle)
    {
        const float half_turn_deg = GIMBAL_FULL_TURN_DEG * 0.5f;
        axis->mechanical_angle_deg = Gimbal_WrapAngle(encoder_deg - axis->applied_zero_encoder_deg + half_turn_deg,
                                                     GIMBAL_FULL_TURN_DEG) - half_turn_deg;
        return;
    }

    // yaw 保留原有连续角展开，跨编码器零点时按相邻帧最短路径累计
    const float relative_deg = Gimbal_WrapAngle(encoder_deg - axis->applied_zero_encoder_deg, GIMBAL_FULL_TURN_DEG);
    float angle_deg = NAN;
    if (!Gimbal_UpdateAngleUnwrap(&axis->unwrap, relative_deg, GIMBAL_FULL_TURN_DEG, &angle_deg))
    {
        Gimbal_RejectAxisFeedback(axis, GIMBAL_AXIS_FAULT);
        return;
    }
    axis->mechanical_angle_deg = angle_deg;
}

/**
 * @brief 清空两环历史及输出，进入指定停止状态
 *
 * @param[in,out] axis 单轴运行数据
 * @param[in] next_state RELAX、OFFLINE、TRIPPED 或 FAULT，其他值按 FAULT 处理
 * @return 无返回值
 */
static void Gimbal_StopAxis(Gimbal_Axis_t *axis, Gimbal_AxisState_t next_state)
{
    PID_Reset(&axis->angle_pid);
    PID_Reset(&axis->speed_pid);
    axis->target_angle_deg = NAN;
    axis->target_speed_rad_s = 0.0f;
    axis->output_current_a = 0.0f;
    axis->state = next_state == GIMBAL_AXIS_RELAX || next_state == GIMBAL_AXIS_OFFLINE ||
        next_state == GIMBAL_AXIS_TRIPPED ? next_state : GIMBAL_AXIS_FAULT;
}
