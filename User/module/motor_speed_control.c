#include "module/motor_speed_control.h"

#include <math.h>
#include <stddef.h>

static float MotorSpeedControl_Clamp(float value, float absolute_limit);
static float MotorSpeedControl_ApplyRamp(float current, float target, float maximum_step);
static bool MotorSpeedControl_IsConfigurationValid(float sample_frequency_hz);
static bool MotorSpeedControl_ApplyPidTune(MotorSpeedControl_t *control, const MotorSpeedPidTune_t *pid_tune);
static void MotorSpeedControl_Reset(MotorSpeedControl_t *control);
static void MotorSpeedControl_PrimeFeedback(MotorSpeedControl_t *control);

/**
 * @brief 初始化单电机速度控制模块
 *
 * @param[in,out] control 速度控制上下文
 * @param[in] sample_frequency_hz 控制采样频率，单位 Hz，必须大于 0
 * @return 成功返回 MOTOR_SPEED_CONTROL_OK，失败返回对应状态码
 */
int8_t MotorSpeedControl_Init(MotorSpeedControl_t *control, float sample_frequency_hz)
{
    if (control == NULL)
    {
        return MOTOR_SPEED_CONTROL_NULL_ERROR;
    }

    // 清除调用者提供的上下文，避免沿用未定义的控制状态
    *control = (MotorSpeedControl_t)
    {
        0
    };

    const MotorSpeedPidTune_t default_pid_tune =
    {
        .kp = MOTOR_SPEED_PID_KP,
        .ki = MOTOR_SPEED_PID_KI,
        .kd = MOTOR_SPEED_PID_KD,
    };
    control->pid_param = (KPID_Params_t)
    {
        .k = 1.0F,
        .p = MOTOR_SPEED_PID_KP,
        .i = MOTOR_SPEED_PID_KI,
        .d = MOTOR_SPEED_PID_KD,
        .i_limit = MOTOR_SPEED_PID_INTEGRAL_LIMIT,
        .out_limit = MOTOR_SPEED_CURRENT_LIMIT_A,
        .d_cutoff_freq = MOTOR_SPEED_PID_D_CUTOFF_HZ,
        .range = 0.0F,
    };

    // 在创建 PID 和滤波器状态前拒绝非法固定参数
    if (!MotorSpeedControl_IsConfigurationValid(sample_frequency_hz) ||
        !MotorSpeedControl_ApplyPidTune(control, &default_pid_tune))
    {
        control->feedback.status = MOTOR_SPEED_CONTROL_CONFIG_ERROR;
        return control->feedback.status;
    }

    // 初始化 PID 与两级滤波器，使控制器从零输出安全启动
    if (PID_Init(&control->pid, KPID_MODE_CALC_D, sample_frequency_hz, &control->pid_param) != 0)
    {
        control->feedback.status = MOTOR_SPEED_CONTROL_INIT_ERROR;
        return control->feedback.status;
    }
    LowPassFilter2p_Init(&control->feedback_filter, sample_frequency_hz, MOTOR_SPEED_FEEDBACK_LPF_CUTOFF_HZ);
    LowPassFilter2p_Init(&control->current_filter, sample_frequency_hz, MOTOR_SPEED_CURRENT_LPF_CUTOFF_HZ);

    control->feedback.initialized = true;
    control->feedback.status = MOTOR_SPEED_CONTROL_OK;
    return control->feedback.status;
}

/**
 * @brief 更新单电机实际转速反馈
 *
 * @param[in,out] control 速度控制上下文
 * @param[in] actual_speed_rpm 输出轴实际转速，单位 rpm
 * @return 成功返回 MOTOR_SPEED_CONTROL_OK，失败返回对应状态码
 */
int8_t MotorSpeedControl_UpdateFeedback(MotorSpeedControl_t *control, float actual_speed_rpm)
{
    if (control == NULL)
    {
        return MOTOR_SPEED_CONTROL_NULL_ERROR;
    }

    // 非有限反馈不得覆盖上一份合法转速
    if (!isfinite(actual_speed_rpm))
    {
        return MOTOR_SPEED_CONTROL_INVALID_VALUE;
    }

    control->feedback.actual_speed_rpm = actual_speed_rpm;
    return MOTOR_SPEED_CONTROL_OK;
}

/**
 * @brief 执行一次单电机速度控制计算
 *
 * @param[in,out] control 速度控制上下文
 * @param[in] requested_speed_rpm 请求的输出轴目标转速，单位 rpm
 * @param[in] pid_tune 本周期速度 PID 参数
 * @param[in] enabled 是否允许产生非零电流指令
 * @param[in] control_period_s 本周期控制间隔，单位 s，必须大于 0
 * @return 本周期速度控制状态，取值见 MotorSpeedControlStatus_t
 */
int8_t MotorSpeedControl_Control(MotorSpeedControl_t *control, float requested_speed_rpm,
                                 const MotorSpeedPidTune_t *pid_tune, bool enabled, float control_period_s)
{
    if (control == NULL || pid_tune == NULL)
    {
        return MOTOR_SPEED_CONTROL_NULL_ERROR;
    }

    // 复制本周期输入并优先建立安全的零电流输出
    MotorSpeedControlFeedback_t *feedback = &control->feedback;
    feedback->enabled = enabled;
    feedback->requested_speed_rpm = requested_speed_rpm;
    feedback->current_command_a = 0.0F;

    // 校验并应用在线 PID 参数，非法配置必须清除控制状态
    const bool pid_tune_valid = MotorSpeedControl_ApplyPidTune(control, pid_tune);
    feedback->pid_kp = control->pid_param.p;
    feedback->pid_ki = control->pid_param.i;
    feedback->pid_kd = control->pid_param.d;
    feedback->limited_target_speed_rpm =
        isfinite(requested_speed_rpm) ? MotorSpeedControl_Clamp(requested_speed_rpm, MOTOR_SPEED_LIMIT_RPM) : 0.0F;

    if (!feedback->initialized)
    {
        feedback->status = MOTOR_SPEED_CONTROL_INIT_ERROR;
        MotorSpeedControl_Reset(control);
    }
    else if (!pid_tune_valid)
    {
        feedback->status = MOTOR_SPEED_CONTROL_CONFIG_ERROR;
        MotorSpeedControl_Reset(control);
    }
    else if (!isfinite(requested_speed_rpm) || !isfinite(control_period_s) || control_period_s <= 0.0F)
    {
        feedback->status = MOTOR_SPEED_CONTROL_INVALID_VALUE;
        MotorSpeedControl_Reset(control);
    }
    else if (!enabled)
    {
        feedback->status = MOTOR_SPEED_CONTROL_DISABLED;
        MotorSpeedControl_Reset(control);
    }
    else
    {
        // 使能恢复时预置反馈状态，避免反馈微分产生突跳
        if (!control->was_enabled)
        {
            MotorSpeedControl_PrimeFeedback(control);
        }

        // 依次执行目标斜坡、反馈滤波、PID 计算和电流滤波
        control->ramped_target_speed_rpm =
            MotorSpeedControl_ApplyRamp(control->ramped_target_speed_rpm, feedback->limited_target_speed_rpm,
                                        MOTOR_SPEED_RAMP_RATE_RPM_S * control_period_s);
        feedback->filtered_speed_rpm = LowPassFilter2p_Apply(&control->feedback_filter, feedback->actual_speed_rpm);
        const float pid_current_command_a =
            PID_Calc(&control->pid, control->ramped_target_speed_rpm, feedback->filtered_speed_rpm, 0.0F,
                     control_period_s);
        feedback->current_command_a = LowPassFilter2p_Apply(&control->current_filter, pid_current_command_a);

        // 拒绝异常计算结果，仅发布有限且已限幅的电流指令
        if (!isfinite(feedback->current_command_a))
        {
            feedback->status = MOTOR_SPEED_CONTROL_INVALID_VALUE;
            MotorSpeedControl_Reset(control);
        }
        else
        {
            feedback->current_command_a = MotorSpeedControl_Clamp(feedback->current_command_a,
                                                                  MOTOR_SPEED_CURRENT_LIMIT_A);
            feedback->status = MOTOR_SPEED_CONTROL_OK;
        }
    }

    // 汇总本周期目标与误差，供所属任务发布诊断快照
    feedback->target_speed_rpm = control->ramped_target_speed_rpm;
    feedback->speed_error_rpm =
        feedback->status == MOTOR_SPEED_CONTROL_OK ? feedback->target_speed_rpm - feedback->filtered_speed_rpm : 0.0F;
    return feedback->status;
}

/**
 * @brief 导出单电机转子侧电流指令
 *
 * @param[in] control 速度控制上下文
 * @param[out] current_command_a 转子侧电流指令，单位 A
 * @return 成功返回 MOTOR_SPEED_CONTROL_OK，失败返回对应状态码
 */
int8_t MotorSpeedControl_DumpOutput(const MotorSpeedControl_t *control, float *current_command_a)
{
    if (control == NULL || current_command_a == NULL)
    {
        return MOTOR_SPEED_CONTROL_NULL_ERROR;
    }

    *current_command_a = control->feedback.current_command_a;
    return MOTOR_SPEED_CONTROL_OK;
}

/**
 * @brief 对浮点值执行正负对称限幅
 *
 * @param[in] value 待限幅数值
 * @param[in] absolute_limit 正数形式的绝对值上限
 * @return 限幅后的数值
 */
static float MotorSpeedControl_Clamp(float value, float absolute_limit)
{
    if (value > absolute_limit)
    {
        return absolute_limit;
    }
    if (value < -absolute_limit)
    {
        return -absolute_limit;
    }
    return value;
}

/**
 * @brief 按单周期最大变化量逼近目标值
 *
 * @param[in] current 当前值
 * @param[in] target 目标值
 * @param[in] maximum_step 单周期允许的最大变化量
 * @return 本周期更新后的数值
 */
static float MotorSpeedControl_ApplyRamp(float current, float target, float maximum_step)
{
    const float difference = target - current;
    if (difference > maximum_step)
    {
        return current + maximum_step;
    }
    if (difference < -maximum_step)
    {
        return current - maximum_step;
    }
    return target;
}

/**
 * @brief 检查固定速度控制参数是否合法
 *
 * @param[in] sample_frequency_hz 控制采样频率，单位 Hz
 * @return 所有参数合法时返回 true，否则返回 false
 */
static bool MotorSpeedControl_IsConfigurationValid(float sample_frequency_hz)
{
    return isfinite(sample_frequency_hz) && sample_frequency_hz > 0.0F && isfinite(MOTOR_SPEED_LIMIT_RPM) &&
           MOTOR_SPEED_LIMIT_RPM > 0.0F && isfinite(MOTOR_SPEED_RAMP_RATE_RPM_S) &&
           MOTOR_SPEED_RAMP_RATE_RPM_S > 0.0F && isfinite(MOTOR_SPEED_PID_D_CUTOFF_HZ) &&
           MOTOR_SPEED_PID_D_CUTOFF_HZ < sample_frequency_hz * 0.5F &&
           isfinite(MOTOR_SPEED_FEEDBACK_LPF_CUTOFF_HZ) &&
           MOTOR_SPEED_FEEDBACK_LPF_CUTOFF_HZ < sample_frequency_hz * 0.5F &&
           isfinite(MOTOR_SPEED_CURRENT_LPF_CUTOFF_HZ) &&
           MOTOR_SPEED_CURRENT_LPF_CUTOFF_HZ < sample_frequency_hz * 0.5F &&
           isfinite(MOTOR_SPEED_PID_INTEGRAL_LIMIT) && MOTOR_SPEED_PID_INTEGRAL_LIMIT >= 0.0F &&
           isfinite(MOTOR_SPEED_CURRENT_LIMIT_A) && MOTOR_SPEED_CURRENT_LIMIT_A > 0.0F;
}

/**
 * @brief 校验并应用本周期速度 PID 参数
 *
 * @param[in,out] control 速度控制上下文
 * @param[in] pid_tune 待应用的速度 PID 参数
 * @return 参数合法并成功应用时返回 true，否则返回 false
 */
static bool MotorSpeedControl_ApplyPidTune(MotorSpeedControl_t *control, const MotorSpeedPidTune_t *pid_tune)
{
    if (control == NULL || pid_tune == NULL)
    {
        return false;
    }

    const float kp = pid_tune->kp;
    const float ki = pid_tune->ki;
    const float kd = pid_tune->kd;
    if (!isfinite(kp) || kp < 0.0F || !isfinite(ki) || ki < 0.0F || !isfinite(kd) || kd < 0.0F)
    {
        return false;
    }

    control->pid_param.p = kp;
    control->pid_param.i = ki;
    control->pid_param.d = kd;
    return true;
}

/**
 * @brief 清除速度 PID、滤波器和目标斜坡状态
 *
 * @param[in,out] control 速度控制上下文
 * @return 无返回值
 */
static void MotorSpeedControl_Reset(MotorSpeedControl_t *control)
{
    control->ramped_target_speed_rpm = 0.0F;
    control->was_enabled = false;
    control->feedback.filtered_speed_rpm = 0.0F;
    control->feedback.current_command_a = 0.0F;

    if (control->feedback.initialized)
    {
        PID_Reset(&control->pid);
        LowPassFilter2p_Reset(&control->feedback_filter, 0.0F);
        LowPassFilter2p_Reset(&control->current_filter, 0.0F);
    }
}

/**
 * @brief 使用当前实际转速预置反馈微分状态
 *
 * @param[in,out] control 速度控制上下文
 * @return 无返回值
 */
static void MotorSpeedControl_PrimeFeedback(MotorSpeedControl_t *control)
{
    // 清除旧 PID 状态并用当前实际转速预置反馈滤波器
    PID_Reset(&control->pid);
    control->feedback.filtered_speed_rpm =
        LowPassFilter2p_Reset(&control->feedback_filter, control->feedback.actual_speed_rpm);
    LowPassFilter2p_Reset(&control->current_filter, 0.0F);

    // 同步微分滤波器和上一反馈，避免重新使能时产生电流冲击
    const float scaled_feedback = control->pid_param.k * control->feedback.filtered_speed_rpm;
    LowPassFilter2p_Reset(&control->pid.dfilter, scaled_feedback);
    control->pid.last.k_fb = scaled_feedback;
    control->was_enabled = true;
}
