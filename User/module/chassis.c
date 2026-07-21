#include "module/chassis.h"

#include "component/pid.h"

#include <math.h>
#include <stddef.h>

/*
 * 底盘单路电机速度控制反馈：
 * - initialized：速度 PID 初始化成功时为 true。
 * - enabled：本周期允许输出非零电流时为 true。
 * - status：本周期运行状态，取值见 Chassis_MotorStatus_t。
 * - requested_speed_rpm：调用者请求的输出轴目标转速，单位 rpm。
 * - limited_target_speed_rpm：限幅后的输出轴目标转速，单位 rpm。
 * - target_speed_rpm：限幅并经过斜坡后的输出轴目标转速，单位 rpm。
 * - actual_speed_rpm：调用者提供的输出轴实际转速，单位 rpm。
 * - filtered_speed_rpm：送入 PID 的滤波后输出轴转速，单位 rpm。
 * - speed_error_rpm：目标转速与滤波后转速之差，单位 rpm。
 * - current_command_a：滤波并限幅后的转子侧电流指令，单位 A。
 * - pid_kp、pid_ki、pid_kd：本周期实际采用的 PID 参数。
 */
typedef struct
{
    bool initialized;
    bool enabled;
    Chassis_MotorStatus_t status;
    float requested_speed_rpm;
    float limited_target_speed_rpm;
    float target_speed_rpm;
    float actual_speed_rpm;
    float filtered_speed_rpm;
    float speed_error_rpm;
    float current_command_a;
    float pid_kp;
    float pid_ki;
    float pid_kd;
} Chassis_MotorFeedback_t;

/*
 * 底盘单路电机速度控制上下文：
 * - pid_param：PID 配置参数。
 * - pid：PID 运行状态。
 * - feedback_filter：速度反馈二阶低通滤波器。
 * - current_filter：电流指令二阶低通滤波器。
 * - feedback：目标、反馈、误差、电流指令和运行状态。
 * - ramped_target_speed_rpm：斜坡内部目标转速，单位 rpm。
 * - was_enabled：上一周期速度控制是否使能。
 */
typedef struct
{
    KPID_Params_t pid_param;
    KPID_t pid;
    LowPassFilter2p_t feedback_filter;
    LowPassFilter2p_t current_filter;
    Chassis_MotorFeedback_t feedback;
    float ramped_target_speed_rpm;
    bool was_enabled;
} Chassis_MotorControl_t;

/*
 * 底盘模块私有运行状态：
 * - initialized：四路速度控制器均可运行时为 true。
 * - control_init_status：四路速度控制器的聚合初始化结果。
 * - motor_init_status[0~3]：四个速度控制器的初始化结果。
 * - motor_control[0~3]：四个相互独立的单电机速度控制上下文。
 */
typedef struct
{
    bool initialized;
    int8_t control_init_status;
    Chassis_MotorStatus_t motor_init_status[CHASSIS_MOTOR_COUNT];
    Chassis_MotorControl_t motor_control[CHASSIS_MOTOR_COUNT];
} Chassis_State_t;

static Chassis_State_t chassis_state;

static void Chassis_ResetState(void);
static void Chassis_ResetSnapshot(Chassis_Snapshot_t *snapshot);
static int8_t Chassis_InitControllers(float sample_frequency_hz);
static int8_t Chassis_Calculate(const Chassis_Input_t *input, const Chassis_Feedback_t *feedback,
                                Chassis_Output_t *output, Chassis_Snapshot_t *snapshot);
static Chassis_MotorStatus_t Chassis_MotorInit(Chassis_MotorControl_t *control, float sample_frequency_hz);
static Chassis_MotorStatus_t Chassis_MotorUpdateFeedback(Chassis_MotorControl_t *control, float actual_speed_rpm);
static Chassis_MotorStatus_t Chassis_MotorControl(Chassis_MotorControl_t *control, float requested_speed_rpm,
                                                  const Chassis_PidTune_t *pid_tune, bool enabled,
                                                  float control_period_s);
static Chassis_MotorStatus_t Chassis_MotorDumpOutput(const Chassis_MotorControl_t *control,
                                                     float *current_command_a);
static float Chassis_Clamp(float value, float absolute_limit);
static float Chassis_ApplyRamp(float current, float target, float maximum_step);
static bool Chassis_IsConfigurationValid(float sample_frequency_hz);
static bool Chassis_ApplyPidTune(Chassis_MotorControl_t *control, const Chassis_PidTune_t *pid_tune);
static void Chassis_ResetMotorControl(Chassis_MotorControl_t *control);
static void Chassis_PrimeFeedback(Chassis_MotorControl_t *control);

/**
 * @brief 初始化四个相互独立的 M3508 速度控制器
 *
 * @param[in] sample_frequency_hz 控制采样频率，单位 Hz，必须大于 0
 * @param[out] snapshot 底盘初始化结果快照
 * @return 全部速度控制器初始化成功返回 CHASSIS_OK，否则返回对应状态码
 */
int8_t Chassis_Init(float sample_frequency_hz, Chassis_Snapshot_t *snapshot)
{
    if (snapshot == NULL)
    {
        return CHASSIS_NULL_ERROR;
    }

    // 先清除全部跨周期状态，保证任一初始化失败路径都不能进入周期控制
    Chassis_ResetState();
    Chassis_ResetSnapshot(snapshot);
    if (!isfinite(sample_frequency_hz) || sample_frequency_hz <= 0.0F)
    {
        chassis_state.control_init_status = CHASSIS_CONFIG_ERROR;
        for (uint32_t motor_index = 0U; motor_index < CHASSIS_MOTOR_COUNT; motor_index++)
        {
            chassis_state.motor_init_status[motor_index] = CHASSIS_MOTOR_CONFIG_ERROR;
        }
        Chassis_ResetSnapshot(snapshot);
        return CHASSIS_CONFIG_ERROR;
    }

    // 配置有效后初始化四路速度控制器，任一路初始化失败都禁止模块进入运行态
    const int8_t control_status = Chassis_InitControllers(sample_frequency_hz);
    chassis_state.initialized = control_status == CHASSIS_OK;
    Chassis_ResetSnapshot(snapshot);
    if (!chassis_state.initialized)
    {
        return CHASSIS_CONTROL_INIT_ERROR;
    }
    return CHASSIS_OK;
}

/**
 * @brief 根据反馈执行一次四路速度控制计算
 *
 * @param[in] input 本周期控制输入快照
 * @param[in] feedback 本周期四路电机反馈快照
 * @param[out] output 本周期四路电流计算结果
 * @param[out] snapshot 本周期速度控制结果快照
 * @return 四路速度控制均正常或安全禁用时返回 CHASSIS_OK，存在计算异常时返回 CHASSIS_ERROR
 */
int8_t Chassis_Run(const Chassis_Input_t *input, const Chassis_Feedback_t *feedback, Chassis_Output_t *output,
                   Chassis_Snapshot_t *snapshot)
{
    // 先清除调用方提供的非空结果对象，任一非法调用都不得沿用上一周期数据
    if (output != NULL)
    {
        *output = (Chassis_Output_t)
        {
            0,
        };
    }
    if (snapshot != NULL)
    {
        Chassis_ResetSnapshot(snapshot);
    }
    if (output == NULL || snapshot == NULL)
    {
        return CHASSIS_NULL_ERROR;
    }

    if (input == NULL || feedback == NULL)
    {
        return CHASSIS_NULL_ERROR;
    }
    if (!chassis_state.initialized)
    {
        return CHASSIS_NOT_INITIALIZED;
    }

    // 使用调用方提供的一致反馈完成速度控制，设备读写由上层任务负责
    snapshot->chassis_status = Chassis_Calculate(input, feedback, output, snapshot);
    return snapshot->chassis_status;
}

/**
 * @brief 恢复底盘模块私有状态的安全默认值
 *
 * @return 无返回值
 */
static void Chassis_ResetState(void)
{
    chassis_state = (Chassis_State_t)
    {
        0,
    };
    chassis_state.control_init_status = CHASSIS_NOT_INITIALIZED;
    for (uint32_t motor_index = 0U; motor_index < CHASSIS_MOTOR_COUNT; motor_index++)
    {
        chassis_state.motor_init_status[motor_index] = CHASSIS_MOTOR_INIT_ERROR;
    }
}

/**
 * @brief 使用模块私有状态建立安全一致的公开快照
 *
 * @param[out] snapshot 待复位的底盘结果快照
 * @return 无返回值
 */
static void Chassis_ResetSnapshot(Chassis_Snapshot_t *snapshot)
{
    *snapshot = (Chassis_Snapshot_t)
    {
        0,
    };
    snapshot->initialized = chassis_state.initialized;
    snapshot->control_init_status = chassis_state.control_init_status;
    snapshot->chassis_status = CHASSIS_NOT_INITIALIZED;
    for (uint32_t motor_index = 0U; motor_index < CHASSIS_MOTOR_COUNT; motor_index++)
    {
        snapshot->motor_init_status[motor_index] = chassis_state.motor_init_status[motor_index];
        snapshot->control_status[motor_index] = CHASSIS_MOTOR_INIT_ERROR;
    }
}

/**
 * @brief 初始化四个相互独立的单电机速度控制器
 *
 * @param[in] sample_frequency_hz 控制采样频率，单位 Hz，必须大于 0
 * @return 全部速度控制器初始化成功返回 CHASSIS_OK，否则返回 CHASSIS_CONTROL_INIT_ERROR
 */
static int8_t Chassis_InitControllers(float sample_frequency_hz)
{
    bool all_initialized = true;
    for (uint32_t motor_index = 0U; motor_index < CHASSIS_MOTOR_COUNT; motor_index++)
    {
        chassis_state.motor_init_status[motor_index] =
            Chassis_MotorInit(&chassis_state.motor_control[motor_index], sample_frequency_hz);
        if (chassis_state.motor_init_status[motor_index] != CHASSIS_MOTOR_OK)
        {
            all_initialized = false;
        }
    }

    chassis_state.control_init_status = all_initialized ? CHASSIS_OK : CHASSIS_CONTROL_INIT_ERROR;
    return chassis_state.control_init_status;
}

/**
 * @brief 根据本周期反馈独立计算四路安全电流指令
 *
 * @param[in] input 本周期控制输入快照
 * @param[in] feedback 本周期四路电机反馈快照
 * @param[out] output 本周期四路电流计算结果
 * @param[in,out] snapshot 本周期底盘结果快照
 * @return 全部活动控制器正常返回 CHASSIS_OK，存在控制异常返回 CHASSIS_ERROR
 */
static int8_t Chassis_Calculate(const Chassis_Input_t *input, const Chassis_Feedback_t *feedback,
                                Chassis_Output_t *output, Chassis_Snapshot_t *snapshot)
{
    bool all_control_valid = true;
    for (uint32_t motor_index = 0U; motor_index < CHASSIS_MOTOR_COUNT; motor_index++)
    {
        Chassis_MotorControl_t *motor_control = &chassis_state.motor_control[motor_index];
        const bool feedback_available = feedback->valid[motor_index] && feedback->motor_online[motor_index];
        const float actual_speed_rpm = feedback_available ? feedback->actual_speed_rpm[motor_index] : 0.0F;
        const Chassis_MotorStatus_t feedback_status = Chassis_MotorUpdateFeedback(motor_control, actual_speed_rpm);
        const bool motor_enabled = input->enabled && feedback_available && feedback_status == CHASSIS_MOTOR_OK;
        const Chassis_MotorStatus_t control_status =
            Chassis_MotorControl(motor_control, input->requested_speed_rpm[motor_index], &input->pid_tune,
                                 motor_enabled, input->control_period_s);
        float current_command_a = 0.0F;
        const Chassis_MotorStatus_t output_status = Chassis_MotorDumpOutput(motor_control, &current_command_a);

        // 汇总本路状态，异常反馈或计算结果只清零本路电流
        Chassis_MotorStatus_t published_status = control_status;
        if (feedback_status != CHASSIS_MOTOR_OK)
        {
            published_status = feedback_status;
            all_control_valid = false;
        }
        else if (output_status != CHASSIS_MOTOR_OK)
        {
            published_status = output_status;
            all_control_valid = false;
        }
        else if (control_status != CHASSIS_MOTOR_OK && control_status != CHASSIS_MOTOR_DISABLED)
        {
            all_control_valid = false;
        }
        if (published_status != CHASSIS_MOTOR_OK)
        {
            current_command_a = 0.0F;
        }

        // 保存统一结果快照，调用方不再读取控制器内部状态
        snapshot->control_status[motor_index] = published_status;
        snapshot->motor_enabled[motor_index] = motor_control->feedback.enabled;
        snapshot->limited_target_speed_rpm[motor_index] = motor_control->feedback.limited_target_speed_rpm;
        snapshot->ramped_target_speed_rpm[motor_index] = motor_control->feedback.target_speed_rpm;
        snapshot->actual_speed_rpm[motor_index] = motor_control->feedback.actual_speed_rpm;
        snapshot->current_command_a[motor_index] = current_command_a;
        output->current_command_a[motor_index] = current_command_a;
    }
    return all_control_valid ? CHASSIS_OK : CHASSIS_ERROR;
}

/**
 * @brief 初始化底盘单路电机速度控制器
 *
 * @param[in,out] control 单路电机速度控制上下文
 * @param[in] sample_frequency_hz 控制采样频率，单位 Hz，必须大于 0
 * @return 成功返回 CHASSIS_MOTOR_OK，失败返回对应状态码
 */
static Chassis_MotorStatus_t Chassis_MotorInit(Chassis_MotorControl_t *control, float sample_frequency_hz)
{
    if (control == NULL)
    {
        return CHASSIS_MOTOR_NULL_ERROR;
    }

    // 清除单路上下文，保证初始化失败路径保持零输出
    *control = (Chassis_MotorControl_t)
    {
        0,
    };
    const Chassis_PidTune_t default_pid_tune =
    {
        .kp = CHASSIS_PID_KP,
        .ki = CHASSIS_PID_KI,
        .kd = CHASSIS_PID_KD,
    };
    control->pid_param = (KPID_Params_t)
    {
        .k = 1.0F,
        .p = CHASSIS_PID_KP,
        .i = CHASSIS_PID_KI,
        .d = CHASSIS_PID_KD,
        .i_limit = CHASSIS_PID_INTEGRAL_LIMIT,
        .out_limit = CHASSIS_CURRENT_LIMIT_A,
        .d_cutoff_freq = CHASSIS_PID_D_CUTOFF_HZ,
        .range = 0.0F,
    };

    // 在创建 PID 和滤波器状态前拒绝非法固定参数
    if (!Chassis_IsConfigurationValid(sample_frequency_hz) || !Chassis_ApplyPidTune(control, &default_pid_tune))
    {
        control->feedback.status = CHASSIS_MOTOR_CONFIG_ERROR;
        return control->feedback.status;
    }

    // 初始化 PID 与两级滤波器，使控制器从零输出安全启动
    if (PID_Init(&control->pid, KPID_MODE_CALC_D, sample_frequency_hz, &control->pid_param) != 0)
    {
        control->feedback.status = CHASSIS_MOTOR_INIT_ERROR;
        return control->feedback.status;
    }
    LowPassFilter2p_Init(&control->feedback_filter, sample_frequency_hz, CHASSIS_FEEDBACK_LPF_CUTOFF_HZ);
    LowPassFilter2p_Init(&control->current_filter, sample_frequency_hz, CHASSIS_CURRENT_LPF_CUTOFF_HZ);

    control->feedback.initialized = true;
    control->feedback.status = CHASSIS_MOTOR_OK;
    return control->feedback.status;
}

/**
 * @brief 更新底盘单路电机实际转速反馈
 *
 * @param[in,out] control 单路电机速度控制上下文
 * @param[in] actual_speed_rpm 输出轴实际转速，单位 rpm
 * @return 成功返回 CHASSIS_MOTOR_OK，失败返回对应状态码
 */
static Chassis_MotorStatus_t Chassis_MotorUpdateFeedback(Chassis_MotorControl_t *control, float actual_speed_rpm)
{
    if (control == NULL)
    {
        return CHASSIS_MOTOR_NULL_ERROR;
    }

    // 非有限反馈不得覆盖上一份合法转速
    if (!isfinite(actual_speed_rpm))
    {
        return CHASSIS_MOTOR_INVALID_VALUE;
    }

    control->feedback.actual_speed_rpm = actual_speed_rpm;
    return CHASSIS_MOTOR_OK;
}

/**
 * @brief 执行一次底盘单路电机速度控制计算
 *
 * @param[in,out] control 单路电机速度控制上下文
 * @param[in] requested_speed_rpm 请求的输出轴目标转速，单位 rpm
 * @param[in] pid_tune 本周期速度 PID 参数
 * @param[in] enabled 是否允许产生非零电流指令
 * @param[in] control_period_s 本周期控制间隔，单位 s，必须大于 0
 * @return 本周期速度控制状态，取值见 Chassis_MotorStatus_t
 */
static Chassis_MotorStatus_t Chassis_MotorControl(Chassis_MotorControl_t *control, float requested_speed_rpm,
                                                  const Chassis_PidTune_t *pid_tune, bool enabled,
                                                  float control_period_s)
{
    if (control == NULL || pid_tune == NULL)
    {
        return CHASSIS_MOTOR_NULL_ERROR;
    }

    // 复制本周期输入并优先建立安全的零电流输出
    Chassis_MotorFeedback_t *feedback = &control->feedback;
    feedback->enabled = enabled;
    feedback->requested_speed_rpm = requested_speed_rpm;
    feedback->current_command_a = 0.0F;

    // 校验并应用在线 PID 参数，非法配置必须清除控制状态
    const bool pid_tune_valid = Chassis_ApplyPidTune(control, pid_tune);
    feedback->pid_kp = control->pid_param.p;
    feedback->pid_ki = control->pid_param.i;
    feedback->pid_kd = control->pid_param.d;
    feedback->limited_target_speed_rpm =
        isfinite(requested_speed_rpm) ? Chassis_Clamp(requested_speed_rpm, CHASSIS_SPEED_LIMIT_RPM) : 0.0F;

    if (!feedback->initialized)
    {
        feedback->status = CHASSIS_MOTOR_INIT_ERROR;
        Chassis_ResetMotorControl(control);
    }
    else if (!pid_tune_valid)
    {
        feedback->status = CHASSIS_MOTOR_CONFIG_ERROR;
        Chassis_ResetMotorControl(control);
    }
    else if (!isfinite(requested_speed_rpm) || !isfinite(control_period_s) || control_period_s <= 0.0F)
    {
        feedback->status = CHASSIS_MOTOR_INVALID_VALUE;
        Chassis_ResetMotorControl(control);
    }
    else if (!enabled)
    {
        feedback->status = CHASSIS_MOTOR_DISABLED;
        Chassis_ResetMotorControl(control);
    }
    else
    {
        // 使能恢复时预置反馈状态，避免反馈微分产生突跳
        if (!control->was_enabled)
        {
            Chassis_PrimeFeedback(control);
        }

        // 依次执行目标斜坡、反馈滤波、PID 计算和电流滤波
        control->ramped_target_speed_rpm =
            Chassis_ApplyRamp(control->ramped_target_speed_rpm, feedback->limited_target_speed_rpm,
                              CHASSIS_RAMP_RATE_RPM_S * control_period_s);
        feedback->filtered_speed_rpm = LowPassFilter2p_Apply(&control->feedback_filter, feedback->actual_speed_rpm);
        const float pid_current_command_a =
            PID_Calc(&control->pid, control->ramped_target_speed_rpm, feedback->filtered_speed_rpm, 0.0F,
                     control_period_s);
        feedback->current_command_a = LowPassFilter2p_Apply(&control->current_filter, pid_current_command_a);

        // 拒绝异常计算结果，仅发布有限且已限幅的电流指令
        if (!isfinite(feedback->current_command_a))
        {
            feedback->status = CHASSIS_MOTOR_INVALID_VALUE;
            Chassis_ResetMotorControl(control);
        }
        else
        {
            feedback->current_command_a = Chassis_Clamp(feedback->current_command_a, CHASSIS_CURRENT_LIMIT_A);
            feedback->status = CHASSIS_MOTOR_OK;
        }
    }

    // 汇总本周期目标与误差，供任务发布诊断快照
    feedback->target_speed_rpm = control->ramped_target_speed_rpm;
    feedback->speed_error_rpm =
        feedback->status == CHASSIS_MOTOR_OK ? feedback->target_speed_rpm - feedback->filtered_speed_rpm : 0.0F;
    return feedback->status;
}

/**
 * @brief 导出底盘单路电机转子侧电流指令
 *
 * @param[in] control 单路电机速度控制上下文
 * @param[out] current_command_a 转子侧电流指令，单位 A
 * @return 成功返回 CHASSIS_MOTOR_OK，失败返回对应状态码
 */
static Chassis_MotorStatus_t Chassis_MotorDumpOutput(const Chassis_MotorControl_t *control,
                                                     float *current_command_a)
{
    if (control == NULL || current_command_a == NULL)
    {
        return CHASSIS_MOTOR_NULL_ERROR;
    }

    *current_command_a = control->feedback.current_command_a;
    return CHASSIS_MOTOR_OK;
}

/**
 * @brief 对浮点值执行正负对称限幅
 *
 * @param[in] value 待限幅数值
 * @param[in] absolute_limit 正数形式的绝对值上限
 * @return 限幅后的数值
 */
static float Chassis_Clamp(float value, float absolute_limit)
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
static float Chassis_ApplyRamp(float current, float target, float maximum_step)
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
 * @brief 校验底盘速度控制固定参数
 *
 * @param[in] sample_frequency_hz 控制采样频率，单位 Hz
 * @return 所有参数合法时返回 true，否则返回 false
 */
static bool Chassis_IsConfigurationValid(float sample_frequency_hz)
{
    return isfinite(sample_frequency_hz) && sample_frequency_hz > 0.0F && isfinite(CHASSIS_SPEED_LIMIT_RPM) &&
           CHASSIS_SPEED_LIMIT_RPM > 0.0F && isfinite(CHASSIS_RAMP_RATE_RPM_S) &&
           CHASSIS_RAMP_RATE_RPM_S > 0.0F && isfinite(CHASSIS_PID_D_CUTOFF_HZ) &&
           CHASSIS_PID_D_CUTOFF_HZ < sample_frequency_hz * 0.5F &&
           isfinite(CHASSIS_FEEDBACK_LPF_CUTOFF_HZ) &&
           CHASSIS_FEEDBACK_LPF_CUTOFF_HZ < sample_frequency_hz * 0.5F &&
           isfinite(CHASSIS_CURRENT_LPF_CUTOFF_HZ) &&
           CHASSIS_CURRENT_LPF_CUTOFF_HZ < sample_frequency_hz * 0.5F &&
           isfinite(CHASSIS_PID_INTEGRAL_LIMIT) && CHASSIS_PID_INTEGRAL_LIMIT >= 0.0F &&
           isfinite(CHASSIS_CURRENT_LIMIT_A) && CHASSIS_CURRENT_LIMIT_A > 0.0F;
}

/**
 * @brief 校验并应用底盘速度 PID 在线参数
 *
 * @param[in,out] control 单路电机速度控制上下文
 * @param[in] pid_tune 待应用的速度 PID 参数
 * @return 参数合法并成功应用时返回 true，否则返回 false
 */
static bool Chassis_ApplyPidTune(Chassis_MotorControl_t *control, const Chassis_PidTune_t *pid_tune)
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
 * @brief 清除底盘单路电机的动态控制状态
 *
 * @param[in,out] control 单路电机速度控制上下文
 * @return 无返回值
 */
static void Chassis_ResetMotorControl(Chassis_MotorControl_t *control)
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
 * @brief 预置底盘单路电机反馈状态以平滑恢复使能
 *
 * @param[in,out] control 单路电机速度控制上下文
 * @return 无返回值
 */
static void Chassis_PrimeFeedback(Chassis_MotorControl_t *control)
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
