#include "module/gimbal.h"

#include "component/filter.h"
#include "component/pid.h"

#include <math.h>
#include <stddef.h>

/*
 * GM6020 速度控制运行上下文：
 * - initialized、was_enabled：初始化和上一周期实际使能状态
 * - init_status：初始化结果
 * - ramped_target_speed_rad_s：兼容字段，当前直接采用限幅后的目标角速度，单位 rad/s
 * - pid_param、pid：GM6020 独立 PID 参数和动态状态
 * - feedback_filter、current_filter：速度反馈和电流指令滤波器状态
 */
typedef struct
{
    bool initialized;
    bool was_enabled;
    Gimbal_Status_t init_status;
    float ramped_target_speed_rad_s;
    KPID_Params_t pid_param;
    KPID_t pid;
    LowPassFilter2p_t feedback_filter;
    LowPassFilter2p_t current_filter;
} Gimbal_Control_t;

static Gimbal_Control_t gimbal_control =
{
    .init_status = GIMBAL_NOT_INITIALIZED,
};

static float Gimbal_Clamp(float value, float absolute_limit);
static bool Gimbal_IsConfigurationValid(float sample_frequency_hz);
static bool Gimbal_ApplyPidTune(const Gimbal_PidTune_t *pid_tune);
static void Gimbal_ResetControl(void);
static void Gimbal_PrimeFeedback(float actual_speed_rad_s);

/**
 * @brief 初始化 GM6020 速度控制器
 *
 * @param[in] sample_frequency_hz 控制采样频率，单位 Hz，必须大于 0
 * @param[out] snapshot 初始化结果快照
 * @return 初始化成功返回 GIMBAL_OK，失败返回对应状态码
 */
Gimbal_Status_t Gimbal_Init(float sample_frequency_hz, Gimbal_Snapshot_t *snapshot)
{
    if (snapshot == NULL)
    {
        return GIMBAL_NULL_ERROR;
    }

    // 清除运行上下文和结果快照，保证初始化失败路径保持零输出
    gimbal_control = (Gimbal_Control_t)
    {
        0,
    };
    *snapshot = (Gimbal_Snapshot_t)
    {
        0,
    };
    gimbal_control.init_status = GIMBAL_NOT_INITIALIZED;
    gimbal_control.pid_param = (KPID_Params_t)
    {
        .k = 1.0F,
        .p = GIMBAL_PID_KP,
        .i = GIMBAL_PID_KI,
        .d = GIMBAL_PID_KD,
        .i_limit = GIMBAL_PID_INTEGRAL_LIMIT,
        .out_limit = GIMBAL_CURRENT_LIMIT_A,
        .d_cutoff_freq = GIMBAL_PID_D_CUTOFF_HZ,
        .range = 0.0F,
    };
    const Gimbal_PidTune_t default_pid_tune =
    {
        .kp = GIMBAL_PID_KP,
        .ki = GIMBAL_PID_KI,
        .kd = GIMBAL_PID_KD,
    };

    // 校验固定参数和默认 PID 参数，非法配置不得进入闭环控制
    if (!Gimbal_IsConfigurationValid(sample_frequency_hz) || !Gimbal_ApplyPidTune(&default_pid_tune))
    {
        gimbal_control.init_status = GIMBAL_CONFIG_ERROR;
        snapshot->init_status = gimbal_control.init_status;
        snapshot->control_status = GIMBAL_NOT_INITIALIZED;
        return gimbal_control.init_status;
    }

    // 复用底盘速度环使用的 PID 组件并初始化独立滤波器状态
    if (PID_Init(&gimbal_control.pid, KPID_MODE_CALC_D, sample_frequency_hz, &gimbal_control.pid_param) != 0)
    {
        gimbal_control.init_status = GIMBAL_ERROR;
        snapshot->init_status = gimbal_control.init_status;
        snapshot->control_status = GIMBAL_NOT_INITIALIZED;
        return gimbal_control.init_status;
    }
    LowPassFilter2p_Init(&gimbal_control.feedback_filter, sample_frequency_hz, GIMBAL_FEEDBACK_LPF_CUTOFF_HZ);
    LowPassFilter2p_Init(&gimbal_control.current_filter, sample_frequency_hz, GIMBAL_CURRENT_LPF_CUTOFF_HZ);

    gimbal_control.initialized = true;
    gimbal_control.init_status = GIMBAL_OK;
    snapshot->initialized = true;
    snapshot->init_status = GIMBAL_OK;
    snapshot->control_status = GIMBAL_DISABLED;
    snapshot->pid_kp = gimbal_control.pid_param.p;
    snapshot->pid_ki = gimbal_control.pid_param.i;
    snapshot->pid_kd = gimbal_control.pid_param.d;
    return GIMBAL_OK;
}

/**
 * @brief 执行一次 GM6020 速度闭环控制
 *
 * @param[in] input 本周期控制输入
 * @param[in] feedback 本周期电机反馈
 * @param[out] output 本周期电流输出
 * @param[out] snapshot 本周期控制结果快照
 * @return 控制正常返回 GIMBAL_OK，禁用或异常时返回对应状态码
 */
Gimbal_Status_t Gimbal_Run(const Gimbal_Input_t *input, const Gimbal_Feedback_t *feedback,
                           Gimbal_Output_t *output, Gimbal_Snapshot_t *snapshot)
{
    if (output != NULL)
    {
        *output = (Gimbal_Output_t)
        {
            0,
        };
    }
    if (input == NULL || feedback == NULL || output == NULL || snapshot == NULL)
    {
        return GIMBAL_NULL_ERROR;
    }

    // 复制本周期输入和控制器状态，优先建立安全的零电流快照
    *snapshot = (Gimbal_Snapshot_t)
    {
        .initialized = gimbal_control.initialized,
        .init_status = gimbal_control.init_status,
        .control_status = GIMBAL_NOT_INITIALIZED,
        .enabled = input->enabled && feedback->valid && feedback->online,
        .requested_speed_rad_s = input->target_speed_rad_s,
        .limited_target_speed_rad_s = isfinite(input->target_speed_rad_s) ?
                                      Gimbal_Clamp(input->target_speed_rad_s, GIMBAL_SPEED_LIMIT_RAD_S) : 0.0F,
        .actual_speed_rad_s = isfinite(feedback->actual_speed_rad_s) ? feedback->actual_speed_rad_s : 0.0F,
        .pid_kp = gimbal_control.pid_param.p,
        .pid_ki = gimbal_control.pid_param.i,
        .pid_kd = gimbal_control.pid_param.d,
    };

    // 校验并应用 GM6020 独立 PID 参数，失败时清除动态控制状态
    const bool pid_tune_valid = Gimbal_ApplyPidTune(&input->pid_tune);
    snapshot->pid_kp = gimbal_control.pid_param.p;
    snapshot->pid_ki = gimbal_control.pid_param.i;
    snapshot->pid_kd = gimbal_control.pid_param.d;
    if (!gimbal_control.initialized)
    {
        snapshot->control_status = GIMBAL_NOT_INITIALIZED;
        Gimbal_ResetControl();
    }
    else if (!pid_tune_valid)
    {
        snapshot->control_status = GIMBAL_CONFIG_ERROR;
        Gimbal_ResetControl();
    }
    else if (!isfinite(input->target_speed_rad_s) || !isfinite(input->control_period_s) ||
             input->control_period_s <= 0.0F || !isfinite(feedback->actual_speed_rad_s))
    {
        snapshot->control_status = GIMBAL_INVALID_VALUE;
        Gimbal_ResetControl();
    }
    else if (!snapshot->enabled)
    {
        snapshot->control_status = GIMBAL_DISABLED;
        Gimbal_ResetControl();
    }
    else
    {
        // 使能恢复时预置反馈状态，避免反馈微分产生突跳
        if (!gimbal_control.was_enabled)
        {
            Gimbal_PrimeFeedback(feedback->actual_speed_rad_s);
        }

        // 直接采用限幅后的目标转速，再执行反馈滤波、PID 计算和电流滤波
        gimbal_control.ramped_target_speed_rad_s = snapshot->limited_target_speed_rad_s;
        snapshot->filtered_speed_rad_s =
            LowPassFilter2p_Apply(&gimbal_control.feedback_filter, feedback->actual_speed_rad_s);
        const float pid_current_command_a =
            PID_Calc(&gimbal_control.pid, gimbal_control.ramped_target_speed_rad_s,
                     snapshot->filtered_speed_rad_s, 0.0F, input->control_period_s);
        snapshot->current_command_a =
            LowPassFilter2p_Apply(&gimbal_control.current_filter, pid_current_command_a);

        // 拒绝异常计算结果，仅发布有限且已限幅的电流指令
        if (!isfinite(snapshot->current_command_a))
        {
            snapshot->control_status = GIMBAL_INVALID_VALUE;
            Gimbal_ResetControl();
        }
        else
        {
            snapshot->current_command_a = Gimbal_Clamp(snapshot->current_command_a, GIMBAL_CURRENT_LIMIT_A);
            output->current_command_a = snapshot->current_command_a;
            snapshot->control_status = GIMBAL_OK;
        }
    }

    // 汇总目标与误差，异常路径始终保持零电流
    snapshot->ramped_target_speed_rad_s = gimbal_control.ramped_target_speed_rad_s;
    snapshot->speed_error_rad_s = snapshot->control_status == GIMBAL_OK ?
                                  snapshot->ramped_target_speed_rad_s - snapshot->filtered_speed_rad_s : 0.0F;
    if (snapshot->control_status != GIMBAL_OK)
    {
        snapshot->current_command_a = 0.0F;
        output->current_command_a = 0.0F;
    }
    return snapshot->control_status;
}

/**
 * @brief 对浮点值执行正负对称限幅
 *
 * @param[in] value 待限幅数值
 * @param[in] absolute_limit 正数形式的绝对值上限
 * @return 限幅后的数值
 */
static float Gimbal_Clamp(float value, float absolute_limit)
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
 * @brief 校验 GM6020 速度控制固定参数
 *
 * @param[in] sample_frequency_hz 控制采样频率，单位 Hz
 * @return 所有参数合法时返回 true，否则返回 false
 */
static bool Gimbal_IsConfigurationValid(float sample_frequency_hz)
{
    return isfinite(sample_frequency_hz) && sample_frequency_hz > 0.0F && isfinite(GIMBAL_SPEED_LIMIT_RAD_S) &&
           GIMBAL_SPEED_LIMIT_RAD_S > 0.0F &&
           isfinite(GIMBAL_PID_D_CUTOFF_HZ) && GIMBAL_PID_D_CUTOFF_HZ < sample_frequency_hz * 0.5F &&
           isfinite(GIMBAL_FEEDBACK_LPF_CUTOFF_HZ) &&
           GIMBAL_FEEDBACK_LPF_CUTOFF_HZ < sample_frequency_hz * 0.5F &&
           isfinite(GIMBAL_CURRENT_LPF_CUTOFF_HZ) &&
           GIMBAL_CURRENT_LPF_CUTOFF_HZ < sample_frequency_hz * 0.5F &&
           isfinite(GIMBAL_PID_INTEGRAL_LIMIT) && GIMBAL_PID_INTEGRAL_LIMIT >= 0.0F &&
           isfinite(GIMBAL_CURRENT_LIMIT_A) && GIMBAL_CURRENT_LIMIT_A > 0.0F;
}

/**
 * @brief 校验并应用 GM6020 独立速度 PID 参数
 *
 * @param[in] pid_tune 待应用的速度 PID 参数
 * @return 参数合法并成功应用时返回 true，否则返回 false
 */
static bool Gimbal_ApplyPidTune(const Gimbal_PidTune_t *pid_tune)
{
    if (pid_tune == NULL)
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

    gimbal_control.pid_param.p = kp;
    gimbal_control.pid_param.i = ki;
    gimbal_control.pid_param.d = kd;
    return true;
}

/**
 * @brief 清除 GM6020 动态控制状态并恢复零电流
 *
 * @return 无返回值
 */
static void Gimbal_ResetControl(void)
{
    gimbal_control.ramped_target_speed_rad_s = 0.0F;
    gimbal_control.was_enabled = false;
    if (gimbal_control.initialized)
    {
        PID_Reset(&gimbal_control.pid);
        LowPassFilter2p_Reset(&gimbal_control.feedback_filter, 0.0F);
        LowPassFilter2p_Reset(&gimbal_control.current_filter, 0.0F);
    }
}

/**
 * @brief 预置 GM6020 反馈状态以平滑恢复使能
 *
 * @param[in] actual_speed_rad_s 当前实际角速度，单位 rad/s
 * @return 无返回值
 */
static void Gimbal_PrimeFeedback(float actual_speed_rad_s)
{
    // 清除旧 PID 状态并用当前实际转速预置反馈滤波器
    PID_Reset(&gimbal_control.pid);
    const float filtered_speed_rad_s =
        LowPassFilter2p_Reset(&gimbal_control.feedback_filter, actual_speed_rad_s);
    LowPassFilter2p_Reset(&gimbal_control.current_filter, 0.0F);

    // 同步微分滤波器和上一反馈，避免重新使能时产生电流冲击
    const float scaled_feedback = gimbal_control.pid_param.k * filtered_speed_rad_s;
    LowPassFilter2p_Reset(&gimbal_control.pid.dfilter, scaled_feedback);
    gimbal_control.pid.last.k_fb = scaled_feedback;
    gimbal_control.was_enabled = true;
}
