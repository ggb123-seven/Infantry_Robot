#include "module/chassis.h"

#include <math.h>
#include <stddef.h>

/*
 * 底盘模块私有运行状态：
 * - initialized：四路速度控制器均可运行时为 true。
 * - control_init_status：四路速度控制器的聚合初始化结果。
 * - motor_init_status[0~3]：四个速度控制器的初始化结果。
 * - speed_control[0~3]：四个相互独立的单电机速度控制上下文。
 */
typedef struct
{
    bool initialized;
    int8_t control_init_status;
    Chassis_MotorStatus_t motor_init_status[CHASSIS_MOTOR_COUNT];
    MotorSpeedControl_t speed_control[CHASSIS_MOTOR_COUNT];
} Chassis_State_t;

static Chassis_State_t chassis_state;

static void Chassis_ResetState(void);
static void Chassis_ResetSnapshot(Chassis_Snapshot_t *snapshot);
static int8_t Chassis_InitControllers(float sample_frequency_hz);
static int8_t Chassis_Calculate(const Chassis_Input_t *input, const Chassis_Feedback_t *feedback,
                                Chassis_Output_t *output, Chassis_Snapshot_t *snapshot);
static Chassis_MotorStatus_t Chassis_MapMotorStatus(int8_t motor_status);

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
        const int8_t motor_status =
            MotorSpeedControl_Init(&chassis_state.speed_control[motor_index], sample_frequency_hz);
        chassis_state.motor_init_status[motor_index] = Chassis_MapMotorStatus(motor_status);
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
        MotorSpeedControl_t *speed_control = &chassis_state.speed_control[motor_index];
        const bool feedback_available = feedback->valid[motor_index] && feedback->motor_online[motor_index];
        const float actual_speed_rpm = feedback_available ? feedback->actual_speed_rpm[motor_index] : 0.0F;
        const int8_t feedback_status = MotorSpeedControl_UpdateFeedback(speed_control, actual_speed_rpm);
        const bool motor_enabled = input->enabled && feedback_available && feedback_status == MOTOR_SPEED_CONTROL_OK;
        const int8_t control_status =
            MotorSpeedControl_Control(speed_control, input->requested_speed_rpm[motor_index], &input->pid_tune,
                                      motor_enabled, input->control_period_s);
        float current_command_a = 0.0F;
        const int8_t output_status = MotorSpeedControl_DumpOutput(speed_control, &current_command_a);

        // 汇总本路状态，异常反馈或计算结果只清零本路电流
        int8_t published_status = control_status;
        if (feedback_status != MOTOR_SPEED_CONTROL_OK)
        {
            published_status = feedback_status;
            all_control_valid = false;
        }
        else if (output_status != MOTOR_SPEED_CONTROL_OK)
        {
            published_status = output_status;
            all_control_valid = false;
        }
        else if (control_status != MOTOR_SPEED_CONTROL_OK && control_status != MOTOR_SPEED_CONTROL_DISABLED)
        {
            all_control_valid = false;
        }
        if (published_status != MOTOR_SPEED_CONTROL_OK)
        {
            current_command_a = 0.0F;
        }

        // 保存统一结果快照，调用方不再读取控制器内部状态
        snapshot->control_status[motor_index] = Chassis_MapMotorStatus(published_status);
        snapshot->motor_enabled[motor_index] = speed_control->feedback.enabled;
        snapshot->limited_target_speed_rpm[motor_index] = speed_control->feedback.limited_target_speed_rpm;
        snapshot->ramped_target_speed_rpm[motor_index] = speed_control->feedback.target_speed_rpm;
        snapshot->actual_speed_rpm[motor_index] = speed_control->feedback.actual_speed_rpm;
        snapshot->current_command_a[motor_index] = current_command_a;
        output->current_command_a[motor_index] = current_command_a;
    }
    return all_control_valid ? CHASSIS_OK : CHASSIS_ERROR;
}

/**
 * @brief 将内部单电机速度控制状态转换为 Chassis 公开状态
 *
 * @param[in] motor_status 内部 MotorSpeedControlStatus_t 状态值
 * @return 对应的 Chassis_MotorStatus_t，未知值返回 CHASSIS_MOTOR_UNKNOWN_ERROR
 */
static Chassis_MotorStatus_t Chassis_MapMotorStatus(int8_t motor_status)
{
    switch (motor_status)
    {
        case MOTOR_SPEED_CONTROL_OK:
            return CHASSIS_MOTOR_OK;
        case MOTOR_SPEED_CONTROL_INIT_ERROR:
            return CHASSIS_MOTOR_INIT_ERROR;
        case MOTOR_SPEED_CONTROL_DISABLED:
            return CHASSIS_MOTOR_DISABLED;
        case MOTOR_SPEED_CONTROL_INVALID_VALUE:
            return CHASSIS_MOTOR_INVALID_VALUE;
        case MOTOR_SPEED_CONTROL_CONFIG_ERROR:
            return CHASSIS_MOTOR_CONFIG_ERROR;
        case MOTOR_SPEED_CONTROL_NULL_ERROR:
            return CHASSIS_MOTOR_NULL_ERROR;
        default:
            return CHASSIS_MOTOR_UNKNOWN_ERROR;
    }
}
