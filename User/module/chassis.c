#include "module/chassis.h"

#include <math.h>
#include <stddef.h>

static void Chassis_ResetOutput(Chassis_Output_t *output, int8_t default_status);

/**
 * @brief 初始化四路 Chassis 速度控制器
 *
 * @param[out] chassis Chassis 控制上下文
 * @param[in] sample_frequency_hz 控制采样频率，单位 Hz，必须大于 0
 * @return 全部速度控制器初始化成功返回 CHASSIS_OK，否则返回对应状态码
 */
int8_t Chassis_Init(Chassis_t *chassis, float sample_frequency_hz)
{
    if (chassis == NULL)
    {
        return CHASSIS_NULL_ERROR;
    }

    // 清除全部跨周期状态，确保初始化失败时四路控制器均保持禁用
    *chassis = (Chassis_t)
    {
        0,
    };
    if (!isfinite(sample_frequency_hz) || sample_frequency_hz <= 0.0F)
    {
        for (uint32_t motor_index = 0U; motor_index < CHASSIS_MOTOR_COUNT; motor_index++)
        {
            chassis->init_status[motor_index] = MOTOR_SPEED_CONTROL_CONFIG_ERROR;
        }
        return CHASSIS_CONFIG_ERROR;
    }

    // 使用相同采样频率依次初始化四个相互独立的速度控制器
    bool all_initialized = true;
    for (uint32_t motor_index = 0U; motor_index < CHASSIS_MOTOR_COUNT; motor_index++)
    {
        chassis->init_status[motor_index] =
            MotorSpeedControl_Init(&chassis->speed_control[motor_index], sample_frequency_hz);
        if (chassis->init_status[motor_index] != MOTOR_SPEED_CONTROL_OK)
        {
            all_initialized = false;
        }
    }

    chassis->initialized = all_initialized;
    return all_initialized ? CHASSIS_OK : CHASSIS_INIT_ERROR;
}

/**
 * @brief 执行一次四路 Chassis 速度控制计算
 *
 * 任一路离线、禁用或输入异常时只清零该路输出，其余合法控制器继续独立运行。
 *
 * @param[in,out] chassis Chassis 控制上下文
 * @param[in] input 本周期四路输入快照
 * @param[out] output 本周期四路一致输出快照
 * @return 全部活动控制器正常时返回 CHASSIS_OK，存在控制异常时返回 CHASSIS_CONTROL_ERROR
 */
int8_t Chassis_Control(Chassis_t *chassis, const Chassis_Input_t *input, Chassis_Output_t *output)
{
    if (output == NULL)
    {
        return CHASSIS_NULL_ERROR;
    }

    // 在检查其他输入前建立全零输出，保证所有失败路径都有安全快照
    Chassis_Output_t calculated_output;
    Chassis_ResetOutput(&calculated_output, MOTOR_SPEED_CONTROL_INIT_ERROR);
    if (chassis == NULL || input == NULL)
    {
        *output = calculated_output;
        return CHASSIS_NULL_ERROR;
    }
    if (!chassis->initialized)
    {
        *output = calculated_output;
        return CHASSIS_INIT_ERROR;
    }

    // 四路控制器分别接收反馈和使能条件，单路异常不得阻止其他控制器运行
    bool all_control_valid = true;
    for (uint32_t motor_index = 0U; motor_index < CHASSIS_MOTOR_COUNT; motor_index++)
    {
        MotorSpeedControl_t *speed_control = &chassis->speed_control[motor_index];
        const int8_t feedback_status =
            MotorSpeedControl_UpdateFeedback(speed_control, input->actual_speed_rpm[motor_index]);
        const bool motor_enabled = input->enabled && input->motor_online[motor_index] &&
                                   feedback_status == MOTOR_SPEED_CONTROL_OK;
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

        // 保存四路一致输出快照，调用方只消费公开结果，不读取内部控制器状态
        calculated_output.control_status[motor_index] = published_status;
        calculated_output.motor_enabled[motor_index] = speed_control->feedback.enabled;
        calculated_output.limited_target_speed_rpm[motor_index] = speed_control->feedback.limited_target_speed_rpm;
        calculated_output.ramped_target_speed_rpm[motor_index] = speed_control->feedback.target_speed_rpm;
        calculated_output.actual_speed_rpm[motor_index] = speed_control->feedback.actual_speed_rpm;
        calculated_output.current_command_a[motor_index] = current_command_a;
    }

    *output = calculated_output;
    return all_control_valid ? CHASSIS_OK : CHASSIS_CONTROL_ERROR;
}

/**
 * @brief 将 Chassis 输出恢复为全零并设置四路默认状态
 *
 * @param[out] output 待复位的 Chassis 输出
 * @param[in] default_status 四路控制状态默认值
 * @return 无返回值
 */
static void Chassis_ResetOutput(Chassis_Output_t *output, int8_t default_status)
{
    *output = (Chassis_Output_t)
    {
        0,
    };
    for (uint32_t motor_index = 0U; motor_index < CHASSIS_MOTOR_COUNT; motor_index++)
    {
        output->control_status[motor_index] = default_status;
    }
}
