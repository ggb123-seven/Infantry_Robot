#include "task/motor_chassis.h"

#include "module/chassis.h"
#include "module/chassis_can.h"
#include "task/user_task.h"

/*
 * 调试停机确认参数：
 * - MOTOR_DEBUG_STOP_CONFIRM_CYCLES：关闭调试使能后，连续成功提交四电机零电流帧的控制周期数。
 */
#define MOTOR_DEBUG_STOP_CONFIRM_CYCLES (5U)

_Static_assert(MOTOR_CHASSIS_MOTOR_COUNT == CHASSIS_MOTOR_COUNT, "Chassis 电机数量必须与任务设备数量一致");
_Static_assert(MOTOR_CHASSIS_MOTOR_COUNT == CHASSIS_CAN_MOTOR_COUNT,
               "Chassis CAN 电机数量必须与任务设备数量一致");

/**
 * @brief 四个 M3508 的在线调试参数初值
 *
 * 上电后关闭调试使能并将全部目标转速清零，只有调试器明确写入目标并开启使能后才允许非零输出。
 */
volatile MotorChassisTune_t g_motor_chassis_tune =
{
    .motor_debug_enable = false,
    .requested_speed_rpm =
    {
        0.0F,
        0.0F,
        0.0F,
        0.0F,
    },
    .pid_kp = MOTOR_SPEED_PID_KP,
    .pid_ki = MOTOR_SPEED_PID_KI,
    .pid_kd = MOTOR_SPEED_PID_KD,
    .actual_speed_rpm =
    {
        0.0F,
        0.0F,
        0.0F,
        0.0F,
    },
};

/**
 * @brief 四个 M3508 的运行监视数据初值
 *
 * 任务启动前所有设备和速度控制均视为不可用，数值反馈清零，避免把尚未运行的状态误认为有效数据。
 */
volatile MotorChassisMonitor_t g_motor_chassis_monitor =
{
    .motor_online =
    {
        false,
        false,
        false,
        false,
    },
    .current_saturated =
    {
        false,
        false,
        false,
        false,
    },
    .debug_stop_ready = false,
    .register_status =
    {
        CHASSIS_CAN_DEVICE_UNAVAILABLE,
        CHASSIS_CAN_DEVICE_UNAVAILABLE,
        CHASSIS_CAN_DEVICE_UNAVAILABLE,
        CHASSIS_CAN_DEVICE_UNAVAILABLE,
    },
    .control_init_status =
    {
        MOTOR_SPEED_CONTROL_INIT_ERROR,
        MOTOR_SPEED_CONTROL_INIT_ERROR,
        MOTOR_SPEED_CONTROL_INIT_ERROR,
        MOTOR_SPEED_CONTROL_INIT_ERROR,
    },
    .feedback_update_status =
    {
        CHASSIS_CAN_DEVICE_UNAVAILABLE,
        CHASSIS_CAN_DEVICE_UNAVAILABLE,
        CHASSIS_CAN_DEVICE_UNAVAILABLE,
        CHASSIS_CAN_DEVICE_UNAVAILABLE,
    },
    .current_set_status =
    {
        CHASSIS_CAN_DEVICE_UNAVAILABLE,
        CHASSIS_CAN_DEVICE_UNAVAILABLE,
        CHASSIS_CAN_DEVICE_UNAVAILABLE,
        CHASSIS_CAN_DEVICE_UNAVAILABLE,
    },
    .chassis_status = CHASSIS_INIT_ERROR,
    .control_status =
    {
        MOTOR_SPEED_CONTROL_INIT_ERROR,
        MOTOR_SPEED_CONTROL_INIT_ERROR,
        MOTOR_SPEED_CONTROL_INIT_ERROR,
        MOTOR_SPEED_CONTROL_INIT_ERROR,
    },
    .can_tx_status = CHASSIS_CAN_DEVICE_UNAVAILABLE,
    .debug_stop_zero_tx_count = 0U,
    .limited_target_speed_rpm =
    {
        0.0F,
        0.0F,
        0.0F,
        0.0F,
    },
    .ramped_target_speed_rpm =
    {
        0.0F,
        0.0F,
        0.0F,
        0.0F,
    },
    .current_command_a =
    {
        0.0F,
        0.0F,
        0.0F,
        0.0F,
    },
    .temperature_c =
    {
        0.0F,
        0.0F,
        0.0F,
        0.0F,
    },
};

/*
 * 四电机任务私有边界与模块状态：
 * - chassis_can：统一拥有四电机注册、反馈读取、电流写槽和 CAN 发送的边界上下文。
 * - chassis_can_feedback：CAN 边界本周期发布的四路反馈快照。
 * - chassis_can_output_status：CAN 边界本周期发布的四路电流写入和统一发送状态。
 * - chassis：组合四路单电机速度控制器的 Chassis 模块上下文。
 * - chassis_output：Chassis 模块本周期发布的四路一致输出快照。
 * - debug_stop_zero_tx_count：关闭调试使能后连续成功提交零电流帧的周期数。
 */
#ifdef DEBUG
ChassisCAN_t chassis_can;
ChassisCAN_Feedback_t chassis_can_feedback;
ChassisCAN_OutputStatus_t chassis_can_output_status;
Chassis_t chassis;
Chassis_Output_t chassis_output;
#else
static ChassisCAN_t chassis_can;
static ChassisCAN_Feedback_t chassis_can_feedback;
static ChassisCAN_OutputStatus_t chassis_can_output_status;
static Chassis_t chassis;
static Chassis_Output_t chassis_output;
#endif
static uint32_t debug_stop_zero_tx_count;

static void MotorChassis_UpdateOzoneData(const ChassisCAN_Feedback_t *feedback,
                                         const ChassisCAN_OutputStatus_t *output_status,
                                         const Chassis_Output_t *output, bool motor_debug_enable);

/**
 * @brief 初始化并周期运行四个 M3508 的速度控制链
 *
 * 本任务只负责编排 CAN 边界反馈、速度控制模块和四路电流输出快照。
 * 调试使能关闭、设备离线或任一控制步骤失败时，对应电机电流指令保持为零。
 *
 * @param[in] argument 任务参数，本任务不使用
 * @return 本任务不会返回
 */
void Task_motor_chassis(void *argument)
{
    (void)argument;

    const uint32_t delay_tick = osKernelGetTickFreq() / MOTOR_CHASSIS_FREQ;
    const float control_period_s = 1.0F / (float)MOTOR_CHASSIS_FREQ;

    // 等待系统外设初始化完成后再接管 CAN 控制链
    osDelay(MOTOR_CHASSIS_INIT_DELAY);

    // 通过唯一 CAN 边界初始化总线和四个电机，并发布逐路注册诊断
    const int8_t chassis_can_init_status = ChassisCAN_Init(&chassis_can);
    for (uint32_t motor_index = 0U; motor_index < MOTOR_CHASSIS_MOTOR_COUNT; motor_index++)
    {
        g_motor_chassis_monitor.register_status[motor_index] = chassis_can.register_status[motor_index];
    }
    if (!chassis_can.initialized)
    {
        g_motor_chassis_monitor.can_tx_status = chassis_can_init_status;
        osThreadExit();
        return;
    }

    // 由 Chassis 模块统一创建四个相互独立的速度控制器
    g_motor_chassis_monitor.chassis_status = Chassis_Init(&chassis, (float)MOTOR_CHASSIS_FREQ);
    for (uint32_t motor_index = 0U; motor_index < MOTOR_CHASSIS_MOTOR_COUNT; motor_index++)
    {
        g_motor_chassis_monitor.control_init_status[motor_index] = chassis.init_status[motor_index];
    }

    // 建立绝对周期基准，避免控制周期累计漂移
    uint32_t tick = osKernelGetTickCount();
    while (1)
    {
        Chassis_Input_t chassis_input =
        {
            .enabled = g_motor_chassis_tune.motor_debug_enable,
            .pid_tune =
            {
                .kp = g_motor_chassis_tune.pid_kp,
                .ki = g_motor_chassis_tune.pid_ki,
                .kd = g_motor_chassis_tune.pid_kd,
            },
            .control_period_s = control_period_s,
        };
        const bool motor_debug_enable = chassis_input.enabled;

        // 通过 CAN 边界独立刷新四路设备反馈并记录本周期更新结果
        ChassisCAN_ReadFeedback(&chassis_can, &chassis_can_feedback);

        // 仅将本周期收到的新反馈交给控制器，保持迁移前的保守失帧处理语义
        for (uint32_t motor_index = 0U; motor_index < MOTOR_CHASSIS_MOTOR_COUNT; motor_index++)
        {
            chassis_input.requested_speed_rpm[motor_index] = g_motor_chassis_tune.requested_speed_rpm[motor_index];
            const bool feedback_updated = chassis_can_feedback.feedback_update_status[motor_index] == CHASSIS_CAN_OK;
            chassis_input.actual_speed_rpm[motor_index] =
                feedback_updated ? chassis_can_feedback.actual_speed_rpm[motor_index] : 0.0F;
            chassis_input.motor_online[motor_index] =
                feedback_updated && chassis_can_feedback.motor_online[motor_index];
        }

        // Chassis 模块一次性计算四路速度控制，任一路异常只清零对应输出
        g_motor_chassis_monitor.chassis_status = Chassis_Control(&chassis, &chassis_input, &chassis_output);

        // 通过 CAN 边界写入四路安全电流并只发送一次同组控制帧
        ChassisCAN_WriteCurrent(&chassis_can, chassis_output.current_command_a, &chassis_can_output_status);

        // 写入异常的单路按实际安全结果归零，避免诊断继续显示未提交的控制命令
        for (uint32_t motor_index = 0U; motor_index < MOTOR_CHASSIS_MOTOR_COUNT; motor_index++)
        {
            if (chassis_can_output_status.current_set_status[motor_index] != CHASSIS_CAN_OK)
            {
                chassis_output.current_command_a[motor_index] = 0.0F;
            }
        }

        // 汇总 CAN 边界与 Chassis 结果，发布本周期 Ozone 诊断快照
        MotorChassis_UpdateOzoneData(&chassis_can_feedback, &chassis_can_output_status, &chassis_output,
                                     motor_debug_enable);

        tick += delay_tick;
        // 等待至下一个绝对任务周期，保持稳定控制频率
        osDelayUntil(tick);
    }
}

/**
 * @brief 汇总本周期结果并刷新 Ozone 四电机诊断数据
 *
 * 除更新每个电机的反馈、控制和电流状态外，还统计连续成功发送零电流帧的周期数，
 * 用于确认关闭调试使能后的安全停机状态。
 *
 * @param[in] feedback CAN 边界本周期四电机反馈快照
 * @param[in] output_status CAN 边界本周期电流写入与统一发送状态
 * @param[in] output Chassis 模块本周期四路输出快照
 * @param[in] motor_debug_enable 本周期采用的四电机全局调试使能状态
 * @return 无返回值
 */
static void MotorChassis_UpdateOzoneData(const ChassisCAN_Feedback_t *feedback,
                                         const ChassisCAN_OutputStatus_t *output_status,
                                         const Chassis_Output_t *output, bool motor_debug_enable)
{
    bool all_zero_current_set = !motor_debug_enable && output_status->can_tx_status == CHASSIS_CAN_OK;

    for (uint32_t motor_index = 0U; motor_index < MOTOR_CHASSIS_MOTOR_COUNT; motor_index++)
    {
        if (output->current_command_a[motor_index] != 0.0F ||
            output_status->current_set_status[motor_index] != CHASSIS_CAN_OK)
        {
            all_zero_current_set = false;
        }

        g_motor_chassis_monitor.motor_online[motor_index] = feedback->motor_online[motor_index];
        g_motor_chassis_monitor.current_saturated[motor_index] =
            output->current_command_a[motor_index] >= MOTOR_SPEED_CURRENT_LIMIT_A ||
            output->current_command_a[motor_index] <= -MOTOR_SPEED_CURRENT_LIMIT_A;
        g_motor_chassis_monitor.feedback_update_status[motor_index] = feedback->feedback_update_status[motor_index];
        g_motor_chassis_monitor.current_set_status[motor_index] = output_status->current_set_status[motor_index];
        g_motor_chassis_monitor.control_status[motor_index] = output->control_status[motor_index];
        g_motor_chassis_monitor.limited_target_speed_rpm[motor_index] = output->limited_target_speed_rpm[motor_index];
        g_motor_chassis_monitor.ramped_target_speed_rpm[motor_index] = output->ramped_target_speed_rpm[motor_index];
        g_motor_chassis_tune.actual_speed_rpm[motor_index] = output->actual_speed_rpm[motor_index];
        g_motor_chassis_monitor.current_command_a[motor_index] = output->current_command_a[motor_index];
        g_motor_chassis_monitor.temperature_c[motor_index] = feedback->temperature_c[motor_index];
    }

    if (all_zero_current_set)
    {
        if (debug_stop_zero_tx_count < MOTOR_DEBUG_STOP_CONFIRM_CYCLES)
        {
            debug_stop_zero_tx_count++;
        }
    }
    else
    {
        debug_stop_zero_tx_count = 0U;
    }

    g_motor_chassis_monitor.debug_stop_ready =
        debug_stop_zero_tx_count >= MOTOR_DEBUG_STOP_CONFIRM_CYCLES;
    g_motor_chassis_monitor.can_tx_status = output_status->can_tx_status;
    g_motor_chassis_monitor.debug_stop_zero_tx_count = debug_stop_zero_tx_count;
}
