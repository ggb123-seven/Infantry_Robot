#include "task/motor_chassis.h"

#include "bsp/can.h"
#include "device/motor_rm.h"
#include "module/chassis.h"
#include "task/user_task.h"

#include <stddef.h>

/*
 * 调试停机确认参数：
 * - MOTOR_DEBUG_STOP_CONFIRM_CYCLES：关闭调试使能后，连续成功提交四电机零电流帧的控制周期数。
 */
#define MOTOR_DEBUG_STOP_CONFIRM_CYCLES (5U)

_Static_assert(MOTOR_CHASSIS_MOTOR_COUNT == CHASSIS_MOTOR_COUNT, "Chassis 电机数量必须与任务设备数量一致");

/*
 * 四个 M3508 电机参数：
 * - CAN 总线：全部使用 CAN1。
 * - C620 电调 ID：依次为 1~4，对应反馈标准帧 ID 0x201~0x204。
 * - 电机型号：全部为 M3508。
 * - 安装方向：当前全部不反向，最终方向需通过低速上板测试确认。
 * - 减速箱：全部启用，设备层将转子转速除以 3591/187 后输出 rpm。
 * @datasheet RoboMaster C620 用户手册“CAN 通信协议”章节
 */
static MOTOR_RM_Param_t motor_3508_param[MOTOR_CHASSIS_MOTOR_COUNT] =
{
    {
        .can = BSP_CAN_1,
        .id = 0x201U,
        .module = MOTOR_M3508,
        .reverse = false,
        .gear = true,
    },
    {
        .can = BSP_CAN_1,
        .id = 0x202U,
        .module = MOTOR_M3508,
        .reverse = false,
        .gear = true,
    },
    {
        .can = BSP_CAN_1,
        .id = 0x203U,
        .module = MOTOR_M3508,
        .reverse = false,
        .gear = true,
    },
    {
        .can = BSP_CAN_1,
        .id = 0x204U,
        .module = MOTOR_M3508,
        .reverse = false,
        .gear = true,
    },
};

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
        DEVICE_ERR_NO_DEV,
        DEVICE_ERR_NO_DEV,
        DEVICE_ERR_NO_DEV,
        DEVICE_ERR_NO_DEV,
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
        DEVICE_ERR_NO_DEV,
        DEVICE_ERR_NO_DEV,
        DEVICE_ERR_NO_DEV,
        DEVICE_ERR_NO_DEV,
    },
    .current_set_status =
    {
        DEVICE_ERR_NO_DEV,
        DEVICE_ERR_NO_DEV,
        DEVICE_ERR_NO_DEV,
        DEVICE_ERR_NO_DEV,
    },
    .chassis_status = CHASSIS_INIT_ERROR,
    .control_status =
    {
        MOTOR_SPEED_CONTROL_INIT_ERROR,
        MOTOR_SPEED_CONTROL_INIT_ERROR,
        MOTOR_SPEED_CONTROL_INIT_ERROR,
        MOTOR_SPEED_CONTROL_INIT_ERROR,
    },
    .can_tx_status = DEVICE_ERR_NO_DEV,
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
 * 四电机任务私有设备与模块状态：
 * - motor_3508[0~3]：注册后的 M3508 设备实例，注册失败时对应元素为 NULL。
 * - chassis：组合四路单电机速度控制器的 Chassis 模块上下文。
 * - chassis_output：Chassis 模块本周期发布的四路一致输出快照。
 * - debug_stop_zero_tx_count：关闭调试使能后连续成功提交零电流帧的周期数。
 */
#ifdef DEBUG
MOTOR_RM_t *motor_3508[MOTOR_CHASSIS_MOTOR_COUNT];
Chassis_t chassis;
Chassis_Output_t chassis_output;
#else
static MOTOR_RM_t *motor_3508[MOTOR_CHASSIS_MOTOR_COUNT];
static Chassis_t chassis;
static Chassis_Output_t chassis_output;
#endif
static uint32_t debug_stop_zero_tx_count;

static void MotorChassis_UpdateOzoneData(const int8_t feedback_update_status[MOTOR_CHASSIS_MOTOR_COUNT],
                                         const int8_t current_set_status[MOTOR_CHASSIS_MOTOR_COUNT],
                                         const Chassis_Output_t *output, int8_t can_tx_status,
                                         bool motor_debug_enable);

/**
 * @brief 初始化并周期运行四个 M3508 的速度控制链
 *
 * 本任务只负责编排设备反馈、速度控制模块、电流缓存和 CAN 统一发送。
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

    // 初始化 CAN 总线，失败时保持默认零输出并终止本任务
    const int8_t can_init_status = BSP_CAN_Init();
    if (can_init_status != BSP_OK)
    {
        g_motor_chassis_monitor.can_tx_status = can_init_status;
        osThreadExit();
        return;
    }

    // 注册四个电机设备，任务只保存后续设备 I/O 所需实例
    for (uint32_t motor_index = 0U; motor_index < MOTOR_CHASSIS_MOTOR_COUNT; motor_index++)
    {
        const int8_t register_status = MOTOR_RM_Register(&motor_3508_param[motor_index]);
        g_motor_chassis_monitor.register_status[motor_index] = register_status;
        motor_3508[motor_index] =
            register_status == DEVICE_OK ? MOTOR_RM_GetMotor(&motor_3508_param[motor_index]) : NULL;
        if (motor_3508[motor_index] == NULL && register_status == DEVICE_OK)
        {
            g_motor_chassis_monitor.register_status[motor_index] = DEVICE_ERR_NO_DEV;
        }
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
        int8_t feedback_update_status[MOTOR_CHASSIS_MOTOR_COUNT];
        int8_t current_set_status[MOTOR_CHASSIS_MOTOR_COUNT];
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

        // 采集四路目标与设备反馈，建立本周期 Chassis 一致输入快照
        for (uint32_t motor_index = 0U; motor_index < MOTOR_CHASSIS_MOTOR_COUNT; motor_index++)
        {
            chassis_input.requested_speed_rpm[motor_index] = g_motor_chassis_tune.requested_speed_rpm[motor_index];
            chassis_input.actual_speed_rpm[motor_index] = 0.0F;
            chassis_input.motor_online[motor_index] = false;
            feedback_update_status[motor_index] = DEVICE_ERR_NO_DEV;
            current_set_status[motor_index] = DEVICE_ERR_NO_DEV;

            // 从 CAN 接收缓存刷新本电机设备反馈
            if (motor_3508[motor_index] != NULL)
            {
                feedback_update_status[motor_index] = MOTOR_RM_Update(&motor_3508_param[motor_index]);
            }

            if (motor_3508[motor_index] != NULL && feedback_update_status[motor_index] == DEVICE_OK)
            {
                chassis_input.actual_speed_rpm[motor_index] = motor_3508[motor_index]->feedback.rotor_speed;
                chassis_input.motor_online[motor_index] = motor_3508[motor_index]->motor.header.online;
            }
        }

        // Chassis 模块一次性计算四路速度控制，任一路异常只清零对应输出
        g_motor_chassis_monitor.chassis_status = Chassis_Control(&chassis, &chassis_input, &chassis_output);

        // 将四路 Chassis 安全电流输出写入对应设备发送槽位
        for (uint32_t motor_index = 0U; motor_index < MOTOR_CHASSIS_MOTOR_COUNT; motor_index++)
        {
            if (motor_3508[motor_index] != NULL)
            {
                current_set_status[motor_index] =
                    MOTOR_RM_SetTorqueCurrent(&motor_3508_param[motor_index],
                                              chassis_output.current_command_a[motor_index]);
                if (current_set_status[motor_index] != DEVICE_OK)
                {
                    chassis_output.current_command_a[motor_index] = 0.0F;

                    // 写入失败时立即用零电流覆盖旧槽位，原失败状态继续用于诊断
                    const int8_t zero_set_status =
                        MOTOR_RM_SetTorqueCurrent(&motor_3508_param[motor_index], 0.0F);
                    if (zero_set_status != DEVICE_OK)
                    {
                        current_set_status[motor_index] = zero_set_status;
                    }
                }
            }
        }

        // 四个槽位更新完成后统一发送控制帧并发布本周期诊断快照
        const int8_t can_tx_status = MOTOR_RM_FlushGroup(&motor_3508_param[0]);
        MotorChassis_UpdateOzoneData(feedback_update_status, current_set_status, &chassis_output, can_tx_status,
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
 * @param[in] feedback_update_status 本周期四电机反馈更新结果
 * @param[in] current_set_status 本周期四电机电流指令写入结果
 * @param[in] output Chassis 模块本周期四路输出快照
 * @param[in] can_tx_status 本周期 CAN 控制帧发送结果
 * @param[in] motor_debug_enable 本周期采用的四电机全局调试使能状态
 * @return 无返回值
 */
static void MotorChassis_UpdateOzoneData(const int8_t feedback_update_status[MOTOR_CHASSIS_MOTOR_COUNT],
                                         const int8_t current_set_status[MOTOR_CHASSIS_MOTOR_COUNT],
                                         const Chassis_Output_t *output, int8_t can_tx_status,
                                         bool motor_debug_enable)
{
    bool all_zero_current_set = !motor_debug_enable && can_tx_status == DEVICE_OK;

    for (uint32_t motor_index = 0U; motor_index < MOTOR_CHASSIS_MOTOR_COUNT; motor_index++)
    {
        const bool motor_online =
            motor_3508[motor_index] != NULL && motor_3508[motor_index]->motor.header.online;
        if (output->current_command_a[motor_index] != 0.0F || current_set_status[motor_index] != DEVICE_OK)
        {
            all_zero_current_set = false;
        }

        g_motor_chassis_monitor.motor_online[motor_index] = motor_online;
        g_motor_chassis_monitor.current_saturated[motor_index] =
            output->current_command_a[motor_index] >= MOTOR_SPEED_CURRENT_LIMIT_A ||
            output->current_command_a[motor_index] <= -MOTOR_SPEED_CURRENT_LIMIT_A;
        g_motor_chassis_monitor.feedback_update_status[motor_index] = feedback_update_status[motor_index];
        g_motor_chassis_monitor.current_set_status[motor_index] = current_set_status[motor_index];
        g_motor_chassis_monitor.control_status[motor_index] = output->control_status[motor_index];
        g_motor_chassis_monitor.limited_target_speed_rpm[motor_index] = output->limited_target_speed_rpm[motor_index];
        g_motor_chassis_monitor.ramped_target_speed_rpm[motor_index] = output->ramped_target_speed_rpm[motor_index];
        g_motor_chassis_tune.actual_speed_rpm[motor_index] = output->actual_speed_rpm[motor_index];
        g_motor_chassis_monitor.current_command_a[motor_index] = output->current_command_a[motor_index];
        g_motor_chassis_monitor.temperature_c[motor_index] =
            motor_3508[motor_index] != NULL ? motor_3508[motor_index]->feedback.temp : 0.0F;
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
    g_motor_chassis_monitor.can_tx_status = can_tx_status;
    g_motor_chassis_monitor.debug_stop_zero_tx_count = debug_stop_zero_tx_count;
}
