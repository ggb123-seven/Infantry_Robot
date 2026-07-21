#include "task/motor_chassis.h"

#include "device/can_devices.h"
#include "module/chassis.h"
#include "task/ozone_debug.h"
#include "task/user_task.h"

#include <stddef.h>

_Static_assert(CHASSIS_MOTOR_COUNT == CAN_DEVICES_CHASSIS_MOTOR_COUNT, "底盘控制与 CAN 设备数量必须一致");

/*
 * 底盘任务私有状态：
 * - can_devices_snapshot：保存从反馈邮箱取得的最新 CAN 设备快照。
 * - chassis_feedback：保存从 CAN 设备快照映射出的本周期底盘控制反馈。
 * - chassis_output：保存底盘模块本周期计算得到的四路电流命令。
 * - chassis_snapshot：保存底盘速度控制器初始化和本周期控制结果。
 */
static CANDevices_Snapshot_t can_devices_snapshot;
static Chassis_Feedback_t chassis_feedback;
static Chassis_Output_t chassis_output;
static Chassis_Snapshot_t chassis_snapshot;

static bool MotorChassis_ReadLatestFeedback(void);
static bool MotorChassis_PublishCommand(void);

/**
 * @brief 初始化并周期运行四个 M3508 的速度控制链
 *
 * 调试使能关闭、设备离线或任一控制步骤失败时，对应电机电流指令保持为零。
 *
 * @param[in] argument 任务参数，本任务不使用
 * @return 本任务不会返回
 */
void Task_motor_chassis(void *argument)
{
    (void)argument;

    const uint32_t delay_tick = osKernelGetTickFreq() / MOTOR_CHASSIS_FREQ;

    // 等待系统外设与业务模块完成统一初始化后再进入周期控制
    osDelay(MOTOR_CHASSIS_INIT_DELAY);

    // 建立绝对周期基准，避免控制周期累计漂移
    uint32_t tick = osKernelGetTickCount();
    while (1)
    {
        Chassis_Input_t chassis_input;

        // 消费并映射最新 CAN 反馈，邮箱没有新快照时四路反馈保持无效
        const bool feedback_received = MotorChassis_ReadLatestFeedback();

        // 取得一致控制参数并完成纯速度控制计算，再发布最新电流命令
        OzoneDebug_GetMotorChassisInput(&chassis_input, 1.0F / (float)MOTOR_CHASSIS_FREQ);
        Chassis_Run(&chassis_input, &chassis_feedback, &chassis_output, &chassis_snapshot);
        MotorChassis_PublishCommand();

        // 将设备与控制结果集中发布到独立 Ozone 调试区，不在任务内维护诊断字段
        OzoneDebug_UpdateMotorChassis(&chassis_input, feedback_received ? &can_devices_snapshot : NULL,
                                      &chassis_snapshot);

        tick += delay_tick;
        // 等待至下一个绝对任务周期，保持稳定控制频率
        osDelayUntil(tick);
    }
}

/**
 * @brief 初始化底盘任务私有速度控制器
 *
 * @return 四路速度控制器可运行时返回 true，否则返回 false
 */
bool Task_motor_chassis_Init(void)
{
    // 初始化四路底盘速度控制器，CAN 设备集合由独立通信任务负责初始化
    Chassis_Init((float)MOTOR_CHASSIS_FREQ, &chassis_snapshot);
    OzoneDebug_UpdateChassisInit(&chassis_snapshot);
    return chassis_snapshot.initialized;
}

/**
 * @brief 从容量为 1 的反馈邮箱读取并映射最新 CAN 设备快照
 *
 * @return 成功取得新快照时返回 true，否则返回 false 且四路反馈无效
 */
static bool MotorChassis_ReadLatestFeedback(void)
{
    chassis_feedback = (Chassis_Feedback_t)
    {
        0,
    };
    if (task_runtime.msgq.can_feedback == NULL ||
        osMessageQueueGet(task_runtime.msgq.can_feedback, &can_devices_snapshot, NULL, 0U) != osOK)
    {
        return false;
    }

    // 只把本周期成功更新且在线的设备反馈标记为可用于速度控制
    for (uint32_t motor_index = 0U; motor_index < CHASSIS_MOTOR_COUNT; motor_index++)
    {
        chassis_feedback.valid[motor_index] =
            can_devices_snapshot.feedback_update_status[motor_index] == CAN_DEVICES_OK;
        chassis_feedback.motor_online[motor_index] = can_devices_snapshot.motor_online[motor_index];
        chassis_feedback.actual_speed_rpm[motor_index] = can_devices_snapshot.actual_speed_rpm[motor_index];
    }
    return true;
}

/**
 * @brief 向容量为 1 的命令邮箱发布最新底盘电流命令
 *
 * @return 邮箱重置并写入成功时返回 true，否则返回 false
 */
static bool MotorChassis_PublishCommand(void)
{
    CANDevices_Command_t command =
    {
        .sequence = can_devices_snapshot.sequence,
    };
    for (uint32_t motor_index = 0U; motor_index < CHASSIS_MOTOR_COUNT; motor_index++)
    {
        command.current_a[motor_index] = chassis_output.current_command_a[motor_index];
    }
    if (task_runtime.msgq.can_command == NULL)
    {
        return false;
    }

    // 邮箱只保留最新电流命令，重置和写入任一失败都由 CAN task 的零命令默认值兜底
    return osMessageQueueReset(task_runtime.msgq.can_command) == osOK &&
           osMessageQueuePut(task_runtime.msgq.can_command, &command, 0U, 0U) == osOK;
}
