#include "task/motor_gimbal.h"

#include "device/can_devices.h"
#include "module/gimbal.h"
#include "task/ozone_debug.h"
#include "task/user_task.h"

#include <stddef.h>

/*
 * 云台任务运行快照：
 * - motor_gimbal_can_snapshot：CAN 任务发布的最新 GM6020 反馈
 * - motor_gimbal_control_snapshot：GM6020 速度控制模块的最新计算结果
 * 云台任务只负责编排模块与邮箱，不直接访问 GM6020 私有参数或 CAN 外设
 */
static CANDevices_Snapshot_t motor_gimbal_can_snapshot;
static Gimbal_Snapshot_t motor_gimbal_control_snapshot;

static bool MotorGimbal_ReadLatestFeedback(void);
static bool MotorGimbal_PublishCommand(const CANDevices_GimbalCommand_t *command);

/**
 * @brief 周期执行 GM6020 速度闭环并发布电流命令
 *
 * @param[in] argument 任务参数，本任务不使用
 * @return 本任务不会返回
 */
void Task_motor_gimbal(void *argument)
{
    (void)argument;

    const uint32_t delay_tick = osKernelGetTickFreq() / MOTOR_GIMBAL_FREQ;
    const float control_period_s = 1.0F / (float)MOTOR_GIMBAL_FREQ;
    osDelay(MOTOR_GIMBAL_INIT_DELAY);

    uint32_t tick = osKernelGetTickCount();
    while (true)
    {
        // 读取 Ozone 中的速度目标和 GM6020 独立 PID 参数
        Gimbal_Input_t input;
        OzoneDebug_GetMotorGimbalInput(&input, control_period_s);

        // 读取 CAN 任务发布的最新 GM6020 反馈，并建立本周期模块反馈输入
        const bool feedback_received = MotorGimbal_ReadLatestFeedback();
        const Gimbal_Feedback_t feedback =
        {
            .valid = feedback_received &&
                     motor_gimbal_can_snapshot.gm6020_feedback_update_status == CAN_DEVICES_OK,
            .online = feedback_received && motor_gimbal_can_snapshot.gm6020_online,
            .actual_speed_rad_s = feedback_received ? motor_gimbal_can_snapshot.gm6020_speed_rad_s : 0.0F,
        };

        // 执行速度闭环，禁用、离线或非法输入时模块输出零电流
        Gimbal_Output_t output;
        Gimbal_Run(&input, &feedback, &output, &motor_gimbal_control_snapshot);
        CANDevices_GimbalCommand_t command =
        {
            .sequence = motor_gimbal_can_snapshot.sequence,
            .current_a = output.current_command_a,
        };

        // 通过独立最新值邮箱交给 CAN 任务统一发送
        MotorGimbal_PublishCommand(&command);

        // 发布速度控制结果和 GM6020 物理反馈，供 Ozone 在线整定
        OzoneDebug_UpdateMotorGimbal(feedback_received ? &motor_gimbal_can_snapshot : NULL,
                                     &motor_gimbal_control_snapshot);

        tick += delay_tick;
        // 按绝对时间等待下一周期，避免控制频率随处理耗时漂移
        osDelayUntil(tick);
    }
}

/**
 * @brief 初始化云台 GM6020 速度控制任务
 *
 * @return 初始化成功返回 true
 */
bool Task_motor_gimbal_Init(void)
{
    motor_gimbal_can_snapshot = (CANDevices_Snapshot_t)
    {
        0,
    };
    const Gimbal_Status_t init_status = Gimbal_Init((float)MOTOR_GIMBAL_FREQ, &motor_gimbal_control_snapshot);
    OzoneDebug_UpdateMotorGimbal(NULL, &motor_gimbal_control_snapshot);
    return init_status == GIMBAL_OK;
}

/**
 * @brief 读取云台反馈邮箱中的最新快照
 *
 * @return 取到新快照返回 true，否则返回 false
 */
static bool MotorGimbal_ReadLatestFeedback(void)
{
    if (task_runtime.msgq.can_gimbal_feedback == NULL)
    {
        return false;
    }
    return osMessageQueueGet(task_runtime.msgq.can_gimbal_feedback, &motor_gimbal_can_snapshot, NULL, 0U) == osOK;
}

/**
 * @brief 发布最新 GM6020 电流命令
 *
 * @param[in] command 待发布的云台电流命令
 * @return 邮箱可用且写入成功返回 true
 */
static bool MotorGimbal_PublishCommand(const CANDevices_GimbalCommand_t *command)
{
    if (command == NULL || task_runtime.msgq.can_gimbal_command == NULL)
    {
        return false;
    }
    if (osMessageQueueReset(task_runtime.msgq.can_gimbal_command) != osOK)
    {
        return false;
    }
    return osMessageQueuePut(task_runtime.msgq.can_gimbal_command, command, 0U, 0U) == osOK;
}
