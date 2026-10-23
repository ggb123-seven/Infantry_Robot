#include "task/can_task.h"

#include "device/can_devices.h"
#include "task/user_task.h"

#include <stddef.h>

// CAN 设备观测变量，声明见 can_task.h
CANDevices_Snapshot_t can_devices_snapshot;

static bool CANTask_ReadLatestCommand(CANDevices_Command_t *command);
static bool CANTask_PublishSnapshot(void);
static bool CANTask_PublishGimbalFeedback(uint8_t axis, osMessageQueueId_t queue);
static int8_t gimbal_output_status = CAN_DEVICES_NOT_INITIALIZED;// 独立保留云台发送结果，避免覆盖底盘快照的发送状态

/**
 * @brief 运行 CAN 设备反馈采集和电流命令发送任务
 *
 * @param[in] argument 任务参数，本任务不使用
 * @return 本任务不会返回
 */
void Task_can(void *argument)
{
    (void)argument;
    const uint32_t delay_tick = osKernelGetTickFreq() / CAN_TASK_FREQ;
    osDelay(CAN_TASK_INIT_DELAY);// 等待业务初始化完成后再进入 CAN 周期通信

    // 建立绝对周期基准，避免通信周期累计漂移
    uint32_t tick = osKernelGetTickCount();
    while (true)
    {
        CANDevices_Command_t command;
        // 只读取本周期可用的最新命令，邮箱为空或异常时保留四路零电流默认值
        const bool command_received = CANTask_ReadLatestCommand(&command);
        // 刷新四路底盘反馈后应用本周期电流命令，保持同组命令一致提交
        CANDevices_UpdateFeedback(&can_devices_snapshot);
        if (!command_received)
        {
            command.sequence = can_devices_snapshot.sequence;
        }
        CANDevices_ApplyCurrent(&command, &can_devices_snapshot);
        // 只使用本周期取得的 yaw 电流，邮箱为空或读取失败时立即回到零输出
        float yaw_current_a = 0.0f;
        if (task_runtime.msgq.gimbal_yaw_command == NULL ||
            osMessageQueueGet(task_runtime.msgq.gimbal_yaw_command, &yaw_current_a, NULL, 0U) != osOK)
        {
            yaw_current_a = 0.0f;
        }
        // 两轴缓存统一组帧，yaw 使用速度环输出，pitch 保持零电流
        gimbal_output_status = CANDevices_ApplyGimbalCurrent(yaw_current_a);
        // 发布包含本周期反馈与实际输出的最新设备快照
        CANTask_PublishSnapshot();
        tick += delay_tick;
        // 等待至下一个绝对任务周期，保持稳定 CAN 通信频率
        osDelayUntil(tick);
    }
}

/**
 * @brief 初始化 CAN 设备集合
 *
 * @return CAN 总线可运行时返回 true，否则返回 false
 */
bool Task_can_Init(void)
{
    // 初始化两条 CAN 总线及底盘、云台电机，允许单路注册失败后隔离运行
    CANDevices_Init(&can_devices_snapshot);
    return can_devices_snapshot.initialized;
}

/**
 * @brief 从容量为 1 的命令邮箱读取最新电流命令
 *
 * @param[out] command 本周期电流命令，读取失败时四路电流均为 0
 * @return 成功取得新命令时返回 true，否则返回 false
 */
static bool CANTask_ReadLatestCommand(CANDevices_Command_t *command)
{
    *command = (CANDevices_Command_t)
    {
        0,
    };
    if (task_runtime.msgq.can_command == NULL)
    {
        return false;
    }
    return osMessageQueueGet(task_runtime.msgq.can_command, command, NULL, 0U) == osOK;
}

/**
 * @brief 向容量为 1 的反馈邮箱发布最新 CAN 设备快照
 *
 * @return 邮箱重置并写入成功时返回 true，否则返回 false
 */
static bool CANTask_PublishSnapshot(void)
{
    bool success = true;

    // 底盘邮箱只保留最新设备状态，重置和写入失败均记录为发布失败
    if (task_runtime.msgq.can_feedback == NULL ||
        osMessageQueueReset(task_runtime.msgq.can_feedback) != osOK ||
        osMessageQueuePut(task_runtime.msgq.can_feedback, &can_devices_snapshot, 0U, 0U) != osOK)
    {
        success = false;
    }

    // 两轴分别发布最新有效反馈，任一轴丢帧不会覆盖另一轴邮箱
    if (!CANTask_PublishGimbalFeedback(CAN_DEVICES_GIMBAL_YAW, task_runtime.msgq.gimbal_yaw_feedback))
    {
        success = false;
    }
    if (!CANTask_PublishGimbalFeedback(CAN_DEVICES_GIMBAL_PITCH, task_runtime.msgq.gimbal_pitch_feedback))
    {
        success = false;
    }
    return success;
}

/**
 * @brief 将单轴新反馈发布到对应的最新值邮箱
 *
 * @param[in] axis 云台轴索引，yaw 为 0，pitch 为 1
 * @param[in] queue 对应轴的容量为 1 的 MOTOR_RM_Feedback_t 邮箱
 * @return 新反馈发布成功时返回 true，没有新反馈或邮箱操作失败时返回 false
 */
static bool CANTask_PublishGimbalFeedback(uint8_t axis, osMessageQueueId_t queue)
{
    MOTOR_RM_Feedback_t feedback;

    // 只接纳新帧，消费者根据反馈中的微秒时间戳识别离线与过期数据
    if (queue == NULL || CANDevices_UpdateGimbalFeedback(axis, &feedback) != CAN_DEVICES_OK)
    {
        return false;
    }

    // 邮箱保留最新一帧，清空及写入的结果均交给调用方检查
    return osMessageQueueReset(queue) == osOK && osMessageQueuePut(queue, &feedback, 0U, 0U) == osOK;
}
