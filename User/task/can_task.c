#include "task/can_task.h"

#include "device/can_devices.h"
#include "task/ozone_debug.h"
#include "task/user_task.h"

#include <stddef.h>

/*
 * CAN 任务私有状态：
 * - can_devices_snapshot：保存 CAN 设备初始化、本周期反馈和电流提交结果。
 */
static CANDevices_Snapshot_t can_devices_snapshot;

static bool CANTask_ReadLatestCommand(CANDevices_Command_t *command);
static bool CANTask_PublishSnapshot(void);

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

    // 等待业务初始化完成后再进入 CAN 周期通信
    osDelay(CAN_TASK_INIT_DELAY);

    // 建立绝对周期基准，避免通信周期累计漂移
    uint32_t tick = osKernelGetTickCount();
    while (true)
    {
        CANDevices_Command_t command;

        // 只读取本周期可用的最新命令，邮箱为空或异常时保留四路零电流默认值
        const bool command_received = CANTask_ReadLatestCommand(&command);

        // 刷新全部设备反馈后应用命令，设备集合负责逐路校验和统一组发送
        CANDevices_UpdateFeedback(&can_devices_snapshot);
        if (!command_received)
        {
            command.sequence = can_devices_snapshot.sequence;
        }
        CANDevices_ApplyCurrent(&command, &can_devices_snapshot);

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
    // 初始化 CAN 总线和当前实际存在的四个底盘电机，允许单路注册失败后隔离运行
    CANDevices_Init(&can_devices_snapshot);
    OzoneDebug_UpdateCANDevicesInit(&can_devices_snapshot);
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
    if (task_runtime.msgq.can_feedback == NULL)
    {
        return false;
    }

    // 邮箱只保留最新设备状态，重置和写入任一失败都视为本周期发布失败
    return osMessageQueueReset(task_runtime.msgq.can_feedback) == osOK &&
           osMessageQueuePut(task_runtime.msgq.can_feedback, &can_devices_snapshot, 0U, 0U) == osOK;
}
