#include "task/can_task.h"

#include "device/can_devices.h"
#include "module/gimbal.h"
#include "module/shoot.h"
#include "module/trigger.h"
#include "task/user_task.h"

#include <stddef.h>

// CAN 设备观测变量，声明见 can_task.h
CANDevices_Snapshot_t can_devices_snapshot;

static bool CANTask_ReadLatestCommand(CANDevices_Command_t *command);
static bool CANTask_PublishSnapshot(void);
// 独立保留云台发送结果，避免覆盖底盘快照的发送状态
static int8_t gimbal_output_status = CAN_DEVICES_NOT_INITIALIZED;
// 发射机构独立记录发送结果，不覆盖底盘和云台的诊断
static int8_t shoot_output_status = CAN_DEVICES_NOT_INITIALIZED;

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
        // 刷新四路底盘反馈后应用本周期电流命令，保持同组命令一致提交
        CANDevices_UpdateFeedback(&can_devices_snapshot);
        if (!command_received)
        {
            command.sequence = can_devices_snapshot.sequence;
        }
        CANDevices_ApplyCurrent(&command, &can_devices_snapshot);
        // 只使用本周期取得的云台输出，邮箱为空或读取失败时立即回到零输出
        Gimbal_Output_t gimbal_output;
        if (osMessageQueueGet(task_runtime.msgq.gimbal_output, &gimbal_output, NULL, 0U) != osOK)
        {
            gimbal_output.yaw_current_a = 0.0f;
            gimbal_output.pitch_current_a = 0.0f;
        }
        // 两轴独立电流交给设备层写入同组槽位，再统一提交一帧
        gimbal_output_status = CANDevices_ApplyGimbalCurrent(gimbal_output.yaw_current_a,
                                                           gimbal_output.pitch_current_a);

        // 分别消费两个业务模块的新输出，缺少任一路消息时只清零对应业务电流
        Shoot_Output_t shoot_output;
        Trigger_Output_t trigger_output;
        if (osMessageQueueGet(task_runtime.msgq.shoot_output, &shoot_output, NULL, 0U) != osOK)
        {
            shoot_output.current_a[0] = 0.0f;
            shoot_output.current_a[1] = 0.0f;
        }
        // 拨弹邮箱与摩擦轮邮箱各有唯一消费者，任何读取失败均不沿用旧电流
        if (osMessageQueueGet(task_runtime.msgq.trigger_output, &trigger_output, NULL, 0U) != osOK)
        {
            trigger_output.current_a = 0.0f;
        }
        // 写完 ID5、ID6、ID7 后统一提交一帧 0x1FF，禁止两个业务任务各自发送整帧
        shoot_output_status = CANDevices_ApplyShootCurrent(shoot_output.current_a[0],
            shoot_output.current_a[1], trigger_output.current_a);
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
    // 初始化两条总线及底盘、云台、摩擦轮与拨弹电机，允许单路注册失败后隔离运行
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
    if (osMessageQueueReset(task_runtime.msgq.can_feedback) != osOK ||
        osMessageQueuePut(task_runtime.msgq.can_feedback, &can_devices_snapshot, 0U, 0U) != osOK)
    {
        success = false;
    }

    // 队列按设备索引排列：云台 yaw、pitch，随后为摩擦轮 ID5、ID6 和拨弹 ID7
    const osMessageQueueId_t queues[CAN_DEVICES_GIMBAL_MOTOR_COUNT + CAN_DEVICES_SHOOT_MOTOR_COUNT] =
    {
        task_runtime.msgq.gimbal_yaw_feedback,
        task_runtime.msgq.gimbal_pitch_feedback,
        task_runtime.msgq.shoot_fric1_feedback,
        task_runtime.msgq.shoot_fric2_feedback,
        task_runtime.msgq.trigger_feedback,
    };
    for (uint8_t index = 0U; index < sizeof(queues) / sizeof(queues[0]); index++)
    {
        MOTOR_RM_Feedback_t feedback;
        const int8_t status = index < CAN_DEVICES_GIMBAL_MOTOR_COUNT ?
            CANDevices_UpdateGimbalFeedback(index, &feedback) :
            CANDevices_UpdateShootFeedback(index - CAN_DEVICES_GIMBAL_MOTOR_COUNT, &feedback);

        // 各路仅发布新帧，单路无新帧或邮箱失败仍继续分发其他路
        if (status != CAN_DEVICES_OK || osMessageQueueReset(queues[index]) != osOK ||
            osMessageQueuePut(queues[index], &feedback, 0U, 0U) != osOK)
        {
            success = false;
        }
    }
    return success;
}
