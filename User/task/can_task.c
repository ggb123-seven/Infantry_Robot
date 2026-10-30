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
static bool CANTask_PublishGimbalFeedback(uint8_t axis, osMessageQueueId_t queue);
static bool CANTask_PublishShootFeedback(uint8_t index, osMessageQueueId_t queue);
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
        Gimbal_Output_t gimbal_output =
        {
            0,
        };
        if (task_runtime.msgq.gimbal_output == NULL ||
            osMessageQueueGet(task_runtime.msgq.gimbal_output, &gimbal_output, NULL, 0U) != osOK)
        {
            gimbal_output.yaw_current_a = 0.0f;
            gimbal_output.pitch_current_a = 0.0f;
        }
        // 两轴独立电流交给设备层写入同组槽位，再统一提交一帧
        gimbal_output_status = CANDevices_ApplyGimbalCurrent(gimbal_output.yaw_current_a,
                                                           gimbal_output.pitch_current_a);

        // 分别消费两个业务模块的新输出，缺少任一路消息时只清零对应业务电流
        Shoot_Output_t shoot_output =
        {
            0,
        };
        Trigger_Output_t trigger_output =
        {
            0,
        };
        if (task_runtime.msgq.shoot_output == NULL ||
            osMessageQueueGet(task_runtime.msgq.shoot_output, &shoot_output, NULL, 0U) != osOK)
        {
            shoot_output.current_a[0] = 0.0f;
            shoot_output.current_a[1] = 0.0f;
        }
        // 拨弹邮箱与摩擦轮邮箱各有唯一消费者，任何读取失败均不沿用旧电流
        if (task_runtime.msgq.trigger_output == NULL ||
            osMessageQueueGet(task_runtime.msgq.trigger_output, &trigger_output, NULL, 0U) != osOK)
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

    // 发射机构三路独立分发，拨弹与摩擦轮任务不会争抢同一反馈消息
    if (!CANTask_PublishShootFeedback(CAN_DEVICES_SHOOT_FRIC1, task_runtime.msgq.shoot_fric1_feedback))
    {
        success = false;
    }
    if (!CANTask_PublishShootFeedback(CAN_DEVICES_SHOOT_FRIC2, task_runtime.msgq.shoot_fric2_feedback))
    {
        success = false;
    }
    if (!CANTask_PublishShootFeedback(CAN_DEVICES_SHOOT_TRIGGER, task_runtime.msgq.trigger_feedback))
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

/**
 * @brief 发布单路摩擦轮或拨弹电机的新反馈
 *
 * @param[in] index 发射机构电机索引，0、1 为摩擦轮，2 为拨弹
 * @param[in] queue 该电机独占的容量为 1 的 MOTOR_RM_Feedback_t 邮箱
 * @return 新帧发布成功返回 true，无新帧或邮箱操作失败返回 false
 */
static bool CANTask_PublishShootFeedback(uint8_t index, osMessageQueueId_t queue)
{
    MOTOR_RM_Feedback_t feedback;

    // 只发布本周期取得的新帧，过期判定由消费模块使用源时间完成
    if (queue == NULL || CANDevices_UpdateShootFeedback(index, &feedback) != CAN_DEVICES_OK)
    {
        return false;
    }

    // 邮箱只保存最新反馈，写入失败时不刷新消费者的本地源时间
    return osMessageQueueReset(queue) == osOK && osMessageQueuePut(queue, &feedback, 0U, 0U) == osOK;
}
