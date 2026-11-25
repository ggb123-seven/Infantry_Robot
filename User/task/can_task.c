/*
 * CAN 任务
 *
 * 只做总线路由：把设备反馈分投到各业务邮箱，再汇聚底盘、云台、摩擦轮与拨弹的输出统一发送
 * 本任务不实现任何业务控制策略
 */

#include "task/can_task.h"

#include "device/can_devices.h"
#include "module/gimbal.h"
#include "module/shoot.h"
#include "task/user_task.h"

#include <stddef.h>

// CAN 设备观测变量，声明见 can_task.h
CANDevices_Snapshot_t can_devices_snapshot;

static bool CANTask_ReadLatestCommand(CANDevices_Command_t *command);
static bool CANTask_PublishSnapshot(void);
static bool CANTask_PublishGimbalFeedback(uint8_t axis, osMessageQueueId_t queue);
static int8_t CANTask_PublishShootFeedback(uint8_t index, osMessageQueueId_t queue);

// 独立保留云台发送结果，避免覆盖底盘快照的发送状态
static int8_t gimbal_output_status = CAN_DEVICES_NOT_INITIALIZED;
// 独立记录三路发射反馈结果，设备注册失败或丢帧不改变底盘和云台状态
static int8_t shoot_feedback_status[CAN_DEVICES_SHOOT_MOTOR_COUNT] =
{
    CAN_DEVICES_NOT_INITIALIZED,
    CAN_DEVICES_NOT_INITIALIZED,
    CAN_DEVICES_NOT_INITIALIZED,
};
// 独立记录三路发射合并发送结果，不与底盘和云台发送状态混用
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
    tick += delay_tick;
    // 首次也等待完整周期再开始收发，避免启动阶段与其他任务抢占总线
    osDelayUntil(tick);
    while (true)
    {
        // 第一段：底盘命令、反馈与电流保持原有同组一致提交
        CANDevices_Command_t command;
        // 只读取本周期可用的最新命令，邮箱为空或异常时保留四路零电流默认值
        const bool command_received = CANTask_ReadLatestCommand(&command);
        // 刷新四路底盘反馈后应用本周期电流命令，保持同组命令一致提交
        CANDevices_UpdateFeedback(&can_devices_snapshot);
        if (!command_received)
        {
            command.sequence = can_devices_snapshot.sequence;
        }
        // 提交底盘四路电流，0x200 帧组与发射机构缓存互不影响
        CANDevices_ApplyCurrent(&command, &can_devices_snapshot);

        // 第二段：三路发射反馈分投独立邮箱，只有取得新帧才覆盖邮箱最新值
        shoot_feedback_status[CAN_DEVICES_SHOOT_FRIC1] =
            CANTask_PublishShootFeedback(CAN_DEVICES_SHOOT_FRIC1, task_runtime.msgq.shoot_fric1_feedback);
        shoot_feedback_status[CAN_DEVICES_SHOOT_FRIC2] =
            CANTask_PublishShootFeedback(CAN_DEVICES_SHOOT_FRIC2, task_runtime.msgq.shoot_fric2_feedback);
        shoot_feedback_status[CAN_DEVICES_SHOOT_TRIGGER] =
            CANTask_PublishShootFeedback(CAN_DEVICES_SHOOT_TRIGGER, task_runtime.msgq.trigger_feedback);

        // 第三段：只使用本周期取得的业务输出，邮箱为空或读取失败时对应电流按零处理
        Shoot_Output_t shoot_output;
        if (task_runtime.msgq.shoot_output == NULL ||
            osMessageQueueGet(task_runtime.msgq.shoot_output, &shoot_output, NULL, 0U) != osOK)
        {
            shoot_output.current_a[CAN_DEVICES_SHOOT_FRIC1] = 0.0f;
            shoot_output.current_a[CAN_DEVICES_SHOOT_FRIC2] = 0.0f;
        }
        float trigger_current_a = 0.0f;
        if (task_runtime.msgq.trigger_output == NULL ||
            osMessageQueueGet(task_runtime.msgq.trigger_output, &trigger_current_a, NULL, 0U) != osOK)
        {
            trigger_current_a = 0.0f;
        }
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

        // 第四段：两路摩擦轮与拨弹电流合并为一帧 0x1FF，一次调用只发送一帧
        const float fric1_current_a = shoot_output.current_a[CAN_DEVICES_SHOOT_FRIC1];
        const float fric2_current_a = shoot_output.current_a[CAN_DEVICES_SHOOT_FRIC2];
        shoot_output_status = CANDevices_ApplyShootCurrent(fric1_current_a, fric2_current_a, trigger_current_a);
        // 第五段：两轴云台电流交给设备层写入同组槽位，再统一提交 0x2FE 帧
        gimbal_output_status =
            CANDevices_ApplyGimbalCurrent(gimbal_output.yaw_current_a, gimbal_output.pitch_current_a);
        // 第六段：发布包含本周期反馈与实际输出的最新设备快照
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
    // 初始化两条 CAN 总线及底盘、云台与发射机构电机，允许单路注册失败后隔离运行
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
    bool success = osMessageQueueReset(queue) == osOK && osMessageQueuePut(queue, &feedback, 0U, 0U) == osOK;

    // 同一有效 yaw 帧另投底盘独占邮箱，云台邮箱失败不阻止底盘跟随
    if (axis == CAN_DEVICES_GIMBAL_YAW)
    {
        const osMessageQueueId_t chassis_queue = task_runtime.msgq.chassis_yaw_feedback;
        if (chassis_queue == NULL || osMessageQueueReset(chassis_queue) != osOK ||
            osMessageQueuePut(chassis_queue, &feedback, 0U, 0U) != osOK)
        {
            success = false;
        }
    }
    return success;
}

/**
 * @brief 将单路发射机构新反馈发布到对应的最新值邮箱
 *
 * @param[in] index 发射机构电机索引，0、1 为摩擦轮，2 为拨弹
 * @param[in] queue 对应电机的容量为 1 的 MOTOR_RM_Feedback_t 邮箱
 * @return 取得新帧且发布成功返回 CAN_DEVICES_OK，无新帧或邮箱操作失败返回设备错误码
 */
static int8_t CANTask_PublishShootFeedback(uint8_t index, osMessageQueueId_t queue)
{
    MOTOR_RM_Feedback_t feedback;

    // 只接纳新帧，消费者根据反馈中的微秒时间戳识别离线与过期数据
    const int8_t status = CANDevices_UpdateShootFeedback(index, &feedback);
    if (status != CAN_DEVICES_OK)
    {
        return status;
    }

    // 邮箱保留最新一帧，邮箱未创建或写入失败按发送失败记录
    if (queue == NULL || osMessageQueueReset(queue) != osOK || osMessageQueuePut(queue, &feedback, 0U, 0U) != osOK)
    {
        return CAN_DEVICES_ERROR;
    }
    return CAN_DEVICES_OK;
}
