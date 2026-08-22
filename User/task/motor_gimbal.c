#include "task/motor_gimbal.h"

#include "device/can_devices.h"
#include "task/ozone_debug.h"
#include "task/user_task.h"

#include <math.h>
#include <stddef.h>

// GM6020 往复测试电流为 0.1 A，每个方向持续 10000 ms
#define MOTOR_GIMBAL_TEST_CURRENT_A (0.1F)
#define MOTOR_GIMBAL_TEST_DIRECTION_TIME_MS (10000U)

// 测试状态仅包含正向和反向，非法状态在状态机默认分支恢复为正向并输出零电流
typedef enum
{
    MOTOR_GIMBAL_TEST_STATE_FORWARD = 0,
    MOTOR_GIMBAL_TEST_STATE_REVERSE,
} MotorGimbal_TestState_t;

/*
 * 云台任务只保存最新 CAN 快照，不直接访问 GM6020 私有参数或 CAN 外设
 */
static CANDevices_Snapshot_t motor_gimbal_snapshot;
static MotorGimbal_TestState_t motor_gimbal_test_state;
static uint32_t motor_gimbal_test_state_enter_tick;

static bool MotorGimbal_ReadLatestFeedback(void);
static bool MotorGimbal_PublishCommand(const CANDevices_GimbalCommand_t *command);
static float MotorGimbal_RunTestStateMachine(uint32_t now_tick);
static float MotorGimbal_LimitCurrent(float target_current_a, bool enabled, bool feedback_received);

/**
 * @brief 周期读取 GM6020 反馈并发布电流命令
 *
 * @param[in] argument 任务参数，本任务不使用
 * @return 本任务不会返回
 */
void Task_motor_gimbal(void *argument)
{
    (void)argument;

    const uint32_t delay_tick = osKernelGetTickFreq() / MOTOR_GIMBAL_FREQ;
    osDelay(MOTOR_GIMBAL_INIT_DELAY);

    uint32_t tick = osKernelGetTickCount();
    while (true)
    {
        const bool enabled = true;

        // 读取 CAN 任务发布的最新 GM6020 反馈，队列无新数据时保持安全状态
        const bool feedback_received = MotorGimbal_ReadLatestFeedback();

        // 驱动正反转测试状态机，每 10 秒切换一次目标电流方向
        const float target_current_a = MotorGimbal_RunTestStateMachine(osKernelGetTickCount());

        // 仅在测试启用、反馈在线且目标值有限时生成受限电流命令
        float limited_current_a = MotorGimbal_LimitCurrent(target_current_a, enabled, feedback_received);
        CANDevices_GimbalCommand_t command =
        {
            .sequence = motor_gimbal_snapshot.sequence,
            .current_a = limited_current_a,
        };

        // 通过独立最新值邮箱交给 CAN 任务统一发送，发布失败时下一周期会回到零命令
        if (!MotorGimbal_PublishCommand(&command))
        {
            limited_current_a = 0.0F;
        }

        // 发布命令、反馈和发送结果，区分 Ozone 目标值与设备实际应用值
        OzoneDebug_UpdateMotorGimbal(feedback_received ? &motor_gimbal_snapshot : NULL, enabled,
                                     target_current_a, limited_current_a);

        tick += delay_tick;
        // 按绝对时间等待下一周期，避免控制频率随处理耗时漂移
        osDelayUntil(tick);
    }
}

/**
 * @brief 初始化云台 GM6020 电流测试任务
 *
 * @return 初始化成功返回 true
 */
bool Task_motor_gimbal_Init(void)
{
    motor_gimbal_snapshot = (CANDevices_Snapshot_t)
    {
        0,
    };
    motor_gimbal_test_state = MOTOR_GIMBAL_TEST_STATE_FORWARD;
    motor_gimbal_test_state_enter_tick = osKernelGetTickCount();
    OzoneDebug_UpdateMotorGimbal(NULL, false, 0.0F, 0.0F);
    return true;
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
    return osMessageQueueGet(task_runtime.msgq.can_gimbal_feedback, &motor_gimbal_snapshot, NULL, 0U) == osOK;
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

/**
 * @brief 执行 GM6020 正反转定时测试状态机
 *
 * @param[in] now_tick 当前 RTOS 节拍
 * @return 当前状态对应的目标电流，单位 A，非法状态返回 0 A
 */
static float MotorGimbal_RunTestStateMachine(uint32_t now_tick)
{
    const uint32_t state_duration_tick = (MOTOR_GIMBAL_TEST_DIRECTION_TIME_MS * osKernelGetTickFreq()) / 1000U;
    const bool state_timeout = (uint32_t)(now_tick - motor_gimbal_test_state_enter_tick) >= state_duration_tick;

    switch (motor_gimbal_test_state)
    {
        case MOTOR_GIMBAL_TEST_STATE_FORWARD:
            if (state_timeout)
            {
                motor_gimbal_test_state = MOTOR_GIMBAL_TEST_STATE_REVERSE;
                motor_gimbal_test_state_enter_tick = now_tick;
                return -MOTOR_GIMBAL_TEST_CURRENT_A;
            }
            return MOTOR_GIMBAL_TEST_CURRENT_A;

        case MOTOR_GIMBAL_TEST_STATE_REVERSE:
            if (state_timeout)
            {
                motor_gimbal_test_state = MOTOR_GIMBAL_TEST_STATE_FORWARD;
                motor_gimbal_test_state_enter_tick = now_tick;
                return MOTOR_GIMBAL_TEST_CURRENT_A;
            }
            return -MOTOR_GIMBAL_TEST_CURRENT_A;

        default:
            motor_gimbal_test_state = MOTOR_GIMBAL_TEST_STATE_FORWARD;
            motor_gimbal_test_state_enter_tick = now_tick;
            return 0.0F;
    }
}

/**
 * @brief 根据在线状态和协议范围限制 GM6020 电流
 *
 * @param[in] target_current_a 自动往复测试目标电流，单位 A
 * @param[in] enabled 电流测试启用状态
 * @param[in] feedback_received 本周期是否收到新反馈
 * @return 可发送的安全电流，单位 A
 */
static float MotorGimbal_LimitCurrent(float target_current_a, bool enabled, bool feedback_received)
{
    if (!enabled || !feedback_received || !motor_gimbal_snapshot.gm6020_online || !isfinite(target_current_a))
    {
        return 0.0F;
    }
    if (target_current_a > CAN_DEVICES_GM6020_CURRENT_LIMIT_A)
    {
        return CAN_DEVICES_GM6020_CURRENT_LIMIT_A;
    }
    if (target_current_a < -CAN_DEVICES_GM6020_CURRENT_LIMIT_A)
    {
        return -CAN_DEVICES_GM6020_CURRENT_LIMIT_A;
    }
    return target_current_a;
}
