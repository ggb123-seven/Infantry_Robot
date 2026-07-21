#include "task/user_task.h"

#include "device/can_devices.h"
#include "device/dr16.h"
#include "task/can_task.h"
#include "task/dr16_task.h"
#include "task/motor_chassis.h"

#include <stdbool.h>
#include <stddef.h>

static bool Task_InitCreateObjects(void);
static bool Task_InitModules(void);
static bool Task_InitReleaseObjects(void);

/**
 * @brief 创建业务任务和消息队列
 *
 * @param[in] argument 任务参数，本任务不使用
 * @return 本任务不会返回
 */
void Task_Init(void *argument)
{
    (void)argument;
    task_runtime.init_status = TASK_INIT_NOT_STARTED;

    // 在锁定调度器前初始化需要使用 RTOS 同步对象的业务模块，避免锁定期间发生阻塞等待
    if (!Task_InitModules())
    {
        osThreadExit();
    }

    // 锁定任务调度，确保所有业务对象创建并检查完成前没有业务任务运行
    if (osKernelLock() < 0)
    {
        task_runtime.init_status = TASK_INIT_KERNEL_LOCK_FAILED;
        osThreadExit();
    }

    // 集中创建 RTOS 对象，任一失败时删除已创建对象
    if (!Task_InitCreateObjects())
    {
        if (!Task_InitReleaseObjects())
        {
            task_runtime.init_status = TASK_INIT_CLEANUP_FAILED;
        }

        // 清理完成后恢复调度，避免任何残留的部分业务系统继续启动
        if (osKernelUnlock() < 0)
        {
            task_runtime.init_status = TASK_INIT_KERNEL_UNLOCK_FAILED;
        }
        osThreadExit();
    }

    task_runtime.init_status = TASK_INIT_OK;

    // 所有对象就绪后恢复调度，让业务任务从一致的运行时状态开始执行
    if (osKernelUnlock() < 0)
    {
        task_runtime.init_status = TASK_INIT_KERNEL_UNLOCK_FAILED;
    }
    osThreadExit();
}

/**
 * @brief 按依赖顺序创建 RTOS 对象
 *
 * @return 全部对象创建成功时返回 true，否则返回 false
 */
static bool Task_InitCreateObjects(void)
{
    // 先创建 DR16 状态邮箱，保证 DR16 任务开始运行时发布目标已经存在
    task_runtime.init_status = TASK_INIT_DR16_MAILBOX_FAILED;
    task_runtime.msgq.dr16_state = osMessageQueueNew(1U, sizeof(DR16_State_t), NULL);
    if (task_runtime.msgq.dr16_state == NULL)
    {
        return false;
    }

    // 创建 CAN 反馈邮箱，保证 CAN task 启动后已有明确发布目标
    task_runtime.init_status = TASK_INIT_CAN_FEEDBACK_MAILBOX_FAILED;
    task_runtime.msgq.can_feedback = osMessageQueueNew(1U, sizeof(CANDevices_Snapshot_t), NULL);
    if (task_runtime.msgq.can_feedback == NULL)
    {
        return false;
    }

    // 创建 CAN 命令邮箱，保证底盘 task 启动后已有明确发布目标
    task_runtime.init_status = TASK_INIT_CAN_COMMAND_MAILBOX_FAILED;
    task_runtime.msgq.can_command = osMessageQueueNew(1U, sizeof(CANDevices_Command_t), NULL);
    if (task_runtime.msgq.can_command == NULL)
    {
        return false;
    }

    // 先创建 CAN 通信任务，使其优先发布底盘控制所需的设备反馈
    task_runtime.init_status = TASK_INIT_CAN_THREAD_FAILED;
    task_runtime.thread.can = osThreadNew(Task_can, NULL, &attr_can);
    if (task_runtime.thread.can == NULL)
    {
        return false;
    }

    // 创建底盘任务并检查句柄，当前 DR16 状态不会连接到底盘控制
    task_runtime.init_status = TASK_INIT_MOTOR_THREAD_FAILED;
    task_runtime.thread.motor_chassis = osThreadNew(Task_motor_chassis, NULL, &attr_motor_chassis);
    if (task_runtime.thread.motor_chassis == NULL)
    {
        return false;
    }

    // 最后创建 DR16 接收任务，全部对象成功后才由调用方恢复调度
    task_runtime.init_status = TASK_INIT_DR16_THREAD_FAILED;
    task_runtime.thread.dr16 = osThreadNew(Task_dr16, NULL, &attr_dr16);
    return task_runtime.thread.dr16 != NULL;
}

/**
 * @brief 按任务依赖顺序初始化业务模块
 *
 * @return 全部业务模块初始化成功时返回 true，否则返回 false
 */
static bool Task_InitModules(void)
{
    // 先初始化 CAN 设备集合，失败时禁止任何依赖设备反馈的业务任务启动
    task_runtime.init_status = TASK_INIT_CAN_DEVICES_FAILED;
    if (!Task_can_Init())
    {
        return false;
    }

    // CAN 总线可运行后初始化底盘速度控制器，失败时不创建业务任务
    task_runtime.init_status = TASK_INIT_MOTOR_CHASSIS_FAILED;
    return Task_motor_chassis_Init();
}

/**
 * @brief 删除初始化失败前已经创建的全部业务 RTOS 对象
 *
 * @return 所有存在的对象均删除成功时返回 true，否则返回 false
 */
static bool Task_InitReleaseObjects(void)
{
    bool success = true;

    // 先终止尚未获得调度机会的业务任务，再删除它们依赖的状态邮箱
    if (task_runtime.thread.dr16 != NULL)
    {
        if (osThreadTerminate(task_runtime.thread.dr16) != osOK)
        {
            success = false;
        }
        task_runtime.thread.dr16 = NULL;
    }
    if (task_runtime.thread.motor_chassis != NULL)
    {
        if (osThreadTerminate(task_runtime.thread.motor_chassis) != osOK)
        {
            success = false;
        }
        task_runtime.thread.motor_chassis = NULL;
    }
    if (task_runtime.thread.can != NULL)
    {
        if (osThreadTerminate(task_runtime.thread.can) != osOK)
        {
            success = false;
        }
        task_runtime.thread.can = NULL;
    }
    if (task_runtime.msgq.can_command != NULL)
    {
        if (osMessageQueueDelete(task_runtime.msgq.can_command) != osOK)
        {
            success = false;
        }
        task_runtime.msgq.can_command = NULL;
    }
    if (task_runtime.msgq.can_feedback != NULL)
    {
        if (osMessageQueueDelete(task_runtime.msgq.can_feedback) != osOK)
        {
            success = false;
        }
        task_runtime.msgq.can_feedback = NULL;
    }
    if (task_runtime.msgq.dr16_state != NULL)
    {
        if (osMessageQueueDelete(task_runtime.msgq.dr16_state) != osOK)
        {
            success = false;
        }
        task_runtime.msgq.dr16_state = NULL;
    }
    return success;
}
