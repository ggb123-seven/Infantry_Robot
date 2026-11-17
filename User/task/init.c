/*
 * 业务任务装配
 *
 * 按依赖顺序初始化模块并创建业务任务与消息队列
 * 全部对象就绪后解锁调度器，失败时回收已创建对象并退出
 */

/* Includes ----------------------------------------------------------------- */
#include "task/user_task.h"

#include "device/can_devices.h"
#include "device/dr16.h"
#include "module/gimbal.h"
#include "module/trigger.h"
#include "task/can_task.h"
#include "task/dr16_task.h"
#include "task/ins_task.h"
#include "task/motor_chassis.h"
#include "task/motor_gimbal.h"
#include "task/motor_shoot.h"
#include "task/motor_trigger.h"

#include <stdbool.h>
#include <stddef.h>

/* Private function --------------------------------------------------------- */
static bool Task_InitCreateObjects(void);
static bool Task_InitModules(void);
static bool Task_InitReleaseObjects(void);

/* Exported functions ------------------------------------------------------- */
/**
 * @brief 装配业务系统并创建任务与消息队列
 *
 * @param argument 任务参数，本任务不使用
 * @return 无返回值
 */
void Task_Init(void *argument)
{
    (void)argument;
    task_runtime.init_status = TASK_INIT_NOT_STARTED;

    /* 初始化需要 RTOS 同步对象的业务模块，必须早于调度器锁定 */
    if (!Task_InitModules())
    {
        osThreadExit();
    }

    /* 锁定任务调度，保证对象创建完成前没有业务任务运行 */
    if (osKernelLock() < 0)
    {
        task_runtime.init_status = TASK_INIT_KERNEL_LOCK_FAILED;
        osThreadExit();
    }

    /* 集中创建 RTOS 对象，任一失败时回收已创建对象 */
    if (!Task_InitCreateObjects())
    {
        if (!Task_InitReleaseObjects())
        {
            task_runtime.init_status = TASK_INIT_CLEANUP_FAILED;
        }

        /* 清理后恢复调度，避免残留的部分业务系统继续启动 */
        if (osKernelUnlock() < 0)
        {
            task_runtime.init_status = TASK_INIT_KERNEL_UNLOCK_FAILED;
        }
        osThreadExit();
    }

    task_runtime.init_status = TASK_INIT_OK;

    /* 全部对象就绪后恢复调度，让业务任务看到一致的运行时状态 */
    if (osKernelUnlock() < 0)
    {
        task_runtime.init_status = TASK_INIT_KERNEL_UNLOCK_FAILED;
    }
    osThreadExit();
}

/**
 * @brief 按依赖顺序创建 RTOS 对象
 *
 * @return bool 全部对象创建成功时返回 true，否则返回 false
 */
static bool Task_InitCreateObjects(void)
{
    /* 遥控状态邮箱先于 DR16 任务创建 */
    task_runtime.init_status = TASK_INIT_DR16_MAILBOX_FAILED;
    task_runtime.msgq.dr16_state = osMessageQueueNew(1U, sizeof(DR16_State_t), NULL);
    if (task_runtime.msgq.dr16_state == NULL)
    {
        return false;
    }

    /* 云台使用独立遥控邮箱，避免与底盘争抢同一条消息 */
    task_runtime.init_status = TASK_INIT_DR16_GIMBAL_MAILBOX_FAILED;
    task_runtime.msgq.dr16_gimbal_state = osMessageQueueNew(1U, sizeof(DR16_State_t), NULL);
    if (task_runtime.msgq.dr16_gimbal_state == NULL)
    {
        return false;
    }

    /* CAN 反馈与命令邮箱供底盘和 CAN 任务配对使用 */
    task_runtime.init_status = TASK_INIT_CAN_FEEDBACK_MAILBOX_FAILED;
    task_runtime.msgq.can_feedback = osMessageQueueNew(1U, sizeof(CANDevices_Snapshot_t), NULL);
    if (task_runtime.msgq.can_feedback == NULL)
    {
        return false;
    }

    task_runtime.init_status = TASK_INIT_CAN_COMMAND_MAILBOX_FAILED;
    task_runtime.msgq.can_command = osMessageQueueNew(1U, sizeof(CANDevices_Command_t), NULL);
    if (task_runtime.msgq.can_command == NULL)
    {
        return false;
    }

    /* 两轴反馈、云台输出与惯性快照邮箱先于消费者任务创建 */
    task_runtime.init_status = TASK_INIT_GIMBAL_YAW_FEEDBACK_MAILBOX_FAILED;
    task_runtime.msgq.gimbal_yaw_feedback = osMessageQueueNew(1U, sizeof(MOTOR_RM_Feedback_t), NULL);
    if (task_runtime.msgq.gimbal_yaw_feedback == NULL)
    {
        return false;
    }
    task_runtime.init_status = TASK_INIT_GIMBAL_PITCH_FEEDBACK_MAILBOX_FAILED;
    task_runtime.msgq.gimbal_pitch_feedback = osMessageQueueNew(1U, sizeof(MOTOR_RM_Feedback_t), NULL);
    if (task_runtime.msgq.gimbal_pitch_feedback == NULL)
    {
        return false;
    }

    task_runtime.init_status = TASK_INIT_GIMBAL_OUTPUT_MAILBOX_FAILED;
    task_runtime.msgq.gimbal_output = osMessageQueueNew(1U, sizeof(Gimbal_Output_t), NULL);
    if (task_runtime.msgq.gimbal_output == NULL)
    {
        return false;
    }

    /* 姿态和角速度共用邮箱，避免云台读到不同周期的数据 */
    task_runtime.init_status = TASK_INIT_GIMBAL_IMU_MAILBOX_FAILED;
    task_runtime.msgq.gimbal_imu = osMessageQueueNew(1U, sizeof(Gimbal_ImuFeedback_t), NULL);
    if (task_runtime.msgq.gimbal_imu == NULL)
    {
        return false;
    }

    /* 底盘在线状态与小陀螺状态共用邮箱 */
    task_runtime.init_status = TASK_INIT_CHASSIS_GIMBAL_MAILBOX_FAILED;
    task_runtime.msgq.chassis_gimbal = osMessageQueueNew(1U, sizeof(Gimbal_ChassisState_t), NULL);
    if (task_runtime.msgq.chassis_gimbal == NULL)
    {
        return false;
    }

    /* 底盘独占一份 yaw 反馈，避免与云台争抢同一邮箱 */
    task_runtime.init_status = TASK_INIT_CHASSIS_YAW_FEEDBACK_MAILBOX_FAILED;
    task_runtime.msgq.chassis_yaw_feedback = osMessageQueueNew(1U, sizeof(MOTOR_RM_Feedback_t), NULL);
    if (task_runtime.msgq.chassis_yaw_feedback == NULL)
    {
        return false;
    }

    /* 三路电机反馈与两个模块的命令、输出使用独立邮箱 */
    task_runtime.init_status = TASK_INIT_SHOOT_FRIC1_FEEDBACK_MAILBOX_FAILED;
    task_runtime.msgq.shoot_fric1_feedback = osMessageQueueNew(1U, sizeof(MOTOR_RM_Feedback_t), NULL);
    if (task_runtime.msgq.shoot_fric1_feedback == NULL)
    {
        return false;
    }

    task_runtime.init_status = TASK_INIT_SHOOT_FRIC2_FEEDBACK_MAILBOX_FAILED;
    task_runtime.msgq.shoot_fric2_feedback = osMessageQueueNew(1U, sizeof(MOTOR_RM_Feedback_t), NULL);
    if (task_runtime.msgq.shoot_fric2_feedback == NULL)
    {
        return false;
    }

    task_runtime.init_status = TASK_INIT_TRIGGER_FEEDBACK_MAILBOX_FAILED;
    task_runtime.msgq.trigger_feedback = osMessageQueueNew(1U, sizeof(MOTOR_RM_Feedback_t), NULL);
    if (task_runtime.msgq.trigger_feedback == NULL)
    {
        return false;
    }

    task_runtime.init_status = TASK_INIT_SHOOT_COMMAND_MAILBOX_FAILED;
    task_runtime.msgq.shoot_command = osMessageQueueNew(1U, sizeof(Shoot_Command_t), NULL);
    if (task_runtime.msgq.shoot_command == NULL)
    {
        return false;
    }

    task_runtime.init_status = TASK_INIT_TRIGGER_COMMAND_MAILBOX_FAILED;
    task_runtime.msgq.trigger_command = osMessageQueueNew(1U, sizeof(DR16_State_t), NULL);
    if (task_runtime.msgq.trigger_command == NULL)
    {
        return false;
    }

    task_runtime.init_status = TASK_INIT_SHOOT_OUTPUT_MAILBOX_FAILED;
    task_runtime.msgq.shoot_output = osMessageQueueNew(1U, sizeof(Shoot_Output_t), NULL);
    if (task_runtime.msgq.shoot_output == NULL)
    {
        return false;
    }

    task_runtime.init_status = TASK_INIT_TRIGGER_OUTPUT_MAILBOX_FAILED;
    task_runtime.msgq.trigger_output = osMessageQueueNew(1U, sizeof(Trigger_Output_t), NULL);
    if (task_runtime.msgq.trigger_output == NULL)
    {
        return false;
    }

    /* 邮箱就绪后按依赖顺序创建业务任务 */
    task_runtime.init_status = TASK_INIT_INS_THREAD_FAILED;
    task_runtime.thread.ins = osThreadNew(INS_Task, NULL, &attr_ins);
    if (task_runtime.thread.ins == NULL)
    {
        return false;
    }

    task_runtime.init_status = TASK_INIT_CAN_THREAD_FAILED;
    task_runtime.thread.can = osThreadNew(Task_can, NULL, &attr_can);
    if (task_runtime.thread.can == NULL)
    {
        return false;
    }

    task_runtime.init_status = TASK_INIT_MOTOR_THREAD_FAILED;
    task_runtime.thread.motor_chassis = osThreadNew(Task_motor_chassis, NULL, &attr_motor_chassis);
    if (task_runtime.thread.motor_chassis == NULL)
    {
        return false;
    }

    task_runtime.init_status = TASK_INIT_GIMBAL_THREAD_FAILED;
    task_runtime.thread.motor_gimbal = osThreadNew(Task_motor_gimbal, NULL, &attr_motor_gimbal);
    if (task_runtime.thread.motor_gimbal == NULL)
    {
        return false;
    }

    task_runtime.init_status = TASK_INIT_SHOOT_THREAD_FAILED;
    task_runtime.thread.motor_shoot = osThreadNew(Task_motor_shoot, NULL, &attr_motor_shoot);
    if (task_runtime.thread.motor_shoot == NULL)
    {
        return false;
    }

    task_runtime.init_status = TASK_INIT_TRIGGER_THREAD_FAILED;
    task_runtime.thread.motor_trigger = osThreadNew(Task_motor_trigger, NULL, &attr_motor_trigger);
    if (task_runtime.thread.motor_trigger == NULL)
    {
        return false;
    }

    /* 最后创建 DR16 接收任务，全部对象成功后才恢复调度 */
    task_runtime.init_status = TASK_INIT_DR16_THREAD_FAILED;
    task_runtime.thread.dr16 = osThreadNew(Task_dr16, NULL, &attr_dr16);
    return task_runtime.thread.dr16 != NULL;
}

/**
 * @brief 按任务依赖顺序初始化业务模块
 *
 * @return bool 全部业务模块初始化成功时返回 true，否则返回 false
 */
static bool Task_InitModules(void)
{
    /* 先初始化 CAN 设备集合，失败时禁止依赖设备反馈的业务任务启动 */
    task_runtime.init_status = TASK_INIT_CAN_DEVICES_FAILED;
    if (!Task_can_Init())
    {
        return false;
    }

    return true;
}

/**
 * @brief 删除初始化失败前已经创建的全部业务 RTOS 对象
 *
 * @return bool 所有存在的对象均删除成功时返回 true，否则返回 false
 */
static bool Task_InitReleaseObjects(void)
{
    bool success = true;

    /* 先终止尚未获得调度机会的业务任务，再删除其依赖的邮箱 */
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
    if (task_runtime.thread.motor_gimbal != NULL)
    {
        if (osThreadTerminate(task_runtime.thread.motor_gimbal) != osOK)
        {
            success = false;
        }
        task_runtime.thread.motor_gimbal = NULL;
    }
    if (task_runtime.thread.motor_trigger != NULL)
    {
        if (osThreadTerminate(task_runtime.thread.motor_trigger) != osOK)
        {
            success = false;
        }
        task_runtime.thread.motor_trigger = NULL;
    }
    if (task_runtime.thread.motor_shoot != NULL)
    {
        if (osThreadTerminate(task_runtime.thread.motor_shoot) != osOK)
        {
            success = false;
        }
        task_runtime.thread.motor_shoot = NULL;
    }
    if (task_runtime.thread.can != NULL)
    {
        if (osThreadTerminate(task_runtime.thread.can) != osOK)
        {
            success = false;
        }
        task_runtime.thread.can = NULL;
    }
    if (task_runtime.thread.ins != NULL)
    {
        if (osThreadTerminate(task_runtime.thread.ins) != osOK)
        {
            success = false;
        }
        task_runtime.thread.ins = NULL;
    }
    /* 业务任务终止后统一回收其反馈、命令与输出邮箱 */
    if (task_runtime.msgq.trigger_output != NULL)
    {
        if (osMessageQueueDelete(task_runtime.msgq.trigger_output) != osOK)
        {
            success = false;
        }
        task_runtime.msgq.trigger_output = NULL;
    }
    if (task_runtime.msgq.shoot_output != NULL)
    {
        if (osMessageQueueDelete(task_runtime.msgq.shoot_output) != osOK)
        {
            success = false;
        }
        task_runtime.msgq.shoot_output = NULL;
    }
    if (task_runtime.msgq.trigger_command != NULL)
    {
        if (osMessageQueueDelete(task_runtime.msgq.trigger_command) != osOK)
        {
            success = false;
        }
        task_runtime.msgq.trigger_command = NULL;
    }
    if (task_runtime.msgq.shoot_command != NULL)
    {
        if (osMessageQueueDelete(task_runtime.msgq.shoot_command) != osOK)
        {
            success = false;
        }
        task_runtime.msgq.shoot_command = NULL;
    }
    if (task_runtime.msgq.trigger_feedback != NULL)
    {
        if (osMessageQueueDelete(task_runtime.msgq.trigger_feedback) != osOK)
        {
            success = false;
        }
        task_runtime.msgq.trigger_feedback = NULL;
    }
    if (task_runtime.msgq.shoot_fric2_feedback != NULL)
    {
        if (osMessageQueueDelete(task_runtime.msgq.shoot_fric2_feedback) != osOK)
        {
            success = false;
        }
        task_runtime.msgq.shoot_fric2_feedback = NULL;
    }
    if (task_runtime.msgq.shoot_fric1_feedback != NULL)
    {
        if (osMessageQueueDelete(task_runtime.msgq.shoot_fric1_feedback) != osOK)
        {
            success = false;
        }
        task_runtime.msgq.shoot_fric1_feedback = NULL;
    }
    if (task_runtime.msgq.chassis_yaw_feedback != NULL)
    {
        if (osMessageQueueDelete(task_runtime.msgq.chassis_yaw_feedback) != osOK)
        {
            success = false;
        }
        task_runtime.msgq.chassis_yaw_feedback = NULL;
    }
    if (task_runtime.msgq.chassis_gimbal != NULL)
    {
        if (osMessageQueueDelete(task_runtime.msgq.chassis_gimbal) != osOK)
        {
            success = false;
        }
        task_runtime.msgq.chassis_gimbal = NULL;
    }
    if (task_runtime.msgq.can_command != NULL)
    {
        if (osMessageQueueDelete(task_runtime.msgq.can_command) != osOK)
        {
            success = false;
        }
        task_runtime.msgq.can_command = NULL;
    }
    /* 任一后续对象创建失败时，也回收惯性反馈、云台输出与两路电机反馈邮箱 */
    if (task_runtime.msgq.gimbal_imu != NULL)
    {
        if (osMessageQueueDelete(task_runtime.msgq.gimbal_imu) != osOK)
        {
            success = false;
        }
        task_runtime.msgq.gimbal_imu = NULL;
    }
    if (task_runtime.msgq.gimbal_output != NULL)
    {
        if (osMessageQueueDelete(task_runtime.msgq.gimbal_output) != osOK)
        {
            success = false;
        }
        task_runtime.msgq.gimbal_output = NULL;
    }
    if (task_runtime.msgq.gimbal_pitch_feedback != NULL)
    {
        if (osMessageQueueDelete(task_runtime.msgq.gimbal_pitch_feedback) != osOK)
        {
            success = false;
        }
        task_runtime.msgq.gimbal_pitch_feedback = NULL;
    }
    if (task_runtime.msgq.gimbal_yaw_feedback != NULL)
    {
        if (osMessageQueueDelete(task_runtime.msgq.gimbal_yaw_feedback) != osOK)
        {
            success = false;
        }
        task_runtime.msgq.gimbal_yaw_feedback = NULL;
    }
    if (task_runtime.msgq.can_feedback != NULL)
    {
        if (osMessageQueueDelete(task_runtime.msgq.can_feedback) != osOK)
        {
            success = false;
        }
        task_runtime.msgq.can_feedback = NULL;
    }
    /* 遥控分发邮箱同样由初始化任务回收 */
    if (task_runtime.msgq.dr16_gimbal_state != NULL)
    {
        if (osMessageQueueDelete(task_runtime.msgq.dr16_gimbal_state) != osOK)
        {
            success = false;
        }
        task_runtime.msgq.dr16_gimbal_state = NULL;
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
