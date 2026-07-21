#pragma once

#ifdef __cplusplus
extern "C"
{
#endif

#include <cmsis_os2.h>

/*
 * CAN 通信任务运行参数：
 * - CAN_TASK_FREQ：CAN 设备反馈采集和电流发送频率，单位赫兹。
 * - CAN_TASK_INIT_DELAY：CAN 通信任务启动延时，单位内核节拍。
 */
#define CAN_TASK_FREQ (500U)
#define CAN_TASK_INIT_DELAY (0U)

/*
 * 底盘任务运行参数：
 * - MOTOR_CHASSIS_FREQ：底盘控制频率，单位赫兹。
 * - MOTOR_CHASSIS_INIT_DELAY：底盘任务启动延时，单位内核节拍。
 */
#define MOTOR_CHASSIS_FREQ (500U)
#define MOTOR_CHASSIS_INIT_DELAY (0U)

/**
 * @brief 业务对象初始化结果
 */
typedef enum
{
    TASK_INIT_NOT_STARTED = 0,
    TASK_INIT_OK,
    TASK_INIT_CAN_DEVICES_FAILED,
    TASK_INIT_KERNEL_LOCK_FAILED,
    TASK_INIT_DR16_MAILBOX_FAILED,
    TASK_INIT_CAN_FEEDBACK_MAILBOX_FAILED,
    TASK_INIT_CAN_COMMAND_MAILBOX_FAILED,
    TASK_INIT_CAN_THREAD_FAILED,
    TASK_INIT_MOTOR_THREAD_FAILED,
    TASK_INIT_DR16_THREAD_FAILED,
    TASK_INIT_MOTOR_CHASSIS_FAILED,
    TASK_INIT_CLEANUP_FAILED,
    TASK_INIT_KERNEL_UNLOCK_FAILED,
} Task_InitStatus_t;

/*
 * 任务运行时对象：
 * - thread.can：CAN 设备反馈采集和电流发送任务句柄。
 * - thread.motor_chassis：底盘周期控制任务句柄。
 * - thread.dr16：DR16 字节流接收和状态发布任务句柄。
 * - msgq.dr16_state：长度为 1 的 DR16 最新状态邮箱，由 Task_dr16 写入。
 * - msgq.can_feedback：长度为 1 的 CAN 设备最新反馈邮箱，由 Task_can 写入、Task_motor_chassis 读取。
 * - msgq.can_command：长度为 1 的 CAN 设备最新电流命令邮箱，由 Task_motor_chassis 写入、Task_can 读取。
 * - init_status：业务模块初始化、RTOS 对象创建、清理和内核解锁的最终结果。
 */
typedef struct
{
    struct
    {
        osThreadId_t can;
        osThreadId_t motor_chassis;
        osThreadId_t dr16;
    } thread;
    struct
    {
        osMessageQueueId_t dr16_state;
        osMessageQueueId_t can_feedback;
        osMessageQueueId_t can_command;
    } msgq;
    volatile Task_InitStatus_t init_status;
} Task_Runtime_t;

extern Task_Runtime_t task_runtime;

extern const osThreadAttr_t attr_init;
extern const osThreadAttr_t attr_can;
extern const osThreadAttr_t attr_motor_chassis;
extern const osThreadAttr_t attr_dr16;

/**
 * @brief 创建业务任务和消息队列
 *
 * @param[in] argument 任务参数，本任务不使用
 * @return 本任务不会返回
 */
void Task_Init(void *argument);

#ifdef __cplusplus
}
#endif
