#pragma once

#ifdef __cplusplus
extern "C"
{
#endif

#include <cmsis_os2.h>

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
    TASK_INIT_KERNEL_LOCK_FAILED,
    TASK_INIT_DR16_MAILBOX_FAILED,
    TASK_INIT_MOTOR_THREAD_FAILED,
    TASK_INIT_DR16_THREAD_FAILED,
    TASK_INIT_CLEANUP_FAILED,
    TASK_INIT_KERNEL_UNLOCK_FAILED,
} Task_InitStatus_t;

/*
 * 任务运行时对象：
 * - thread.motor_chassis：底盘周期控制任务句柄。
 * - thread.dr16：DR16 字节流接收和状态发布任务句柄。
 * - msgq.dr16_state：长度为 1 的 DR16 最新状态邮箱，由 Task_dr16 写入。
 * - init_status：业务 RTOS 对象创建、清理和内核解锁的最终结果。
 */
typedef struct
{
    struct
    {
        osThreadId_t motor_chassis;
        osThreadId_t dr16;
    } thread;
    struct
    {
        osMessageQueueId_t dr16_state;
    } msgq;
    volatile Task_InitStatus_t init_status;
} Task_Runtime_t;

extern Task_Runtime_t task_runtime;

extern const osThreadAttr_t attr_init;
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
