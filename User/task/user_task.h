#pragma once

#ifdef __cplusplus
extern "C"
{
#endif

#include <cmsis_os2.h>
#include "FreeRTOS.h"

#include "device/can_devices.h"

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

/*
 * 云台任务参数：MOTOR_GIMBAL_FREQ 为云台两轴控制频率，单位赫兹
 * MOTOR_GIMBAL_INIT_DELAY 为启动延时，单位内核节拍
 */
#define MOTOR_GIMBAL_FREQ (500U)
#define MOTOR_GIMBAL_INIT_DELAY (0U)

/*
 * 发射机构任务参数：MOTOR_SHOOT_FREQ、MOTOR_TRIGGER_FREQ 分别为双摩擦轮与拨弹频率，单位 Hz
 * MOTOR_SHOOT_INIT_DELAY、MOTOR_TRIGGER_INIT_DELAY 为启动延时，单位内核节拍
 */
#define MOTOR_SHOOT_FREQ (500U)
#define MOTOR_SHOOT_INIT_DELAY (0U)
#define MOTOR_TRIGGER_FREQ (500U)
#define MOTOR_TRIGGER_INIT_DELAY (0U)

/*
 * INS 任务运行参数：
 * - INS_TASK_FREQ：BMI088 采集和姿态更新频率，单位赫兹
 * - INS_TASK_INIT_DELAY：INS 任务启动延时，单位内核节拍
 * 温控由 INS 模块按二分频执行
 */
#define INS_TASK_FREQ (1000U)
#define INS_TASK_INIT_DELAY (0U)

// 固定频率必须由内核节拍精确表达，配置错误在编译期报告
_Static_assert(CAN_TASK_FREQ > 0U && configTICK_RATE_HZ % CAN_TASK_FREQ == 0U,
               "CAN_TASK_FREQ 必须非零且整除内核节拍频率");
_Static_assert(MOTOR_CHASSIS_FREQ > 0U && configTICK_RATE_HZ % MOTOR_CHASSIS_FREQ == 0U,
               "MOTOR_CHASSIS_FREQ 必须非零且整除内核节拍频率");
_Static_assert(MOTOR_GIMBAL_FREQ > 0U && configTICK_RATE_HZ % MOTOR_GIMBAL_FREQ == 0U,
               "MOTOR_GIMBAL_FREQ 必须非零且整除内核节拍频率");
_Static_assert(MOTOR_SHOOT_FREQ > 0U && configTICK_RATE_HZ % MOTOR_SHOOT_FREQ == 0U,
               "MOTOR_SHOOT_FREQ 必须非零且整除内核节拍频率");
_Static_assert(MOTOR_TRIGGER_FREQ > 0U && configTICK_RATE_HZ % MOTOR_TRIGGER_FREQ == 0U,
               "MOTOR_TRIGGER_FREQ 必须非零且整除内核节拍频率");
_Static_assert(INS_TASK_FREQ > 0U && configTICK_RATE_HZ % INS_TASK_FREQ == 0U,
               "INS_TASK_FREQ 必须非零且整除内核节拍频率");

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
    TASK_INIT_GIMBAL_YAW_FEEDBACK_MAILBOX_FAILED,
    TASK_INIT_GIMBAL_PITCH_FEEDBACK_MAILBOX_FAILED,
    TASK_INIT_GIMBAL_OUTPUT_MAILBOX_FAILED,
    TASK_INIT_GIMBAL_IMU_MAILBOX_FAILED,
    TASK_INIT_GIMBAL_THREAD_FAILED,
    TASK_INIT_CAN_THREAD_FAILED,
    TASK_INIT_MOTOR_THREAD_FAILED,
    TASK_INIT_DR16_THREAD_FAILED,
    TASK_INIT_INS_THREAD_FAILED,
    TASK_INIT_CLEANUP_FAILED,
    TASK_INIT_KERNEL_UNLOCK_FAILED,
    TASK_INIT_DR16_GIMBAL_MAILBOX_FAILED,
    TASK_INIT_CHASSIS_GIMBAL_MAILBOX_FAILED,
    TASK_INIT_SHOOT_FRIC1_FEEDBACK_MAILBOX_FAILED,
    TASK_INIT_SHOOT_FRIC2_FEEDBACK_MAILBOX_FAILED,
    TASK_INIT_TRIGGER_FEEDBACK_MAILBOX_FAILED,
    TASK_INIT_SHOOT_COMMAND_MAILBOX_FAILED,
    TASK_INIT_TRIGGER_COMMAND_MAILBOX_FAILED,
    TASK_INIT_SHOOT_OUTPUT_MAILBOX_FAILED,
    TASK_INIT_TRIGGER_OUTPUT_MAILBOX_FAILED,
    TASK_INIT_SHOOT_THREAD_FAILED,
    TASK_INIT_TRIGGER_THREAD_FAILED,
} Task_InitStatus_t;

/*
 * 任务运行时对象：
 * - thread.can：CAN 设备反馈采集和电流发送任务句柄
 * - thread.motor_chassis：底盘周期控制任务句柄
 * - thread.motor_gimbal：云台模块初始化与两轴周期控制任务句柄
 * - thread.motor_shoot：双摩擦轮周期控制任务句柄
 * - thread.motor_trigger：M2006 拨弹周期控制任务句柄
 * - thread.dr16：DR16 字节流接收和状态发布任务句柄
 * - thread.ins：BMI088 采集、姿态解算和温控任务句柄
 * - msgq.dr16_state：长度为 1 的 DR16 最新状态邮箱，由 Task_dr16 写入、底盘任务读取
 * - msgq.dr16_gimbal_state：长度为 1 的独立 DR16 状态邮箱，元素为 DR16_State_t，由 DR16 任务写入、云台任务读取
 * - msgq.can_feedback：长度为 1 的 CAN 设备最新反馈邮箱，由 Task_can 写入、Task_motor_chassis 读取
 * - msgq.can_command：长度为 1 的 CAN 设备最新电流命令邮箱，由 Task_motor_chassis 写入、Task_can 读取
 * - msgq.gimbal_yaw_feedback：长度为 1 的 yaw 最新反馈邮箱，元素为 MOTOR_RM_Feedback_t，由 Task_can 写入
 * - msgq.gimbal_pitch_feedback：长度为 1 的 pitch 最新反馈邮箱，元素为 MOTOR_RM_Feedback_t，由 Task_can 写入
 * - 两个云台反馈邮箱的速度单位为 rpm、时间戳单位为微秒，均由 Task_motor_gimbal 消费
 * - msgq.gimbal_output：长度为 1 的云台输出邮箱，元素为 Gimbal_Output_t，单位 A，由云台任务写入、CAN 任务读取
 * - msgq.gimbal_imu：长度为 1 的同帧惯性反馈邮箱，元素为 Gimbal_ImuFeedback_t，由 INS 任务写入、云台任务读取
 * - msgq.chassis_gimbal：长度为 1 的底盘运行状态邮箱，由底盘任务写入、云台任务读取
 *   元素为 Gimbal_ChassisState_t，区分四轮离线、在线未自旋和在线小陀螺
 * - msgq.shoot_fric1_feedback、shoot_fric2_feedback：CAN 任务到摩擦轮任务的独立最新反馈邮箱
 *   元素为 MOTOR_RM_Feedback_t，转子速度单位 rpm，源时间单位微秒
 * - msgq.trigger_feedback：CAN 任务到拨弹任务的最新反馈邮箱，元素为 MOTOR_RM_Feedback_t
 *   转速为 36:1 减速后的输出轴 rpm，源时间单位微秒
 * - msgq.shoot_command：摩擦轮连续速度命令邮箱，元素为 Shoot_Command_t，当前没有自动发布者
 *   容量为 1，每个控制周期缺少新命令时失能
 * - msgq.trigger_command：沿用既有句柄的拨弹遥控快照邮箱，元素改为 DR16_State_t，容量为 1
 *   DR16 任务发布，拨弹任务缓存，模块根据源时间检查 100 ms 遥控时效，不再传递速度命令
 * - msgq.shoot_output、trigger_output：两个控制任务到 CAN 任务的最新电流邮箱，元素为 Shoot_Output_t、Trigger_Output_t
 *   电流单位 A，容量均为 1，CAN 任务合并后只发送一次 0x1FF
 * - init_status：业务模块初始化、RTOS 对象创建、清理和内核解锁的最终结果
 */
typedef struct
{
    struct
    {
        osThreadId_t can;
        osThreadId_t motor_chassis;
        osThreadId_t motor_gimbal;
        osThreadId_t motor_shoot;
        osThreadId_t motor_trigger;
        osThreadId_t dr16;
        osThreadId_t ins;
    } thread;
    struct
    {
        osMessageQueueId_t dr16_state;
        osMessageQueueId_t dr16_gimbal_state;
        osMessageQueueId_t can_feedback;
        osMessageQueueId_t can_command;
        osMessageQueueId_t gimbal_yaw_feedback;
        osMessageQueueId_t gimbal_pitch_feedback;
        osMessageQueueId_t gimbal_output;
        osMessageQueueId_t gimbal_imu;
        osMessageQueueId_t chassis_gimbal;
        osMessageQueueId_t shoot_fric1_feedback;
        osMessageQueueId_t shoot_fric2_feedback;
        osMessageQueueId_t trigger_feedback;
        osMessageQueueId_t shoot_command;
        osMessageQueueId_t trigger_command;
        osMessageQueueId_t shoot_output;
        osMessageQueueId_t trigger_output;
    } msgq;
    volatile Task_InitStatus_t init_status;
} Task_Runtime_t;

extern Task_Runtime_t task_runtime;

extern const osThreadAttr_t attr_init;
extern const osThreadAttr_t attr_can;
extern const osThreadAttr_t attr_motor_chassis;
extern const osThreadAttr_t attr_motor_gimbal;
extern const osThreadAttr_t attr_motor_shoot;
extern const osThreadAttr_t attr_motor_trigger;
extern const osThreadAttr_t attr_dr16;
extern const osThreadAttr_t attr_ins;

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
