/*
 * 任务运行时公共接口
 *
 * 集中声明任务运行参数、初始化状态、运行时对象和相关任务入口
 */

#pragma once

#ifdef __cplusplus
extern "C"
{
#endif

/* Includes ----------------------------------------------------------------- */
#include <cmsis_os2.h>
#include "FreeRTOS.h"

#include "device/can_devices.h"

/* Exported constants ------------------------------------------------------- */
/* CAN 任务运行参数，启动延时单位均为内核节拍 */
#define CAN_TASK_FREQ (500U)          /* 采集频率，单位：Hz */
#define CAN_TASK_INIT_DELAY (0U)      /* 启动延时 */

/* 底盘任务运行参数，启动延时单位均为内核节拍 */
#define MOTOR_CHASSIS_FREQ (500U)         /* 控制频率，单位：Hz */
#define MOTOR_CHASSIS_INIT_DELAY (0U)     /* 启动延时 */

/* 云台任务运行参数，启动延时单位均为内核节拍 */
#define MOTOR_GIMBAL_FREQ (500U)          /* 控制频率，单位：Hz */
#define MOTOR_GIMBAL_INIT_DELAY (0U)      /* 启动延时 */

/* 发射机构任务运行参数，启动延时单位均为内核节拍 */
#define MOTOR_SHOOT_FREQ (500U)           /* 摩擦轮频率，单位：Hz */
#define MOTOR_SHOOT_INIT_DELAY (0U)       /* 摩擦轮启动延时 */
#define MOTOR_TRIGGER_FREQ (500U)         /* 拨弹控制频率，单位：Hz */
#define MOTOR_TRIGGER_INIT_DELAY (0U)     /* 拨弹启动延时 */

/* INS 任务运行参数，启动延时单位均为内核节拍 */
#define INS_TASK_FREQ (1000U)             /* 姿态更新频率，单位：Hz */
#define INS_TASK_INIT_DELAY (0U)          /* 启动延时 */

/* 固定频率必须由内核节拍精确表达，配置错误在编译期报告 */
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

/* Exported types ----------------------------------------------------------- */
/* 业务对象初始化结果 */
typedef enum
{
    TASK_INIT_NOT_STARTED = 0,                                      /* 尚未开始 */
    TASK_INIT_OK,                                                   /* 全部对象创建成功 */
    TASK_INIT_CAN_DEVICES_FAILED,                                   /* CAN 设备初始化失败 */
    TASK_INIT_KERNEL_LOCK_FAILED,                                   /* 调度器锁定失败 */
    TASK_INIT_DR16_MAILBOX_FAILED,                                  /* DR16 邮箱创建失败 */
    TASK_INIT_CAN_FEEDBACK_MAILBOX_FAILED,                          /* CAN 反馈邮箱创建失败 */
    TASK_INIT_CAN_COMMAND_MAILBOX_FAILED,                           /* CAN 命令邮箱创建失败 */
    TASK_INIT_GIMBAL_YAW_FEEDBACK_MAILBOX_FAILED,                   /* 云台偏航邮箱失败 */
    TASK_INIT_GIMBAL_PITCH_FEEDBACK_MAILBOX_FAILED,                 /* 云台俯仰邮箱失败 */
    TASK_INIT_GIMBAL_OUTPUT_MAILBOX_FAILED,                         /* 云台输出邮箱创建失败 */
    TASK_INIT_GIMBAL_IMU_MAILBOX_FAILED,                            /* 惯性反馈邮箱创建失败 */
    TASK_INIT_GIMBAL_THREAD_FAILED,                                 /* 云台任务创建失败 */
    TASK_INIT_CAN_THREAD_FAILED,                                    /* CAN 任务创建失败 */
    TASK_INIT_MOTOR_THREAD_FAILED,                                  /* 底盘任务创建失败 */
    TASK_INIT_DR16_THREAD_FAILED,                                   /* DR16 任务创建失败 */
    TASK_INIT_INS_THREAD_FAILED,                                    /* INS 任务创建失败 */
    TASK_INIT_CLEANUP_FAILED,                                       /* 失败回收未全部成功 */
    TASK_INIT_KERNEL_UNLOCK_FAILED,                                 /* 调度器解锁失败 */
    TASK_INIT_DR16_GIMBAL_MAILBOX_FAILED,                           /* 云台遥控邮箱创建失败 */
    TASK_INIT_CHASSIS_GIMBAL_MAILBOX_FAILED,                        /* 底盘状态邮箱创建失败 */
    TASK_INIT_SHOOT_FRIC1_FEEDBACK_MAILBOX_FAILED,                  /* 摩擦轮 1 邮箱失败 */
    TASK_INIT_SHOOT_FRIC2_FEEDBACK_MAILBOX_FAILED,                  /* 摩擦轮 2 邮箱失败 */
    TASK_INIT_TRIGGER_FEEDBACK_MAILBOX_FAILED,                      /* 拨弹反馈邮箱创建失败 */
    TASK_INIT_SHOOT_COMMAND_MAILBOX_FAILED,                         /* 摩擦轮命令邮箱创建失败 */
    TASK_INIT_TRIGGER_COMMAND_MAILBOX_FAILED,                       /* 拨弹遥控邮箱创建失败 */
    TASK_INIT_SHOOT_OUTPUT_MAILBOX_FAILED,                          /* 摩擦轮输出邮箱创建失败 */
    TASK_INIT_TRIGGER_OUTPUT_MAILBOX_FAILED,                        /* 拨弹输出邮箱创建失败 */
    TASK_INIT_SHOOT_THREAD_FAILED,                                  /* 摩擦轮任务创建失败 */
    TASK_INIT_TRIGGER_THREAD_FAILED,                                /* 拨弹任务创建失败 */
    TASK_INIT_CHASSIS_YAW_FEEDBACK_MAILBOX_FAILED,                  /* 底盘 yaw 邮箱失败 */
} Task_InitStatus_t;

/*
 * 任务运行时对象：任务句柄、消息队列和初始化结果集中保存
 * 所有邮箱容量为 1，只保留最新值
 */
typedef struct
{
    /* 任务句柄 */
    struct
    {
        osThreadId_t can;                 /* CAN 通信 */
        osThreadId_t motor_chassis;       /* 底盘控制 */
        osThreadId_t motor_gimbal;        /* 云台控制 */
        osThreadId_t motor_shoot;         /* 双摩擦轮控制 */
        osThreadId_t motor_trigger;       /* 拨弹控制 */
        osThreadId_t dr16;                /* DR16 接收 */
        osThreadId_t ins;                 /* INS 采集 */
    } thread;
    /* 消息队列 */
    struct
    {
        osMessageQueueId_t dr16_state;                 /* DR16 写、底盘读 */
        osMessageQueueId_t dr16_gimbal_state;          /* DR16 写、云台读 */
        osMessageQueueId_t can_feedback;               /* CAN 写、底盘读 */
        osMessageQueueId_t can_command;                /* 底盘写、CAN 读 */
        osMessageQueueId_t gimbal_yaw_feedback;        /* CAN 写、云台读 */
        osMessageQueueId_t gimbal_pitch_feedback;      /* CAN 写、云台读 */
        osMessageQueueId_t gimbal_output;              /* 云台输出，单位：A */
        osMessageQueueId_t gimbal_imu;                 /* INS 写、云台读 */
        osMessageQueueId_t chassis_gimbal;             /* 底盘写、云台读 */
        osMessageQueueId_t chassis_yaw_feedback;       /* CAN 写、底盘读 */
        osMessageQueueId_t shoot_fric1_feedback;       /* CAN 写、摩擦轮读 */
        osMessageQueueId_t shoot_fric2_feedback;       /* CAN 写、摩擦轮读 */
        osMessageQueueId_t trigger_feedback;           /* CAN 写、拨弹读 */
        osMessageQueueId_t shoot_command;              /* 拨弹写、摩擦轮读 */
        osMessageQueueId_t trigger_command;            /* DR16 写、拨弹读 */
        osMessageQueueId_t shoot_output;               /* 摩擦轮输出，单位：A */
        osMessageQueueId_t trigger_output;             /* 拨弹输出，单位：A */
    } msgq;
    volatile Task_InitStatus_t init_status;            /* 初始化最终结果 */
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

/* Exported functions ------------------------------------------------------- */
/**
 * @brief 创建业务任务和消息队列
 *
 * @param argument 任务参数，本任务不使用
 * @return 无返回值
 */
void Task_Init(void *argument);

#ifdef __cplusplus
}
#endif
