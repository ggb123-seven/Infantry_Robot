/*
 * 任务运行时对象定义
 *
 * 定义静态任务栈、控制块和任务属性，避免占用 FreeRTOS 动态堆
 */

#include "task/user_task.h"

#include "task.h"

Task_Runtime_t task_runtime;

/* Private variables -------------------------------------------------------- */
/*
 * 云台、遥控、摩擦轮和拨弹任务使用静态存储，避免耗尽 15360 字节的 FreeRTOS 动态堆
 * 云台栈为 3072 字节，其余三个栈各为 1024 字节
 * StackType_t 数组按 8 字节对齐，满足 Cortex-M4F 的任务栈对齐要求
 */
static StaticTask_t dr16_task_control_block;                  /* DR16 任务控制块 */
static StaticTask_t motor_shoot_task_control_block;           /* 摩擦轮任务控制块 */
static StaticTask_t motor_trigger_task_control_block;         /* 拨弹任务控制块 */
static StaticTask_t motor_gimbal_task_control_block;          /* 云台任务控制块 */
static StackType_t dr16_task_stack[1024U / sizeof(StackType_t)] __ALIGNED(8);          /* DR16 任务栈 */
static StackType_t motor_shoot_task_stack[1024U / sizeof(StackType_t)] __ALIGNED(8);   /* 摩擦轮任务栈 */
static StackType_t motor_trigger_task_stack[1024U / sizeof(StackType_t)] __ALIGNED(8); /* 拨弹任务栈 */
static StackType_t motor_gimbal_task_stack[3072U / sizeof(StackType_t)] __ALIGNED(8);  /* 云台任务栈 */

/* Exported constants ------------------------------------------------------- */
/* 初始化任务：实时优先级，栈 2048 字节 */
const osThreadAttr_t attr_init =
{
    .name = "Task_Init",
    .priority = osPriorityRealtime,
    .stack_size = 2048U,
};

/* CAN 任务：500 Hz，高于普通优先级，栈 1024 字节 */
const osThreadAttr_t attr_can =
{
    .name = "can",
    .priority = osPriorityAboveNormal,
    .stack_size = 256U * 4U,
};

/* 底盘任务：500 Hz，普通优先级，栈 1024 字节 */
const osThreadAttr_t attr_motor_chassis =
{
    .name = "motor_chassis",
    .priority = osPriorityNormal,
    .stack_size = 256U * 4U,
};

/* DR16 任务：高于普通优先级，静态栈 1024 字节 */
const osThreadAttr_t attr_dr16 =
{
    .name = "dr16",
    .priority = osPriorityAboveNormal,
    .cb_mem = &dr16_task_control_block,
    .cb_size = sizeof(dr16_task_control_block),
    .stack_mem = dr16_task_stack,
    .stack_size = sizeof(dr16_task_stack),
};

/* 云台任务：500 Hz，普通优先级，静态栈 3072 字节 */
const osThreadAttr_t attr_motor_gimbal =
{
    .name = "motor_gimbal",
    .priority = osPriorityNormal,
    .cb_mem = &motor_gimbal_task_control_block,
    .cb_size = sizeof(motor_gimbal_task_control_block),
    .stack_mem = motor_gimbal_task_stack,
    .stack_size = sizeof(motor_gimbal_task_stack),
};

/* INS 任务：1000 Hz，高于普通优先级，栈 2048 字节 */
const osThreadAttr_t attr_ins =
{
    .name = "INS",
    .priority = osPriorityAboveNormal,
    .stack_size = 512U * 4U,
};

/* 摩擦轮任务：500 Hz，普通优先级，静态栈 1024 字节 */
const osThreadAttr_t attr_motor_shoot =
{
    .name = "motor_shoot",
    .priority = osPriorityNormal,
    .cb_mem = &motor_shoot_task_control_block,
    .cb_size = sizeof(motor_shoot_task_control_block),
    .stack_mem = motor_shoot_task_stack,
    .stack_size = sizeof(motor_shoot_task_stack),
};

/* 拨弹任务：500 Hz，普通优先级，静态栈 1024 字节 */
const osThreadAttr_t attr_motor_trigger =
{
    .name = "motor_trigger",
    .priority = osPriorityNormal,
    .cb_mem = &motor_trigger_task_control_block,
    .cb_size = sizeof(motor_trigger_task_control_block),
    .stack_mem = motor_trigger_task_stack,
    .stack_size = sizeof(motor_trigger_task_stack),
};
