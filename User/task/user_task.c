#include "task/user_task.h"

#include "task.h"

Task_Runtime_t task_runtime;

/*
 * 启动末尾创建的三个任务使用静态存储，避免耗尽 15360 字节的 FreeRTOS 动态堆
 * 控制块各保存一个任务的内核状态，栈各为 1024 字节，生命周期覆盖任务创建和失败回收
 * StackType_t 数组按 8 字节对齐，满足 Cortex-M4F 的任务栈对齐要求
 */
static StaticTask_t dr16_task_control_block;
static StaticTask_t motor_shoot_task_control_block;
static StaticTask_t motor_trigger_task_control_block;
static StackType_t dr16_task_stack[1024U / sizeof(StackType_t)] __ALIGNED(8);
static StackType_t motor_shoot_task_stack[1024U / sizeof(StackType_t)] __ALIGNED(8);
static StackType_t motor_trigger_task_stack[1024U / sizeof(StackType_t)] __ALIGNED(8);

/*
 * 业务任务静态属性：
 * - Task_Init：实时优先级，栈空间 2048 字节，初始化业务模块并创建业务对象后退出
 * - can：高于普通优先级，栈空间 1024 字节，执行 500 赫兹 CAN 设备通信
 * - motor_chassis：普通优先级，栈空间 1024 字节，执行 500 赫兹底盘周期控制
 * - motor_gimbal：普通优先级，栈空间 1024 字节，执行 500 赫兹双轴惯性串级控制及编码器机械保护
 * - motor_shoot：普通优先级，栈空间 1024 字节，执行 500 赫兹双摩擦轮速度控制
 * - motor_trigger：普通优先级，栈空间 1024 字节，执行 500 赫兹拨弹速度控制
 * - dr16：高于普通优先级，栈空间 1024 字节，消费 UART 字节流并发布遥控状态
 * - INS：高于普通优先级，栈空间 2048 字节，执行 1000 赫兹 BMI088 采集和姿态解算
 */
const osThreadAttr_t attr_init =
{
    .name = "Task_Init",
    .priority = osPriorityRealtime,
    .stack_size = 2048U,
};

const osThreadAttr_t attr_can =
{
    .name = "can",
    .priority = osPriorityAboveNormal,
    .stack_size = 256U * 4U,
};

const osThreadAttr_t attr_motor_chassis =
{
    .name = "motor_chassis",
    .priority = osPriorityNormal,
    .stack_size = 256U * 4U,
};

const osThreadAttr_t attr_dr16 =
{
    .name = "dr16",
    .priority = osPriorityAboveNormal,
    .cb_mem = &dr16_task_control_block,
    .cb_size = sizeof(dr16_task_control_block),
    .stack_mem = dr16_task_stack,
    .stack_size = sizeof(dr16_task_stack),
};

const osThreadAttr_t attr_motor_gimbal =
{
    .name = "motor_gimbal",
    .priority = osPriorityNormal,
    .stack_size = 256U * 4U,
};

const osThreadAttr_t attr_ins =
{
    .name = "INS",
    .priority = osPriorityAboveNormal,
    .stack_size = 512U * 4U,
};

const osThreadAttr_t attr_motor_shoot =
{
    .name = "motor_shoot",
    .priority = osPriorityNormal,
    .cb_mem = &motor_shoot_task_control_block,
    .cb_size = sizeof(motor_shoot_task_control_block),
    .stack_mem = motor_shoot_task_stack,
    .stack_size = sizeof(motor_shoot_task_stack),
};

const osThreadAttr_t attr_motor_trigger =
{
    .name = "motor_trigger",
    .priority = osPriorityNormal,
    .cb_mem = &motor_trigger_task_control_block,
    .cb_size = sizeof(motor_trigger_task_control_block),
    .stack_mem = motor_trigger_task_stack,
    .stack_size = sizeof(motor_trigger_task_stack),
};
