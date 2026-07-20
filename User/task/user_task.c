#include "task/user_task.h"

Task_Runtime_t task_runtime;

/*
 * 业务任务静态属性：
 * - Task_Init：实时优先级，栈空间 4096 字节，初始化业务模块并创建业务对象后退出。
 * - motor_chassis：普通优先级，栈空间 1024 字节，执行 500 赫兹底盘周期控制。
 * - dr16：高于普通优先级，栈空间 1024 字节，消费 UART 字节流并发布遥控状态。
 */
const osThreadAttr_t attr_init =
{
    .name = "Task_Init",
    .priority = osPriorityRealtime,
    .stack_size = 4096U,
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
    .stack_size = 256U * 4U,
};
