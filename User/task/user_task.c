#include "task/user_task.h"

#include <FreeRTOS.h>
#include <task.h>

Task_Runtime_t task_runtime;

/*
 * 堆容量编译期门禁：ucHeap 是固定大小的静态数组，堆耗尽不会改变构建报告的 RAM 占用
 * 任务与邮箱创建会静默返回 NULL，表现为遥控与所有业务任务全部失效
 * 本工程需要至少 24576 字节，低于该值直接编译失败，避免再次出现上板才发现的隐性故障
 */
_Static_assert(configTOTAL_HEAP_SIZE >= 24576U, "FreeRTOS 堆不足 24576 字节，任务与邮箱将创建失败");

/*
 * 运行期兜底：堆分配失败时记录次数并停机，供调试器读取以定位失败发生阶段
 * 停机后已下发的电机电流由电调超时自行撤驱，不会保持输出
 */
volatile uint32_t freertos_malloc_failed_count;

/**
 * @brief FreeRTOS 堆分配失败回调
 *
 * @return 无返回值，本函数不会返回
 */
void vApplicationMallocFailedHook(void)
{
    freertos_malloc_failed_count++;
    // 关闭中断后停机等待调试器接管，禁止在堆结构已损坏时继续调度
    taskDISABLE_INTERRUPTS();
    for (;;)
    {
    }
}

/*
 * 业务任务静态属性：
 * - Task_Init：实时优先级，栈空间 2048 字节，初始化业务模块并创建业务对象后退出
 * - can：高于普通优先级，栈空间 1024 字节，执行 500 赫兹 CAN 设备通信
 * - motor_chassis：普通优先级，栈空间 1024 字节，执行 500 赫兹底盘周期控制
 * - motor_gimbal：普通优先级，栈空间 1024 字节，执行 500 赫兹双轴惯性串级控制及编码器机械保护
 * - motor_shoot：普通优先级，栈空间 1024 字节，执行 500 赫兹双摩擦轮速度闭环
 * - motor_trigger：普通优先级，栈空间 1024 字节，执行 500 赫兹拨弹逐齿控制并联动摩擦轮命令
 * - dr16：高于普通优先级，栈空间 1024 字节，消费 UART 字节流并发布遥控状态
 * - INS：高于普通优先级，栈空间 2048 字节，执行 1000 赫兹 BMI088 采集和姿态解算
 */
const osThreadAttr_t attr_init =
{
    .name = "Task_Init",
    .priority = osPriorityRealtime,
    .stack_size = 3000U,
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
    .stack_size = 256U * 4U,
};

const osThreadAttr_t attr_motor_gimbal =
{
    .name = "motor_gimbal",
    .priority = osPriorityNormal,
    .stack_size = 256U * 4U,
};

const osThreadAttr_t attr_motor_shoot =
{
    .name = "motor_shoot",
    .priority = osPriorityNormal,
    .stack_size = 256U * 4U,
};

const osThreadAttr_t attr_motor_trigger =
{
    .name = "motor_trigger",
    .priority = osPriorityNormal,
    .stack_size = 256U * 4U,
};

const osThreadAttr_t attr_ins =
{
    .name = "INS",
    .priority = osPriorityAboveNormal,
    .stack_size = 512U * 4U,
};
