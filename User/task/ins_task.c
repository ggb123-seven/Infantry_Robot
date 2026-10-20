#include "task/ins_task.h"

#include <stddef.h>

#include "bsp/time.h"
#include "task/user_task.h"

INS_t INS;

/**
 * @brief 初始化 INS 任务使用的 BMI088、姿态滤波器和温控模块
 *
 * @return INS 模块进入运行状态时返回 true，否则返回 false
 */
bool INS_Task_Init(void)
{
    return INS_Init(&INS) == INS_STATUS_OK;
}

/**
 * @brief 以 1000 Hz 采集 BMI088 并更新 INS 姿态解算
 *
 * @param[in] argument 任务参数，本任务不使用
 * @return 本任务不会返回
 */
void INS_Task(void *argument)
{
    (void)argument;

    // 任务获得调度后再初始化设备和加热器，任务创建失败时保持硬件安全输出
    if (!INS_Task_Init())
    {
        task_runtime.thread.ins = NULL;
        osThreadExit();
    }

    const uint32_t delay_tick = osKernelGetTickFreq() / INS_TASK_FREQ;
    if (delay_tick == 0U)
    {
        INS_Update(&INS, 0.0F);
        task_runtime.thread.ins = NULL;
        osThreadExit();
    }

    // 建立绝对周期和微秒时间基准，用实测周期驱动姿态滤波
    uint32_t tick = osKernelGetTickCount();
    uint64_t previous_time_us = BSP_TIME_Get();
    while (1)
    {
        tick += delay_tick;

        // 等待至下一个绝对任务周期，保持 1000 Hz 采集节拍
        osDelayUntil(tick);

        // 读取本周期实际时间间隔，时基异常时由 INS 模块进入故障安全状态
        const uint64_t current_time_us = BSP_TIME_Get();
        float dt_s = 0.0F;
        if (current_time_us > previous_time_us)
        {
            dt_s = (float)(current_time_us - previous_time_us) * 1.0e-6F;
        }
        previous_time_us = current_time_us;

        // 采集 BMI088 并更新 Madgwick、坐标变换、运动加速度和二分频温控
        if (INS_Update(&INS, dt_s) != INS_STATUS_OK)
        {
            // 模块已进入安全故障状态，清除运行句柄并结束当前任务
            task_runtime.thread.ins = NULL;
            osThreadExit();
        }
    }
}
