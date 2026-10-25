#include "task/ins_task.h"

#include <stddef.h>

#include "bsp/time.h"
#include "module/gimbal.h"
#include "task/user_task.h"

INS_t INS;

static bool INS_Task_Publish(uint64_t sample_time_us, bool valid);

/**
 * @brief 初始化 INS 任务使用的 BMI088、姿态滤波器和温控模块
 *
 * @return INS 模块进入运行状态时返回 true，否则返回 false
 */
bool INS_Task_Init(void)
{
    // 先撤销上一轮反馈，再进行可能阻塞的传感器校准
    if (!INS_Task_Publish(BSP_TIME_Get(), false))
    {
        return false;
    }
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

    const uint32_t tick_frequency = osKernelGetTickFreq();
    const uint32_t delay_tick = tick_frequency / INS_TASK_FREQ;
    if (delay_tick == 0U || tick_frequency % INS_TASK_FREQ != 0U)
    {
        INS_Update(&INS, 0.0F);
        INS_Task_Publish(BSP_TIME_Get(), false);
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
            // 模块已关闭加热器，尽力通知云台失效，发布失败时由源时间超时兜底
            INS_Task_Publish(current_time_us, false);
            task_runtime.thread.ins = NULL;
            osThreadExit();
        }

        // 更新完成后一次发布整帧，保留采集前时间，发布失败时关闭加热器并退出
        if (!INS_Task_Publish(current_time_us, true))
        {
            INS_Update(&INS, 0.0F);
            task_runtime.thread.ins = NULL;
            osThreadExit();
        }

        // 延迟超过一个周期时放弃追赶，避免以过短间隔连续采集
        const uint32_t now_tick = osKernelGetTickCount();
        if ((uint32_t)(now_tick - tick) >= delay_tick)
        {
            tick = now_tick;
        }
    }
}

/**
 * @brief 由 INS 所有者将一次完整更新发布为最新值快照
 *
 * @param[in] sample_time_us 本次开始采集的单调时刻，单位微秒
 * @param[in] valid 本次完整更新是否成功，false 时只发布失效通知
 * @return 邮箱重置和写入成功时返回 true，否则返回 false
 */
static bool INS_Task_Publish(uint64_t sample_time_us, bool valid)
{
    Gimbal_ImuFeedback_t feedback =
    {
        .sample_time_us = sample_time_us,
        .valid = valid && INS.module_state == INS_MODULE_STATE_RUNNING && INS.state.valid,
    };

    // 仅当前任务读取自己刚完成的状态，不引入跨任务裸读或额外坐标修正
    if (feedback.valid)
    {
        feedback.angle_deg[0] = INS.state.Roll;
        feedback.angle_deg[1] = INS.state.Pitch;
        feedback.angle_deg[2] = INS.state.YawTotalAngle;
        for (uint32_t i = 0U; i < 3U; ++i)
        {
            feedback.gyro_rad_s[i] = INS.state.Gyro[i];
        }
    }

    // 最新值邮箱允许覆盖未消费样本，队列本身保证整帧复制
    osMessageQueueId_t queue = task_runtime.msgq.gimbal_imu;
    return queue != NULL && osMessageQueueReset(queue) == osOK &&
           osMessageQueuePut(queue, &feedback, 0U, 0U) == osOK;
}
