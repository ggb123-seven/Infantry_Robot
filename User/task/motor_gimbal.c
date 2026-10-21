#include "task/motor_gimbal.h"

#include "bsp/time.h"
#include "module/gimbal.h"
#include "task/user_task.h"

#include <math.h>
#include <stddef.h>

/**
 * @brief 消费 yaw 反馈并周期调用云台模块，通过邮箱发布电流
 *
 * @param[in] argument 任务参数，本任务不使用
 * @return 本任务不会返回
 */
void Task_motor_gimbal(void *argument)
{
    (void)argument;

    const uint32_t tick_frequency = osKernelGetTickFreq();
    const uint32_t delay_tick = tick_frequency / MOTOR_GIMBAL_FREQ;

    // 等待统一初始化完成，反馈未到达时以无效标记阻止速度环输出
    osDelay(MOTOR_GIMBAL_INIT_DELAY);
    float feedback_rpm = NAN;
    uint64_t feedback_time_us = UINT64_MAX;
    uint32_t tick = osKernelGetTickCount();
    uint32_t previous_tick = tick - delay_tick;
    while (true)
    {
        // 取得实际调度周期与单调时间，供模块判断控制间隔和反馈时效
        const uint32_t now_tick = osKernelGetTickCount();
        const float period_s = (float)(now_tick - previous_tick) / (float)tick_frequency;
        previous_tick = now_tick;
        const uint64_t now_us = BSP_TIME_Get();

        // 缓存最新反馈，将驱动的低 32 位时间戳恢复为完整时刻，空邮箱继续使用最近有效帧
        MOTOR_RM_Feedback_t feedback;
        const osStatus_t status = osMessageQueueGet(task_runtime.msgq.gimbal_yaw_feedback, &feedback, NULL, 0U);
        if (status == osOK)
        {
            const uint32_t age_us = (uint32_t)now_us - feedback.last_update_time;
            feedback_time_us = now_us >= age_us ? now_us - age_us : UINT64_MAX;
            feedback_rpm = feedback.rotor_speed;
        }
        else if (status != osErrorResource)
        {
            feedback_time_us = UINT64_MAX;
            feedback_rpm = NAN;
        }

        // 速度单位转换、反馈超时、参数校验、状态切换与 PID 均由模块处理
        const float current_a = Gimbal_YawSpeedRunAt(feedback_rpm, feedback_time_us, now_us, true, period_s);

        // 电流邮箱只保留最新输出，发布失败时清空控制历史并由 CAN 的空邮箱路径归零
        if (osMessageQueueReset(task_runtime.msgq.gimbal_yaw_command) != osOK ||
            osMessageQueuePut(task_runtime.msgq.gimbal_yaw_command, &current_a, 0U, 0U) != osOK)
        {
            Gimbal_YawSpeedRunAt(feedback_rpm, feedback_time_us, now_us, false, period_s);
        }

        // 正常运行使用绝对节拍，错过周期时跳过补算，避免追赶产生连续重复输出
        if ((uint32_t)(now_tick - tick) >= delay_tick)
        {
            tick = now_tick;
        }
        tick += delay_tick;
        osDelayUntil(tick);
    }
}

/**
 * @brief 初始化 yaw 速度环，失败时阻止业务任务创建
 *
 * @return 控制器初始化成功且内核节拍支持目标频率时返回 true，否则返回 false
 */
bool Task_motor_gimbal_Init(void)
{
    const uint32_t tick_frequency = osKernelGetTickFreq();
    if (tick_frequency < MOTOR_GIMBAL_FREQ || tick_frequency % MOTOR_GIMBAL_FREQ != 0U)
    {
        return false;
    }
    return Gimbal_Init((float)MOTOR_GIMBAL_FREQ) == 0;
}
