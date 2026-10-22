#include "task/motor_gimbal.h"

#include "module/gimbal.h"
#include "task/user_task.h"

/**
 * @brief 启动并周期调度云台模块，业务数据处理与电流发布由模块完成
 *
 * @param[in] argument 任务参数，本任务不使用
 * @return 本任务不会返回
 */
void Task_motor_gimbal(void *argument)
{
    (void)argument;

    const uint32_t tick_frequency = osKernelGetTickFreq();
    const uint32_t delay_tick = tick_frequency / MOTOR_GIMBAL_FREQ;

    // 等待统一初始化完成，再将已创建的邮箱绑定给业务模块
    osDelay(MOTOR_GIMBAL_INIT_DELAY);
    Gimbal_BindQueues(task_runtime.msgq.gimbal_yaw_feedback, task_runtime.msgq.gimbal_yaw_command);
    uint32_t tick = osKernelGetTickCount();
    uint32_t previous_tick = tick - delay_tick;
    while (true)
    {
        // 按实际调度间隔驱动业务模块，任务不处理反馈或控制输出
        const uint32_t now_tick = osKernelGetTickCount();
        const float period_s = (float)(now_tick - previous_tick) / (float)tick_frequency;
        previous_tick = now_tick;
        Gimbal_Update(period_s);

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
 * @brief 初始化 yaw 角度与速度串级控制，失败时阻止业务任务创建
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
