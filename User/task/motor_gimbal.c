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
        // 按实际调度间隔驱动模块，外部角速度未接入时沿用编码器反馈，测试轨迹与前馈由模块生成
        const uint32_t now_tick = osKernelGetTickCount();
        const float period_s = (float)(now_tick - previous_tick) / (float)tick_frequency;
        previous_tick = now_tick;
        Gimbal_Update(period_s, 0.0f, false, 0.0f);

        // 正常运行使用绝对节拍，错过周期时跳过补算，避免追赶产生连续重复输出
        if ((uint32_t)(now_tick - tick) >= delay_tick)
        {
            tick = now_tick;
        }
        tick += delay_tick;
        osDelayUntil(tick);
    }
}
