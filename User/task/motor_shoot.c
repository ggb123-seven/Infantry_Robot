#include "task/motor_shoot.h"

#include "bsp/time.h"
#include "module/config.h"
#include "task/user_task.h"

#include <stddef.h>
#include <string.h>

// 双摩擦轮实例及最近消费的命令由本任务独占，跨任务交互只经过邮箱
Shoot_t shoot;
Shoot_Command_t shoot_command;

/**
 * @brief 周期编排双摩擦轮反馈、速度控制与电流邮箱发布
 *
 * @param[in] argument 任务参数，本任务不使用
 * @return 本任务不会返回
 */
void Task_motor_shoot(void *argument)
{
    (void)argument;

    // 节拍不能精确表达控制频率时保留任务并持续发布两轮零电流
    const uint32_t tick_frequency = osKernelGetTickFreq();
    const bool timing_valid = tick_frequency >= MOTOR_SHOOT_FREQ && tick_frequency % MOTOR_SHOOT_FREQ == 0U;
    const uint32_t delay_tick = timing_valid ? tick_frequency / MOTOR_SHOOT_FREQ : 1U;
    osDelay(MOTOR_SHOOT_INIT_DELAY);
    const int8_t init_status = Shoot_Init(&shoot, &shoot_config, timing_valid ? (float)MOTOR_SHOOT_FREQ : 0.0f);

    // 首周期使用名义间隔，后续使用实际无符号节拍差
    uint32_t tick = osKernelGetTickCount();
    uint32_t previous_tick = tick - delay_tick;
    while (true)
    {
        // 两路反馈分别消费后再取时刻，模块独立检查各轮反馈时效
        MOTOR_RM_Feedback_t fric1;
        MOTOR_RM_Feedback_t fric2;
        const bool received1 = task_runtime.msgq.shoot_fric1_feedback != NULL &&
            osMessageQueueGet(task_runtime.msgq.shoot_fric1_feedback, &fric1, NULL, 0U) == osOK;
        const bool received2 = task_runtime.msgq.shoot_fric2_feedback != NULL &&
            osMessageQueueGet(task_runtime.msgq.shoot_fric2_feedback, &fric2, NULL, 0U) == osOK;
        Shoot_UpdateFeedback(&shoot, received1 ? &fric1 : NULL, received2 ? &fric2 : NULL, BSP_TIME_Get());

        // 只采用本周期新命令，失去命令时两轮一起失能
        memset(&shoot_command, 0, sizeof(shoot_command));
        if (task_runtime.msgq.shoot_command == NULL ||
            osMessageQueueGet(task_runtime.msgq.shoot_command, &shoot_command, NULL, 0U) != osOK)
        {
            memset(&shoot_command, 0, sizeof(shoot_command));
        }

        // 状态机和两轮闭环均由模块执行，任务只计算实测周期和导出电流
        const uint32_t now_tick = osKernelGetTickCount();
        const float period_s = timing_valid ? (float)(now_tick - previous_tick) / (float)tick_frequency : 0.0f;
        previous_tick = now_tick;
        Shoot_Output_t output;
        Shoot_Control(&shoot, init_status == 0 ? &shoot_command : NULL, period_s);
        Shoot_DumpOutput(&shoot, &output);

        // 双轮电流保持同一消息，发布失败时清除本地控制历史
        if (task_runtime.msgq.shoot_output == NULL ||
            osMessageQueueReset(task_runtime.msgq.shoot_output) != osOK ||
            osMessageQueuePut(task_runtime.msgq.shoot_output, &output, 0U, 0U) != osOK)
        {
            Shoot_Control(&shoot, NULL, period_s);
        }

        // 延迟时跳过错过的周期，禁止连续补算历史输出
        if ((uint32_t)(now_tick - tick) >= delay_tick)
        {
            tick = now_tick;
        }
        tick += delay_tick;
        // 等待绝对周期边界，保持与总线通信相同的运行频率
        osDelayUntil(tick);
    }
}
