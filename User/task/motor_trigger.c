#include "task/motor_trigger.h"

#include "bsp/time.h"
#include "module/config.h"
#include "task/user_task.h"

#include <stddef.h>
#include <string.h>

// 拨弹实例及最近消费的命令由本任务独占，跨任务交互只经过邮箱
Trigger_t trigger;
Trigger_Command_t trigger_command;

/**
 * @brief 周期编排拨弹反馈、速度控制与电流邮箱发布
 *
 * @param[in] argument 任务参数，本任务不使用
 * @return 本任务不会返回
 */
void Task_motor_trigger(void *argument)
{
    (void)argument;

    // 节拍不能精确表达控制频率时保留任务并持续发布零电流
    const uint32_t tick_frequency = osKernelGetTickFreq();
    const bool timing_valid = tick_frequency >= MOTOR_TRIGGER_FREQ && tick_frequency % MOTOR_TRIGGER_FREQ == 0U;
    const uint32_t delay_tick = timing_valid ? tick_frequency / MOTOR_TRIGGER_FREQ : 1U;
    osDelay(MOTOR_TRIGGER_INIT_DELAY);
    const int8_t init_status = Trigger_Init(&trigger, &trigger_config, timing_valid ? (float)MOTOR_TRIGGER_FREQ : 0.0f);

    // 首周期使用名义间隔，之后使用实际无符号节拍差以支持计时回绕
    uint32_t tick = osKernelGetTickCount();
    uint32_t previous_tick = tick - delay_tick;
    while (true)
    {
        // 先读取反馈，再采完整时刻，避免抢占使合法新反馈被判断为未来数据
        MOTOR_RM_Feedback_t feedback;
        const bool received = task_runtime.msgq.trigger_feedback != NULL &&
            osMessageQueueGet(task_runtime.msgq.trigger_feedback, &feedback, NULL, 0U) == osOK;
        Trigger_UpdateFeedback(&trigger, received ? &feedback : NULL, BSP_TIME_Get());

        // 每周期仅接受新命令，邮箱为空或操作失败时明确失能
        memset(&trigger_command, 0, sizeof(trigger_command));
        if (task_runtime.msgq.trigger_command == NULL ||
            osMessageQueueGet(task_runtime.msgq.trigger_command, &trigger_command, NULL, 0U) != osOK)
        {
            memset(&trigger_command, 0, sizeof(trigger_command));
        }

        // 模块承担状态机与闭环，任务只计算实际周期并导出本周期电流
        const uint32_t now_tick = osKernelGetTickCount();
        const float period_s = timing_valid ? (float)(now_tick - previous_tick) / (float)tick_frequency : 0.0f;
        previous_tick = now_tick;
        Trigger_Output_t output;
        Trigger_Control(&trigger, init_status == 0 ? &trigger_command : NULL, period_s);
        Trigger_DumpOutput(&trigger, &output);

        // 最新电流只交给 CAN 任务，发布失败时复位本地控制历史
        if (task_runtime.msgq.trigger_output == NULL ||
            osMessageQueueReset(task_runtime.msgq.trigger_output) != osOK ||
            osMessageQueuePut(task_runtime.msgq.trigger_output, &output, 0U, 0U) != osOK)
        {
            Trigger_Control(&trigger, NULL, period_s);
        }

        // 跳过错过的周期，防止任务延迟后连续追赶计算
        if ((uint32_t)(now_tick - tick) >= delay_tick)
        {
            tick = now_tick;
        }
        tick += delay_tick;
        // 等待绝对周期边界，保持与总线通信相同的运行频率
        osDelayUntil(tick);
    }
}
