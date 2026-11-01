#include "task/motor_trigger.h"

#include "bsp/time.h"
#include "module/trigger.h"
#include "task/user_task.h"

#include <stddef.h>

/*
 * feeder 为任务独占的逐齿位置与速度串级实例，config 用于在线调参
 * 位置内部使用弧度，角度镜像使用度数，速度内环与前馈目标使用输出轴 rpm
 * trigger_remote 为最近消费的遥控快照，源时间保持不变，空邮箱不撤销有效缓存
 * trigger_remote_valid 表示缓存可用，除空邮箱外的读取错误会撤销缓存有效性
 * trigger_output_published 为上周期电流发布结果，失败后下一周期撤销请求以清除控制历史
 */
Feeder_t feeder;
DR16_State_t trigger_remote;
static bool trigger_remote_valid;
static bool trigger_output_published = true;

/**
 * @brief 周期编排拨弹遥控、反馈、逐齿串级控制与电流邮箱发布
 *
 * @param[in] argument 任务参数，本任务不使用
 * @return 本任务不会返回
 */
void Task_motor_trigger(void *argument)
{
    (void)argument;

    // 建立固定控制节拍，模块保存初始化结果并保证失败时的安全输出
    const uint32_t tick_frequency = osKernelGetTickFreq();
    const uint32_t delay_tick = tick_frequency / MOTOR_TRIGGER_FREQ;
    osDelay(MOTOR_TRIGGER_INIT_DELAY);
    Feeder_Init(&feeder, (float)MOTOR_TRIGGER_FREQ);

    // 首周期使用名义间隔，后续按无符号节拍差计算以支持计时回绕
    uint32_t tick = osKernelGetTickCount();
    uint32_t previous_tick = tick - delay_tick;
    while (true)
    {
        // 遥控邮箱只在新帧或状态变化时发布，空邮箱保留缓存且不刷新源时间
        DR16_State_t received_remote;
        const osStatus_t remote_status =
            osMessageQueueGet(task_runtime.msgq.trigger_command, &received_remote, NULL, 0U);
        if (remote_status == osOK)
        {
            trigger_remote = received_remote;
            trigger_remote_valid = true;
        }
        else if (remote_status != osErrorResource)
        {
            trigger_remote_valid = false;
        }

        // 在读取快照后采时，反馈合法性和缓存时效只在模块输入边界判断
        MOTOR_RM_Feedback_t feedback;
        const bool received = osMessageQueueGet(task_runtime.msgq.trigger_feedback, &feedback, NULL, 0U) == osOK;
        const uint64_t now_us = BSP_TIME_Get();
        Feeder_UpdateFeedback(&feeder, &feedback, received, now_us);

        // 模块统一处理配置快照、遥控时效、状态转换和串级闭环，发布失败后先撤销一次请求
        const uint32_t now_tick = osKernelGetTickCount();
        const float period_s = (float)(now_tick - previous_tick) / (float)tick_frequency;
        previous_tick = now_tick;
        Feeder_Control(&feeder, &trigger_remote, trigger_remote_valid && trigger_output_published, now_us, period_s);
        const Trigger_Output_t output =
        {
            .current_a = Feeder_DumpOutput(&feeder),
        };

        // CAN 任务独占总线输出，记录发布结果供下一周期撤驱，禁止在任务内运行第二次控制
        trigger_output_published = osMessageQueueReset(task_runtime.msgq.trigger_output) == osOK &&
            osMessageQueuePut(task_runtime.msgq.trigger_output, &output, 0U, 0U) == osOK;

        // 错过周期时跳过追赶计算，避免同一时刻推进多个齿距
        if ((uint32_t)(now_tick - tick) >= delay_tick)
        {
            tick = now_tick;
        }
        tick += delay_tick;
        // 等待绝对周期边界，与 CAN 通信保持相同运行频率
        osDelayUntil(tick);
    }
}
