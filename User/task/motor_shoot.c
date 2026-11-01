#include "task/motor_shoot.h"

#include "bsp/time.h"
#include "module/config.h"
#include "task/user_task.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

/*
 * 双摩擦轮实例及最近消费的命令由本任务独占，跨任务交互只经过邮箱
 * shoot.config.speed_pid 为可调转子 rpm 速度参数，初值复制底盘参数并服从摩擦轮电流上限
 * shoot_speed_feedforward_gain_a_per_rpm 为既有目标差分前馈入口，单位 A/rpm
 */
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

    // 固定频率由编译期断言保证可表达，运行期只记录实际控制周期
    const uint32_t tick_frequency = osKernelGetTickFreq();
    const uint32_t delay_tick = tick_frequency / MOTOR_SHOOT_FREQ;
    osDelay(MOTOR_SHOOT_INIT_DELAY);
    // 从配置源复制边界并采用底盘速度参数，模块再复制为实例内独立的在线可调配置
    Shoot_Config_t initial_config = shoot_config;
    initial_config.speed_pid = chassis_config.wheel_speed_pid;
    initial_config.speed_pid.out_limit = fminf(initial_config.speed_pid.out_limit, initial_config.max_current_a);
    Shoot_Init(&shoot, &initial_config, (float)MOTOR_SHOOT_FREQ);

    // 首周期使用名义间隔，后续使用实际无符号节拍差
    uint32_t tick = osKernelGetTickCount();
    uint32_t previous_tick = tick - delay_tick;
    while (true)
    {
        // 两路反馈分别消费后再取时刻，模块独立检查各轮反馈时效
        MOTOR_RM_Feedback_t fric1;
        MOTOR_RM_Feedback_t fric2;
        const bool received1 = osMessageQueueGet(task_runtime.msgq.shoot_fric1_feedback, &fric1, NULL, 0U) == osOK;
        const bool received2 = osMessageQueueGet(task_runtime.msgq.shoot_fric2_feedback, &fric2, NULL, 0U) == osOK;
        Shoot_UpdateFeedback(&shoot, received1 ? &fric1 : NULL, received2 ? &fric2 : NULL, BSP_TIME_Get());

        // 只采用本周期新命令，失去命令时两轮一起失能
        if (osMessageQueueGet(task_runtime.msgq.shoot_command, &shoot_command, NULL, 0U) != osOK)
        {
            memset(&shoot_command, 0, sizeof(shoot_command));
        }

        // 状态机和两轮闭环均由模块执行，任务只计算实测周期和导出电流
        const uint32_t now_tick = osKernelGetTickCount();
        const float period_s = (float)(now_tick - previous_tick) / (float)tick_frequency;
        previous_tick = now_tick;
        Shoot_Output_t output;
        Shoot_Control(&shoot, &shoot_command, period_s);
        Shoot_DumpOutput(&shoot, &output);

        // 双轮电流保持同一消息，发布失败时清除本地控制历史
        if (osMessageQueueReset(task_runtime.msgq.shoot_output) != osOK ||
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
