#include "task/motor_trigger.h"

#include "bsp/time.h"
#include "module/cmd.h"
#include "module/shoot.h"
#include "module/trigger.h"
#include "task/user_task.h"

#include <stddef.h>

// 拨弹运行时两只摩擦轮的转子目标转速，单位 rpm
#define MOTOR_TRIGGER_SHOOT_SPEED_RPM (8000.0f)

/*
 * feeder 为任务独占的拨弹观测实例，config 用于在线调参，控制历史由拨弹模块私有保存
 * feeder_command 为命令层每周期生成的拨弹命令，speed_ratio 为连发归一化转速倍率
 * feeder.target_angle_deg 为手动目标角，feeder.target_speed_rpm 与 measured_speed_rpm 为速度内环目标与实测
 * 角度连续性失效时角度观察量为非数值，帧超时后角度和转速均为非数值
 * 右拨杆下位为运行使能，连发时摇杆回中撤驱，模式 1 忽略摇杆并采用手动目标角，其余档位及失联撤驱
 * trigger_remote 为最近消费的遥控快照，源时间保持不变，空邮箱不撤销有效缓存
 * trigger_remote_valid 表示缓存可用，除空邮箱外的读取错误会撤销缓存有效性
 * trigger_output_published 表示上周期拨弹输出与摩擦轮命令均发布成功，失败后下一周期不提交拨弹命令
 */
Feeder_t feeder;
DR16_State_t trigger_remote;
Feeder_Command_t feeder_command;
static bool trigger_remote_valid;
static bool trigger_output_published = true;

/**
 * @brief 周期编排拨弹遥控、反馈控制及摩擦轮联动命令发布
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

        // 命令层把遥控快照翻译成拨弹命令，发布失败时本周期不提交命令
        Cmd_ParseFeeder(&trigger_remote, trigger_remote_valid, now_us, &feeder_command);
        const uint32_t now_tick = osKernelGetTickCount();
        const float period_s = (float)(now_tick - previous_tick) / (float)tick_frequency;
        previous_tick = now_tick;
        Feeder_Control(&feeder, trigger_output_published ? &feeder_command : NULL, period_s);
        const Trigger_Output_t output =
        {
            .current_a = Feeder_DumpOutput(&feeder),
        };

        // 右拨杆下位直接启动两只摩擦轮，遥控失效或上一周期发布失败时保持失能
        const Shoot_Command_t shoot_command =
        {
            .enabled = trigger_output_published && feeder_command.mode == FEEDER_CMD_RUN,
            .target_speed_rpm =
            {
                MOTOR_TRIGGER_SHOOT_SPEED_RPM,
                MOTOR_TRIGGER_SHOOT_SPEED_RPM,
            },
        };
        const bool shoot_command_published = osMessageQueueReset(task_runtime.msgq.shoot_command) == osOK &&
            osMessageQueuePut(task_runtime.msgq.shoot_command, &shoot_command, 0U, 0U) == osOK;

        // CAN 任务独占总线输出，记录发布结果供下一周期撤驱，禁止在任务内运行第二次控制
        trigger_output_published = shoot_command_published &&
            osMessageQueueReset(task_runtime.msgq.trigger_output) == osOK &&
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
