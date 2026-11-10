#include "task/motor_trigger.h"

#include "bsp/time.h"
#include "module/cmd.h"
#include "module/shoot.h"
#include "module/trigger.h"
#include "task/user_task.h"

#include <stddef.h>

// 备弹与开火档的双摩擦轮转子目标为 8000 rpm，安全档持续零速闭环
#define MOTOR_TRIGGER_SHOOT_SPEED_RPM (8000.0f)

// 遥控进给模式在线入口，每周期只读一次，0 连发、1 单发、2 五连发
volatile uint8_t feeder_remote_fire_mode = FEEDER_FIRE_MODE_BURST;

/*
 * feeder 为任务独占的拨弹观测实例，config 用于在线调参，控制历史由拨弹模块私有保存
 * feeder_command 为命令层每周期生成的拨弹命令，speed_ratio 为连发归一化转速倍率
 * feeder.target_angle_deg 为模块持有的逐发目标，feeder.target_speed_rpm 与 measured_speed_rpm 为速度内环目标与实测
 * 角度连续性失效时角度观察量为非数值，帧超时后角度和转速均为非数值
 * 右拨杆上位安全、中位备弹、下位开火，右摇杆仅控制云台
 * trigger_command_context 保存命令层模式选择和回中阶段，单发执行阶段由拨弹模块独占
 * trigger_remote 为最近消费的遥控快照，源时间保持不变，空邮箱不撤销有效缓存
 * trigger_remote_valid 表示缓存可用，除空邮箱外的读取错误会撤销缓存有效性
 * trigger_output_published 表示上周期拨弹输出与摩擦轮命令均发布成功，失败后下一周期不提交拨弹命令
 */
Feeder_t feeder;
DR16_State_t trigger_remote;
Feeder_Command_t feeder_command;
static bool trigger_remote_valid;
static bool trigger_output_published = true;
static Cmd_Feeder_t trigger_command_context;

static Shoot_Command_t MotorTrigger_ParseShootCommand(Feeder_CmdMode_t feeder_mode);

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
        const uint8_t remote_fire_mode = feeder_remote_fire_mode;
        Cmd_ParseFeeder(&trigger_command_context, &trigger_remote, trigger_remote_valid, now_us,
                         remote_fire_mode, &feeder_command);
        const uint32_t now_tick = osKernelGetTickCount();
        const float period_s = (float)(now_tick - previous_tick) / (float)tick_frequency;
        previous_tick = now_tick;
        Feeder_Control(&feeder, trigger_output_published ? &feeder_command : NULL, period_s);
        const Trigger_Output_t output =
        {
            .current_a = Feeder_DumpOutput(&feeder),
        };

        // 备弹和开火保持摩擦轮运转，安全档持续零速闭环，离线立即撤驱
        const Shoot_Command_t shoot_command = MotorTrigger_ParseShootCommand(feeder_command.mode);
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

/**
 * @brief 将已校验的档位映射为双摩擦轮运行、持续零速闭环或撤驱
 *
 * @param[in] feeder_mode 命令层已校验的档位，离线档包含遥控失效和非法输入
 * @return 备弹与开火为 8000 rpm，安全为使能零速，离线为失能零速
 */
static Shoot_Command_t MotorTrigger_ParseShootCommand(Feeder_CmdMode_t feeder_mode)
{
    // 复用命令层边界检查，摩擦轮不再单独保存拨杆转换或定时撤驱状态
    Shoot_Command_t command =
    {
        0,
    };
    switch (feeder_mode)
    {
        case FEEDER_CMD_RUN:
            command.enabled = true;
            command.target_speed_rpm[0] = MOTOR_TRIGGER_SHOOT_SPEED_RPM;
            command.target_speed_rpm[1] = MOTOR_TRIGGER_SHOOT_SPEED_RPM;
            break;

        case FEEDER_CMD_RELAX:
            command.enabled = true;
            break;

        default:
            break;
    }
    return command;
}
