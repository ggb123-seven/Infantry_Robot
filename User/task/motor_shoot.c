#include "task/motor_shoot.h"

#include "bsp/time.h"
#include "module/config.h"
#include "task/user_task.h"

#include <stddef.h>
#include <string.h>

/*
 * 下位回中位首次收到零速制动命令时开始计时，经过 3 秒后关闭速度环
 * 时长由用户确认，单位微秒；只按实际经过时间撤驱，不再判断两轮转速是否稳定
 * 重复零速命令、反馈波动和空命令邮箱不重置计时，上位失能或新非零目标取消本轮制动
 */
#define MOTOR_SHOOT_BRAKE_DURATION_US (3000000ULL)

// 空闲、定时制动和已撤驱互斥，撤驱后持续零目标不再重新开启速度环
typedef enum
{
    MOTOR_SHOOT_STOP_IDLE = 0,
    MOTOR_SHOOT_STOP_BRAKING,
    MOTOR_SHOOT_STOPPED,
} MotorShoot_StopState_t;

/*
 * 双摩擦轮实例及最近消费的命令由本任务独占，跨任务交互只经过邮箱
 * shoot.config.speed_pid 为直接复制 shoot_config 的可调转子 rpm 速度参数
 * shoot_speed_feedforward_gain_a_per_rpm 为既有目标差分前馈入口，单位 A/rpm
 * shoot_command 为本周期实际提交模块的命令，制动满 3 秒后 enabled 为 false
 * shoot_stop_state 保存定时制动阶段，shoot_stop_since_us 保存本轮制动起点，单位微秒
 */
Shoot_t shoot;
Shoot_Command_t shoot_command;
static MotorShoot_StopState_t shoot_stop_state = MOTOR_SHOOT_STOP_IDLE;
static uint64_t shoot_stop_since_us;

static void MotorShoot_UpdateStopState(bool command_received, bool brake_requested, uint64_t now_us);

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
    // 摩擦轮 PID 直接沿用 config.c 的独立默认值，模块再复制为实例内在线可调配置
    Shoot_Init(&shoot, &shoot_config, (float)MOTOR_SHOOT_FREQ);

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
        const uint64_t now_us = BSP_TIME_Get();
        Shoot_UpdateFeedback(&shoot, received1 ? &fric1 : NULL, received2 ? &fric2 : NULL, now_us);

        // 只采用本周期新命令，失去命令时两轮一起失能
        const bool command_received =
            osMessageQueueGet(task_runtime.msgq.shoot_command, &shoot_command, NULL, 0U) == osOK;
        if (!command_received)
        {
            memset(&shoot_command, 0, sizeof(shoot_command));
        }

        // 按首次零速命令计时，满 3 秒的本周期即提交失能，持续中位和空邮箱不重新使能
        const bool brake_requested = shoot_command.enabled &&
            shoot_command.target_speed_rpm[0] == 0.0f && shoot_command.target_speed_rpm[1] == 0.0f;
        MotorShoot_UpdateStopState(command_received, brake_requested, now_us);
        if (shoot_stop_state == MOTOR_SHOOT_STOPPED)
        {
            shoot_command.enabled = false;
        }

        // 模块集中校验本周期输入并计算电流，失能时清除控制历史并由唯一输出闸门撤驱
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

/**
 * @brief 从首次零速制动命令开始计时，满 3 秒锁存撤驱阶段
 *
 * 调用方保证 now_us 为本周期单调微秒时间，计时独立于反馈更新和模块运行状态
 *
 * @param[in] command_received 本周期成功读取新命令时为 true
 * @param[in] brake_requested 原始命令使能且两轮目标均为零时为 true
 * @param[in] now_us 本周期单调时间，单位微秒，来自 BSP_TIME_Get
 * @return 无返回值
 */
static void MotorShoot_UpdateStopState(bool command_received, bool brake_requested, uint64_t now_us)
{
    // 空邮箱沿用计时阶段，明确的新命令才能取消计时或解除已撤驱阶段
    switch (shoot_stop_state)
    {
        case MOTOR_SHOOT_STOP_IDLE:
            if (brake_requested)
            {
                shoot_stop_since_us = now_us;
                shoot_stop_state = MOTOR_SHOOT_STOP_BRAKING;
            }
            break;

        case MOTOR_SHOOT_STOP_BRAKING:
            if (command_received && !brake_requested)
            {
                shoot_stop_state = MOTOR_SHOOT_STOP_IDLE;
            }
            else if (now_us - shoot_stop_since_us >= MOTOR_SHOOT_BRAKE_DURATION_US)
            {
                shoot_stop_state = MOTOR_SHOOT_STOPPED;
            }
            break;

        case MOTOR_SHOOT_STOPPED:
            if (command_received && !brake_requested)
            {
                shoot_stop_state = MOTOR_SHOOT_STOP_IDLE;
            }
            break;

        default:
            shoot_stop_state = MOTOR_SHOOT_STOPPED;
            break;
    }
}
