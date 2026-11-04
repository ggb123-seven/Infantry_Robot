#include "task/motor_shoot.h"

#include "bsp/time.h"
#include "module/config.h"
#include "task/user_task.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

/*
 * 停稳撤驱首版判据：两轮转子速度绝对值均不超过 1 rpm，各自新反馈连续覆盖至少 20 ms
 * 阈值用于实现用户要求的停稳后关闭速度环，尚待实物验证；任一轮超速或缺新反馈重新确认
 * 计时采用模块已接纳的完整反馈源时间，重复读取旧零速反馈不能累计确认时间
 */
#define MOTOR_SHOOT_STOP_SPEED_RPM (1.0f)
#define MOTOR_SHOOT_STOP_SETTLE_US (20000ULL)

// 空闲、制动、停稳确认和已撤驱互斥，撤驱后持续零目标不再重新开启速度环
typedef enum
{
    MOTOR_SHOOT_STOP_IDLE = 0,
    MOTOR_SHOOT_STOP_BRAKING,
    MOTOR_SHOOT_STOP_CONFIRMING,
    MOTOR_SHOOT_STOPPED,
} MotorShoot_StopState_t;

/*
 * 双摩擦轮实例及最近消费的命令由本任务独占，跨任务交互只经过邮箱
 * shoot.config.speed_pid 为直接复制 shoot_config 的可调转子 rpm 速度参数
 * shoot_speed_feedforward_gain_a_per_rpm 为既有目标差分前馈入口，单位 A/rpm
 * shoot_command 为本周期实际提交模块的命令，停稳撤驱后 enabled 为 false
 * shoot_stop_state 保存零速命令的停稳撤驱阶段，shoot_stop_since_us 保存两轮确认起点，单位微秒
 */
Shoot_t shoot;
Shoot_Command_t shoot_command;
static MotorShoot_StopState_t shoot_stop_state = MOTOR_SHOOT_STOP_IDLE;
static uint64_t shoot_stop_since_us[SHOOT_MOTOR_COUNT];

static void MotorShoot_UpdateStopState(bool command_received, bool brake_requested, bool fresh_feedback);

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
        Shoot_UpdateFeedback(&shoot, received1 ? &fric1 : NULL, received2 ? &fric2 : NULL, BSP_TIME_Get());

        // 只采用本周期新命令，失去命令时两轮一起失能
        const bool command_received =
            osMessageQueueGet(task_runtime.msgq.shoot_command, &shoot_command, NULL, 0U) == osOK;
        if (!command_received)
        {
            memset(&shoot_command, 0, sizeof(shoot_command));
        }

        // 已完成的零速制动只在新非零目标或明确失能命令后解除，空邮箱不复活速度环
        const bool brake_requested = shoot_command.enabled &&
            shoot_command.target_speed_rpm[0] == 0.0f && shoot_command.target_speed_rpm[1] == 0.0f;
        switch (shoot_stop_state)
        {
            case MOTOR_SHOOT_STOP_IDLE:
            case MOTOR_SHOOT_STOP_BRAKING:
            case MOTOR_SHOOT_STOP_CONFIRMING:
                break;

            case MOTOR_SHOOT_STOPPED:
                shoot_command.enabled = shoot_command.enabled && !brake_requested;
                break;

            default:
                shoot_command.enabled = false;
                break;
        }

        // 模块集中校验本周期输入并计算电流，只有成功闭环且两轮均有新反馈才推进停稳确认
        const uint32_t now_tick = osKernelGetTickCount();
        const float period_s = (float)(now_tick - previous_tick) / (float)tick_frequency;
        previous_tick = now_tick;
        Shoot_Output_t output;
        Shoot_Control(&shoot, &shoot_command, period_s);
        MotorShoot_UpdateStopState(command_received, brake_requested, received1 && received2);
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
 * @brief 根据已完成控制的模块状态和双轮反馈确认零速制动完成
 *
 * 停稳后下周期提交失能命令，由模块清除 PID 历史并通过唯一输出闸门发布零电流
 *
 * @param[in] command_received 本周期成功读取新命令时为 true
 * @param[in] brake_requested 原始命令使能且两轮目标均为零时为 true
 * @param[in] fresh_feedback 本周期两轮均收到反馈时为 true，模块运行状态保证其已被接纳
 * @return 无返回值
 */
static void MotorShoot_UpdateStopState(bool command_received, bool brake_requested, bool fresh_feedback)
{
    // 仅消费模块已校验的结果，不重复校验配置、反馈源时间或控制周期
    switch (shoot_stop_state)
    {
        case MOTOR_SHOOT_STOP_IDLE:
        case MOTOR_SHOOT_STOP_BRAKING:
        case MOTOR_SHOOT_STOP_CONFIRMING:
            if (!brake_requested)
            {
                shoot_stop_state = MOTOR_SHOOT_STOP_IDLE;
            }
            else if (shoot.state != SHOOT_RUNNING || !fresh_feedback)
            {
                shoot_stop_state = MOTOR_SHOOT_STOP_BRAKING;
            }
            else if (fabsf(shoot.feedback[0].rotor_speed) > MOTOR_SHOOT_STOP_SPEED_RPM ||
                     fabsf(shoot.feedback[1].rotor_speed) > MOTOR_SHOOT_STOP_SPEED_RPM)
            {
                shoot_stop_state = MOTOR_SHOOT_STOP_BRAKING;
            }
            else if (shoot_stop_state != MOTOR_SHOOT_STOP_CONFIRMING)
            {
                shoot_stop_since_us[0] = shoot.feedback_time_us[0];
                shoot_stop_since_us[1] = shoot.feedback_time_us[1];
                shoot_stop_state = MOTOR_SHOOT_STOP_CONFIRMING;
            }
            else if (shoot.feedback_time_us[0] - shoot_stop_since_us[0] >= MOTOR_SHOOT_STOP_SETTLE_US &&
                     shoot.feedback_time_us[1] - shoot_stop_since_us[1] >= MOTOR_SHOOT_STOP_SETTLE_US)
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
