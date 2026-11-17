/*
 * 云台任务
 *
 * 每周期消费惯性快照、两轴反馈与遥控命令，推进角度速度串级控制
 * 输出两轴电流；发布失败时同步停机，避免零输出期间积分累计
 */

#include "task/motor_gimbal.h"

#include <stddef.h>

#include "bsp/time.h"
#include "module/cmd.h"
#include "device/dr16.h"
#include "module/config.h"
#include "task/user_task.h"

/* 调试观测与命令入口，声明及使用方式见 motor_gimbal.h */
Gimbal_t gimbal;
Gimbal_Command_t gimbal_command =
{
    .mode = GIMBAL_MODE_RELAX,
};
volatile bool gimbal_debug_enabled = false;
volatile bool gimbal_remote_speed_loop_enabled = true;
volatile bool gimbal_remote_angle_loop_enabled = true;
#if MOTOR_GIMBAL_ENABLE_SINE_TEST
volatile bool gimbal_sine_test_enabled = false;
volatile bool gimbal_sine_feedforward_enabled = true;

#endif

/* 遥控缓存及输入状态仅由云台任务访问，不与 ISR 共享 */
static DR16_State_t remote_cache;
static bool remote_cache_valid;
static Cmd_Gimbal_t remote_command;
/* 底盘状态仅由本任务消费，零初始化对应离线，在线未自旋与在线小陀螺独立表示 */
static Gimbal_ChassisState_t chassis_state_cache;

/* Private function --------------------------------------------------------- */
static bool MotorGimbal_ReadFeedback(osMessageQueueId_t queue, MOTOR_RM_Feedback_t *feedback);
static bool MotorGimbal_PublishOutput(const Gimbal_Output_t *output);

/* Exported functions ------------------------------------------------------- */
/**
 * @brief 初始化云台模块并周期执行反馈读取、控制计算和电流发布
 *
 * @param argument 任务参数，本任务不使用
 * @return 无返回值
 */
void Task_motor_gimbal(void *argument)
{
    (void)argument;

    const uint32_t tick_frequency = osKernelGetTickFreq();
    const uint32_t delay_tick = tick_frequency / MOTOR_GIMBAL_FREQ;

    /* 等待统一装配完成后初始化云台模块，失败时只发布零电流 */
    osDelay(MOTOR_GIMBAL_INIT_DELAY);
    const bool initialized = Gimbal_Init(&gimbal, &gimbal_config, (float)MOTOR_GIMBAL_FREQ) == 0;
    uint32_t tick = osKernelGetTickCount();
    uint32_t previous_tick = tick - delay_tick;
    while (true)
    {
        /* 按实际调度间隔计算控制周期，错过节拍时由模块拒绝过长周期 */
        const uint32_t now_tick = osKernelGetTickCount();
        const float period_s = (float)(now_tick - previous_tick) / (float)tick_frequency;
        previous_tick = now_tick;
        Gimbal_Output_t output =
        {
            0,
        };

        if (initialized)
        {
            /* 消费完整惯性快照，无新消息时由模块保留源时间并检查帧龄 */
            Gimbal_ImuFeedback_t imu;
            const osMessageQueueId_t imu_queue = task_runtime.msgq.gimbal_imu;
            const bool imu_received = osMessageQueueGet(imu_queue, &imu, NULL, 0U) == osOK;

            /* 消费两轴最新反馈帧，邮箱为空时传 NULL 由模块检查旧帧时效 */
            MOTOR_RM_Feedback_t yaw_feedback;
            MOTOR_RM_Feedback_t pitch_feedback;
            const bool yaw_received = MotorGimbal_ReadFeedback(task_runtime.msgq.gimbal_yaw_feedback, &yaw_feedback);
            const bool pitch_received = MotorGimbal_ReadFeedback(
                task_runtime.msgq.gimbal_pitch_feedback, &pitch_feedback);
            Gimbal_UpdateFeedback(&gimbal, yaw_received ? &yaw_feedback : NULL,
                                  pitch_received ? &pitch_feedback : NULL, imu_received ? &imu : NULL, BSP_TIME_Get());

            /* 底盘在线与小陀螺共用同一状态快照，空邮箱保留上次状态，读取错误按离线确认 */
            Gimbal_ChassisState_t received_state;
            const osStatus_t chassis_status = osMessageQueueGet(task_runtime.msgq.chassis_gimbal, &received_state,
                                                                NULL, 0U);
            if (chassis_status == osOK)
            {
                chassis_state_cache = received_state;
            }
            else if (chassis_status != osErrorResource)
            {
                chassis_state_cache = GIMBAL_CHASSIS_OFFLINE;
            }

            /* 每周期固定命令来源，遥控来源检查在线许可，手动来源沿用独立调试路径 */
            const bool debug_enabled = gimbal_debug_enabled;
            if (!debug_enabled)
            {
                /* 接收最新遥控帧并保留源时间，读取失败时撤销缓存 */
                DR16_State_t received;
                const osStatus_t status = osMessageQueueGet(task_runtime.msgq.dr16_gimbal_state, &received, NULL, 0U);
                if (status == osOK)
                {
                    remote_cache = received;
                    remote_cache_valid = true;
                }
                else if (status != osErrorResource)
                {
                    remote_cache_valid = false;
                }

                /* 根据遥控快照和环路开关生成本周期命令 */
                const uint64_t now_us = BSP_TIME_Get();
                Gimbal_Command_t parsed_command;
                remote_cache_valid = Cmd_ParseGimbal(&remote_command, &remote_cache, remote_cache_valid,
                    now_us, period_s, Gimbal_HasLatchedFault(&gimbal), &parsed_command);
                const bool speed_enabled = gimbal_remote_speed_loop_enabled;
                const bool angle_enabled = gimbal_remote_angle_loop_enabled;
                Cmd_SelectGimbalLoops(&parsed_command, speed_enabled, angle_enabled);
                gimbal_command = parsed_command;
            }
#if MOTOR_GIMBAL_ENABLE_SINE_TEST
            /* 由云台模块生成遥控测试轨迹，手动调试时重置轨迹状态 */
            Cmd_UpdateGimbalSineTest(&gimbal, &gimbal_command, period_s, debug_enabled,
                                     gimbal_sine_test_enabled, gimbal_sine_feedforward_enabled);
#endif
            /* 从调试可写命令读取本周期快照，模块统一校验并执行闭环与保护后导出电流 */
            const Gimbal_Command_t command = *(const volatile Gimbal_Command_t *)&gimbal_command;
            Gimbal_Control(&gimbal, &command, chassis_state_cache, period_s);
            Gimbal_DumpOutput(&gimbal, &output);
        }

        /* 发布失败时 CAN 任务按空邮箱输出零电流，同步让两轴停机，避免积分在零输出期间累计 */
        if (!MotorGimbal_PublishOutput(&output) && initialized)
        {
            Gimbal_Command_t relax_command =
            {
                .mode = GIMBAL_MODE_RELAX,
            };
            Gimbal_Control(&gimbal, &relax_command, chassis_state_cache, period_s);
            Cmd_ResetGimbal(&remote_command);
            gimbal_command.mode = GIMBAL_MODE_RELAX;
            gimbal_command.yaw.rate_rad_s = 0.0f;
            gimbal_command.pitch.rate_rad_s = 0.0f;
        }

        /* 正常运行使用绝对节拍，错过周期时跳过补算，避免追赶产生连续重复输出 */
        if ((uint32_t)(now_tick - tick) >= delay_tick)
        {
            tick = now_tick;
        }
        tick += delay_tick;
        osDelayUntil(tick);
    }
}

/* Private function --------------------------------------------------------- */
/**
 * @brief 从容量为 1 的反馈邮箱取得一帧新反馈
 *
 * @param queue 元素为 MOTOR_RM_Feedback_t 的反馈邮箱
 * @param feedback 新反馈帧
 * @return bool 取得新帧时返回 true，邮箱为空或读取失败时返回 false
 */
static bool MotorGimbal_ReadFeedback(osMessageQueueId_t queue, MOTOR_RM_Feedback_t *feedback)
{
    return osMessageQueueGet(queue, feedback, NULL, 0U) == osOK;
}

/**
 * @brief 向容量为 1 的云台输出邮箱发布最新电流
 *
 * @param output 本周期两轴输出电流
 * @return bool 邮箱重置并写入成功时返回 true，否则返回 false
 */
static bool MotorGimbal_PublishOutput(const Gimbal_Output_t *output)
{
    osMessageQueueId_t queue = task_runtime.msgq.gimbal_output;
    return osMessageQueueReset(queue) == osOK && osMessageQueuePut(queue, output, 0U, 0U) == osOK;
}
