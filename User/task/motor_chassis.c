#include "task/motor_chassis.h"

#include "bsp/time.h"
#include "module/cmd.h"
#include "device/dr16.h"
#include "module/config.h"
#include "task/user_task.h"

#include <stddef.h>

_Static_assert(CHASSIS_WHEEL_COUNT == CAN_DEVICES_CHASSIS_MOTOR_COUNT, "底盘与 CAN 电机数量必须一致");

// 调试观测入口，字段职责见头文件；运动请求每周期由 DR16 生成
Chassis_t chassis;
Chassis_Command_t chassis_command;
FaultDetect_Snapshot_t fault_detect_snapshot;

/*
 * 底盘任务私有状态：
 * - can_feedback_cache：本周期从反馈邮箱取得的 CAN 快照，读取失败时不使用缓存
 * - dr16_state_cache：最近一次从 DR16 邮箱取得的状态，按完整微秒时间检查时效
 * - dr16_state_cache_valid：至少取得过一帧 DR16 状态时为 true
 */
static CANDevices_Snapshot_t can_feedback_cache;
static DR16_State_t dr16_state_cache;
static bool dr16_state_cache_valid;

static bool MotorChassis_ReadLatestFeedback(void);
static bool MotorChassis_ReadLatestDR16(void);
static bool MotorChassis_PublishCommand(const CANDevices_Command_t *command);
static bool MotorChassis_PublishGimbalFeedback(const Gimbal_ChassisFeedback_t *feedback);

/**
 * @brief 初始化底盘模块并周期执行反馈读取、控制计算和电流发布
 *
 * @param[in] argument 任务参数，本任务不使用
 * @return 本任务不会返回
 */
void Task_motor_chassis(void *argument)
{
    (void)argument;

    // 节拍必须精确表达控制频率，非法时仍保留任务以持续发布零电流和诊断
    const uint32_t tick_frequency = osKernelGetTickFreq();
    const bool timing_valid = tick_frequency >= MOTOR_CHASSIS_FREQ && tick_frequency % MOTOR_CHASSIS_FREQ == 0U;
    const uint32_t delay_tick = timing_valid ? tick_frequency / MOTOR_CHASSIS_FREQ : 1U;
    osDelay(MOTOR_CHASSIS_INIT_DELAY);
    const int8_t init_status = Chassis_Init(&chassis, &chassis_config, timing_valid ? (float)MOTOR_CHASSIS_FREQ : 0.0F);
    if (init_status != 0)
    {
        FaultDetect_UpdateMotorChassis(NULL, &chassis, &fault_detect_snapshot);
    }

    // 首周期使用一个名义间隔，后续周期按实际节拍差计算，并支持无符号节拍回绕
    uint32_t tick = osKernelGetTickCount();
    uint32_t previous_tick = tick - delay_tick;
    while (true)
    {
        // 获取本周期节拍，计算实际控制间隔
        const uint32_t now_tick = osKernelGetTickCount();
        const float period_s = timing_valid ? (float)(now_tick - previous_tick) / (float)tick_frequency : 0.0F;
        previous_tick = now_tick;

        // 消费最新 CAN 快照，没有新快照时由模块将四轮反馈标记为无效
        const bool feedback_received = MotorChassis_ReadLatestFeedback();
        Chassis_UpdateFeedback(&chassis, feedback_received ? &can_feedback_cache : NULL);

        // DR16 独占运动命令来源，邮箱为空时保留缓存并持续检查 100 ms 时效
        MotorChassis_ReadLatestDR16();
        // 读取遥控快照后再采当前时间，避免接收任务抢占时将合法新帧误判为未来时间
        const uint64_t remote_time_us = BSP_TIME_Get();
        Cmd_ParseChassis(&dr16_state_cache, dr16_state_cache_valid, remote_time_us, &chassis_command);
        Gimbal_ChassisMotion_t motion =
            Cmd_ParseChassisMotion(&dr16_state_cache, dr16_state_cache_valid, remote_time_us);

        // 模块完成状态转换与速度闭环，任务只补充命令序号并发布最新电流
        CANDevices_Command_t output =
        {
            .sequence = feedback_received ? can_feedback_cache.sequence : 0U,
        };
        Chassis_Control(&chassis, &chassis_command, period_s);
        Chassis_DumpOutput(&chassis, &output);
        if (!MotorChassis_PublishCommand(&output))
        {
            // 发布失败时 CAN 任务空邮箱归零，本地同步复位，避免未执行电流期间积累控制历史
            Chassis_Control(&chassis, NULL, period_s);
            motion = GIMBAL_CHASSIS_MOTION_UNKNOWN;
        }

        // 发布独立轮速观测，失败时云台按源时间超时撤驱，不读取跨任务裸全局变量
        Gimbal_ChassisFeedback_t gimbal_feedback;
        Chassis_DumpGimbalFeedback(&chassis, feedback_received ? &can_feedback_cache : NULL, motion,
                                   chassis_yaw_rad_s_per_rpm, BSP_TIME_Get(), &gimbal_feedback);
        MotorChassis_PublishGimbalFeedback(&gimbal_feedback);

        // 汇总本周期设备与控制诊断，不复用缺失 CAN 快照的历史故障源
        FaultDetect_UpdateMotorChassis(feedback_received ? &can_feedback_cache : NULL, &chassis,
                                       &fault_detect_snapshot);

        // 错过节拍时跳过补算，防止追赶循环连续发布同一时刻的控制结果
        if ((uint32_t)(now_tick - tick) >= delay_tick)
        {
            tick = now_tick;
        }
        tick += delay_tick;
        osDelayUntil(tick);
    }
}

/**
 * @brief 从 DR16 最新状态邮箱取得一帧并更新底盘本地缓存
 *
 * @return 取得新状态时返回 true，否则返回 false
 */
static bool MotorChassis_ReadLatestDR16(void)
{
    if (task_runtime.msgq.dr16_state == NULL)
    {
        return false;
    }

    DR16_State_t received_state;
    if (osMessageQueueGet(task_runtime.msgq.dr16_state, &received_state, NULL, 0U) != osOK)
    {
        return false;
    }

    dr16_state_cache = received_state;
    dr16_state_cache_valid = true;
    return true;
}

/**
 * @brief 从容量为 1 的反馈邮箱读取最新 CAN 设备快照
 *
 * @return 成功取得新快照时返回 true，否则返回 false
 */
static bool MotorChassis_ReadLatestFeedback(void)
{
    return task_runtime.msgq.can_feedback != NULL &&
           osMessageQueueGet(task_runtime.msgq.can_feedback, &can_feedback_cache, NULL, 0U) == osOK;
}

/**
 * @brief 向容量为 1 的命令邮箱发布最新底盘电流
 *
 * @param[in] command 已由底盘模块填写的四轮电流和任务填写的序号
 * @return 邮箱重置并写入成功时返回 true，否则返回 false
 */
static bool MotorChassis_PublishCommand(const CANDevices_Command_t *command)
{
    if (task_runtime.msgq.can_command == NULL)
    {
        return false;
    }

    // 最新值邮箱不保存历史电流，重置或写入失败时由 CAN 任务默认零命令兜底
    return osMessageQueueReset(task_runtime.msgq.can_command) == osOK &&
           osMessageQueuePut(task_runtime.msgq.can_command, command, 0U, 0U) == osOK;
}

/**
 * @brief 发布仅供云台消费的最新底盘旋转观测
 *
 * @param[in] feedback 本周期完整观测，包含源时间与有效性
 * @return 邮箱存在且覆盖写入成功时为真，否则为假
 */
static bool MotorChassis_PublishGimbalFeedback(const Gimbal_ChassisFeedback_t *feedback)
{
    const osMessageQueueId_t queue = task_runtime.msgq.chassis_gimbal;
    return queue != NULL && osMessageQueueReset(queue) == osOK && osMessageQueuePut(queue, feedback, 0U, 0U) == osOK;
}
