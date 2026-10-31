#include "task/motor_chassis.h"

#include "bsp/time.h"
#include "module/cmd.h"
#include "device/dr16.h"
#include "module/config.h"
#include "module/trigger.h"
#include "task/user_task.h"

#include <stddef.h>

_Static_assert(CHASSIS_WHEEL_COUNT == CAN_DEVICES_CHASSIS_MOTOR_COUNT, "底盘与 CAN 电机数量必须一致");

/*
 * 拨弹遥控映射参数：
 * - 右拨杆下位持续请求输出轴 -120 rpm，其余档位或遥控无效时失能
 * - 速度和方向沿用原拨弹每圈 10 齿、每秒 20 齿、方向 -1 的设定
 */
#define MOTOR_TRIGGER_REMOTE_SPEED_RPM (-120.0f)

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
static bool MotorChassis_PublishState(void);
static bool MotorChassis_PublishTriggerCommand(bool remote_valid);

/**
 * @brief 初始化底盘模块并周期执行反馈读取、控制计算和电流发布
 *
 * @param[in] argument 任务参数，本任务不使用
 * @return 本任务不会返回
 */
void Task_motor_chassis(void *argument)
{
    (void)argument;

    // 固定节拍约束在编译期检查，此处直接按配置建立控制周期
    const uint32_t tick_frequency = osKernelGetTickFreq();
    const uint32_t delay_tick = tick_frequency / MOTOR_CHASSIS_FREQ;
    osDelay(MOTOR_CHASSIS_INIT_DELAY);
    const int8_t init_status = Chassis_Init(&chassis, &chassis_config, (float)MOTOR_CHASSIS_FREQ);
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
        const float period_s = (float)(now_tick - previous_tick) / (float)tick_frequency;
        previous_tick = now_tick;

        // 消费最新 CAN 快照，没有新快照时由模块将四轮反馈标记为无效
        const bool feedback_received = MotorChassis_ReadLatestFeedback();
        Chassis_UpdateFeedback(&chassis, feedback_received ? &can_feedback_cache : NULL);

        // DR16 独占运动命令来源，邮箱为空时保留缓存并持续检查 100 ms 时效
        MotorChassis_ReadLatestDR16();
        // 读取遥控快照后再采当前时间，避免接收任务抢占时将合法新帧误判为未来时间
        const uint64_t remote_time_us = BSP_TIME_Get();
        const bool remote_valid =
            Cmd_ParseChassis(&dr16_state_cache, dr16_state_cache_valid, remote_time_us, &chassis_command);

        // 复用本周期已校验的遥控快照持续发布拨弹命令，避免遥控帧间隙导致失能
        MotorChassis_PublishTriggerCommand(remote_valid);

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
        }

        // 将底盘在线与已采纳的自旋标志合并发布，云台不读取跨任务裸全局变量
        MotorChassis_PublishState();

        // 汇总本周期设备与控制诊断，不复用缺失 CAN 快照的历史故障源
        FaultDetect_UpdateMotorChassis(feedback_received ? &can_feedback_cache : NULL, &chassis,
                                       &fault_detect_snapshot);

        // 错过节拍时跳过补算，防止追赶循环连续发布同一时刻的控制结果
        if ((uint32_t)(now_tick - tick) >= delay_tick)
        {
            tick = now_tick;
        }
        tick += delay_tick;
        // 等待绝对周期边界，保持底盘与拨弹命令的发布节拍
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
    return osMessageQueueGet(task_runtime.msgq.can_feedback, &can_feedback_cache, NULL, 0U) == osOK;
}

/**
 * @brief 向容量为 1 的命令邮箱发布最新底盘电流
 *
 * @param[in] command 已由底盘模块填写的四轮电流和任务填写的序号
 * @return 邮箱重置并写入成功时返回 true，否则返回 false
 */
static bool MotorChassis_PublishCommand(const CANDevices_Command_t *command)
{
    // 最新值邮箱不保存历史电流，重置或写入失败时由 CAN 任务默认零命令兜底
    return osMessageQueueReset(task_runtime.msgq.can_command) == osOK &&
           osMessageQueuePut(task_runtime.msgq.can_command, command, 0U, 0U) == osOK;
}

/**
 * @brief 发布仅供云台消费的底盘在线及小陀螺状态
 *
 * @return 邮箱覆盖写入成功时为真，失败时云台按邮箱为空保留上次状态
 */
static bool MotorChassis_PublishState(void)
{
    // 命令已由命令层校验，当前非零旋转分量只来自左拨杆下位的小陀螺映射
    const bool spinning = chassis.mode == CHASSIS_MODE_INDEPENDENT && chassis_command.move.wz != 0.0f;
    const Gimbal_ChassisState_t state = !Chassis_HasOnlineWheel(&chassis) ? GIMBAL_CHASSIS_OFFLINE :
        (spinning ? GIMBAL_CHASSIS_SPINNING : GIMBAL_CHASSIS_ONLINE);
    const osMessageQueueId_t queue = task_runtime.msgq.chassis_gimbal;
    return osMessageQueueReset(queue) == osOK && osMessageQueuePut(queue, &state, 0U, 0U) == osOK;
}

/**
 * @brief 将已校验遥控快照的右拨杆请求路由到拨弹命令邮箱
 *
 * @param[in] remote_valid 本周期命令解析确认遥控在线且未超时，调用方保证先更新遥控缓存
 * @return 邮箱重置并写入成功时为真，失败时拨弹任务按缺命令失能，底盘控制继续运行
 */
static bool MotorChassis_PublishTriggerCommand(bool remote_valid)
{
    // 命令层已完成在线与时间边界校验，此处只映射开关，不跨任务写拨弹实例
    const bool enabled = remote_valid && dr16_state_cache.data.sw_r == DR16_SWITCH_DOWN;
    const Trigger_Command_t command =
    {
        .enabled = enabled,
        .target_speed_rpm = enabled ? MOTOR_TRIGGER_REMOTE_SPEED_RPM : 0.0f,
    };

    // 单槽邮箱只保存最新请求，发布失败的撤驱沿用拨弹任务的空邮箱处理
    const osMessageQueueId_t queue = task_runtime.msgq.trigger_command;
    return osMessageQueueReset(queue) == osOK && osMessageQueuePut(queue, &command, 0U, 0U) == osOK;
}
