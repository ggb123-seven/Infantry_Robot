#include "task/motor_chassis.h"

#include "bsp/time.h"
#include "device/dr16.h"
#include "module/config.h"
#include "task/user_task.h"

#include <stddef.h>

/*
 * DR16 底盘输入参数，沿用底盘重构方案：
 * - CHANNEL_LIMIT：摇杆去中心后的有效计数上限 660
 * - DEADZONE：摇杆中心死区比例 5%
 * - SPIN_WZ：左拨杆下位固定自旋归一化值 +0.30，正值为俯视逆时针
 * - SCALE_RPM：归一化轮速尺度，单位输出轴 rpm
 */
#define MOTOR_CHASSIS_DR16_CHANNEL_LIMIT (660.0F)
#define MOTOR_CHASSIS_DR16_DEADZONE (0.05F)
#define MOTOR_CHASSIS_DR16_SPIN_WZ (0.30F)
#define MOTOR_CHASSIS_SCALE_RPM (100.0F)

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
static void MotorChassis_ApplyDR16Input(Chassis_Command_t *command, uint64_t now_us);
static float MotorChassis_NormalizeDR16Channel(int16_t channel);
static float MotorChassis_ApplyDeadzone(float value);
static bool MotorChassis_PublishCommand(const CANDevices_Command_t *command);

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
        MotorChassis_ApplyDR16Input(&chassis_command, BSP_TIME_Get());

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
 * @brief 将 DR16 状态转换为当前底盘控制模式和运动向量
 *
 * @param[out] command 待覆盖的底盘控制命令
 * @param[in] now_us 当前时间，单位微秒
 * @return 无返回值
 */
static void MotorChassis_ApplyDR16Input(Chassis_Command_t *command, uint64_t now_us)
{
    if (command == NULL)
    {
        return;
    }

    // 先复位运动请求，确保遥控无效或超时不会沿用上一周期输出
    command->mode = CHASSIS_MODE_RELAX;
    command->wheel_speed_scale_rpm = MOTOR_CHASSIS_SCALE_RPM;
    command->move = (MoveVector_t)
    {
        0,
    };
    if (!dr16_state_cache_valid || !dr16_state_cache.header.online ||
        now_us < dr16_state_cache.header.last_online_time ||
        now_us - dr16_state_cache.header.last_online_time > DR16_RECEIVER_OFFLINE_TIMEOUT_US)
    {
        return;
    }

    // 对遥控通道限幅和去死区，再按拨杆枚举集中选择运动模式
    // 上位和下位为运动档，摇杆回中时保持速度环并以零目标主动制动
    const float lateral_channel = MotorChassis_NormalizeDR16Channel(dr16_state_cache.data.ch_l_x);
    const float forward_channel = MotorChassis_NormalizeDR16Channel(dr16_state_cache.data.ch_l_y);
    switch (dr16_state_cache.data.sw_l)
    {
        case DR16_SWITCH_UP:
            command->mode = CHASSIS_MODE_INDEPENDENT;
            command->move.vx = -forward_channel;
            command->move.vy = lateral_channel;
            break;

        case DR16_SWITCH_DOWN:
            command->mode = CHASSIS_MODE_INDEPENDENT;
            command->move.vx = -forward_channel;
            command->move.vy = lateral_channel;
            command->move.wz = MOTOR_CHASSIS_DR16_SPIN_WZ;
            break;

        // 中位和异常拨杆作为停机档，关闭速度环并输出零电流
        case DR16_SWITCH_MIDDLE:
        case DR16_SWITCH_ERROR:
        default:
            break;
    }
}

/**
 * @brief 将 DR16 通道原始值归一化并执行中心死区处理
 *
 * @param[in] channel DR16 已减去中心值的通道原始值，典型范围 -660~660
 * @return 归一化后的通道值，范围 [-1, 1]
 */
static float MotorChassis_NormalizeDR16Channel(int16_t channel)
{
    float normalized = (float)channel / MOTOR_CHASSIS_DR16_CHANNEL_LIMIT;
    if (normalized > 1.0F)
    {
        normalized = 1.0F;
    }
    else if (normalized < -1.0F)
    {
        normalized = -1.0F;
    }
    return MotorChassis_ApplyDeadzone(normalized);
}

/**
 * @brief 对归一化摇杆值执行中心死区处理
 *
 * @param[in] value 归一化摇杆值
 * @return 处理后的归一化摇杆值
 */
static float MotorChassis_ApplyDeadzone(float value)
{
    if (value > -MOTOR_CHASSIS_DR16_DEADZONE && value < MOTOR_CHASSIS_DR16_DEADZONE)
    {
        return 0.0F;
    }

    if (value > 0.0F)
    {
        return (value - MOTOR_CHASSIS_DR16_DEADZONE) / (1.0F - MOTOR_CHASSIS_DR16_DEADZONE);
    }
    return (value + MOTOR_CHASSIS_DR16_DEADZONE) / (1.0F - MOTOR_CHASSIS_DR16_DEADZONE);
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
