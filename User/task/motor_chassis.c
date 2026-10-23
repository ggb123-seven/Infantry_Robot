#include "task/motor_chassis.h"

#include "bsp/time.h"
#include "device/can_devices.h"
#include "device/dr16.h"
#include "module/chassis.h"
#include "module/fault_detect.h"
#include "task/user_task.h"

#include <stddef.h>

/*
 * DR16 底盘输入参数：
 * - MOTOR_CHASSIS_DR16_CHANNEL_LIMIT：摇杆去中心后的有效计数上限
 * - MOTOR_CHASSIS_DR16_DEADZONE：摇杆中心死区比例
 * - MOTOR_CHASSIS_DR16_SPIN_WZ：左拨杆下位时的固定自旋角速度归一化值，正值为俯视逆时针
 */
#define MOTOR_CHASSIS_DR16_CHANNEL_LIMIT (660.0F)
#define MOTOR_CHASSIS_DR16_DEADZONE (0.05F)
#define MOTOR_CHASSIS_DR16_SPIN_WZ (0.30F)

_Static_assert(CHASSIS_MOTOR_COUNT == CAN_DEVICES_CHASSIS_MOTOR_COUNT, "底盘控制与 CAN 设备数量必须一致");

/*
 * 底盘在线调参初值：轮速尺度 100 rpm，ID1~3 使用固化公共 PID 参数
 */
volatile MotorChassis_Tune_t motor_chassis_tune =
{
    .scale_rpm = 100.0F,
    .pid =
    {
        .kp = CHASSIS_PID_KP,
        .ki = CHASSIS_PID_KI,
        .kd = CHASSIS_PID_KD,
    },
};

Chassis_Snapshot_t chassis_snapshot;
FaultDetect_Snapshot_t fault_detect_snapshot;

/*
 * 底盘任务私有状态：
 * - can_feedback_cache：从反馈邮箱取得的最新 CAN 快照
 * - chassis_feedback：从 CAN 快照映射出的本周期底盘控制反馈
 * - chassis_output：底盘模块本周期计算得到的四路电流命令
 * - dr16_state_cache：最近一次从 DR16 邮箱取得的状态快照
 * - dr16_state_cache_valid：至少取得过一帧 DR16 状态时为 true
 */
static CANDevices_Snapshot_t can_feedback_cache;
static Chassis_Feedback_t chassis_feedback;
static Chassis_Output_t chassis_output;
static DR16_State_t dr16_state_cache;
static bool dr16_state_cache_valid;

static bool MotorChassis_ReadLatestFeedback(void);
static bool MotorChassis_ReadLatestDR16(void);
static void MotorChassis_ApplyDR16Input(Chassis_Input_t *input, uint64_t now_us);
static float MotorChassis_NormalizeDR16Channel(int16_t channel);
static float MotorChassis_ApplyDeadzone(float value);
static bool MotorChassis_PublishCommand(void);

/**
 * @brief 初始化并周期运行 X 型全向轮底盘控制链
 *
 * DR16 未使能、设备离线或任一控制步骤失败时，对应电机电流指令保持为零
 *
 * @param[in] argument 任务参数，本任务不使用
 * @return 本任务不会返回
 */
void Task_motor_chassis(void *argument)
{
    (void)argument;

    const uint32_t delay_tick = osKernelGetTickFreq() / MOTOR_CHASSIS_FREQ;

    // 等待系统外设与业务模块完成统一初始化后再进入周期控制
    osDelay(MOTOR_CHASSIS_INIT_DELAY);

    // 建立绝对周期基准，避免控制周期累计漂移
    uint32_t tick = osKernelGetTickCount();
    while (1)
    {
        // 建立默认禁用的控制输入，并读取在线调参
        Chassis_Input_t chassis_input =
        {
            .enabled = false,
            .scale_rpm = motor_chassis_tune.scale_rpm,
            .pid_tune =
            {
                .kp = motor_chassis_tune.pid.kp,
                .ki = motor_chassis_tune.pid.ki,
                .kd = motor_chassis_tune.pid.kd,
            },
            .control_period_s = 1.0F / (float)MOTOR_CHASSIS_FREQ,
        };

        // 取得时间基准，用于拒绝过期遥控状态
        const uint64_t now_us = BSP_TIME_Get();

        // 消费并映射最新 CAN 反馈，邮箱没有新快照时四路反馈保持无效
        const bool feedback_received = MotorChassis_ReadLatestFeedback();

        // 消费最新 DR16 状态并保留本地缓存，邮箱没有新状态时继续使用最近快照
        MotorChassis_ReadLatestDR16();

        // 由 DR16 唯一生成运动使能和运动向量，失联时保持禁用
        MotorChassis_ApplyDR16Input(&chassis_input, now_us);

        // 完成运动学和速度控制，再发布最新电流命令
        Chassis_Run(&chassis_input, &chassis_feedback, &chassis_output, &chassis_snapshot);
        MotorChassis_PublishCommand();

        // 汇总本周期设备和控制故障
        FaultDetect_UpdateMotorChassis(feedback_received ? &can_feedback_cache : NULL, &chassis_snapshot,
                                       &fault_detect_snapshot);

        tick += delay_tick;
        // 等待至下一个绝对任务周期，保持稳定控制频率
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
 * @param[in,out] input 待覆盖的底盘控制输入
 * @param[in] now_us 当前时间，单位微秒
 * @return 无返回值
 */
static void MotorChassis_ApplyDR16Input(Chassis_Input_t *input, uint64_t now_us)
{
    if (input == NULL)
    {
        return;
    }

    // 先复位运动请求，确保遥控无效或超时不会沿用上一周期输出
    input->enabled = false;
    input->move_vector = (MoveVector_t)
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
            input->enabled = true;
            input->move_vector.vx = -forward_channel;
            input->move_vector.vy = lateral_channel;
            break;

        case DR16_SWITCH_DOWN:
            input->enabled = true;
            input->move_vector.vx = -forward_channel;
            input->move_vector.vy = lateral_channel;
            input->move_vector.wz = MOTOR_CHASSIS_DR16_SPIN_WZ;
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
 * @brief 初始化底盘任务私有运动学与速度控制器
 *
 * @return X 型全向轮混合器和四路速度控制器可运行时返回 true，否则返回 false
 */
bool Task_motor_chassis_Init(void)
{
    // 选择四角 X 型全向轮布局并初始化底盘控制链，CAN 设备集合由独立通信任务负责初始化
    Chassis_Init((float)MOTOR_CHASSIS_FREQ, MIXER_OMNICROSS, &chassis_snapshot);

    // 生成初始化阶段诊断快照，区分未诊断和启动故障
    FaultDetect_UpdateMotorChassis(NULL, &chassis_snapshot, &fault_detect_snapshot);
    return chassis_snapshot.initialized;
}

/**
 * @brief 从容量为 1 的反馈邮箱读取并映射最新 CAN 设备快照
 *
 * @return 成功取得新快照时返回 true，否则返回 false 且四路反馈无效
 */
static bool MotorChassis_ReadLatestFeedback(void)
{
    chassis_feedback = (Chassis_Feedback_t)
    {
        0,
    };
    if (task_runtime.msgq.can_feedback == NULL ||
        osMessageQueueGet(task_runtime.msgq.can_feedback, &can_feedback_cache, NULL, 0U) != osOK)
    {
        return false;
    }

    // 只把本周期成功更新且在线的设备反馈标记为可用于速度控制
    for (uint32_t motor_index = 0U; motor_index < CHASSIS_MOTOR_COUNT; motor_index++)
    {
        chassis_feedback.valid[motor_index] =
            can_feedback_cache.feedback_update_status[motor_index] == CAN_DEVICES_OK;
        chassis_feedback.motor_online[motor_index] = can_feedback_cache.motor_online[motor_index];
        chassis_feedback.actual_speed_rpm[motor_index] = can_feedback_cache.actual_speed_rpm[motor_index];
    }
    return true;
}

/**
 * @brief 向容量为 1 的命令邮箱发布最新底盘电流命令
 *
 * @return 邮箱重置并写入成功时返回 true，否则返回 false
 */
static bool MotorChassis_PublishCommand(void)
{
    CANDevices_Command_t command =
    {
        .sequence = can_feedback_cache.sequence,
    };
    for (uint32_t motor_index = 0U; motor_index < CHASSIS_MOTOR_COUNT; motor_index++)
    {
        command.current_a[motor_index] = chassis_output.current_command_a[motor_index];
    }
    if (task_runtime.msgq.can_command == NULL)
    {
        return false;
    }

    // 邮箱只保留最新电流命令，重置和写入任一失败都由 CAN task 的零命令默认值兜底
    return osMessageQueueReset(task_runtime.msgq.can_command) == osOK &&
           osMessageQueuePut(task_runtime.msgq.can_command, &command, 0U, 0U) == osOK;
}
