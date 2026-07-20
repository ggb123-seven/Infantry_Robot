#include "task/ozone_debug.h"

#include "module/chassis.h"
#include "module/motor_speed_control.h"

#include <stddef.h>

/*
 * Ozone 底盘停机确认参数：
 * - MOTOR_DEBUG_STOP_CONFIRM_CYCLES：关闭调试使能后，连续成功提交四电机零电流帧的控制周期数。
 */
#define MOTOR_DEBUG_STOP_CONFIRM_CYCLES (5U)

_Static_assert(OZONE_MOTOR_CHASSIS_COUNT == CHASSIS_MOTOR_COUNT,
               "Ozone 底盘调试电机数量必须与底盘控制链一致");

volatile DR16_Monitor_t g_dr16_monitor;

/*
 * 四个 M3508 的 Ozone 在线调试参数初值：
 * - 上电默认开启调试使能，四路目标转速均为 100 rpm，反馈保持清零等待 CAN 更新。
 * - PID 初值使用速度控制模块的当前默认参数。
 */
volatile MotorChassisTune_t g_motor_chassis_tune =
{
    .motor_debug_enable = true,
    .requested_speed_rpm =
    {
        100.0F,
        100.0F,
        100.0F,
        100.0F,
    },
    .pid_kp = MOTOR_SPEED_PID_KP,
    .pid_ki = MOTOR_SPEED_PID_KI,
    .pid_kd = MOTOR_SPEED_PID_KD,
    .actual_speed_rpm =
    {
        0.0F,
        0.0F,
        0.0F,
        0.0F,
    },
};

/*
 * 四个 M3508 的 Ozone 运行监视数据初值：
 * - 任务启动前所有设备和控制器均标记为不可用。
 * - 数值反馈保持为零，避免尚未运行的状态被误认为有效数据。
 */
volatile MotorChassisMonitor_t g_motor_chassis_monitor =
{
    .motor_online =
    {
        false,
        false,
        false,
        false,
    },
    .current_saturated =
    {
        false,
        false,
        false,
        false,
    },
    .debug_stop_ready = false,
    .register_status =
    {
        CHASSIS_DEVICE_UNAVAILABLE,
        CHASSIS_DEVICE_UNAVAILABLE,
        CHASSIS_DEVICE_UNAVAILABLE,
        CHASSIS_DEVICE_UNAVAILABLE,
    },
    .control_init_status =
    {
        MOTOR_SPEED_CONTROL_INIT_ERROR,
        MOTOR_SPEED_CONTROL_INIT_ERROR,
        MOTOR_SPEED_CONTROL_INIT_ERROR,
        MOTOR_SPEED_CONTROL_INIT_ERROR,
    },
    .feedback_update_status =
    {
        CHASSIS_DEVICE_UNAVAILABLE,
        CHASSIS_DEVICE_UNAVAILABLE,
        CHASSIS_DEVICE_UNAVAILABLE,
        CHASSIS_DEVICE_UNAVAILABLE,
    },
    .current_set_status =
    {
        CHASSIS_DEVICE_UNAVAILABLE,
        CHASSIS_DEVICE_UNAVAILABLE,
        CHASSIS_DEVICE_UNAVAILABLE,
        CHASSIS_DEVICE_UNAVAILABLE,
    },
    .chassis_status = CHASSIS_NOT_INITIALIZED,
    .control_status =
    {
        MOTOR_SPEED_CONTROL_INIT_ERROR,
        MOTOR_SPEED_CONTROL_INIT_ERROR,
        MOTOR_SPEED_CONTROL_INIT_ERROR,
        MOTOR_SPEED_CONTROL_INIT_ERROR,
    },
    .can_tx_status = CHASSIS_DEVICE_UNAVAILABLE,
    .debug_stop_zero_tx_count = 0U,
    .limited_target_speed_rpm =
    {
        0.0F,
        0.0F,
        0.0F,
        0.0F,
    },
    .ramped_target_speed_rpm =
    {
        0.0F,
        0.0F,
        0.0F,
        0.0F,
    },
    .current_command_a =
    {
        0.0F,
        0.0F,
        0.0F,
        0.0F,
    },
    .temperature_c =
    {
        0.0F,
        0.0F,
        0.0F,
        0.0F,
    },
};

static uint32_t debug_stop_zero_tx_count;

/**
 * @brief 将底盘模块初始化结果发布到 Ozone 监控区
 *
 * @param[in] snapshot 底盘初始化结果快照
 * @return 无返回值
 */
void OzoneDebug_UpdateMotorChassisInit(const Chassis_Snapshot_t *snapshot)
{
    if (snapshot == NULL)
    {
        return;
    }

    // 发布设备注册和速度控制器初始化状态，失败启动不得显示为可用
    for (uint32_t motor_index = 0U; motor_index < CHASSIS_MOTOR_COUNT; motor_index++)
    {
        g_motor_chassis_monitor.register_status[motor_index] = snapshot->register_status[motor_index];
        g_motor_chassis_monitor.control_init_status[motor_index] = snapshot->motor_init_status[motor_index];
    }
    g_motor_chassis_monitor.chassis_status = snapshot->control_init_status;
    g_motor_chassis_monitor.can_tx_status = snapshot->can_init_status;
    debug_stop_zero_tx_count = 0U;
}

/**
 * @brief 从 Ozone 在线参数生成本周期底盘控制输入快照
 *
 * @param[out] input 待写入的底盘控制输入快照
 * @param[in] control_period_s 控制周期，单位 s，必须大于 0
 * @return 无返回值
 */
void OzoneDebug_GetMotorChassisInput(Chassis_Input_t *input, float control_period_s)
{
    if (input == NULL)
    {
        return;
    }

    // 先建立全零快照，再集中读取本周期允许由 Ozone 修改的控制参数
    *input = (Chassis_Input_t)
    {
        .enabled = g_motor_chassis_tune.motor_debug_enable,
        .pid_tune =
        {
            .kp = g_motor_chassis_tune.pid_kp,
            .ki = g_motor_chassis_tune.pid_ki,
            .kd = g_motor_chassis_tune.pid_kd,
        },
        .control_period_s = control_period_s,
    };
    for (uint32_t motor_index = 0U; motor_index < CHASSIS_MOTOR_COUNT; motor_index++)
    {
        input->requested_speed_rpm[motor_index] = g_motor_chassis_tune.requested_speed_rpm[motor_index];
    }
}

/**
 * @brief 将底盘模块单周期结果发布到 Ozone 监控区
 *
 * @param[in] input 本周期实际采用的底盘控制输入快照
 * @param[in] snapshot 本周期底盘模块结果快照
 * @return 无返回值
 */
void OzoneDebug_UpdateMotorChassis(const Chassis_Input_t *input, const Chassis_Snapshot_t *snapshot)
{
    if (input == NULL || snapshot == NULL)
    {
        return;
    }

    bool all_zero_current_set = !input->enabled && snapshot->can_tx_status == CHASSIS_OK;

    // 汇总四路反馈、控制和电流提交结果，确保调试数据与本周期实际输出一致
    for (uint32_t motor_index = 0U; motor_index < CHASSIS_MOTOR_COUNT; motor_index++)
    {
        if (snapshot->current_command_a[motor_index] != 0.0F ||
            snapshot->current_set_status[motor_index] != CHASSIS_OK)
        {
            all_zero_current_set = false;
        }

        g_motor_chassis_monitor.motor_online[motor_index] = snapshot->motor_online[motor_index];
        g_motor_chassis_monitor.current_saturated[motor_index] =
            snapshot->current_command_a[motor_index] >= MOTOR_SPEED_CURRENT_LIMIT_A ||
            snapshot->current_command_a[motor_index] <= -MOTOR_SPEED_CURRENT_LIMIT_A;
        g_motor_chassis_monitor.feedback_update_status[motor_index] = snapshot->feedback_update_status[motor_index];
        g_motor_chassis_monitor.current_set_status[motor_index] = snapshot->current_set_status[motor_index];
        g_motor_chassis_monitor.control_status[motor_index] = snapshot->control_status[motor_index];
        g_motor_chassis_monitor.limited_target_speed_rpm[motor_index] =
            snapshot->limited_target_speed_rpm[motor_index];
        g_motor_chassis_monitor.ramped_target_speed_rpm[motor_index] =
            snapshot->ramped_target_speed_rpm[motor_index];
        g_motor_chassis_tune.actual_speed_rpm[motor_index] = snapshot->actual_speed_rpm[motor_index];
        g_motor_chassis_monitor.current_command_a[motor_index] = snapshot->current_command_a[motor_index];
        g_motor_chassis_monitor.temperature_c[motor_index] = snapshot->temperature_c[motor_index];
    }

    // 连续确认零电流帧发送成功，供调试器判断关闭使能后的安全停机状态
    if (all_zero_current_set)
    {
        if (debug_stop_zero_tx_count < MOTOR_DEBUG_STOP_CONFIRM_CYCLES)
        {
            debug_stop_zero_tx_count++;
        }
    }
    else
    {
        debug_stop_zero_tx_count = 0U;
    }

    g_motor_chassis_monitor.debug_stop_ready =
        debug_stop_zero_tx_count >= MOTOR_DEBUG_STOP_CONFIRM_CYCLES;
    g_motor_chassis_monitor.chassis_status = snapshot->chassis_status;
    g_motor_chassis_monitor.can_tx_status = snapshot->can_tx_status;
    g_motor_chassis_monitor.debug_stop_zero_tx_count = debug_stop_zero_tx_count;
}
