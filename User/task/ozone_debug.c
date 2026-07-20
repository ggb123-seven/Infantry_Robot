#include "task/ozone_debug.h"

#include "module/chassis.h"
#include "module/chassis_can.h"
#include "module/motor_speed_control.h"

volatile DR16_Monitor_t g_dr16_monitor;

/*
 * 四个 M3508 的 Ozone 在线调试参数初值：
 * - 上电保持调试使能关闭，全部目标转速和反馈清零。
 * - PID 初值使用速度控制模块的当前默认参数。
 */
volatile MotorChassisTune_t g_motor_chassis_tune =
{
    .motor_debug_enable = false,
    .requested_speed_rpm =
    {
        0.0F,
        0.0F,
        0.0F,
        0.0F,
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
        CHASSIS_CAN_DEVICE_UNAVAILABLE,
        CHASSIS_CAN_DEVICE_UNAVAILABLE,
        CHASSIS_CAN_DEVICE_UNAVAILABLE,
        CHASSIS_CAN_DEVICE_UNAVAILABLE,
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
        CHASSIS_CAN_DEVICE_UNAVAILABLE,
        CHASSIS_CAN_DEVICE_UNAVAILABLE,
        CHASSIS_CAN_DEVICE_UNAVAILABLE,
        CHASSIS_CAN_DEVICE_UNAVAILABLE,
    },
    .current_set_status =
    {
        CHASSIS_CAN_DEVICE_UNAVAILABLE,
        CHASSIS_CAN_DEVICE_UNAVAILABLE,
        CHASSIS_CAN_DEVICE_UNAVAILABLE,
        CHASSIS_CAN_DEVICE_UNAVAILABLE,
    },
    .chassis_status = CHASSIS_INIT_ERROR,
    .control_status =
    {
        MOTOR_SPEED_CONTROL_INIT_ERROR,
        MOTOR_SPEED_CONTROL_INIT_ERROR,
        MOTOR_SPEED_CONTROL_INIT_ERROR,
        MOTOR_SPEED_CONTROL_INIT_ERROR,
    },
    .can_tx_status = CHASSIS_CAN_DEVICE_UNAVAILABLE,
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
