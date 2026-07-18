#pragma once

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdint.h>

/*
 * Ozone 速度调参与波形字段：
 * - requested_speed_rpm：可修改的输出轴目标转速，单位 rpm。
 * - pid_kp、pid_ki、pid_kd：可修改的速度 PID 参数。
 * - actual_speed_rpm：只读的输出轴真实转速，单位 rpm。
 */
typedef struct
{
  float requested_speed_rpm;
  float pid_kp;
  float pid_ki;
  float pid_kd;
  float actual_speed_rpm;
} MotorChassisTune_t;

/*
 * Ozone 电机运行监视字段：
 * - motor_online：最近 100 ms 内收到电机反馈时为 true。
 * - current_saturated：电流指令达到正负限幅时为 true。
 * - register_status：电机注册结果，0 表示成功。
 * - control_init_status：速度环初始化结果，0 表示成功。
 * - control_status：本周期速度环状态，0 表示正常，-2 表示反馈离线导致禁用。
 * - can_tx_status：本周期 CAN 控制帧发送结果，0 表示成功。
 * - limited_target_speed_rpm：限幅后的输出轴目标转速，单位 rpm。
 * - ramped_target_speed_rpm：缓启动后的输出轴目标转速，单位 rpm。
 * - current_command_a：下发给 C620 的转子侧电流指令，单位 A。
 * - temperature_c：C620 反馈的电机温度，单位摄氏度。
 */
typedef struct
{
  bool motor_online;
  bool current_saturated;
  int8_t register_status;
  int8_t control_init_status;
  int8_t control_status;
  int8_t can_tx_status;
  float limited_target_speed_rpm;
  float ramped_target_speed_rpm;
  float current_command_a;
  float temperature_c;
} MotorChassisMonitor_t;

extern volatile MotorChassisTune_t g_motor_chassis_tune;
extern volatile MotorChassisMonitor_t g_motor_chassis_monitor;

/**
 * @brief 运行单个 M3508 电机速度控制任务
 *
 * @param[in] argument 任务参数，本任务不使用
 * @return 本任务不会返回
 */
void Task_motor_chassis(void *argument);

#ifdef __cplusplus
}
#endif
