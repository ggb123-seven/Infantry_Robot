#pragma once

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdint.h>

/*
 * Ozone 电机监视与在线调参字段：
 * - requested_speed_rpm：可修改的输出轴目标转速，单位 rpm。
 * - pid_kp、pid_ki、pid_kd：可修改的速度 PID 参数。
 * - motor_found：电机注册后能取得实例时为 true。
 * - motor_online：最近 100 ms 内收到电机反馈时为 true。
 * - current_saturated：电流指令达到正负限幅时为 true。
 * - register_status：电机注册结果，0 表示成功。
 * - control_init_status：速度环初始化结果，0 表示成功。
 * - feedback_update_status：本周期电机反馈更新结果，0 表示取得新反馈。
 * - control_status：本周期速度环状态，0 表示正常，-2 表示反馈离线导致禁用。
 * - current_set_status：本周期电流指令写入结果，0 表示成功。
 * - can_tx_status：本周期 CAN 控制帧发送结果，0 表示成功。
 * - loop_count：任务控制周期累计次数。
 * - online_drop_count：电机在线状态从在线变为离线的累计次数。
 * - feedback_age_us：距最近一次有效反馈的时间，单位 us。
 * - limited_target_speed_rpm：限幅后的输出轴目标转速，单位 rpm。
 * - ramped_target_speed_rpm：缓启动后的输出轴目标转速，单位 rpm。
 * - actual_speed_rpm：反馈的输出轴真实转速，单位 rpm。
 * - speed_error_rpm：目标转速与真实转速之差，单位 rpm。
 * - current_command_a：下发给 C620 的转子侧电流指令，单位 A。
 * - pid_integral_error_rpm_s：PID 积分状态，单位 rpm*s。
 * - applied_pid_kp、applied_pid_ki、applied_pid_kd：速度环本周期实际采用的 PID 参数。
 * - raw_rotor_speed_rpm：C620 反馈的原始转子转速，单位 rpm。
 * - raw_current：C620 反馈的原始电流编码值。
 * - temperature_c：C620 反馈的电机温度，单位摄氏度。
 */
typedef struct {
  float requested_speed_rpm;
  float pid_kp;
  float pid_ki;
  float pid_kd;
  bool motor_found;
  bool motor_online;
  bool current_saturated;
  int8_t register_status;
  int8_t control_init_status;
  int8_t feedback_update_status;
  int8_t control_status;
  int8_t current_set_status;
  int8_t can_tx_status;
  uint32_t loop_count;
  uint32_t online_drop_count;
  uint32_t feedback_age_us;
  float limited_target_speed_rpm;
  float ramped_target_speed_rpm;
  float actual_speed_rpm;
  float speed_error_rpm;
  float current_command_a;
  float pid_integral_error_rpm_s;
  float applied_pid_kp;
  float applied_pid_ki;
  float applied_pid_kd;
  int16_t raw_rotor_speed_rpm;
  int16_t raw_current;
  float temperature_c;
} MotorChassisMonitor_t;

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
