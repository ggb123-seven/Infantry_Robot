#pragma once

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdint.h>

#define MOTOR_CHASSIS_MOTOR_COUNT (4U)

/**
 * @brief 四个 M3508 速度环的在线调试参数
 *
 * Ozone 速度调参与波形字段：
 * - motor_debug_enable：可修改的四电机全局调试使能；设为 false 后所有速度环清零并持续发送零电流。
 * - requested_speed_rpm[0~3]：可修改的输出轴目标转速，依次对应 C620 电调 ID 1~4，单位 rpm。
 * - pid_kp、pid_ki、pid_kd：可修改的速度 PID 参数。
 * - actual_speed_rpm[0~3]：只读的输出轴真实转速，依次对应 C620 电调 ID 1~4，单位 rpm。
 */
typedef struct
{
  bool motor_debug_enable;
  float requested_speed_rpm[MOTOR_CHASSIS_MOTOR_COUNT];
  float pid_kp;
  float pid_ki;
  float pid_kd;
  float actual_speed_rpm[MOTOR_CHASSIS_MOTOR_COUNT];
} MotorChassisTune_t;

/**
 * @brief 四个 M3508 任务的运行状态与诊断数据
 *
 * Ozone 四电机运行监视字段：
 * - motor_online[0~3]：最近 100 ms 内收到对应电机反馈时为 true。
 * - current_saturated[0~3]：对应电机电流指令达到正负限幅时为 true。
 * - debug_stop_ready：关闭调试使能后，连续成功提交 5 个周期的四电机零电流帧时为 true。
 * - register_status[0~3]：对应电机注册结果，0 表示成功。
 * - control_init_status[0~3]：对应速度环初始化结果，0 表示成功。
 * - feedback_update_status[0~3]：本周期对应电机反馈更新结果，0 表示成功。
 * - current_set_status[0~3]：本周期对应电流指令写入结果，0 表示成功。
 * - control_status[0~3]：本周期对应速度环状态，0 表示正常，-2 表示使能关闭或反馈离线。
 * - can_tx_status：本周期 CAN 控制帧发送结果，0 表示成功。
 * - debug_stop_zero_tx_count：关闭调试使能后连续成功提交的零电流帧周期数。
 * - limited_target_speed_rpm[0~3]：限幅后的对应输出轴目标转速，单位 rpm。
 * - ramped_target_speed_rpm[0~3]：缓启动后的对应输出轴目标转速，单位 rpm。
 * - current_command_a[0~3]：下发给对应 C620 的转子侧电流指令，单位 A。
 * - temperature_c[0~3]：对应 C620 反馈的电机温度，单位摄氏度。
 */
typedef struct
{
  bool motor_online[MOTOR_CHASSIS_MOTOR_COUNT];
  bool current_saturated[MOTOR_CHASSIS_MOTOR_COUNT];
  bool debug_stop_ready;
  int8_t register_status[MOTOR_CHASSIS_MOTOR_COUNT];
  int8_t control_init_status[MOTOR_CHASSIS_MOTOR_COUNT];
  int8_t feedback_update_status[MOTOR_CHASSIS_MOTOR_COUNT];
  int8_t current_set_status[MOTOR_CHASSIS_MOTOR_COUNT];
  int8_t control_status[MOTOR_CHASSIS_MOTOR_COUNT];
  int8_t can_tx_status;
  uint32_t debug_stop_zero_tx_count;
  float limited_target_speed_rpm[MOTOR_CHASSIS_MOTOR_COUNT];
  float ramped_target_speed_rpm[MOTOR_CHASSIS_MOTOR_COUNT];
  float current_command_a[MOTOR_CHASSIS_MOTOR_COUNT];
  float temperature_c[MOTOR_CHASSIS_MOTOR_COUNT];
} MotorChassisMonitor_t;

extern volatile MotorChassisTune_t g_motor_chassis_tune;
extern volatile MotorChassisMonitor_t g_motor_chassis_monitor;

/**
 * @brief 运行四个 M3508 电机速度控制任务
 *
 * @param[in] argument 任务参数，本任务不使用
 * @return 本任务不会返回
 */
void Task_motor_chassis(void *argument);

#ifdef __cplusplus
}
#endif
