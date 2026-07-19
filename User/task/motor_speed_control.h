#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

#include "component/pid.h"

/*
 * 本模块仅闭环控制一台 M3508 电机的减速箱输出轴转速。
 * 不包含底盘运动学解算、车体速度闭环或多电机同步控制。
 */

/*
 * 速度控制参数：
 * - MOTOR_SPEED_TARGET_RPM：单个 M3508 电机的目标速度接口，单位为减速箱输出轴
 * rpm。
 * - MOTOR_SPEED_LIMIT_RPM：目标速度正负对称限幅，单位为输出轴 rpm。
 * - MOTOR_SPEED_RAMP_RATE_RPM_S：缓启动最大变化斜率，单位 rpm/s。
 * - MOTOR_SPEED_PID_KP：比例增益默认值，单位 A/rpm。
 * - MOTOR_SPEED_PID_KI：积分增益默认值，单位 A/(rpm*s)。
 * - MOTOR_SPEED_PID_KD：反馈微分增益默认值，默认 0。
 * - MOTOR_SPEED_PID_D_CUTOFF_HZ：反馈微分低通截止频率，单位 Hz；小于等于 0
 * 时直通。
 * - MOTOR_SPEED_FEEDBACK_LPF_CUTOFF_HZ：速度反馈二阶低通截止频率，单位
 * Hz；小于等于 0 时直通。
 * - MOTOR_SPEED_CURRENT_LPF_CUTOFF_HZ：电流指令二阶低通截止频率，单位
 * Hz；小于等于 0 时直通。
 * - MOTOR_SPEED_PID_INTEGRAL_LIMIT：积分状态限幅，单位 rpm*s。
 * - MOTOR_SPEED_CURRENT_LIMIT_A：PID 输出的转子侧电流指令限幅，单位 A。
 * 默认目标为 100 rpm；PID 参数为上板整定结果，其余参数按实车测试需要配置。
 */
#ifndef MOTOR_SPEED_TARGET_RPM
#define MOTOR_SPEED_TARGET_RPM (100.0F)
#endif

#ifndef MOTOR_SPEED_LIMIT_RPM
#define MOTOR_SPEED_LIMIT_RPM (300.0F)
#endif

#ifndef MOTOR_SPEED_RAMP_RATE_RPM_S
#define MOTOR_SPEED_RAMP_RATE_RPM_S (150.0F)
#endif

#ifndef MOTOR_SPEED_PID_KP
#define MOTOR_SPEED_PID_KP (0.23F)
#endif

#ifndef MOTOR_SPEED_PID_KI
#define MOTOR_SPEED_PID_KI (0.14F)
#endif

#ifndef MOTOR_SPEED_PID_KD
#define MOTOR_SPEED_PID_KD (0.0F)
#endif

#ifndef MOTOR_SPEED_PID_D_CUTOFF_HZ
#define MOTOR_SPEED_PID_D_CUTOFF_HZ (20.0F)
#endif

#ifndef MOTOR_SPEED_FEEDBACK_LPF_CUTOFF_HZ
#define MOTOR_SPEED_FEEDBACK_LPF_CUTOFF_HZ (40.0F)
#endif

#ifndef MOTOR_SPEED_CURRENT_LPF_CUTOFF_HZ
#define MOTOR_SPEED_CURRENT_LPF_CUTOFF_HZ (-1.0F)
#endif

#ifndef MOTOR_SPEED_PID_INTEGRAL_LIMIT
#define MOTOR_SPEED_PID_INTEGRAL_LIMIT (20.0F)
#endif

#ifndef MOTOR_SPEED_CURRENT_LIMIT_A
#define MOTOR_SPEED_CURRENT_LIMIT_A (4.0F)
#endif

/**
 * @brief 速度 PID 在线调节参数
 *
 * 速度 PID 在线参数：
 * - kp：速度环比例增益，单位 A/rpm。
 * - ki：速度环积分增益，单位 A/(rpm*s)。
 * - kd：速度环反馈微分增益，默认值为 0。
 * 参数由任务层从 Ozone 监视结构体传入，任一字段为非有限值或负数时，
 * 速度环清除 PID 状态并输出零电流指令。
 */
typedef struct {
  float kp;
  float ki;
  float kd;
} MotorSpeedPidTune_t;

/**
 * @brief 速度环运行状态
 */
typedef enum {
  MOTOR_SPEED_CONTROL_OK = 0,
  MOTOR_SPEED_CONTROL_INIT_ERROR = -1,
  MOTOR_SPEED_CONTROL_DISABLED = -2,
  MOTOR_SPEED_CONTROL_INVALID_VALUE = -3,
  MOTOR_SPEED_CONTROL_CONFIG_ERROR = -4,
  MOTOR_SPEED_CONTROL_NULL_ERROR = -5,
} MotorSpeedControlStatus_t;

/**
 * @brief 速度环调试反馈结构体
 *
 * 字段说明：
 * - initialized：速度 PID 初始化成功时为 true。
 * - enabled：M3508 电机任务允许速度环输出电流指令时为 true。
 * - status：本周期速度环状态，取值见 MotorSpeedControlStatus_t。
 * - requested_speed_rpm：M3508 电机任务传入的原始目标速度，单位为输出轴 rpm。
 * - limited_target_speed_rpm：经过速度限幅后的目标速度，单位为输出轴 rpm。
 * - target_speed_rpm：经过限幅和缓启动后送入 PID 的目标速度，单位为输出轴 rpm。
 * - actual_speed_rpm：M3508 电机任务传入的真实输出轴速度，单位 rpm。
 * - filtered_speed_rpm：经过二阶低通滤波后送入 PID 的速度反馈，单位 rpm。
 * - speed_error_rpm：目标速度与滤波后速度之差，单位 rpm；速度环未使能时为 0。
 * - current_command_a：速度 PID
 * 输出经过二阶低通滤波和限幅后的转子侧电流指令，单位 A。
 * - pid_kp：本周期实际应用的比例增益，单位 A/rpm。
 * - pid_ki：本周期实际应用的积分增益，单位 A/(rpm*s)。
 * - pid_kd：本周期实际应用的反馈微分增益。
 */
typedef struct {
  bool initialized;
  bool enabled;
  int8_t status;
  float requested_speed_rpm;
  float limited_target_speed_rpm;
  float target_speed_rpm;
  float actual_speed_rpm;
  float filtered_speed_rpm;
  float speed_error_rpm;
  float current_command_a;
  float pid_kp;
  float pid_ki;
  float pid_kd;
} MotorSpeedControlFeedback_t;

/**
 * @brief 单个 M3508 输出轴速度环的完整运行上下文
 *
 * 速度环主结构体：
 * - pid_param：PID 参数，默认值在初始化时写入，运行中由 Ozone 调参值更新。
 * - pid：现有 PID 库的运行状态。
 * - feedback_filter：速度反馈二阶低通滤波器。
 * - current_filter：电流指令二阶低通滤波器。
 * - feedback：目标速度、真实速度、误差、电流输出和实际 PID 参数的调试反馈。
 * - ramped_target_speed_rpm：缓启动内部状态，单位为输出轴 rpm。
 * - was_enabled：上一周期使能状态，用于恢复时预置反馈微分。
 */
typedef struct {
  KPID_Params_t pid_param;
  KPID_t pid;
  LowPassFilter2p_t feedback_filter;
  LowPassFilter2p_t current_filter;
  MotorSpeedControlFeedback_t feedback;
  float ramped_target_speed_rpm;
  bool was_enabled;
} MotorSpeedControl_t;

/**
 * @brief 初始化速度环模块
 *
 * @param[in,out] control 速度环主结构体
 * @param[in] sample_frequency_hz 速度环采样频率，单位 Hz，必须大于 0
 * @return 成功返回 MOTOR_SPEED_CONTROL_OK，失败返回对应状态码
 */
int8_t MotorSpeedControl_Init(MotorSpeedControl_t *control,
                              float sample_frequency_hz);

/**
 * @brief 更新速度环真实速度反馈
 *
 * @param[in,out] control 速度环主结构体
 * @param[in] actual_speed_rpm 真实速度，单位为输出轴 rpm
 * @return 成功返回 MOTOR_SPEED_CONTROL_OK，失败返回对应状态码
 */
int8_t MotorSpeedControl_UpdateFeedback(MotorSpeedControl_t *control,
                                        float actual_speed_rpm);

/**
 * @brief 执行一次速度环控制计算
 *
 * @param[in,out] control 速度环主结构体
 * @param[in] requested_speed_rpm 原始目标速度，单位为输出轴 rpm
 * @param[in] pid_tune 本周期速度 PID 参数
 * @param[in] enabled 是否允许速度环产生非零电流指令
 * @param[in] control_period_s 本周期控制间隔，单位 s，必须大于 0
 * @return 本周期速度环状态，取值见 MotorSpeedControlStatus_t
 */
int8_t MotorSpeedControl_Control(MotorSpeedControl_t *control,
                                 float requested_speed_rpm,
                                 const MotorSpeedPidTune_t *pid_tune,
                                 bool enabled, float control_period_s);

/**
 * @brief 导出速度环电流指令
 *
 * @param[in] control 速度环主结构体
 * @param[out] current_command_a 转子侧电流指令，单位 A
 * @return 成功返回 MOTOR_SPEED_CONTROL_OK，参数为空时返回
 * MOTOR_SPEED_CONTROL_NULL_ERROR
 */
int8_t MotorSpeedControl_DumpOutput(const MotorSpeedControl_t *control,
                                    float *current_command_a);

#ifdef __cplusplus
}
#endif
