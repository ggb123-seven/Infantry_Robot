#pragma once

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdint.h>

#include "component/pid.h"

/*
 * 单电机速度控制参数：
 * - MOTOR_SPEED_LIMIT_RPM：输出轴目标转速的正负对称限幅，单位 rpm。
 * - MOTOR_SPEED_RAMP_RATE_RPM_S：目标转速最大变化斜率，单位 rpm/s。
 * - MOTOR_SPEED_PID_KP：比例增益默认值，单位 A/rpm。
 * - MOTOR_SPEED_PID_KI：积分增益默认值，单位 A/(rpm*s)。
 * - MOTOR_SPEED_PID_KD：反馈微分增益默认值。
 * - MOTOR_SPEED_PID_D_CUTOFF_HZ：反馈微分低通截止频率，单位 Hz；小于等于 0 时直通。
 * - MOTOR_SPEED_FEEDBACK_LPF_CUTOFF_HZ：速度反馈二阶低通截止频率，单位 Hz；小于等于 0 时直通。
 * - MOTOR_SPEED_CURRENT_LPF_CUTOFF_HZ：电流指令二阶低通截止频率，单位 Hz；小于等于 0 时直通。
 * - MOTOR_SPEED_PID_INTEGRAL_LIMIT：积分状态限幅，单位 rpm*s。
 * - MOTOR_SPEED_CURRENT_LIMIT_A：转子侧电流指令正负对称限幅，单位 A。
 * 当前固化的上板整定结果为 Kp=0.23、Ki=0.14、Kd=0，PID 微分滤波截止频率为 20 Hz。
 * 速度反馈二阶低通截止频率为 45 Hz，电流输出二阶滤波保持直通。
 */
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
#define MOTOR_SPEED_FEEDBACK_LPF_CUTOFF_HZ (45.0F)
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

/*
 * 速度 PID 在线参数：
 * - kp：速度环比例增益，单位 A/rpm。
 * - ki：速度环积分增益，单位 A/(rpm*s)。
 * - kd：速度环反馈微分增益。
 */
typedef struct
{
    float kp;
    float ki;
    float kd;
} MotorSpeedPidTune_t;

/**
 * @brief 速度控制运行状态
 */
typedef enum
{
    MOTOR_SPEED_CONTROL_OK = 0,
    MOTOR_SPEED_CONTROL_INIT_ERROR = -1,
    MOTOR_SPEED_CONTROL_DISABLED = -2,
    MOTOR_SPEED_CONTROL_INVALID_VALUE = -3,
    MOTOR_SPEED_CONTROL_CONFIG_ERROR = -4,
    MOTOR_SPEED_CONTROL_NULL_ERROR = -5,
} MotorSpeedControlStatus_t;

/*
 * 速度控制反馈字段：
 * - initialized：速度 PID 初始化成功时为 true。
 * - enabled：本周期允许输出非零电流时为 true。
 * - status：本周期运行状态，取值见 MotorSpeedControlStatus_t。
 * - requested_speed_rpm：调用者请求的输出轴目标转速，单位 rpm。
 * - limited_target_speed_rpm：限幅后的输出轴目标转速，单位 rpm。
 * - target_speed_rpm：限幅并经过斜坡后的输出轴目标转速，单位 rpm。
 * - actual_speed_rpm：调用者提供的输出轴实际转速，单位 rpm。
 * - filtered_speed_rpm：送入 PID 的滤波后输出轴转速，单位 rpm。
 * - speed_error_rpm：目标转速与滤波后转速之差，单位 rpm。
 * - current_command_a：滤波并限幅后的转子侧电流指令，单位 A。
 * - pid_kp、pid_ki、pid_kd：本周期实际采用的 PID 参数。
 */
typedef struct
{
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

/*
 * 单电机速度控制上下文字段：
 * - pid_param：PID 配置参数。
 * - pid：PID 运行状态。
 * - feedback_filter：速度反馈二阶低通滤波器。
 * - current_filter：电流指令二阶低通滤波器。
 * - feedback：目标、反馈、误差、电流指令和运行状态。
 * - ramped_target_speed_rpm：斜坡内部目标转速，单位 rpm。
 * - was_enabled：上一周期速度控制是否使能。
 */
typedef struct
{
    KPID_Params_t pid_param;
    KPID_t pid;
    LowPassFilter2p_t feedback_filter;
    LowPassFilter2p_t current_filter;
    MotorSpeedControlFeedback_t feedback;
    float ramped_target_speed_rpm;
    bool was_enabled;
} MotorSpeedControl_t;

/**
 * @brief 初始化单电机速度控制模块
 *
 * @param[in,out] control 速度控制上下文
 * @param[in] sample_frequency_hz 控制采样频率，单位 Hz，必须大于 0
 * @return 成功返回 MOTOR_SPEED_CONTROL_OK，失败返回对应状态码
 */
int8_t MotorSpeedControl_Init(MotorSpeedControl_t *control, float sample_frequency_hz);

/**
 * @brief 更新单电机实际转速反馈
 *
 * @param[in,out] control 速度控制上下文
 * @param[in] actual_speed_rpm 输出轴实际转速，单位 rpm
 * @return 成功返回 MOTOR_SPEED_CONTROL_OK，失败返回对应状态码
 */
int8_t MotorSpeedControl_UpdateFeedback(MotorSpeedControl_t *control, float actual_speed_rpm);

/**
 * @brief 执行一次单电机速度控制计算
 *
 * @param[in,out] control 速度控制上下文
 * @param[in] requested_speed_rpm 请求的输出轴目标转速，单位 rpm
 * @param[in] pid_tune 本周期速度 PID 参数
 * @param[in] enabled 是否允许产生非零电流指令
 * @param[in] control_period_s 本周期控制间隔，单位 s，必须大于 0
 * @return 本周期速度控制状态，取值见 MotorSpeedControlStatus_t
 */
int8_t MotorSpeedControl_Control(MotorSpeedControl_t *control, float requested_speed_rpm,
                                 const MotorSpeedPidTune_t *pid_tune, bool enabled, float control_period_s);

/**
 * @brief 导出单电机转子侧电流指令
 *
 * @param[in] control 速度控制上下文
 * @param[out] current_command_a 转子侧电流指令，单位 A
 * @return 成功返回 MOTOR_SPEED_CONTROL_OK，失败返回对应状态码
 */
int8_t MotorSpeedControl_DumpOutput(const MotorSpeedControl_t *control, float *current_command_a);

#ifdef __cplusplus
}
#endif
