#pragma once

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdint.h>

/*
 * GM6020 速度控制参数：
 * - GIMBAL_SPEED_LIMIT_RAD_S：目标角速度正负对称限幅，单位 rad/s
 * - GIMBAL_PID_KP、GIMBAL_PID_KI、GIMBAL_PID_KD：GM6020 独立速度 PID 初始参数
 * - GIMBAL_PID_D_CUTOFF_HZ：反馈微分低通截止频率，单位 Hz，小于等于 0 时直通
 * - GIMBAL_FEEDBACK_LPF_CUTOFF_HZ：速度反馈二阶低通截止频率，单位 Hz，小于等于 0 时直通
 * - GIMBAL_CURRENT_LPF_CUTOFF_HZ：电流指令二阶低通截止频率，单位 Hz，小于等于 0 时直通
 * - GIMBAL_PID_INTEGRAL_LIMIT：积分状态正负对称限幅，单位 rad
 * - GIMBAL_CURRENT_LIMIT_A：速度环输出电流正负对称限幅，单位 A
 * 当前参数仅作为低电流上板整定初值，必须结合 GM6020 实际负载继续整定
 */
#ifndef GIMBAL_SPEED_LIMIT_RAD_S
#define GIMBAL_SPEED_LIMIT_RAD_S (10.4719755F)
#endif

#ifndef GIMBAL_PID_KP
#define GIMBAL_PID_KP (0.03F)
#endif

#ifndef GIMBAL_PID_KI
#define GIMBAL_PID_KI (0.3F)
#endif

#ifndef GIMBAL_PID_KD
#define GIMBAL_PID_KD (0.0F)
#endif

#ifndef GIMBAL_PID_D_CUTOFF_HZ
#define GIMBAL_PID_D_CUTOFF_HZ (-1.0F)
#endif

#ifndef GIMBAL_FEEDBACK_LPF_CUTOFF_HZ
#define GIMBAL_FEEDBACK_LPF_CUTOFF_HZ (-1.0F)
#endif

#ifndef GIMBAL_CURRENT_LPF_CUTOFF_HZ
#define GIMBAL_CURRENT_LPF_CUTOFF_HZ (-1.0F)
#endif

#ifndef GIMBAL_PID_INTEGRAL_LIMIT
#define GIMBAL_PID_INTEGRAL_LIMIT (10.0F)
#endif

#ifndef GIMBAL_CURRENT_LIMIT_A
#define GIMBAL_CURRENT_LIMIT_A (1.0F)
#endif

/**
 * @brief GM6020 速度控制状态
 */
typedef enum
{
    GIMBAL_OK = 0,
    GIMBAL_ERROR = -1,
    GIMBAL_NULL_ERROR = -2,
    GIMBAL_NOT_INITIALIZED = -3,
    GIMBAL_DISABLED = -4,
    GIMBAL_INVALID_VALUE = -5,
    GIMBAL_CONFIG_ERROR = -6,
} Gimbal_Status_t;

/*
 * GM6020 速度 PID 在线参数：
 * - kp：比例增益，单位 A/(rad/s)
 * - ki：积分增益，单位 A/rad
 * - kd：反馈微分增益
 */
typedef struct
{
    float kp;
    float ki;
    float kd;
} Gimbal_PidTune_t;

/*
 * GM6020 单周期速度控制输入：
 * - enabled：速度控制使能，false 时复位控制器并输出零电流
 * - target_speed_rad_s：目标角速度，单位 rad/s，正负值决定旋转方向
 * - pid_tune：本周期采用的 GM6020 独立 PID 参数
 * - control_period_s：本周期控制间隔，单位 s，必须大于 0
 */
typedef struct
{
    bool enabled;
    float target_speed_rad_s;
    Gimbal_PidTune_t pid_tune;
    float control_period_s;
} Gimbal_Input_t;

/*
 * GM6020 单周期反馈输入：
 * - valid：本周期反馈可用于闭环控制时为 true
 * - online：最近通信周期内电机在线时为 true
 * - actual_speed_rad_s：电机实际角速度，单位 rad/s
 */
typedef struct
{
    bool valid;
    bool online;
    float actual_speed_rad_s;
} Gimbal_Feedback_t;

/*
 * GM6020 单周期控制输出：
 * - current_command_a：速度 PID 计算得到的转子侧电流，单位 A，异常路径为 0
 */
typedef struct
{
    float current_command_a;
} Gimbal_Output_t;

/*
 * GM6020 速度控制结果快照：
 * - initialized：速度控制器初始化成功时为 true
 * - init_status、control_status：初始化和本周期控制状态，取值见 Gimbal_Status_t
 * - enabled：本周期反馈有效、设备在线且控制使能时为 true
 * - requested_speed_rad_s：外部请求的目标角速度，单位 rad/s
 * - limited_target_speed_rad_s：经过安全限幅的目标角速度，单位 rad/s
 * - ramped_target_speed_rad_s：兼容字段，当前直接等于限幅后的 PID 目标角速度，单位 rad/s
 * - actual_speed_rad_s、filtered_speed_rad_s：实际与滤波后反馈角速度，单位 rad/s
 * - speed_error_rad_s：PID 目标与滤波反馈之差，单位 rad/s
 * - current_command_a：本周期转子侧电流指令，单位 A，异常路径为 0
 * - pid_kp、pid_ki、pid_kd：本周期实际采用的 GM6020 独立 PID 参数
 */
typedef struct
{
    bool initialized;
    Gimbal_Status_t init_status;
    Gimbal_Status_t control_status;
    bool enabled;
    float requested_speed_rad_s;
    float limited_target_speed_rad_s;
    float ramped_target_speed_rad_s;
    float actual_speed_rad_s;
    float filtered_speed_rad_s;
    float speed_error_rad_s;
    float current_command_a;
    float pid_kp;
    float pid_ki;
    float pid_kd;
} Gimbal_Snapshot_t;

/**
 * @brief 初始化 GM6020 速度控制器
 *
 * @param[in] sample_frequency_hz 控制采样频率，单位 Hz，必须大于 0
 * @param[out] snapshot 初始化结果快照
 * @return 初始化成功返回 GIMBAL_OK，失败返回对应状态码
 */
Gimbal_Status_t Gimbal_Init(float sample_frequency_hz, Gimbal_Snapshot_t *snapshot);

/**
 * @brief 执行一次 GM6020 速度闭环控制
 *
 * @param[in] input 本周期控制输入
 * @param[in] feedback 本周期电机反馈
 * @param[out] output 本周期电流输出
 * @param[out] snapshot 本周期控制结果快照
 * @return 控制正常返回 GIMBAL_OK，禁用或异常时返回对应状态码
 */
Gimbal_Status_t Gimbal_Run(const Gimbal_Input_t *input, const Gimbal_Feedback_t *feedback,
                           Gimbal_Output_t *output, Gimbal_Snapshot_t *snapshot);

#ifdef __cplusplus
}
#endif
