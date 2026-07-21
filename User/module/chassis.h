#pragma once

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdint.h>

#include "module/motor_speed_control.h"

#define CHASSIS_MOTOR_COUNT (4U)

/**
 * @brief 底盘完整控制链状态
 */
typedef enum
{
    CHASSIS_OK = 0,
    CHASSIS_ERROR = -1,
    CHASSIS_NULL_ERROR = -2,
    CHASSIS_NOT_INITIALIZED = -3,
    CHASSIS_CONTROL_INIT_ERROR = -5,
    CHASSIS_CONFIG_ERROR = -6,
} Chassis_Status_t;

/*
 * 底盘单周期输入：
 * - enabled：四路速度控制统一使能，false 时所有电流输出清零。
 * - requested_speed_rpm[0~3]：四个输出轴目标转速，单位 rpm。
 * - pid_tune：本周期四路共用的速度 PID 参数快照。
 * - control_period_s：本周期控制间隔，单位 s，必须大于 0。
 */
typedef struct Chassis_Input
{
    bool enabled;
    float requested_speed_rpm[CHASSIS_MOTOR_COUNT];
    MotorSpeedPidTune_t pid_tune;
    float control_period_s;
} Chassis_Input_t;

/*
 * 底盘单周期反馈输入：
 * - valid[0~3]：本周期对应电机反馈可用于控制时为 true。
 * - motor_online[0~3]：对应电机处于在线状态时为 true。
 * - actual_speed_rpm[0~3]：四个输出轴实际转速，单位 rpm。
 */
typedef struct
{
    bool valid[CHASSIS_MOTOR_COUNT];
    bool motor_online[CHASSIS_MOTOR_COUNT];
    float actual_speed_rpm[CHASSIS_MOTOR_COUNT];
} Chassis_Feedback_t;

/*
 * 底盘单周期控制输出：
 * - current_command_a[0~3]：四路速度控制计算得到的转子侧电流命令，单位 A；异常路径为 0。
 */
typedef struct
{
    float current_command_a[CHASSIS_MOTOR_COUNT];
} Chassis_Output_t;

/*
 * 底盘速度控制初始化与单周期结果快照：
 * - initialized：四路速度控制器均可运行时为 true。
 * - control_init_status：四路速度控制器的聚合初始化结果。
 * - chassis_status：本周期四路速度控制的聚合结果。
 * - motor_init_status[0~3]、control_status[0~3]：四路速度控制器的初始化和本周期运行结果。
 * - motor_enabled[0~3]：本周期对应速度控制器实际使能状态。
 * - limited_target_speed_rpm[0~3]、ramped_target_speed_rpm[0~3]：限幅和缓启动后的目标转速，单位 rpm。
 * - actual_speed_rpm[0~3]：本周期参与速度控制的输出轴实际转速，单位 rpm。
 * - current_command_a[0~3]：本周期计算得到的转子侧电流命令，单位 A；对应路径异常时为 0。
 */
typedef struct Chassis_Snapshot
{
    bool initialized;
    int8_t control_init_status;
    int8_t chassis_status;
    int8_t motor_init_status[CHASSIS_MOTOR_COUNT];
    int8_t control_status[CHASSIS_MOTOR_COUNT];
    bool motor_enabled[CHASSIS_MOTOR_COUNT];
    float actual_speed_rpm[CHASSIS_MOTOR_COUNT];
    float limited_target_speed_rpm[CHASSIS_MOTOR_COUNT];
    float ramped_target_speed_rpm[CHASSIS_MOTOR_COUNT];
    float current_command_a[CHASSIS_MOTOR_COUNT];
} Chassis_Snapshot_t;

/**
 * @brief 初始化四个相互独立的 M3508 速度控制器
 *
 * @param[in] sample_frequency_hz 控制采样频率，单位 Hz，必须大于 0
 * @param[out] snapshot 底盘初始化结果快照
 * @return 全部速度控制器初始化成功返回 CHASSIS_OK，否则返回对应状态码
 */
int8_t Chassis_Init(float sample_frequency_hz, Chassis_Snapshot_t *snapshot);

/**
 * @brief 根据反馈执行一次四路速度控制计算
 *
 * 单路反馈无效、离线或控制计算失败时只清零对应输出，其他合法控制器继续运行。
 *
 * @param[in] input 本周期控制输入快照
 * @param[in] feedback 本周期四路电机反馈快照
 * @param[out] output 本周期四路电流计算结果
 * @param[out] snapshot 本周期速度控制结果快照
 * @return 四路速度控制均正常或安全禁用时返回 CHASSIS_OK，存在计算异常时返回 CHASSIS_ERROR
 */
int8_t Chassis_Run(const Chassis_Input_t *input, const Chassis_Feedback_t *feedback, Chassis_Output_t *output,
                   Chassis_Snapshot_t *snapshot);

#ifdef __cplusplus
}
#endif
