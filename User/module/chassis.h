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
 * @brief Chassis 模块状态
 */
typedef enum
{
    CHASSIS_OK = 0,
    CHASSIS_NULL_ERROR = -1,
    CHASSIS_CONFIG_ERROR = -2,
    CHASSIS_INIT_ERROR = -3,
    CHASSIS_CONTROL_ERROR = -4,
} Chassis_Status_t;

/*
 * Chassis 单周期输入：
 * - enabled：四路速度控制的统一使能，false 时所有电流输出清零。
 * - requested_speed_rpm[0~3]：四个输出轴目标转速，单位 rpm。
 * - actual_speed_rpm[0~3]：四个输出轴实际转速，单位 rpm。
 * - motor_online[0~3]：四个电机设备反馈是否在线。
 * - pid_tune：本周期四路共用的速度 PID 参数快照。
 * - control_period_s：本周期控制间隔，单位 s，必须大于 0。
 */
typedef struct
{
    bool enabled;
    float requested_speed_rpm[CHASSIS_MOTOR_COUNT];
    float actual_speed_rpm[CHASSIS_MOTOR_COUNT];
    bool motor_online[CHASSIS_MOTOR_COUNT];
    MotorSpeedPidTune_t pid_tune;
    float control_period_s;
} Chassis_Input_t;

/*
 * Chassis 单周期输出：
 * - control_status[0~3]：四路单电机速度控制状态。
 * - motor_enabled[0~3]：四路速度控制本周期是否实际使能。
 * - limited_target_speed_rpm[0~3]：四路限幅后目标转速，单位 rpm。
 * - ramped_target_speed_rpm[0~3]：四路斜坡后目标转速，单位 rpm。
 * - actual_speed_rpm[0~3]：四路速度控制采用的实际转速，单位 rpm。
 * - current_command_a[0~3]：四路安全转子侧电流指令，单位 A；任一路异常时该路为 0。
 */
typedef struct
{
    int8_t control_status[CHASSIS_MOTOR_COUNT];
    bool motor_enabled[CHASSIS_MOTOR_COUNT];
    float limited_target_speed_rpm[CHASSIS_MOTOR_COUNT];
    float ramped_target_speed_rpm[CHASSIS_MOTOR_COUNT];
    float actual_speed_rpm[CHASSIS_MOTOR_COUNT];
    float current_command_a[CHASSIS_MOTOR_COUNT];
} Chassis_Output_t;

/*
 * Chassis 跨周期控制上下文：
 * - initialized：四个速度控制器全部初始化成功时为 true。
 * - init_status[0~3]：四个单电机速度控制器的初始化结果。
 * - speed_control[0~3]：四个相互独立的单电机速度控制上下文。
 */
typedef struct
{
    bool initialized;
    int8_t init_status[CHASSIS_MOTOR_COUNT];
    MotorSpeedControl_t speed_control[CHASSIS_MOTOR_COUNT];
} Chassis_t;

/**
 * @brief 初始化四路 Chassis 速度控制器
 *
 * @param[out] chassis Chassis 控制上下文
 * @param[in] sample_frequency_hz 控制采样频率，单位 Hz，必须大于 0
 * @return 全部速度控制器初始化成功返回 CHASSIS_OK，否则返回对应状态码
 */
int8_t Chassis_Init(Chassis_t *chassis, float sample_frequency_hz);

/**
 * @brief 执行一次四路 Chassis 速度控制计算
 *
 * 任一路离线、禁用或输入异常时只清零该路输出，其余合法控制器继续独立运行。
 *
 * @param[in,out] chassis Chassis 控制上下文
 * @param[in] input 本周期四路输入快照
 * @param[out] output 本周期四路一致输出快照
 * @return 全部活动控制器正常时返回 CHASSIS_OK，存在控制异常时返回 CHASSIS_CONTROL_ERROR
 */
int8_t Chassis_Control(Chassis_t *chassis, const Chassis_Input_t *input, Chassis_Output_t *output);

#ifdef __cplusplus
}
#endif
