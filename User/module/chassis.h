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
    CHASSIS_CAN_INIT_ERROR = -4,
    CHASSIS_CONTROL_INIT_ERROR = -5,
    CHASSIS_CONFIG_ERROR = -6,
    CHASSIS_DEVICE_UNAVAILABLE = -7,
    CHASSIS_INVALID_CURRENT = -8,
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
 * 底盘初始化与单周期结果快照：
 * - initialized：CAN 总线和四路速度控制器可运行时为 true，允许单个电机注册失败后隔离运行。
 * - can_init_status、control_init_status：CAN 设备边界和四路速度控制器的聚合初始化结果。
 * - feedback_status、chassis_status、output_status：本周期反馈、控制和电流提交的聚合结果。
 * - can_tx_status：本周期四路同组 CAN 控制帧统一发送结果。
 * - register_status[0~3]、motor_init_status[0~3]：四个设备和速度控制器的初始化结果。
 * - feedback_update_status[0~3]、control_status[0~3]、current_set_status[0~3]：本周期逐路处理结果。
 * - motor_online[0~3]、motor_enabled[0~3]：设备在线状态和速度控制实际使能状态。
 * - actual_speed_rpm[0~3]、temperature_c[0~3]：四路设备反馈，单位分别为 rpm 和摄氏度。
 * - limited_target_speed_rpm[0~3]、ramped_target_speed_rpm[0~3]：限幅和缓启动后的目标转速，单位 rpm。
 * - current_command_a[0~3]：实际提交的转子侧电流指令，单位 A；对应路径异常时为 0。
 */
typedef struct Chassis_Snapshot
{
    bool initialized;
    int8_t can_init_status;
    int8_t control_init_status;
    int8_t feedback_status;
    int8_t chassis_status;
    int8_t output_status;
    int8_t can_tx_status;
    int8_t register_status[CHASSIS_MOTOR_COUNT];
    int8_t motor_init_status[CHASSIS_MOTOR_COUNT];
    int8_t feedback_update_status[CHASSIS_MOTOR_COUNT];
    int8_t control_status[CHASSIS_MOTOR_COUNT];
    int8_t current_set_status[CHASSIS_MOTOR_COUNT];
    bool motor_online[CHASSIS_MOTOR_COUNT];
    bool motor_enabled[CHASSIS_MOTOR_COUNT];
    float actual_speed_rpm[CHASSIS_MOTOR_COUNT];
    float temperature_c[CHASSIS_MOTOR_COUNT];
    float limited_target_speed_rpm[CHASSIS_MOTOR_COUNT];
    float ramped_target_speed_rpm[CHASSIS_MOTOR_COUNT];
    float current_command_a[CHASSIS_MOTOR_COUNT];
} Chassis_Snapshot_t;

/**
 * @brief 初始化四个 M3508 的 CAN 设备边界和速度控制器
 *
 * 单个电机注册失败时保留其他电机运行能力，并在快照中记录对应注册状态。
 *
 * @param[in] sample_frequency_hz 控制采样频率，单位 Hz，必须大于 0
 * @param[out] snapshot 底盘初始化结果快照
 * @return 完全成功返回 CHASSIS_OK，存在隔离故障或初始化失败时返回对应状态码
 */
int8_t Chassis_Init(float sample_frequency_hz, Chassis_Snapshot_t *snapshot);

/**
 * @brief 执行一次反馈采集、四路速度控制和电流统一发送
 *
 * 单路反馈或电流写入失败时只清零对应电机，其他合法电机继续运行。
 *
 * @param[in] input 本周期控制输入快照
 * @param[out] snapshot 本周期反馈、控制和发送结果快照
 * @return 全链路正常返回 CHASSIS_OK，存在隔离故障返回 CHASSIS_ERROR
 */
int8_t Chassis_Run(const Chassis_Input_t *input, Chassis_Snapshot_t *snapshot);

#ifdef __cplusplus
}
#endif
