#pragma once

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdint.h>

#include "component/user_math.h"
#include "device/can_devices.h"
#include "module/fault_detect.h"

/*
 * Ozone 底盘调试参数：
 * - OZONE_MOTOR_CHASSIS_COUNT：在线调试与监控的 M3508 电机数量，必须与底盘控制链一致。
 */
#define OZONE_MOTOR_CHASSIS_COUNT (4U)

struct Chassis_Input;
struct Chassis_Snapshot;

/*
 * 单个底盘电机的速度 PID 覆盖参数：
 * - pid_override_enable：为 true 时仅对指定电机启用独立参数
 * - pid_override_motor_index：底盘电机 ID，取值 1~4，ID4 对应数组下标 3
 * - pid_override_kp、pid_override_ki、pid_override_kd：指定电机的速度 PID 参数
 */
typedef struct
{
    bool pid_override_enable;
    uint32_t pid_override_motor_index;
    float pid_override_kp;
    float pid_override_ki;
    float pid_override_kd;
} MotorChassisPidOverride_t;

/*
 * 四个 M3508 的 Ozone 调试参数，使能和运动向量统一由 DR16 遥控输入生成：
 * - scale_rpm：可修改的归一化轮速尺度，单位 rpm，底盘模块内限制到 CHASSIS_SPEED_LIMIT_RPM
 * - pid_kp、pid_ki、pid_kd：ID1~3 共用的底盘速度 PID 参数
 * - pid_override：指定单个电机的独立速度 PID 参数，当前默认选择 ID4
 */
typedef struct
{
    float scale_rpm;
    float pid_kp;
    float pid_ki;
    float pid_kd;
    MotorChassisPidOverride_t pid_override;
} MotorChassisTune_t;

/*
 * 底盘目标速度与反馈速度：
 * - requested_speed_rpm[0~3]：运动学解算后的四路输出轴目标转速，单位 rpm
 * - actual_speed_rpm[0~3]：CAN 反馈的四路输出轴实际转速，单位 rpm
 */
typedef struct
{
    float requested_speed_rpm[OZONE_MOTOR_CHASSIS_COUNT];
    float actual_speed_rpm[OZONE_MOTOR_CHASSIS_COUNT];
} MotorChassisSpeed_t;

/*
 * 四个 M3508 的 Ozone 日常监视量，数组下标依次对应 C620 电调 ID 1~4：
 * - motor_online[0~3]：最近一次 CAN 快照中的电机在线标志
 * - temperature_c[0~3]：最近一次 CAN 快照中的电机温度，单位摄氏度
 * - speed_loop_ok[0~3]：本周期对应电机速度环计算正常时为 true
 * - speed：四个电机的目标转速和反馈转速，单位 rpm
 */
typedef struct
{
    bool motor_online[OZONE_MOTOR_CHASSIS_COUNT];
    float temperature_c[OZONE_MOTOR_CHASSIS_COUNT];
    bool speed_loop_ok[OZONE_MOTOR_CHASSIS_COUNT];
    MotorChassisSpeed_t speed;
} MotorChassisMonitor_t;

extern volatile FaultDetect_Snapshot_t g_fault_detect_monitor;
extern volatile MotorChassisTune_t g_motor_chassis_tune;
extern volatile MotorChassisMonitor_t g_motor_chassis_monitor;

/**
 * @brief 将底盘速度控制器初始化结果发布到 Ozone 监控区
 *
 * @param[in] chassis_snapshot 底盘速度控制器初始化结果快照
 * @return 无返回值
 */
void OzoneDebug_UpdateChassisInit(const struct Chassis_Snapshot *chassis_snapshot);

/**
 * @brief 将独立故障检测结果发布到 Ozone 监控区
 *
 * @param[in] fault_snapshot 本周期故障检测结果快照
 * @return 无返回值
 */
void OzoneDebug_UpdateFaultDetect(const FaultDetect_Snapshot_t *fault_snapshot);

/**
 * @brief 将 CAN 设备集合和底盘模块单周期结果发布到 Ozone 监控区
 *
 * @param[in] input 本周期实际采用的底盘控制输入快照
 * @param[in] can_snapshot 本周期 CAN 设备反馈与输出结果快照，允许为 NULL
 * @param[in] chassis_snapshot 本周期底盘速度控制结果快照
 * @return 无返回值
 */
void OzoneDebug_UpdateMotorChassis(const struct Chassis_Input *input, const CANDevices_Snapshot_t *can_snapshot,
                                   const struct Chassis_Snapshot *chassis_snapshot);

#ifdef __cplusplus
}
#endif
