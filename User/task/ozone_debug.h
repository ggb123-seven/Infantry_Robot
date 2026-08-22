#pragma once

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdint.h>

#include "device/can_devices.h"
#include "device/dr16.h"
#include "module/fault_detect.h"
#include "module/gimbal.h"

/*
 * Ozone 底盘调试参数：
 * - OZONE_MOTOR_CHASSIS_COUNT：在线调试与监控的 M3508 电机数量，必须与底盘控制链一致。
 */
#define OZONE_MOTOR_CHASSIS_COUNT (4U)

struct Chassis_Input;
struct Chassis_Snapshot;

/*
 * DR16 Ozone 诊断数据：
 * - received_byte_count：UART BSP 交付的累计字节数。
 * - uart_event_count：UART BSP 交付的累计字节段数量。
 * - valid_frame_count、invalid_frame_count：合法帧数量和重同步期间尝试失败的候选帧数量。
 * - resync_discarded_byte_count：逐字节重同步累计丢弃的字节数。
 * - ring_buffer_overflow_count、uart_error_count：RingBuffer 溢出和 UART 错误次数。
 * - thread_notify_error_count、thread_wait_error_count：线程标志设置和等待异常次数。
 * - mailbox_error_count、stream_restart_error_count：状态邮箱发布和 UART 重启失败次数。
 * - last_uart_error：最近一次 UART HAL 错误位或 BSP 内部错误码。
 * - current_frame_age_us：当前时刻距离最后合法帧的时间，单位微秒，超过 32 位时饱和。
 * - receiver_initialized：DR16 协议字节流接收器初始化成功时为 true
 * - online：最近一帧业务状态的在线标志。
 * - fault_detected、fault_flags：DR16 当前故障汇总状态和故障位
 * - latest_data：最近一帧完整合法的解码值，仅供调试观察，不作为控制输入。
 */
typedef struct
{
    uint32_t received_byte_count;
    uint32_t uart_event_count;
    uint32_t valid_frame_count;
    uint32_t invalid_frame_count;
    uint32_t resync_discarded_byte_count;
    uint32_t ring_buffer_overflow_count;
    uint32_t uart_error_count;
    uint32_t thread_notify_error_count;
    uint32_t thread_wait_error_count;
    uint32_t mailbox_error_count;
    uint32_t stream_restart_error_count;
    uint32_t last_uart_error;
    uint32_t current_frame_age_us;
    bool receiver_initialized;
    bool online;
    bool fault_detected;
    uint32_t fault_flags;
    DR16_Data_t latest_data;
} DR16_Monitor_t;

/*
 * 四个 M3508 的 Ozone 在线调试参数：
 * - motor_debug_enable：可修改的四电机全局调试使能，设为 false 后所有速度环清零并持续发送零电流
 * - vx、vy、wz：可修改的归一化底盘运动分量，正方向依次为前向、左向和俯视逆时针
 * - scale_rpm：可修改的归一化轮速尺度，单位 rpm，底盘模块内限制到 CHASSIS_SPEED_LIMIT_RPM
 * - pid_kp、pid_ki、pid_kd：可修改的速度 PID 参数
 * - actual_speed_rpm[0~3]：只读的输出轴真实转速，依次对应 C620 电调 ID 1~4，单位 rpm
 */
typedef struct
{
    bool motor_debug_enable;
    float vx;
    float vy;
    float wz;
    float scale_rpm;
    float pid_kp;
    float pid_ki;
    float pid_kd;
    float actual_speed_rpm[OZONE_MOTOR_CHASSIS_COUNT];
} MotorChassisTune_t;

/*
 * 四个 M3508 的 Ozone 运行状态与诊断数据：
 * - motor_online[0~3]：最近 100 ms 内收到对应电机反馈时为 true。
 * - current_saturated[0~3]：对应电机电流指令达到正负限幅时为 true。
 * - debug_stop_ready：关闭调试使能后，连续成功提交 5 个周期的四电机零电流帧时为 true。
 * - register_status[0~3]：对应电机注册结果，0 表示成功。
 * - control_init_status[0~3]：对应速度环初始化结果，0 表示成功。
 * - feedback_update_status[0~3]：本周期对应电机反馈更新结果，0 表示成功。
 * - current_set_status[0~3]：本周期对应电流指令写入结果，0 表示成功。
 * - mixer_init_status：运动学混合器初始化结果，取值见 Mixer_Status_t
 * - mixer_status：本周期运动学解算结果，取值见 Mixer_Status_t
 * - chassis_status：本周期 Chassis 四路组合控制结果，0 表示全部活动控制器正常。
 * - control_status[0~3]：本周期对应速度环状态，0 表示正常，-2 表示使能关闭或反馈离线。
 * - can_tx_status：本周期 CAN 控制帧发送结果，0 表示成功。
 * - debug_stop_zero_tx_count：关闭调试使能后连续成功提交的零电流帧周期数。
 * - requested_speed_rpm[0~3]：运动学解算后的输出轴目标转速，依次对应 C620 电调 ID 1~4，单位 rpm
 * - limited_target_speed_rpm[0~3]：限幅后的对应输出轴目标转速，单位 rpm。
 * - ramped_target_speed_rpm[0~3]：缓启动后的对应输出轴目标转速，单位 rpm。
 * - current_command_a[0~3]：下发给对应 C620 的转子侧电流指令，单位 A。
 * - temperature_c[0~3]：对应 C620 反馈的电机温度，单位摄氏度。
 */
typedef struct
{
    bool motor_online[OZONE_MOTOR_CHASSIS_COUNT];
    bool current_saturated[OZONE_MOTOR_CHASSIS_COUNT];
    bool debug_stop_ready;
    int8_t register_status[OZONE_MOTOR_CHASSIS_COUNT];
    int8_t control_init_status[OZONE_MOTOR_CHASSIS_COUNT];
    int8_t feedback_update_status[OZONE_MOTOR_CHASSIS_COUNT];
    int8_t current_set_status[OZONE_MOTOR_CHASSIS_COUNT];
    int8_t mixer_init_status;
    int8_t mixer_status;
    int8_t chassis_status;
    int8_t control_status[OZONE_MOTOR_CHASSIS_COUNT];
    int8_t can_tx_status;
    uint32_t debug_stop_zero_tx_count;
    float requested_speed_rpm[OZONE_MOTOR_CHASSIS_COUNT];
    float limited_target_speed_rpm[OZONE_MOTOR_CHASSIS_COUNT];
    float ramped_target_speed_rpm[OZONE_MOTOR_CHASSIS_COUNT];
    float current_command_a[OZONE_MOTOR_CHASSIS_COUNT];
    float temperature_c[OZONE_MOTOR_CHASSIS_COUNT];
} MotorChassisMonitor_t;

/*
 * GM6020 Ozone 速度环在线参数：
 * - speed_control_enable：速度控制使能，false 时任务持续发布零电流
 * - target_speed_rpm：目标转速，单位 rpm，正负值决定旋转方向
 * - pid_kp、pid_ki、pid_kd：GM6020 独立速度 PID 参数
 */
typedef struct
{
    bool speed_control_enable;
    float target_speed_rpm;
    float pid_kp;
    float pid_ki;
    float pid_kd;
} MotorGM6020Tune_t;

/*
 * GM6020 Ozone 速度控制与物理反馈数据：
 * - control_init_status、control_status：速度环初始化和本周期控制状态，取值见 Gimbal_Status_t
 * - control_enabled：本周期使能、反馈有效且设备在线时为 true
 * - requested_speed_rpm、limited_target_speed_rpm、ramped_target_speed_rpm：请求、限幅和斜坡目标转速
 * - filtered_speed_rpm、speed_error_rpm：滤波后转速和速度误差，单位 rpm
 * - current_command_a：速度环输出的转子侧电流指令，单位 A，异常路径为 0
 * - register_status：CAN1 上 GM6020 的注册结果，0 表示成功
 * - online：最近 100 ms 内收到 GM6020 反馈时为 true
 * - angle_rad：转子单圈角度，范围 [0, 2π)，单位 rad
 * - speed_rpm、raw_current_lsb、torque_current_a、temperature_c：转速、原始电流、换算电流和温度
 *   单位分别为 rpm、LSB、A 和摄氏度
 */
typedef struct
{
    Gimbal_Status_t control_init_status;
    Gimbal_Status_t control_status;
    bool control_enabled;
    float requested_speed_rpm;
    float limited_target_speed_rpm;
    float ramped_target_speed_rpm;
    float filtered_speed_rpm;
    float speed_error_rpm;
    float current_command_a;
    int8_t register_status;
    bool online;
    float angle_rad;
    float speed_rpm;
    int16_t raw_current_lsb;
    float torque_current_a;
    float temperature_c;
} MotorGM6020Monitor_t;

extern volatile DR16_Monitor_t g_dr16_monitor;
extern volatile FaultDetect_Snapshot_t g_fault_detect_monitor;
extern volatile MotorChassisTune_t g_motor_chassis_tune;
extern volatile MotorChassisMonitor_t g_motor_chassis_monitor;
extern volatile MotorGM6020Tune_t g_motor_gm6020_tune;
extern volatile MotorGM6020Monitor_t g_motor_gm6020_monitor;

/**
 * @brief 将 CAN 设备集合初始化结果发布到 Ozone 监控区
 *
 * @param[in] can_snapshot CAN 设备集合初始化结果快照
 * @return 无返回值
 */
void OzoneDebug_UpdateCANDevicesInit(const CANDevices_Snapshot_t *can_snapshot);

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
 * @brief 从 Ozone 在线参数生成本周期底盘控制输入快照
 *
 * @param[out] input 待写入的底盘控制输入快照
 * @param[in] control_period_s 控制周期，单位 s，必须大于 0
 * @return 无返回值
 */
void OzoneDebug_GetMotorChassisInput(struct Chassis_Input *input, float control_period_s);

/**
 * @brief 从 Ozone 在线参数生成本周期 GM6020 速度控制输入
 *
 * @param[out] input 待写入的 GM6020 速度控制输入
 * @param[in] control_period_s 控制周期，单位 s，必须大于 0
 * @return 无返回值
 */
void OzoneDebug_GetMotorGimbalInput(Gimbal_Input_t *input, float control_period_s);

/**
 * @brief 发布 GM6020 本周期速度控制与物理反馈数据
 *
 * @param[in] can_snapshot CAN 设备集合本周期快照，允许为 NULL
 * @param[in] gimbal_snapshot GM6020 速度控制结果快照，允许为 NULL
 * @return 无返回值
 */
void OzoneDebug_UpdateMotorGimbal(const CANDevices_Snapshot_t *can_snapshot,
                                  const Gimbal_Snapshot_t *gimbal_snapshot);

/**
 * @brief 将 CAN 设备集合、GM6020 和底盘模块单周期结果发布到 Ozone 监控区
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
