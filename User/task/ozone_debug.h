#pragma once

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdint.h>

#include "device/dr16.h"

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
 * - online：最近一帧业务状态的在线标志。
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
    bool online;
    DR16_Data_t latest_data;
} DR16_Monitor_t;

/*
 * 四个 M3508 的 Ozone 在线调试参数：
 * - motor_debug_enable：可修改的四电机全局调试使能；设为 false 后所有速度环清零并持续发送零电流。
 * - requested_speed_rpm[0~3]：可修改的输出轴目标转速，依次对应 C620 电调 ID 1~4，单位 rpm。
 * - pid_kp、pid_ki、pid_kd：可修改的速度 PID 参数。
 * - actual_speed_rpm[0~3]：只读的输出轴真实转速，依次对应 C620 电调 ID 1~4，单位 rpm。
 */
typedef struct
{
    bool motor_debug_enable;
    float requested_speed_rpm[OZONE_MOTOR_CHASSIS_COUNT];
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
 * - chassis_status：本周期 Chassis 四路组合控制结果，0 表示全部活动控制器正常。
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
    bool motor_online[OZONE_MOTOR_CHASSIS_COUNT];
    bool current_saturated[OZONE_MOTOR_CHASSIS_COUNT];
    bool debug_stop_ready;
    int8_t register_status[OZONE_MOTOR_CHASSIS_COUNT];
    int8_t control_init_status[OZONE_MOTOR_CHASSIS_COUNT];
    int8_t feedback_update_status[OZONE_MOTOR_CHASSIS_COUNT];
    int8_t current_set_status[OZONE_MOTOR_CHASSIS_COUNT];
    int8_t chassis_status;
    int8_t control_status[OZONE_MOTOR_CHASSIS_COUNT];
    int8_t can_tx_status;
    uint32_t debug_stop_zero_tx_count;
    float limited_target_speed_rpm[OZONE_MOTOR_CHASSIS_COUNT];
    float ramped_target_speed_rpm[OZONE_MOTOR_CHASSIS_COUNT];
    float current_command_a[OZONE_MOTOR_CHASSIS_COUNT];
    float temperature_c[OZONE_MOTOR_CHASSIS_COUNT];
} MotorChassisMonitor_t;

extern volatile DR16_Monitor_t g_dr16_monitor;
extern volatile MotorChassisTune_t g_motor_chassis_tune;
extern volatile MotorChassisMonitor_t g_motor_chassis_monitor;

/**
 * @brief 将底盘模块初始化结果发布到 Ozone 监控区
 *
 * @param[in] snapshot 底盘初始化结果快照
 * @return 无返回值
 */
void OzoneDebug_UpdateMotorChassisInit(const struct Chassis_Snapshot *snapshot);

/**
 * @brief 从 Ozone 在线参数生成本周期底盘控制输入快照
 *
 * @param[out] input 待写入的底盘控制输入快照
 * @param[in] control_period_s 控制周期，单位 s，必须大于 0
 * @return 无返回值
 */
void OzoneDebug_GetMotorChassisInput(struct Chassis_Input *input, float control_period_s);

/**
 * @brief 将底盘模块单周期结果发布到 Ozone 监控区
 *
 * @param[in] input 本周期实际采用的底盘控制输入快照
 * @param[in] snapshot 本周期底盘模块结果快照
 * @return 无返回值
 */
void OzoneDebug_UpdateMotorChassis(const struct Chassis_Input *input, const struct Chassis_Snapshot *snapshot);

#ifdef __cplusplus
}
#endif
