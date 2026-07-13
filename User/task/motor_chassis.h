#pragma once

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdint.h>

/**
 * @brief 电机控制反馈快照
 *
 * 字段说明：
 * - online：电机在线状态；最近 100 ms 内收到有效反馈时为 true，超时后为 false。
 * - output_total_angle_rad：M3508 减速箱输出轴累计角度，单位 rad；首次反馈位置作为零点，正负号表示转动方向。
 * - output_speed_rpm：M3508 减速箱输出轴转速，单位 rpm；正负号表示转动方向。
 * - torque_current_feedback：按现有 RM 设备层和减速比换算的输出侧等效转矩电流，单位 A；正负号表示转矩方向。
 * - temperature_c：C620 反馈帧中的电机温度，单位摄氏度。
 * - feedback_timestamp_us：最近一次成功解析反馈的 BSP 微秒时间戳，单位 us；使用 32 位计数，约 71.6 min 回绕一次。
 */
typedef struct
{
    bool online;
    float output_total_angle_rad;
    float output_speed_rpm;
    float torque_current_feedback;
    float temperature_c;
    uint32_t feedback_timestamp_us;
} MotorChassisFeedbackSnapshot_t;

extern volatile MotorChassisFeedbackSnapshot_t g_motor_chassis_feedback;

/**
 * @brief 获取驱动电机反馈快照
 *
 * @param[out] snapshot 用于接收反馈快照的结构体指针
 * @return 成功返回 0，参数为空时返回负值
 */
int8_t MotorChassis_GetFeedbackSnapshot(MotorChassisFeedbackSnapshot_t *snapshot);

#ifdef __cplusplus
}
#endif
