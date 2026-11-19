/*
 * 云台调试记录实现
 *
 * 只做静态缓冲写入，不加锁不滤波；写满后自动关闭使能，保证最早期数据不被覆盖
 * 整个文件受 MOTOR_GIMBAL_ENABLE_DEBUG_LOG 控制，默认不参与编译
 */

/* Includes ----------------------------------------------------------------- */
#include "task/gimbal_debug_log.h"

#if MOTOR_GIMBAL_ENABLE_DEBUG_LOG

/* Exported variables ------------------------------------------------------- */
Gimbal_DebugLogSample_t gimbal_debug_log[GIMBAL_DEBUG_LOG_CAPACITY];
volatile uint32_t gimbal_debug_log_count = 0U;
volatile bool gimbal_debug_log_enabled = false;

/* Exported functions ------------------------------------------------------- */
/**
 * @brief 按当前使能状态写入一条周期记录
 *
 * @param command_deg 角度指令，单位：度
 * @param angle_deg IMU 世界系 yaw 角，单位：度
 * @param speed_rad_s 世界系 yaw 角速度，单位：rad/s
 * @param current_a yaw 输出电流，单位：A
 * @return 无返回值
 */
void Gimbal_DebugLog_Push(float command_deg, float angle_deg, float speed_rad_s, float current_a)
{
    if (!gimbal_debug_log_enabled)
    {
        return;
    }
    const uint32_t index = gimbal_debug_log_count;
    if (index >= GIMBAL_DEBUG_LOG_CAPACITY)
    {
        gimbal_debug_log_enabled = false;
        return;
    }
    gimbal_debug_log[index] = (Gimbal_DebugLogSample_t)
    {
        .command_deg = command_deg,
        .angle_deg = angle_deg,
        .speed_rad_s = speed_rad_s,
        .current_a = current_a,
    };
    gimbal_debug_log_count = index + 1U;
}

#endif
