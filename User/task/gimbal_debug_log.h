/*
 * 云台调试记录接口
 *
 * 按 500 Hz 控制周期把角度指令、IMU 世界角、角速度与输出电流写入静态缓冲
 * 供 EmberProbe 分片导出后做角度环系统辨识，不参与控制计算
 * 默认编译关闭，正式固件不分配缓冲；做辨识时把 MOTOR_GIMBAL_ENABLE_DEBUG_LOG 置 1 重新构建
 */

#pragma once

#ifdef __cplusplus
extern "C"
{
#endif

/* Includes ----------------------------------------------------------------- */
#include <stdbool.h>
#include <stdint.h>

/* Private define ----------------------------------------------------------- */
/* 记录编译开关，默认关闭；置 1 时占用 64 kB 静态 RAM，供离线辨识使用 */
#ifndef MOTOR_GIMBAL_ENABLE_DEBUG_LOG
#define MOTOR_GIMBAL_ENABLE_DEBUG_LOG (0)
#endif

#if MOTOR_GIMBAL_ENABLE_DEBUG_LOG
/* 缓冲容量按 500 Hz 计算约 8.2 s，四项 float 共占 64 kB 静态 RAM */
#define GIMBAL_DEBUG_LOG_CAPACITY (4096U)

/* Exported types ----------------------------------------------------------- */
/* 单周期记录样本，四项取自同一 Control 周期快照 */
typedef struct
{
    float command_deg; /* 角度指令，单位：度 */
    float angle_deg;   /* IMU 世界系 yaw 角，单位：度 */
    float speed_rad_s; /* 世界系 yaw 角速度，单位：rad/s */
    float current_a;   /* yaw 输出电流，单位：A */
} Gimbal_DebugLogSample_t;

/* Exported variables ------------------------------------------------------- */
/* 样本缓冲，按写入顺序排列，写满后停止记录以便完整导出 */
extern Gimbal_DebugLogSample_t gimbal_debug_log[GIMBAL_DEBUG_LOG_CAPACITY];
/* 已写入样本数，写 0 可清空；写满后模块自动关闭使能 */
extern volatile uint32_t gimbal_debug_log_count;
/* 记录使能，缓冲写满后由模块自动清零 */
extern volatile bool gimbal_debug_log_enabled;

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
void Gimbal_DebugLog_Push(float command_deg, float angle_deg, float speed_rad_s, float current_a);
#endif

#ifdef __cplusplus
}
#endif
