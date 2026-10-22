#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

/*
 * yaw 速度环调试参数与反馈集中区，初始值均为零
 * kp：比例增益，单位 A/(rad/s)，有限非负数，上限 1000
 * ki：积分增益，单位 A/rad，有限非负数，上限 1000
 * kd：反馈微分增益，单位 A/(rad/s²)，有限非负数，上限 1000
 * target_rad_s：速度目标，单位 rad/s，计算时限制至电机速度范围
 * actual_rad_s：实际角速度，单位 rad/s，由模块更新，反馈无效时为非数值
 * 调试器可修改前三个增益及目标值，实际值仅供观察；模块应由单个任务串行调用
 * volatile 保证调试器可见性，不提供多个任务同时写入时的同步保护
 */
typedef struct gimbal_yaw_speed
{
    float kp;
    float ki;
    float kd;
    float target_rad_s;
    float actual_rad_s;
} GimbalYawSpeed_t;

extern volatile GimbalYawSpeed_t gimbal_yaw_speed;

/*
 * 初次调参使用 1 A 电流限幅，低于手册所列 1.62 A 额定电流
 * 目标速度限幅依据 GM6020 使用说明 20231013 的 320 rpm 空载转速，以 2π/60 转为 rad/s
 * 反馈超过 100 ms 时禁止计算，控制周期最大 100 ms，均为当前模块的软件保护配置
 */
#define GIMBAL_YAW_CURRENT_LIMIT_A 1.0f
#define GIMBAL_YAW_SPEED_LIMIT_RAD_S (320.0f * 0.10471975512f)
#define GIMBAL_YAW_FEEDBACK_TIMEOUT_MS 100U
#define GIMBAL_YAW_MAX_PERIOD_S 0.1f

/**
 * @brief 初始化 yaw 速度控制器，保留调试区已有的参数和目标值
 *
 * @param[in] sample_frequency_hz 控制频率，单位 Hz，范围为 10 至 2000
 * @return 成功返回 0，频率非法或控制器初始化失败返回 -1
 */
int8_t Gimbal_Init(float sample_frequency_hz);

/**
 * @brief 根据 yaw 转子反馈计算单周期电流命令，不直接访问总线
 *
 * @param[in] feedback_rpm 驱动原始转子速度，单位 rpm，有效范围为有符号 16 位反馈范围
 * @param[in] feedback_age_ms 距最近有效帧的时间，单位 ms，尚无反馈时传入 UINT32_MAX
 * @param[in] enabled 是否允许速度控制，false 时清空控制器并输出零电流
 * @param[in] period_s 控制间隔，单位 s，须不小于初始化周期且不超过 0.1 s
 * @return 转子侧目标电流，单位 A，范围为正负 1 A，未初始化或输入无效时返回 0
 */
float Gimbal_YawSpeedRun(float feedback_rpm, uint32_t feedback_age_ms, bool enabled, float period_s);

/**
 * @brief 根据完整微秒时间戳检查反馈时效并执行速度闭环
 *
 * @param[in] feedback_rpm 驱动反馈转速，单位 rpm，无有效反馈时传入非数值
 * @param[in] feedback_time_us 最近反馈时刻，单位微秒，未收到反馈时传入 UINT64_MAX
 * @param[in] now_us 当前单调时间，单位微秒
 * @param[in] enabled 是否允许控制，false 时清空历史并输出零电流
 * @param[in] period_s 实际控制间隔，单位秒，须在初始化周期至 0.1 秒之间
 * @return 转子侧目标电流，单位 A，范围为正负 1 A，反馈过期或时间异常时返回零
 */
float Gimbal_YawSpeedRunAt(float feedback_rpm, uint64_t feedback_time_us, uint64_t now_us,
                         bool enabled, float period_s);

#ifdef __cplusplus
}
#endif
