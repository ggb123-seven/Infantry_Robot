#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

/**
 * @brief 将原始通道计数限幅归一化并去除中心死区
 *
 * @param[in] channel 去中心后的原始计数
 * @param[in] channel_limit 满量程计数，须为有限正值
 * @param[in] deadzone 中心死区比例，须为有限值且位于 [0, 1)
 * @return 死区内为零，死区外线性映射到 [-1, 1]
 */
float InputShaping_NormalizeChannel(int16_t channel, float channel_limit, float deadzone);

/**
 * @brief 按加减速上限和一阶平滑逼近速度请求，换向时先经过零速
 *
 * @param[in] current 上周期速度，须为有限值，与 requested 单位一致
 * @param[in] requested 本周期请求速度，须为有限值
 * @param[in] accel 加速上限，单位为速度单位每秒，须为有限正值
 * @param[in] decel 减速上限，单位为速度单位每秒，须为有限正值
 * @param[in] period_s 控制周期，单位秒，须为有限正值
 * @param[in] smooth_time_s 一阶平滑时间常数，单位秒，须为有限正值
 * @param[in] settle 直接收敛的误差阈值，与速度单位一致，须为有限非负值
 * @return 平滑后的本周期速度
 */
float InputShaping_SmoothRate(float current, float requested, float accel, float decel,
                             float period_s, float smooth_time_s, float settle);

#ifdef __cplusplus
}
#endif
