#include "component/input_shaping.h"

#include <math.h>

#include "component/user_math.h"

/**
 * @brief 将原始通道计数限幅归一化并去除中心死区
 *
 * @param[in] channel 去中心后的原始计数
 * @param[in] channel_limit 满量程计数，须为有限正值
 * @param[in] deadzone 中心死区比例，须为有限值且位于 [0, 1)
 * @return 死区内为零，死区外线性映射到 [-1, 1]
 */
float InputShaping_NormalizeChannel(int16_t channel, float channel_limit, float deadzone)
{
    // 先限制满量程，再将死区外的有效行程重新映射到完整输出范围
    const float value = fmaxf(-1.0f, fminf(1.0f, (float)channel / channel_limit));
    if (fabsf(value) <= deadzone)
    {
        return 0.0f;
    }
    return copysignf((fabsf(value) - deadzone) / (1.0f - deadzone), value);
}

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
                             float period_s, float smooth_time_s, float settle)
{
    // 换向先以零速为本周期边界，避免跨过零速后仍沿用减速上限
    const float target = current * requested < 0.0f ? 0.0f : requested;
    const float rate_limit = fabsf(target) < fabsf(current) ? decel : accel;
    const float max_step = rate_limit * period_s;
    const float difference = fabsf(target - current);

    // 小误差有限时间收敛且仍受变化上限约束，避免零速请求留下长尾
    if (difference <= settle && difference <= max_step)
    {
        return target;
    }

    // 实际周期决定平滑比例，较大变化继续由单周期斜坡约束
    const float blend = 1.0f - expf(-period_s / smooth_time_s);
    return RampTo(current, target, fminf(max_step, difference * blend));
}
