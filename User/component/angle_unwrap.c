#include "component/angle_unwrap.h"

#include <math.h>
#include <stddef.h>

/**
 * @brief 将有限角度环绕到单圈区间
 *
 * @param[in] angle 有限角度
 * @param[in] period 单圈周期，须为有限正数，例如 360 或 2π
 * @return 单圈角度，范围 [0, period)
 */
float AngleUnwrap_Wrap(float angle, float period)
{
    float wrapped = fmodf(angle, period);
    if (wrapped < 0.0f)
    {
        wrapped += period;
    }
    // 负的极小角度加整圈后可能舍入为整圈，统一归到零点
    return wrapped >= period ? 0.0f : wrapped;
}

/**
 * @brief 丢弃上一帧参考，下一次更新重新以单圈角度建立连续坐标
 *
 * @param[out] unwrap 展开器状态，为空时不执行任何操作
 * @return 无返回值
 */
void AngleUnwrap_Reset(AngleUnwrap_t *unwrap)
{
    if (unwrap == NULL)
    {
        return;
    }
    unwrap->has_reference = false;
    unwrap->last_wrapped_angle = 0.0f;
    unwrap->continuous_angle = 0.0f;
}

/**
 * @brief 按相邻两帧最短路径展开单圈角度
 *
 * @param[in,out] unwrap 展开器状态
 * @param[in] wrapped_angle 本帧单圈角度，范围 [0, period)
 * @param[in] period 单圈周期，须为有限正数
 * @param[out] continuous_angle 展开后的连续角度
 * @return 成功返回 true；参数非法、半圈歧义或结果溢出时返回 false 并清除参考
 */
bool AngleUnwrap_Update(AngleUnwrap_t *unwrap, float wrapped_angle, float period, float *continuous_angle)
{
    if (unwrap == NULL || continuous_angle == NULL)
    {
        return false;
    }
    if (!isfinite(period) || period <= 0.0f || !isfinite(wrapped_angle) || wrapped_angle < 0.0f ||
        wrapped_angle >= period)
    {
        AngleUnwrap_Reset(unwrap);
        return false;
    }

    // 首帧直接以单圈角度建立连续坐标
    if (!unwrap->has_reference)
    {
        unwrap->has_reference = true;
        unwrap->last_wrapped_angle = wrapped_angle;
        unwrap->continuous_angle = wrapped_angle;
        *continuous_angle = wrapped_angle;
        return true;
    }

    // 将相邻帧差值折算到半圈以内，跨越编码器零点时取最短路径
    const float half_period = period * 0.5f;
    float delta = wrapped_angle - unwrap->last_wrapped_angle;
    if (delta > half_period)
    {
        delta -= period;
    }
    else if (delta < -half_period)
    {
        delta += period;
    }

    // 恰为半圈时无法判断转动方向，丢弃参考避免累计错误圈数
    const float next_angle = unwrap->continuous_angle + delta;
    if (fabsf(delta) >= half_period || !isfinite(next_angle))
    {
        AngleUnwrap_Reset(unwrap);
        return false;
    }

    unwrap->last_wrapped_angle = wrapped_angle;
    unwrap->continuous_angle = next_angle;
    *continuous_angle = next_angle;
    return true;
}
