#pragma once

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>

/*
 * 单圈角度展开器状态，角度单位由调用方决定，周期须与输入单位一致：
 * - has_reference：已记录上一帧单圈角度时为 true
 * - last_wrapped_angle：上一帧单圈角度，范围 [0, period)
 * - continuous_angle：展开后的连续角度，首帧等于单圈角度，之后按相邻帧最短路径累加
 */
typedef struct
{
    bool has_reference;
    float last_wrapped_angle;
    float continuous_angle;
} AngleUnwrap_t;

/**
 * @brief 将有限角度环绕到单圈区间
 *
 * @param[in] angle 有限角度
 * @param[in] period 单圈周期，须为有限正数，例如 360 或 2π
 * @return 单圈角度，范围 [0, period)
 */
float AngleUnwrap_Wrap(float angle, float period);

/**
 * @brief 丢弃上一帧参考，下一次更新重新以单圈角度建立连续坐标
 *
 * @param[out] unwrap 展开器状态，为空时不执行任何操作
 * @return 无返回值
 */
void AngleUnwrap_Reset(AngleUnwrap_t *unwrap);

/**
 * @brief 按相邻两帧最短路径展开单圈角度
 *
 * 相邻两帧差值恰为半圈时方向不确定，此时清除参考并返回失败
 *
 * @param[in,out] unwrap 展开器状态
 * @param[in] wrapped_angle 本帧单圈角度，范围 [0, period)
 * @param[in] period 单圈周期，须为有限正数
 * @param[out] continuous_angle 展开后的连续角度
 * @return 成功返回 true；参数非法、半圈歧义或结果溢出时返回 false 并清除参考
 */
bool AngleUnwrap_Update(AngleUnwrap_t *unwrap, float wrapped_angle, float period, float *continuous_angle);

#ifdef __cplusplus
}
#endif
