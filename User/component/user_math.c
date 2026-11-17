/*
 * 通用数学工具实现
 *
 * 提供快速倒数平方根、限幅、循环量运算与摩擦轮转速换算，供控制层直接调用
 */

/* Includes ----------------------------------------------------------------- */
#include "user_math.h"
#include <string.h>

/* USER INCLUDE BEGIN */

/* USER INCLUDE END */

/* Exported functions ------------------------------------------------------- */
inline float InvSqrt(float x) {
//#if 0
  /* 快速倒数平方根近似，原理见 https://en.wikipedia.org/wiki/Fast_inverse_square_root */
	float halfx = 0.5f * x;
	float y = x;
	long i = *(long*)&y;
	i = 0x5f3759df - (i>>1);
	y = *(float*)&i;
	y = y * (1.5f - (halfx * y * y));
	y = y * (1.5f - (halfx * y * y));
	return y;
//#else
//  return 1.0f / sqrtf(x);
//#endif
}

/**
 * @brief 按对称限幅截断数值
 *
 * @param in 输入值
 * @param limit 限幅绝对值
 * @return float 截断后的值
 */
inline float AbsClip(float in, float limit) {
  return (in < -limit) ? -limit : ((in > limit) ? limit : in);
}

/**
 * @brief 按单周期最大步长逼近目标值
 *
 * @param current 当前值
 * @param target 目标值
 * @param max_step 单周期允许的最大变化量
 * @return float 更新后的值
 */
float RampTo(float current, float target, float max_step)
{
    if (!isfinite(max_step) || max_step < 0.0F)
    {
        return current;
    }

    const float difference = target - current;
    if (difference > max_step)
    {
        return current + max_step;
    }
    if (difference < -max_step)
    {
        return current - max_step;
    }
    return target;
}

/**
 * @brief 取浮点数的绝对值
 *
 * @param in 输入值
 * @return float 绝对值
 */
float fAbs(float in){
  return (in > 0) ? in : -in;
}

/**
 * @brief 将数值限制在指定区间内
 *
 * @param origin 待限制的数值
 * @param min 区间下限
 * @param max 区间上限
 */
inline void Clip(float *origin, float min, float max) {
  if (*origin > max) *origin = max;
  if (*origin < min) *origin = min;
}

/**
 * @brief 取数值的符号
 *
 * @param in 输入值
 * @return float 非负返回 1，负值返回 0
 */
inline float Sign(float in) { return (in > 0) ? 1.0f : 0.0f; }

/**
 * @brief 将运动向量置零
 *
 * @param mv 待清零的运动向量
 */
inline void ResetMoveVector(MoveVector_t *mv) { memset(mv, 0, sizeof(*mv)); }

/**
 * @brief 计算循环量误差
 *
 * 设定值与反馈值位于同一循环区间时按最短路径取差，range 取该区间宽度，
 * 为 0 时退化为线性误差
 *
 * @param sp 设定值
 * @param fb 反馈值
 * @param range 循环区间宽度，与输入同单位
 * @return float 误差，落在区间宽度的一半以内
 */
inline float CircleError(float sp, float fb, float range) {
  float error = sp - fb;
  if (range > 0.0f) {
    float half_range = range / 2.0f;

    if (error > half_range)
      error -= range;
    else if (error < -half_range)
      error += range;
  }
  return error;
}

/**
 * @brief 在循环区间内累加增量
 *
 * 被操作的值始终保持在 [0, range) 内，越过边界时回卷
 *
 * @param origin 被操作的循环量
 * @param delta 增量
 * @param range 循环区间宽度，与输入同单位
 */
inline void CircleAdd(float *origin, float delta, float range) {
  float out = *origin + delta;
  if (range > 0.0f) {
    if (out >= range)
      out -= range;
    else if (out < 0.0f)
      out += range;
  }
  *origin = out;
}

/**
 * @brief 对以 2π 为周期的角度取反
 *
 * @param origin 被操作的角度，单位：rad
 */
inline void CircleReverse(float *origin) { *origin = -(*origin) + M_2PI; }

/**
 * @brief 根据目标弹丸速度换算摩擦轮转速
 *
 * 裁判系统给定的速度直接映射到标定转速档位，其余速度按线速度与摩擦轮周长换算
 *
 * @param bullet_speed 弹丸速度，单位：m/s
 * @param fric_radius 摩擦轮半径，单位：m
 * @param is17mm 是否为 17 mm 弹丸
 * @return float 摩擦轮转速，单位：rpm
 */
inline float CalculateRpm(float bullet_speed, float fric_radius, bool is17mm) {
  if (bullet_speed == 0.0f) return 0.f;
  if (is17mm) {
    if (bullet_speed == 15.0f) return 4670.f;
    if (bullet_speed == 18.0f) return 5200.f;
    if (bullet_speed == 30.0f) return 7350.f;
  } else {
    if (bullet_speed == 10.0f) return 4450.f;
    if (bullet_speed == 16.0f) return 5800.f;
  }

  /* 非标定档位时按线速度与摩擦轮周长换算 */
  return 60.0f * (float)bullet_speed / (M_2PI * fric_radius);
}

// /**
//  * @brief 断言失败处理
//  *
//  * @param file 文件名
//  * @param line 行号
//  */
// void VerifyFailed(const char *file, uint32_t line) {
//   UNUSED(file);
//   UNUSED(line);
//   while (1) {
//     __NOP();
//   }
// }

/* USER FUNCTION BEGIN */

/* USER FUNCTION END */
