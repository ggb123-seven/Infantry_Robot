/*
 * 通用数学工具接口
 *
 * 提供快速倒数平方根、限幅、循环量运算与摩擦轮转速换算，供控制层直接调用
 */

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ----------------------------------------------------------------- */
#include <float.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>

/* USER INCLUDE BEGIN */

/* USER INCLUDE END */

/* Exported constants ------------------------------------------------------- */
#define M_DEG2RAD_MULT (0.01745329251f) /* 角度转弧度系数 */
#define M_RAD2DEG_MULT (57.2957795131f) /* 弧度转角度系数 */

/* 补齐工具链缺失的圆周率常量 */
#ifndef M_PI_2
#define M_PI_2 1.57079632679f /* 圆周率一半 */
#endif

#ifndef M_PI
#define M_PI 3.14159265358979323846f /* 圆周率 */
#endif

#ifndef M_2PI
#define M_2PI 6.28318530717958647692f /* 二倍圆周率 */
#endif

#ifndef __packed
  #define __packed __attribute__((__packed__))
#endif /* __packed */

/* max/min 用 GNU 语句表达式，保证参数只求值一次并保留原类型 */
#define max(a, b)           \
  ({                        \
    __typeof__(a) _a = (a); \
    __typeof__(b) _b = (b); \
    _a > _b ? _a : _b;      \
  })

#define min(a, b)           \
  ({                        \
    __typeof__(a) _a = (a); \
    __typeof__(b) _b = (b); \
    _a < _b ? _a : _b;      \
  })

/* USER DEFINE BEGIN */

/* USER DEFINE END */

/* Exported types ----------------------------------------------------------- */
/* 底盘运动向量，各分量已完成归一化 */
typedef struct {
  float vx; /* 前后平移，前进为正 */
  float vy; /* 左右平移，左移为正 */
  float wz; /* 转动，俯视逆时针为正 */
} MoveVector_t;

/* USER STRUCT BEGIN */

/* USER STRUCT END */

/* Exported functions ------------------------------------------------------- */
/**
 * @brief 快速计算倒数平方根
 *
 * @param x 输入值
 * @return float x 的倒数平方根，精度约千分之一
 */
float InvSqrt(float x);

/**
 * @brief 按对称限幅截断数值
 *
 * @param in 输入值
 * @param limit 限幅绝对值
 * @return float 截断后的值
 */
float AbsClip(float in, float limit);

/**
 * @brief 按单周期最大步长逼近目标值
 *
 * @param current 当前值
 * @param target 目标值
 * @param max_step 单周期允许的最大变化量
 * @return float 更新后的值
 */
float RampTo(float current, float target, float max_step);

/**
 * @brief 取浮点数的绝对值
 *
 * @param in 输入值
 * @return float 绝对值
 */
float fAbs(float in);

/**
 * @brief 将数值限制在指定区间内
 *
 * @param origin 待限制的数值
 * @param min 区间下限
 * @param max 区间上限
 */
void Clip(float *origin, float min, float max);

/**
 * @brief 取数值的符号
 *
 * @param in 输入值
 * @return float 非负返回 1，负值返回 0
 */
float Sign(float in);

/**
 * @brief 将运动向量置零
 *
 * @param mv 待清零的运动向量
 */
void ResetMoveVector(MoveVector_t *mv);

/**
 * @brief 计算循环量误差
 *
 * 设定值与反馈值位于同一循环区间时按最短路径取差，range 为该区间宽度，
 * 取 0 时退化为线性误差
 *
 * @param sp 设定值
 * @param fb 反馈值
 * @param range 循环区间宽度，与输入同单位
 * @return float 误差，落在区间宽度的一半以内
 */
float CircleError(float sp, float fb, float range);

/**
 * @brief 在循环区间内累加增量
 *
 * @param origin 被操作的循环量
 * @param delta 增量
 * @param range 循环区间宽度，与输入同单位
 */
void CircleAdd(float *origin, float delta, float range);

/**
 * @brief 对以 2π 为周期的角度取反
 *
 * @param origin 被操作的角度，单位：rad
 */
void CircleReverse(float *origin);

/**
 * @brief 根据目标弹丸速度换算摩擦轮转速
 *
 * 裁判系统给定的速度直接映射到标定转速档位，其余情况按线速度与轮径换算
 *
 * @param bullet_speed 弹丸速度，单位：m/s
 * @param fric_radius 摩擦轮半径，单位：m
 * @param is17mm 是否为 17 mm 弹丸
 * @return float 摩擦轮转速，单位：rpm
 */
float CalculateRpm(float bullet_speed, float fric_radius, bool is17mm);

#ifdef __cplusplus
}
#endif

/* 调试断言：DEBUG 下表达式为假时进入 VerifyFailed，未定义 DEBUG 时直接忽略 */
#ifdef DEBUG

/**
 * @brief 表达式为假时进入断言失败处理
 */
#define ASSERT(expr)                    \
  do {                                  \
    if (!(expr)) {                      \
      VerifyFailed(__FILE__, __LINE__); \
    }                                   \
  } while (0)
#else

/**
 * @brief 未定义 DEBUG，表达式不运行，断言被忽略
 */
#define ASSERT(expr) ((void)(0))
#endif

#ifdef DEBUG

/**
 * @brief 表达式为假时进入断言失败处理
 */
#define VERIFY(expr)                    \
  do {                                  \
    if (!(expr)) {                      \
      VerifyFailed(__FILE__, __LINE__); \
    }                                   \
  } while (0)
#else

/**
 * @brief 表达式总会运行，只是忽略结果
 */
#define VERIFY(expr) ((void)(expr))
#endif

// /**
//  * @brief 断言失败处理
//  *
//  * @param file 文件名
//  * @param line 行号
//  */
// void VerifyFailed(const char *file, uint32_t line);

/* USER FUNCTION BEGIN */

/* USER FUNCTION END */
