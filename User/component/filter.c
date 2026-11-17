/*
 * 二阶低通与带阻滤波器实现
 *
 * 系数在初始化时按采样频率与截止频率一次算出，Apply 只做两次乘加和延迟单元移位
 */

/* Includes ----------------------------------------------------------------- */
#include "filter.h"

#include "user_math.h"

/* Exported functions ------------------------------------------------------- */
/**
 * @brief 初始化二阶低通滤波器系数
 *
 * @param f 滤波器实例
 * @param sample_freq 采样频率，单位：Hz
 * @param cutoff_freq 截止频率，单位：Hz
 */
void LowPassFilter2p_Init(LowPassFilter2p_t *f, float sample_freq,
                          float cutoff_freq) {
  if (f == NULL) return;

  f->cutoff_freq = cutoff_freq;

  f->delay_element_1 = 0.0f;
  f->delay_element_2 = 0.0f;

  if (f->cutoff_freq <= 0.0f) {
    /* 截止频率非正，退化为直通 */
    f->b0 = 1.0f;
    f->b1 = 0.0f;
    f->b2 = 0.0f;

    f->a1 = 0.0f;
    f->a2 = 0.0f;

    return;
  }
  const float fr = sample_freq / f->cutoff_freq;
  const float ohm = tanf(M_PI / fr);
  const float c = 1.0f + 2.0f * cosf(M_PI / 4.0f) * ohm + ohm * ohm;

  f->b0 = ohm * ohm / c;
  f->b1 = 2.0f * f->b0;
  f->b2 = f->b0;

  f->a1 = 2.0f * (ohm * ohm - 1.0f) / c;
  f->a2 = (1.0f - 2.0f * cosf(M_PI / 4.0f) * ohm + ohm * ohm) / c;
}

/**
 * @brief 对单个采样值执行一次低通滤波
 *
 * @param f 滤波器实例
 * @param sample 输入采样值
 * @return float 滤波后的值
 */
float LowPassFilter2p_Apply(LowPassFilter2p_t *f, float sample) {
  if (f == NULL) return 0.0f;

  /* 按直接 II 型计算本拍中间状态与输出 */
  float delay_element_0 =
      sample - f->delay_element_1 * f->a1 - f->delay_element_2 * f->a2;

  if (isinf(delay_element_0)) {
    /* 异常值不进入反馈链，用原始采样代替 */
    delay_element_0 = sample;
  }

  const float output = delay_element_0 * f->b0 + f->delay_element_1 * f->b1 +
                       f->delay_element_2 * f->b2;

  f->delay_element_2 = f->delay_element_1;
  f->delay_element_1 = delay_element_0;

  /* 输出由输入与系数决定，此处不再限幅 */
  return output;
}

/**
 * @brief 按当前采样值重置低通滤波器延迟单元
 *
 * @param f 滤波器实例
 * @param sample 用于重置的采样值
 * @return float 重置后对该采样值的滤波输出
 */
float LowPassFilter2p_Reset(LowPassFilter2p_t *f, float sample) {
  if (f == NULL) return 0.0f;

  const float dval = sample / (f->b0 + f->b1 + f->b2);

  if (isfinite(dval)) {
    f->delay_element_1 = dval;
    f->delay_element_2 = dval;

  } else {
    f->delay_element_1 = sample;
    f->delay_element_2 = sample;
  }

  return LowPassFilter2p_Apply(f, sample);
}

/**
 * @brief 初始化带阻滤波器系数
 *
 * @param f 滤波器实例
 * @param sample_freq 采样频率，单位：Hz
 * @param notch_freq 中心频率，单位：Hz
 * @param bandwidth 带宽，单位：Hz
 */
void NotchFilter_Init(NotchFilter_t *f, float sample_freq, float notch_freq,
                      float bandwidth) {
  if (f == NULL) return;

  f->notch_freq = notch_freq;
  f->bandwidth = bandwidth;

  f->delay_element_1 = 0.0f;
  f->delay_element_2 = 0.0f;

  if (notch_freq <= 0.0f) {
    /* 中心频率非正，退化为直通 */
    f->b0 = 1.0f;
    f->b1 = 0.0f;
    f->b2 = 0.0f;

    f->a1 = 0.0f;
    f->a2 = 0.0f;

    return;
  }

  const float alpha = tanf(M_PI * bandwidth / sample_freq);
  const float beta = -cosf(M_2PI * notch_freq / sample_freq);
  const float a0_inv = 1.0f / (alpha + 1.0f);

  f->b0 = a0_inv;
  f->b1 = 2.0f * beta * a0_inv;
  f->b2 = a0_inv;

  f->a1 = f->b1;
  f->a2 = (1.0f - alpha) * a0_inv;
}

/**
 * @brief 对单个采样值执行一次带阻滤波
 *
 * @param f 滤波器实例
 * @param sample 输入采样值
 * @return float 滤波后的值
 */
inline float NotchFilter_Apply(NotchFilter_t *f, float sample) {
  if (f == NULL) return 0.0f;

  /* 直接 II 型实现，延迟单元保存中间状态 */
  const float delay_element_0 =
      sample - f->delay_element_1 * f->a1 - f->delay_element_2 * f->a2;
  const float output = delay_element_0 * f->b0 + f->delay_element_1 * f->b1 +
                       f->delay_element_2 * f->b2;

  f->delay_element_2 = f->delay_element_1;
  f->delay_element_1 = delay_element_0;

  return output;
}

/**
 * @brief 按当前采样值重置带阻滤波器延迟单元
 *
 * @param f 滤波器实例
 * @param sample 用于重置的采样值
 * @return float 重置后对该采样值的滤波输出
 */
float NotchFilter_Reset(NotchFilter_t *f, float sample) {
  if (f == NULL) return 0.0f;

  float dval = sample;

  if (fabsf(f->b0 + f->b1 + f->b2) > FLT_EPSILON) {
    dval = dval / (f->b0 + f->b1 + f->b2);
  }

  f->delay_element_1 = dval;
  f->delay_element_2 = dval;

  return NotchFilter_Apply(f, sample);
}
