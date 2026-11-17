/*
 * 二阶低通与带阻滤波器接口
 *
 * 按采样频率与截止频率离线计算双二阶系数，逐拍更新两个延迟单元，运行中不做三角函数运算
 */

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ----------------------------------------------------------------- */
#include "user_math.h"

/* USER INCLUDE BEGIN */

/* USER INCLUDE END */

/* USER DEFINE BEGIN */

/* USER DEFINE END */

/* Exported types ----------------------------------------------------------- */
/* 二阶低通滤波器，双二阶直接 II 型 */
typedef struct {
  float cutoff_freq; /* 截止频率，单位：Hz */

  float a1; /* 反馈系数 a1 */
  float a2; /* 反馈系数 a2 */

  float b0; /* 前馈系数 b0 */
  float b1; /* 前馈系数 b1 */
  float b2; /* 前馈系数 b2 */

  float delay_element_1; /* 上一拍延迟单元 */
  float delay_element_2; /* 上上拍延迟单元 */

} LowPassFilter2p_t;

/* 二阶带阻滤波器，双二阶直接 II 型 */
typedef struct {
  float notch_freq; /* 中心频率，单位：Hz */
  float bandwidth;  /* 带宽，单位：Hz */

  float a1; /* 反馈系数 a1 */
  float a2; /* 反馈系数 a2 */

  float b0; /* 前馈系数 b0 */
  float b1; /* 前馈系数 b1 */
  float b2; /* 前馈系数 b2 */

  float delay_element_1; /* 上一拍延迟单元 */
  float delay_element_2; /* 上上拍延迟单元 */

} NotchFilter_t;

/* USER STRUCT BEGIN */

/* USER STRUCT END */

/**
 * @brief 初始化二阶低通滤波器系数
 *
 * @param f 滤波器实例
 * @param sample_freq 采样频率，单位：Hz
 * @param cutoff_freq 截止频率，单位：Hz
 */
void LowPassFilter2p_Init(LowPassFilter2p_t *f, float sample_freq,
                          float cutoff_freq);

/**
 * @brief 对单个采样值执行一次低通滤波
 *
 * @param f 滤波器实例
 * @param sample 输入采样值
 * @return float 滤波后的值
 */
float LowPassFilter2p_Apply(LowPassFilter2p_t *f, float sample);

/**
 * @brief 按当前采样值重置低通滤波器延迟单元
 *
 * @param f 滤波器实例
 * @param sample 用于重置的采样值
 * @return float 重置后对该采样值的滤波输出
 */
float LowPassFilter2p_Reset(LowPassFilter2p_t *f, float sample);

/**
 * @brief 初始化带阻滤波器系数
 *
 * @param f 滤波器实例
 * @param sample_freq 采样频率，单位：Hz
 * @param notch_freq 中心频率，单位：Hz
 * @param bandwidth 带宽，单位：Hz
 */
void NotchFilter_Init(NotchFilter_t *f, float sample_freq, float notch_freq,
                      float bandwidth);

/**
 * @brief 对单个采样值执行一次带阻滤波
 *
 * @param f 滤波器实例
 * @param sample 输入采样值
 * @return float 滤波后的值
 */
float NotchFilter_Apply(NotchFilter_t *f, float sample);

/**
 * @brief 按当前采样值重置带阻滤波器延迟单元
 *
 * @param f 滤波器实例
 * @param sample 用于重置的采样值
 * @return float 重置后对该采样值的滤波输出
 */
float NotchFilter_Reset(NotchFilter_t *f, float sample);

/* USER FUNCTION BEGIN */

/* USER FUNCTION END */

#ifdef __cplusplus
}
#endif
