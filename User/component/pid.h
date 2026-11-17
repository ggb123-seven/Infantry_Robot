/*
 * PID 控制器接口
 *
 * 移植自 PX4 Firmware pid，支持并行式与标准式整定、D 项低通和循环误差处理
 * 上游：https://github.com/PX4/Firmware/blob/master/src/lib/pid/pid.h
 */

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ----------------------------------------------------------------- */
#include <stdint.h>

#include "filter.h"
#include "user_math.h"

/* USER INCLUDE BEGIN */

/* USER INCLUDE END */

/* USER DEFINE BEGIN */

/* USER DEFINE END */

/* Exported types ----------------------------------------------------------- */
/* PID 微分项计算方式：CALC_D 忽略 fb_dot，SET_D 直接使用调用方微分值 */
typedef enum {
  KPID_MODE_NO_D = 0, /* 无微分项，即 PI 控制 */
  KPID_MODE_CALC_D,   /* 由反馈值离散微分 */
  KPID_MODE_SET_D     /* 使用调用方微分值 */
} KPID_Mode_t;

/* PID 参数，k 取 1 为并行式，p 取 1 为标准式 */
typedef struct {
  float k;             /* 控制器增益 */
  float p;             /* 比例项增益 */
  float i;             /* 积分项增益 */
  float d;             /* 微分项增益 */
  float i_limit;       /* 积分项限幅 */
  float out_limit;     /* 输出限幅 */
  float d_cutoff_freq; /* 截止频率，单位：Hz */
  float range;         /* 循环误差范围 */
} KPID_Params_t;

/* PID 主结构体 */
typedef struct {
  KPID_Mode_t mode;           /* 微分项计算方式 */
  const KPID_Params_t *param; /* 指向控制参数 */

  float dt_min; /* 最短调用间隔，单位：s */
  float i;      /* 积分累计值 */

  struct {
    float err;  /* 上次误差 */
    float k_fb; /* 上次 D 项输入 */
    float out;  /* 上次输出 */
  } last;

  LowPassFilter2p_t dfilter; /* D 项低通滤波器 */
} KPID_t;

/* Exported functions ------------------------------------------------------- */
/**
 * @brief 初始化 PID，校验参数并复位内部状态
 *
 * @param pid PID 实例
 * @param mode 微分项计算方式
 * @param sample_freq 采样频率，单位：Hz
 * @param param PID 参数，须在实例生命周期内有效
 * @return int8_t 0 表示成功，负值表示失败
 */
int8_t PID_Init(KPID_t *pid, KPID_Mode_t mode, float sample_freq,
                const KPID_Params_t *param);

/**
 * @brief 执行一次 PID 计算
 *
 * @param pid PID 实例
 * @param sp 设定值
 * @param fb 反馈值
 * @param fb_dot 反馈值微分
 * @param dt 本次调用间隔，单位：s
 * @return float 计算的输出
 */
float PID_Calc(KPID_t *pid, float sp, float fb, float fb_dot, float dt);

/**
 * @brief 只重置积分项
 *
 * @param pid PID 实例
 * @return int8_t 0 表示成功，负值表示失败
 */
int8_t PID_ResetIntegral(KPID_t *pid);

/**
 * @brief 重置 PID 全部内部状态
 *
 * @param pid PID 实例
 * @return int8_t 0 表示成功，负值表示失败
 */
int8_t PID_Reset(KPID_t *pid);

/* USER FUNCTION BEGIN */

/* USER FUNCTION END */

#ifdef __cplusplus
}
#endif
