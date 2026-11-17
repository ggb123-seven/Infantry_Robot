/*
 * PID 控制器实现
 *
 * 移植自 PX4 Firmware pid，k/p/i/d 可组合作并行式或标准式，D 项先经二阶低通
 * 参考：https://github.com/PX4/Firmware/blob/master/src/lib/pid/pid.cpp
 */

/* Includes ----------------------------------------------------------------- */
#include "pid.h"

/* Private define ----------------------------------------------------------- */
/* 积分增益与输出限幅低于该阈值时视为未启用 */
#define SIGMA 0.000001f

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
                const KPID_Params_t *param) {
  if (pid == NULL) return -1;

  if (!isfinite(param->p)) return -1;
  if (!isfinite(param->i)) return -1;
  if (!isfinite(param->d)) return -1;
  if (!isfinite(param->i_limit)) return -1;
  if (!isfinite(param->out_limit)) return -1;
  pid->param = param;

  float dt_min = 1.0f / sample_freq;
  if (isfinite(dt_min))
    pid->dt_min = dt_min;
  else
    return -1;

  LowPassFilter2p_Init(&(pid->dfilter), sample_freq, pid->param->d_cutoff_freq);

  pid->mode = mode;
  PID_Reset(pid);
  return 0;
}

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
float PID_Calc(KPID_t *pid, float sp, float fb, float fb_dot, float dt) {
  if (!isfinite(sp) || !isfinite(fb) || !isfinite(fb_dot) || !isfinite(dt)) {
    return pid->last.out;
  }

  /* 计算循环误差 */
  const float err = CircleError(sp, fb, pid->param->range);

  /* 计算 P 项 */
  const float k_err = err * pid->param->k;

  /* 计算 D 项 */
  const float k_fb = pid->param->k * fb;
  const float filtered_k_fb = LowPassFilter2p_Apply(&(pid->dfilter), k_fb);

  float d;
  switch (pid->mode) {
    case KPID_MODE_CALC_D:
      /* 用反馈值微分代替误差微分，避免设定值突变引入微分冲击 */
      d = (filtered_k_fb - pid->last.k_fb) / fmaxf(dt, pid->dt_min);
      break;

    case KPID_MODE_SET_D:
      d = fb_dot;
      break;

    case KPID_MODE_NO_D:
      d = 0.0f;
      break;
  }
  pid->last.err = err;
  pid->last.k_fb = filtered_k_fb;

  if (!isfinite(d)) d = 0.0f;

  /* 合成 PD 输出 */
  float output = (k_err * pid->param->p) - (d * pid->param->d);

  /* 计算 I 项 */
  const float i = pid->i + (k_err * dt);
  const float i_out = i * pid->param->i;

  if (pid->param->i > SIGMA) {
    /* 输出与积分均在限幅内才接受新积分，实现抗饱和 */
    if (isfinite(i)) {
      if ((fabsf(output + i_out) <= pid->param->out_limit) &&
          (fabsf(i) <= pid->param->i_limit)) {
        pid->i = i;
      }
    }
  }

  /* 合成 PID 输出 */
  output += i_out;

  /* 输出限幅 */
  if (isfinite(output)) {
    if (pid->param->out_limit > SIGMA) {
      output = AbsClip(output, pid->param->out_limit);
    }
    pid->last.out = output;
  }
  return pid->last.out;
}

/**
 * @brief 只重置积分项
 *
 * @param pid PID 实例
 * @return int8_t 0 表示成功，负值表示失败
 */
int8_t PID_ResetIntegral(KPID_t *pid) {
  if (pid == NULL) return -1;

  pid->i = 0.0f;

  return 0;
}

/**
 * @brief 重置 PID 全部内部状态
 *
 * @param pid PID 实例
 * @return int8_t 0 表示成功，负值表示失败
 */
int8_t PID_Reset(KPID_t *pid) {
  if (pid == NULL) return -1;

  pid->i = 0.0f;
  pid->last.err = 0.0f;
  pid->last.k_fb = 0.0f;
  pid->last.out = 0.0f;
  LowPassFilter2p_Reset(&(pid->dfilter), 0.0f);

  return 0;
}
