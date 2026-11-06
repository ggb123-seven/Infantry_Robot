/*
 * 参考【RM2026 复旦大学 星云EGA战队】云台滑模控制器教学与开源
 * 来源：https://github.com/xinruilee04/smc_controller.git
 */
#include "slidingmodec.h"

#include <math.h>
#include <string.h>

/**
 * @brief 将归一化滑模面限制在连续边界层内
 *
 * @param[in] value 已按边界宽度归一化的滑模面
 * @return 范围为负一到正一的饱和值
 */
static float SMC_Sat(float value)
{
    return fminf(1.0f, fmaxf(-1.0f, value));
}

bool SMC_IsConfigValid(const SMC_t *smc)
{
    // 在线调参可写入零分母或非有限值，统一在调用边界拒绝，计算函数不重复校验
    return isfinite(smc->C) && smc->C > 0.0f && isfinite(smc->K) && smc->K >= 0.0f &&
           isfinite(smc->error_eps) && smc->error_eps > 0.0f &&
           isfinite(smc->q) && isfinite(smc->p) && smc->q > 0.0f && smc->p >= smc->q &&
           smc->q / smc->p > 0.0f && isfinite(smc->speed_max_deg_s) && smc->speed_max_deg_s > 0.0f &&
           isfinite(smc->surface_eps_deg_s) && smc->surface_eps_deg_s > 0.0f;
}

void SMC_Init(SMC_t *smc, const SMC_t *config, float ref)
{
    // 调用方保证实例有效及参数已经校验，初始化只建立参数与历史
    memset(smc, 0, sizeof(*smc));
    smc->C = config->C;
    smc->K = config->K;
    smc->target = ref;
    smc->target_last = ref;
    smc->error_eps = config->error_eps;
    smc->q = config->q;
    smc->p = config->p;
    smc->speed_max_deg_s = config->speed_max_deg_s;
    smc->surface_eps_deg_s = config->surface_eps_deg_s;
}

bool SMC_Tick(SMC_t *smc, float angle_now, float angle_vel)
{
    // 滑模面使用本周期参考速度，确保角度与速度误差属于同一轨迹
    smc->angle = angle_now;
    smc->ang_vel = angle_vel;
    smc->error_last = smc->error;
    smc->error = angle_now - smc->target;
    const float error_dot = angle_vel - smc->target_dot;
    const float ratio = smc->q / smc->p;
    const float normalized_error = fabsf(smc->error) / smc->error_eps;
    float surface_error;

    // 近零三次多项式连接连续幂函数，保持角误差符号与量纲
    if (normalized_error < 1.0f)
    {
        const float t = normalized_error;
        surface_error = smc->error * (1.0f + (ratio - 1.0f) * t * (t - 1.0f));
    }
    else
    {
        const float power = powf(normalized_error, ratio);
        surface_error = copysignf(smc->error_eps * power, smc->error);
    }

    // 实际角度减参考角度形成滑模面，趋近项直接修正速度目标
    smc->s = error_dot + smc->C * surface_error;
    const float speed_target_deg_s = smc->target_dot - smc->C * surface_error -
                                     smc->K * SMC_Sat(smc->s / smc->surface_eps_deg_s);

    // 浮点运算可能溢出，速度目标限幅前统一检查本链出口
    if (!isfinite(speed_target_deg_s))
    {
        return false;
    }
    smc->speed_target_deg_s = fminf(smc->speed_max_deg_s, fmaxf(-smc->speed_max_deg_s, speed_target_deg_s));
    smc->target_last = smc->target;
    return true;
}

void SMC_AdvanceReference(SMC_t *smc, float target_angle, float target_speed, float max_speed,
                          float max_acceleration, float reference_gain, float period_s)
{
    // 命令已有解析速度时直接使用，位置偏差仅用于追踪受限参考，避免重复微分与前馈
    const float feedforward = fminf(max_speed, fmaxf(-max_speed, target_speed));
    const float predicted = smc->target + feedforward * period_s;
    const float requested = feedforward + reference_gain * (target_angle - predicted);
    const float speed_goal = fminf(max_speed, fmaxf(-max_speed, requested));
    const float previous_speed = smc->target_dot;
    const float step = max_acceleration * period_s;
    const float slewed_speed = fminf(previous_speed + step, fmaxf(previous_speed - step, speed_goal));
    // 在线收紧速度上限时立即服从新边界，实际加速度保留该次变化供控制器前馈
    const float reference_speed = fminf(max_speed, fmaxf(-max_speed, slewed_speed));

    // 梯形积分使本周期位置、速度和加速度来自同一受限轨迹
    smc->target += 0.5f * (previous_speed + reference_speed) * period_s;
    smc->target_dot = reference_speed;
    smc->target_ddot = (reference_speed - previous_speed) / period_s;
}
