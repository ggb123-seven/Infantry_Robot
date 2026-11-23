/*
 * 参考【RM2026 复旦大学 星云EGA战队】云台滑模控制器教学与开源
 * 来源：https://github.com/xinruilee04/smc_controller.git
 * 本工程改为线性滑模面并直接输出电流，参数含义见本文件开头说明
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
           isfinite(smc->epsilon_deg_s2) && smc->epsilon_deg_s2 >= 0.0f &&
           isfinite(smc->J) && smc->J > 0.0f &&
           isfinite(smc->surface_eps_deg_s) && smc->surface_eps_deg_s > 0.0f;
}

void SMC_Init(SMC_t *smc, const SMC_t *config, float ref)
{
    // 调用方保证实例有效及参数已经校验，初始化只建立参数与历史
    memset(smc, 0, sizeof(*smc));
    smc->C = config->C;
    smc->K = config->K;
    smc->epsilon_deg_s2 = config->epsilon_deg_s2;
    smc->J = config->J;
    smc->surface_eps_deg_s = config->surface_eps_deg_s;
    smc->target = ref;
    smc->target_last = ref;
}

bool SMC_Tick(SMC_t *smc, float angle_now, float angle_vel)
{
    // 误差与滑模面统一由模块坐标给出，参考转速和参考加速度来自同一条受限轨迹
    smc->angle = angle_now;
    smc->ang_vel = angle_vel;
    smc->error_last = smc->error;
    smc->error = angle_now - smc->target;
    const float error_dot = angle_vel - smc->target_dot;
    smc->s = error_dot + smc->C * smc->error;

    // 线性趋近项直接生成角加速度，再按等效惯量换算为候选电流
    const float surface = SMC_Sat(smc->s / smc->surface_eps_deg_s);
    const float u = smc->J * (smc->target_ddot - smc->C * error_dot - smc->K * smc->s -
                              smc->epsilon_deg_s2 * surface);

    // 浮点运算可能溢出，候选电流在写入观测前统一检查本链出口
    if (!isfinite(u))
    {
        return false;
    }
    smc->target_last = smc->target;
    smc->u = u;
    return true;
}

void SMC_AdvanceReference(SMC_t *smc, float target_angle, float target_speed, float max_speed,
                          float max_acceleration, float reference_gain, float period_s)
{
    // 命令已有解析速度时直接使用，位置偏差仅用于追踪受限参考，避免重复微分与前馈
    const float feedforward = fminf(max_speed, fmaxf(-max_speed, target_speed));
    const float predicted = smc->target + feedforward * period_s;
    const float error = target_angle - predicted;
    // 只按剩余距离限制误差修正项，前馈速度不受影响，避免跟踪时把参考速度压成零
    const float correction_raw = reference_gain * error;
    const float correction_limit = sqrtf(2.0f * max_acceleration * fabsf(error));
    const float correction = fminf(correction_limit, fabsf(correction_raw)) *
                             (correction_raw < 0.0f ? -1.0f : 1.0f);
    const float requested = feedforward + correction;
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
