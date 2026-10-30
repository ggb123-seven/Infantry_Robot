/*
 * 参考【RM2026 复旦大学 星云EGA战队】云台滑模控制器教学与开源
 * 来源：https://github.com/xinruilee04/smc_controller.git
 */
#include "slidingmodec.h"

#include <math.h>
#include <string.h>

/**
 * @brief 将滑模变量限制在固定单位边界层，避免零点硬切换
 *
 * @param[in] y 滑模变量，单位度每秒
 * @return 范围为负一到正一的饱和值
 */
static float SMC_Sat(float y)
{
    return fminf(1.0f, fmaxf(-1.0f, y));
}

bool SMC_IsConfigValid(const SMC_t *smc)
{
    // 在线调参可写入零分母或非有限值，统一在调用边界拒绝，计算函数不重复校验
    return isfinite(smc->C) && smc->C > 0.0f && isfinite(smc->K) && smc->K >= 0.0f &&
           isfinite(smc->error_eps) && smc->error_eps > 0.0f &&
           isfinite(smc->q) && isfinite(smc->p) && smc->q > 0.0f && smc->p >= smc->q &&
           smc->q / smc->p > 0.0f && isfinite(smc->u_max) && smc->u_max > 0.0f &&
           isfinite(smc->J) && smc->J > 0.0f && isfinite(smc->epsilon) && smc->epsilon >= 0.0f;
}

void SMC_Init(SMC_t *smc, float C, float K, float ref, float error_eps,
              float q, float p, float u_max, float J, float epsilon)
{
    // 调用方保证实例有效及参数已经校验，初始化只建立参数与历史
    memset(smc, 0, sizeof(*smc));
    smc->C = C;
    smc->K = K;
    smc->target = ref;
    smc->target_last = ref;
    smc->error_eps = error_eps;
    smc->q = q;
    smc->p = p;
    smc->u_max = u_max;
    smc->J = J;
    smc->epsilon = epsilon;
}

bool SMC_Tick(SMC_t *smc, float angle_now, float angle_vel, float target_speed, float target_acceleration)
{
    // 参考导数由模块按实际周期生成，先使用本周期参考，避免旧导数滞后一拍
    smc->angle = angle_now;
    smc->ang_vel = angle_vel;
    smc->error_last = smc->error;
    smc->error = angle_now - smc->target;
    smc->target_dot = target_speed;
    smc->target_ddot = target_acceleration;
    const float error_dot = angle_vel - target_speed;
    const float ratio = smc->q / smc->p;
    const float normalized_error = fabsf(smc->error) / smc->error_eps;
    float surface_error;
    float surface_derivative;

    // 近零三次多项式在过渡点同时匹配幂函数的值和一阶导数，不以角度死区撤掉制动
    if (normalized_error < 1.0f)
    {
        const float t = normalized_error;
        surface_error = smc->error * (1.0f + (ratio - 1.0f) * t * (t - 1.0f));
        surface_derivative = 1.0f + (ratio - 1.0f) * (3.0f * t * t - 2.0f * t);
    }
    else
    {
        const float power = powf(normalized_error, ratio);
        surface_error = copysignf(smc->error_eps * power, smc->error);
        surface_derivative = ratio * power / normalized_error;
    }

    // 实际值减参考值配合负反馈趋近律，等效增益将角加速度换算为安培
    smc->s = error_dot + smc->C * surface_error;
    smc->ds = -smc->K * smc->s - smc->epsilon * SMC_Sat(smc->s);
    const float current_a = smc->J * (target_acceleration + smc->ds -
                                     smc->C * surface_derivative * error_dot);

    // 浮点运算可能溢出，限幅前检查计算链出口，避免饱和函数吞掉非数值
    if (!isfinite(current_a))
    {
        return false;
    }
    smc->u = fminf(smc->u_max, fmaxf(-smc->u_max, current_a));
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
    const float reference_speed = fminf(previous_speed + step, fmaxf(previous_speed - step, speed_goal));

    // 梯形积分使本周期位置、速度和加速度来自同一受限轨迹
    smc->target += 0.5f * (previous_speed + reference_speed) * period_s;
    smc->target_dot = reference_speed;
    smc->target_ddot = (reference_speed - previous_speed) / period_s;
}
