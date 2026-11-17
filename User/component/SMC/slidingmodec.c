/*
 * 云台滑模控制器实现
 *
 * 终端滑模面叠加饱和趋近律，近零误差时切换线性滑模面，输出按 u_max 限幅
 * 参考【RM2026 复旦大学 星云EGA战队】云台滑模控制器开源，来源 https://github.com/xinruilee04/smc_controller.git
 */

/* Includes ----------------------------------------------------------------- */
#include "slidingmodec.h"

#include <math.h>
#include <string.h>

/* Private function --------------------------------------------------------- */
/**
 * @brief 将滑模面数值饱和限制在单位边界层内
 *
 * @param y 待限制的滑模面数值
 * @return float 位于负一到正一之间的饱和值
 */
static float SMC_Sat(float y)
{
    if (fabsf(y) <= 1.0f)
    {
        return y;
    }
    return y > 0.0f ? 1.0f : -1.0f;
}

/* Exported functions ------------------------------------------------------- */
void SMC_Init(SMC_t *smc, float C, float K, float ref, float error_eps,
              float q, float p, float u_max, float J, float epsilon)
{
    if (smc == NULL)
    {
        return;
    }

    /* 清除历史状态，使局部实例与静态实例的初值一致 */
    memset(smc, 0, sizeof(*smc));
    smc->C = C;
    smc->K = K;
    smc->target = ref;
    smc->error_eps = error_eps;
    smc->q = q;
    smc->p = p;
    smc->u_max = u_max;
    smc->J = J;
    smc->epsilon = epsilon;
}

void SMC_Tick(SMC_t *smc, float angle_now, float angle_vel)
{
    float e_qp;
    float qp;
    float error_dot;

    if (smc == NULL)
    {
        return;
    }

    /* 沿用原算法的目标逐周期差分与反馈计算顺序 */
    smc->angle = angle_now;
    smc->ang_vel = angle_vel;
    qp = smc->q / smc->p;
    smc->error = smc->angle - smc->target;
    error_dot = smc->ang_vel - smc->target_dot;
    smc->target_ddot = (smc->target - smc->target_last) - smc->target_dot;
    smc->target_dot = smc->target - smc->target_last;

    /* 误差进入死区时零输出并提前返回 */
    if (fabsf(smc->error) < smc->error_eps)
    {
        smc->u = 0.0f;
        return;
    }

    /* 用浮点绝对值与幂函数保持原 C++ 浮点运算语义 */
    if (smc->error < 0.0f)
    {
        e_qp = -powf(fabsf(smc->error), qp);
    }
    else
    {
        e_qp = powf(fabsf(smc->error), qp);
    }

    smc->s = error_dot + smc->C * e_qp;
    smc->ds = -smc->epsilon * SMC_Sat(smc->s) - smc->K * smc->s;
    smc->u = smc->J * (smc->target_ddot + smc->ds - smc->C * qp * error_dot * e_qp / smc->error);

    /* 误差小于一度时改用线性滑模面，减小近零误差的非线性影响 */
    if (fabsf(smc->error) < 1.0f)
    {
        smc->error = smc->angle - smc->target;
        smc->s = smc->C * smc->error + error_dot;
        smc->u = smc->J * (smc->target_ddot - smc->C * error_dot - smc->epsilon * SMC_Sat(smc->s) - smc->K * smc->s);
    }

    /* 输出限幅并保存下一周期需要的目标历史值 */
    if (smc->u > smc->u_max)
    {
        smc->u = smc->u_max;
    }
    if (smc->u < -smc->u_max)
    {
        smc->u = -smc->u_max;
    }
    smc->target_last = smc->target;
}
