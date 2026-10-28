#include "module/gimbal_governor.h"

#include <math.h>
#include <stddef.h>

/*
 * 悬空低电流调试的初始包络与反算参数，均须实测确认：
 * - SOFT、HARD、BRAKE：残差阈值 8、12、16 rad/s，依次收紧同向电流、撤销同向电流、达到制动上限
 * - BRAKE_CURRENT_A：反向制动上限 0.3 A，同时不超过轴电流上限，更强的已有反向电流保持不变
 * - TRACKING_TIME_S：积分反算时间常数 0.05 s，每周期跟踪比例不超过 1
 * - INTEGRAL_SLEW_A_S：反算导致的积分电流贡献变化不超过 2 A/s，避免小积分增益放大单步修正
 * - MIN_INTEGRAL_GAIN：沿用通用 PID 的积分启用阈值 0.000001，避免除以接近零的增益
 * 残差为编码器相对转速减上部惯性 yaw 转速加独立底盘实测转速，禁止使用同源估计底盘转速抵消残差
 * 本轮独立底盘项由调用方传 0，小陀螺与底盘跟随兼容尚未完成，倾斜及不同步采样误差仍需实测
 */
#define GIMBAL_GOVERNOR_SOFT_RAD_S (8.0f)
#define GIMBAL_GOVERNOR_HARD_RAD_S (12.0f)
#define GIMBAL_GOVERNOR_BRAKE_RAD_S (16.0f)
#define GIMBAL_GOVERNOR_BRAKE_CURRENT_A (0.3f)
#define GIMBAL_GOVERNOR_TRACKING_TIME_S (0.05f)
#define GIMBAL_GOVERNOR_INTEGRAL_SLEW_A_S (2.0f)
#define GIMBAL_GOVERNOR_MIN_INTEGRAL_GAIN (0.000001f)

/*
 * 异常制动与自动恢复试验起点，尚未经过真实机械验证：
 * - DETECT、IMU_STILL：残差超过 1 rad/s 且上部转速低于 0.3 rad/s 才累计同向异常
 * - DETECT_TIME：同向异常持续 0.15 s 后锁存制动，条件中断或换向重新计时
 * - STOP、RESTART、STOP_TIME：低于 0.2 rad/s 持续 0.1 s 进入受限姿态保持，高于 0.4 rad/s 重新制动
 * - DAMPING：阻尼系数 0.15 A/(rad/s)，制动上限沿用 0.3 A
 * - RECOVERY_TIME：人工起控及未锁存的失联恢复后，用 0.2 s 渐进开放速度和电流权限
 * - HOLD_SPEED、HOLD_CURRENT：受限保持的速度权限 0.5 rad/s、电流上限 0.08 A
 * - MATCH、MATCH_TIME：残差小于 0.15 rad/s 且同向运动差小于上部转速的 25%，持续 0.2 s 后允许恢复
 * - MOVING、SETTLED_ANGLE、SETTLED_TIME：运动确认下限 0.1 rad/s，误差小于 0.5 度且静止持续 0.3 s 也可恢复
 * - RESTORE_TIME：自动恢复持续 0.5 s，期间保留 0.2 至 0.4 rad/s 的低速电流包络
 * - CURRENT_SLEW：电流变化上限 2 A/s，撤销同向推动和电流包络优先于连续性
 */
#define GIMBAL_GOVERNOR_DETECT_RAD_S (1.0f)
#define GIMBAL_GOVERNOR_IMU_STILL_RAD_S (0.3f)
#define GIMBAL_GOVERNOR_DETECT_TIME_S (0.15f)
#define GIMBAL_GOVERNOR_STOP_RAD_S (0.2f)
#define GIMBAL_GOVERNOR_RESTART_RAD_S (0.4f)
#define GIMBAL_GOVERNOR_STOP_TIME_S (0.1f)
#define GIMBAL_GOVERNOR_DAMPING (0.15f)
#define GIMBAL_GOVERNOR_RECOVERY_TIME_S (0.2f)
#define GIMBAL_GOVERNOR_CURRENT_SLEW_A_S (2.0f)
#define GIMBAL_GOVERNOR_HOLD_SPEED_RAD_S (0.5f)
#define GIMBAL_GOVERNOR_HOLD_CURRENT_A (0.08f)
#define GIMBAL_GOVERNOR_MATCH_RAD_S (0.15f)
#define GIMBAL_GOVERNOR_MATCH_RATIO (0.25f)
#define GIMBAL_GOVERNOR_MATCH_TIME_S (0.2f)
#define GIMBAL_GOVERNOR_MOVING_RAD_S (0.1f)
#define GIMBAL_GOVERNOR_SETTLED_ANGLE_DEG (0.5f)
#define GIMBAL_GOVERNOR_SETTLED_TIME_S (0.3f)
#define GIMBAL_GOVERNOR_RESTORE_TIME_S (0.5f)

// 单一云台任务拥有全部保护上下文，不供中断或多个云台实例并发访问
typedef enum
{
    GIMBAL_GOVERNOR_RECOVERING = 0,
    GIMBAL_GOVERNOR_MONITOR,
    GIMBAL_GOVERNOR_BRAKING,
    GIMBAL_GOVERNOR_HOLD,
    GIMBAL_GOVERNOR_RESTORING,
} GimbalGovernor_State_t;

static Gimbal_Axis_t *governor_yaw;
static GimbalGovernor_State_t governor_state;
static float governor_detect_s;
static float governor_stop_s;
static float governor_recovery_s;
static float governor_held_angle_deg;
static float governor_previous_current_a;
static float governor_match_s;
static float governor_settled_s;
static Gimbal_Mode_t governor_mode;

/**
 * @brief 判断是否处于制动、受限保持或自动恢复
 *
 * @return 仍需冻结原目标和约束恢复权限时返回 true
 */
static bool GimbalGovernor_IsLatched(void)
{
    return governor_state == GIMBAL_GOVERNOR_BRAKING || governor_state == GIMBAL_GOVERNOR_HOLD ||
           governor_state == GIMBAL_GOVERNOR_RESTORING;
}

/**
 * @brief 初始化唯一 yaw 轴的保护上下文
 *
 * @param[in,out] axis 已初始化且由云台任务独占的 yaw 轴
 * @return 无返回值
 */
void GimbalGovernor_InitYaw(Gimbal_Axis_t *axis)
{
    governor_yaw = axis;
    governor_state = GIMBAL_GOVERNOR_RECOVERING;
    governor_detect_s = 0.0f;
    governor_stop_s = 0.0f;
    governor_recovery_s = 0.0f;
    governor_held_angle_deg = NAN;
    governor_previous_current_a = 0.0f;
    governor_match_s = governor_settled_s = 0.0f;
    governor_mode = GIMBAL_MODE_RELAX;
}

/**
 * @brief 检查受限纠偏的真实运动响应或已到达姿态目标
 *
 * @param[in] residual_rad_s 已校验的速度残差，单位 rad/s
 * @param[in] imu_speed_rad_s 已校验的上部 yaw 转速，单位 rad/s
 * @param[in] period_s 已校验的周期，单位 s
 * @return 持续满足运动一致性或姿态到达条件时返回 true
 */
static bool GimbalGovernor_CanRestore(float residual_rad_s, float imu_speed_rad_s, float period_s)
{
    // 静止本身不能证明底座获得支撑，运动确认必须同时比较两种反馈
    const bool matched = fabsf(residual_rad_s) < GIMBAL_GOVERNOR_MATCH_RAD_S &&
        fabsf(imu_speed_rad_s) > GIMBAL_GOVERNOR_MOVING_RAD_S &&
        governor_yaw->motor_speed_rad_s * imu_speed_rad_s > 0.0f &&
        fabsf(residual_rad_s) < GIMBAL_GOVERNOR_MATCH_RATIO * fabsf(imu_speed_rad_s);
    const bool settled = governor_mode == GIMBAL_MODE_ANGLE &&
        fabsf(governor_held_angle_deg - governor_yaw->angle_deg) < GIMBAL_GOVERNOR_SETTLED_ANGLE_DEG &&
        fabsf(residual_rad_s) < GIMBAL_GOVERNOR_MATCH_RAD_S &&
        fabsf(imu_speed_rad_s) < GIMBAL_GOVERNOR_MOVING_RAD_S;
    governor_match_s = matched ? governor_match_s + period_s : 0.0f;
    governor_settled_s = settled ? governor_settled_s + period_s : 0.0f;
    return governor_match_s >= GIMBAL_GOVERNOR_MATCH_TIME_S ||
           governor_settled_s >= GIMBAL_GOVERNOR_SETTLED_TIME_S;
}

/**
 * @brief 集中推进异常检测、制动、受限姿态保持及自动恢复状态
 *
 * @param[in] residual_rad_s 有限的同坐标速度残差，单位 rad/s
 * @param[in] imu_speed_rad_s 有限的上部 yaw 转速，单位 rad/s
 * @param[in] period_s 已校验的周期，单位 s
 * @return 状态合法时返回 true，非法状态锁定保持并返回 false
 */
static bool GimbalGovernor_Advance(float residual_rad_s, float imu_speed_rad_s, float period_s)
{
    // 正常运行阶段仅累计连续同向异常，不把断续运动拼成一次触发
    switch (governor_state)
    {
        case GIMBAL_GOVERNOR_RECOVERING:
        case GIMBAL_GOVERNOR_MONITOR:
            governor_recovery_s = fminf(GIMBAL_GOVERNOR_RECOVERY_TIME_S, governor_recovery_s + period_s);
            if (fabsf(residual_rad_s) > GIMBAL_GOVERNOR_DETECT_RAD_S &&
                fabsf(imu_speed_rad_s) < GIMBAL_GOVERNOR_IMU_STILL_RAD_S)
            {
                if (governor_detect_s * residual_rad_s < 0.0f)
                {
                    governor_detect_s = 0.0f;
                }
                governor_detect_s += copysignf(period_s, residual_rad_s);
            }
            else
            {
                governor_detect_s = 0.0f;
            }
            if (fabsf(governor_detect_s) >= GIMBAL_GOVERNOR_DETECT_TIME_S)
            {
                governor_held_angle_deg = isfinite(governor_yaw->target_angle_deg) ?
                    governor_yaw->target_angle_deg : governor_yaw->angle_deg;
                governor_stop_s = 0.0f;
                governor_match_s = governor_settled_s = 0.0f;
                governor_state = GIMBAL_GOVERNOR_BRAKING;
            }
            else if (governor_recovery_s >= GIMBAL_GOVERNOR_RECOVERY_TIME_S)
            {
                governor_state = GIMBAL_GOVERNOR_MONITOR;
            }
            break;

        case GIMBAL_GOVERNOR_BRAKING:
            governor_stop_s = fabsf(residual_rad_s) < GIMBAL_GOVERNOR_STOP_RAD_S ?
                governor_stop_s + period_s : 0.0f;
            if (governor_stop_s >= GIMBAL_GOVERNOR_STOP_TIME_S)
            {
                governor_state = GIMBAL_GOVERNOR_HOLD;
                governor_match_s = governor_settled_s = 0.0f;
            }
            break;

        case GIMBAL_GOVERNOR_HOLD:
            if (fabsf(residual_rad_s) > GIMBAL_GOVERNOR_RESTART_RAD_S)
            {
                governor_stop_s = 0.0f;
                governor_state = GIMBAL_GOVERNOR_BRAKING;
            }
            else if (GimbalGovernor_CanRestore(residual_rad_s, imu_speed_rad_s, period_s))
            {
                governor_recovery_s = 0.0f;
                governor_state = GIMBAL_GOVERNOR_RESTORING;
            }
            break;

        case GIMBAL_GOVERNOR_RESTORING:
            // 恢复中再次出现底座运动迹象立即收回权限，不能等到原高速阈值
            if (fabsf(residual_rad_s) >= GIMBAL_GOVERNOR_STOP_RAD_S)
            {
                governor_match_s = governor_settled_s = governor_stop_s = 0.0f;
                governor_state = fabsf(residual_rad_s) > GIMBAL_GOVERNOR_RESTART_RAD_S ?
                    GIMBAL_GOVERNOR_BRAKING : GIMBAL_GOVERNOR_HOLD;
            }
            else
            {
                governor_recovery_s = fminf(GIMBAL_GOVERNOR_RESTORE_TIME_S, governor_recovery_s + period_s);
                if (governor_recovery_s >= GIMBAL_GOVERNOR_RESTORE_TIME_S)
                {
                    governor_detect_s = 0.0f;
                    governor_state = GIMBAL_GOVERNOR_MONITOR;
                }
            }
            break;

        default:
            governor_state = GIMBAL_GOVERNOR_HOLD;
            governor_previous_current_a = 0.0f;
            return false;
    }
    return true;
}

/**
 * @brief 校验本轮输入并在锁存后冻结 yaw 命令，保留 pitch
 *
 * @param[in,out] gimbal 已绑定的唯一云台实例
 * @param[in,out] command 本轮两轴命令，非法值不得被保护覆盖而隐藏
 * @param[in] period_s 实际控制周期，单位 s
 * @return 可继续原单轴控制路径时返回 true，否则由调用方故障停机
 */
bool GimbalGovernor_PrepareYaw(Gimbal_t *gimbal, Gimbal_Command_t *command, float period_s)
{
    if (gimbal == NULL || command == NULL || governor_yaw != &gimbal->yaw)
    {
        return false;
    }
    // 明确失能立即复位保护，电机失联仍交给原有轴状态机零输出
    if (command->mode == GIMBAL_MODE_RELAX || !gimbal->config.yaw.enabled)
    {
        GimbalGovernor_InitYaw(&gimbal->yaw);
        return true;
    }
    if ((command->mode != GIMBAL_MODE_ANGLE && command->mode != GIMBAL_MODE_SPEED) ||
        !isfinite(period_s) || period_s <= 0.0f || period_s > 0.1f ||
        !isfinite(command->yaw.target_speed_rad_s) ||
        (command->mode == GIMBAL_MODE_ANGLE && !isfinite(command->yaw.target_angle_deg)))
    {
        return false;
    }
    governor_mode = command->mode;
    // 旧反馈不得用于主动制动，等待期间清除计时与电流历史，但保留已有锁存
    if (gimbal->imu_state != GIMBAL_IMU_READY || gimbal->yaw.state == GIMBAL_AXIS_FAULT ||
        !isfinite(gimbal->yaw.motor_speed_rad_s) || !isfinite(gimbal->yaw.speed_rad_s) ||
        !isfinite(gimbal->yaw.mechanical_angle_deg) || !isfinite(gimbal->yaw.angle_deg))
    {
        governor_detect_s = governor_stop_s = governor_previous_current_a = 0.0f;
        governor_match_s = governor_settled_s = 0.0f;
        if (governor_state == GIMBAL_GOVERNOR_RESTORING)
        {
            governor_state = GIMBAL_GOVERNOR_HOLD;
        }
        if (!GimbalGovernor_IsLatched())
        {
            governor_state = GIMBAL_GOVERNOR_RECOVERING;
            governor_recovery_s = 0.0f;
        }
        return true;
    }

    // 独立底盘实测项本轮为零，不使用由同一编码器和惯性数据反推的底座速度
    const float residual_rad_s = gimbal->yaw.motor_speed_rad_s - gimbal->yaw.speed_rad_s;
    if (!isfinite(residual_rad_s) || !GimbalGovernor_Advance(residual_rad_s, gimbal->yaw.speed_rad_s, period_s))
    {
        return false;
    }
    if (GimbalGovernor_IsLatched())
    {
        command->yaw.target_angle_deg = governor_held_angle_deg;
        command->yaw.target_speed_rad_s = 0.0f;
    }
    return true;
}

/**
 * @brief 制动时暂停纠偏，停稳后以受限速度保持姿态并逐步恢复权限
 *
 * @param[in] target_speed_rad_s 有限的速度请求，单位 rad/s
 * @return 允许的速度请求，非法状态返回非数值
 */
float GimbalGovernor_LimitSpeed(float target_speed_rad_s)
{
    if (!isfinite(target_speed_rad_s))
    {
        return NAN;
    }
    switch (governor_state)
    {
        case GIMBAL_GOVERNOR_RECOVERING:
            return target_speed_rad_s * governor_recovery_s / GIMBAL_GOVERNOR_RECOVERY_TIME_S;
        case GIMBAL_GOVERNOR_MONITOR:
            return target_speed_rad_s;
        case GIMBAL_GOVERNOR_BRAKING:
            return 0.0f;
        case GIMBAL_GOVERNOR_HOLD:
            return fminf(GIMBAL_GOVERNOR_HOLD_SPEED_RAD_S, fmaxf(-GIMBAL_GOVERNOR_HOLD_SPEED_RAD_S,
                                                               target_speed_rad_s));
        case GIMBAL_GOVERNOR_RESTORING:
        {
            const float blend = governor_recovery_s / GIMBAL_GOVERNOR_RESTORE_TIME_S;
            const float hold_speed = fminf(GIMBAL_GOVERNOR_HOLD_SPEED_RAD_S,
                                           fmaxf(-GIMBAL_GOVERNOR_HOLD_SPEED_RAD_S, target_speed_rad_s));
            return hold_speed + blend * (target_speed_rad_s - hold_speed);
        }
        default:
            return NAN;
    }
}

/**
 * @brief 给受限姿态电流施加低速残差包络，保留反向制动
 *
 * @param[in] residual_rad_s 有限的残差，单位 rad/s
 * @param[in] requested_a 有限的候选电流，单位 A
 * @param[in] max_current_a 有限正电流上限，单位 A
 * @return 包络内电流，单位 A
 */
static float GimbalGovernor_LimitHolding(float residual_rad_s, float requested_a, float max_current_a)
{
    // 残差从停稳阈值增至重新制动阈值时，连续削减同向电流并叠加阻尼
    const float width_rad_s = GIMBAL_GOVERNOR_RESTART_RAD_S - GIMBAL_GOVERNOR_STOP_RAD_S;
    const float upper = max_current_a * fminf(1.0f, fmaxf(0.0f,
        (GIMBAL_GOVERNOR_RESTART_RAD_S - residual_rad_s) / width_rad_s));
    const float lower = -max_current_a * fminf(1.0f, fmaxf(0.0f,
        (GIMBAL_GOVERNOR_RESTART_RAD_S + residual_rad_s) / width_rad_s));
    const float damped_a = requested_a - GIMBAL_GOVERNOR_DAMPING * residual_rad_s;
    return fminf(upper, fmaxf(lower, fminf(max_current_a, fmaxf(-max_current_a, damped_a))));
}

/**
 * @brief 制动时使用残差阻尼，停稳后受限保持姿态并自动平滑恢复
 *
 * @param[in] motor_speed_rad_s 有限的编码器速度，单位 rad/s
 * @param[in] imu_speed_rad_s 有限的上部 yaw 转速，单位 rad/s
 * @param[in] requested_current_a 有限的候选电流，单位 A
 * @param[in] max_current_a 有限正电流上限，单位 A
 * @param[in] period_s 周期，单位 s，范围 (0, 0.1]
 * @return 保护后的电流，非法输入返回非数值
 */
float GimbalGovernor_ControlCurrent(float motor_speed_rad_s, float imu_speed_rad_s, float requested_current_a,
                                    float max_current_a, float period_s)
{
    float current_a = GimbalGovernor_LimitCurrent(motor_speed_rad_s, imu_speed_rad_s, 0.0f,
                                                 requested_current_a, max_current_a);
    if (!isfinite(current_a) || !isfinite(period_s) || period_s <= 0.0f || period_s > 0.1f)
    {
        return NAN;
    }
    const float residual_rad_s = motor_speed_rad_s - imu_speed_rad_s;
    const float step_a = GIMBAL_GOVERNOR_CURRENT_SLEW_A_S * period_s;
    float previous_a = governor_previous_current_a;
    float permitted_a = max_current_a;
    switch (governor_state)
    {
        case GIMBAL_GOVERNOR_MONITOR:
            break;
        case GIMBAL_GOVERNOR_RECOVERING:
        {
            const float limit_a = max_current_a * governor_recovery_s / GIMBAL_GOVERNOR_RECOVERY_TIME_S;
            permitted_a = limit_a;
            current_a = fminf(limit_a, fmaxf(-limit_a, current_a));
            break;
        }
        case GIMBAL_GOVERNOR_BRAKING:
        {
            const float limit_a = fminf(GIMBAL_GOVERNOR_BRAKE_CURRENT_A, max_current_a);
            permitted_a = limit_a;
            current_a = fminf(limit_a, fmaxf(-limit_a, -GIMBAL_GOVERNOR_DAMPING * residual_rad_s));
            // 进入制动或运动换向时先撤销同向推动，不能为保持斜率继续加速
            if (previous_a * residual_rad_s >= 0.0f)
            {
                previous_a = 0.0f;
            }
            break;
        }
        case GIMBAL_GOVERNOR_HOLD:
        case GIMBAL_GOVERNOR_RESTORING:
        {
            if (governor_yaw == NULL)
            {
                return NAN;
            }
            // 受限保持使用同一组比例增益，避免制动反算留下的积分暂时抵消姿态保持力
            const float blend = governor_state == GIMBAL_GOVERNOR_RESTORING ?
                governor_recovery_s / GIMBAL_GOVERNOR_RESTORE_TIME_S : 0.0f;
            const float hold_limit_a = fminf(GIMBAL_GOVERNOR_HOLD_CURRENT_A, max_current_a);
            permitted_a = hold_limit_a + blend * (max_current_a - hold_limit_a);
            const float proportional_a = governor_yaw->applied_speed_pid.k * governor_yaw->applied_speed_pid.p *
                (governor_yaw->target_speed_rad_s - imu_speed_rad_s);
            if (!isfinite(proportional_a))
            {
                return NAN;
            }
            current_a = proportional_a + blend * (current_a - proportional_a);
            current_a = GimbalGovernor_LimitHolding(residual_rad_s, current_a, permitted_a);
            // 超过重新制动阈值时不沿用上周期的同向推动
            if (fabsf(residual_rad_s) >= GIMBAL_GOVERNOR_RESTART_RAD_S && previous_a * residual_rad_s > 0.0f)
            {
                previous_a = 0.0f;
            }
            break;
        }
        default:
            return NAN;
    }
    current_a = fminf(previous_a + step_a, fmaxf(previous_a - step_a, current_a));
    // 配置电流上限下降时立即遵守新上限，不能被历史电流绕过
    current_a = fminf(permitted_a, fmaxf(-permitted_a, current_a));
    if (governor_state == GIMBAL_GOVERNOR_HOLD || governor_state == GIMBAL_GOVERNOR_RESTORING)
    {
        // 斜率历史不得绕过低速电流包络；此处仅夹紧，不重复叠加阻尼
        current_a = GimbalGovernor_LimitHolding(residual_rad_s,
            current_a + GIMBAL_GOVERNOR_DAMPING * residual_rad_s, permitted_a);
    }
    return GimbalGovernor_IsLatched() ? current_a :
        GimbalGovernor_LimitCurrent(motor_speed_rad_s, imu_speed_rad_s, 0.0f, current_a, max_current_a);
}

/**
 * @brief 将实际执行电流用于反算，并在保护期间冻结角度积分和目标
 *
 * @param[in,out] axis 已绑定的 yaw 轴，输出已经过机械边界处理
 * @param[in] candidate_a 保护前的候选电流，单位 A
 * @param[in] previous_speed_integral 本轮速度环计算前积分
 * @param[in] previous_angle_integral 本轮角度环计算前积分
 * @param[in] period_s 已校验周期，单位 s
 * @return 无返回值，异常结果由上层检查后停机
 */
void GimbalGovernor_TrackCurrent(Gimbal_Axis_t *axis, float candidate_a, float previous_speed_integral,
                                 float previous_angle_integral, float period_s)
{
    if (axis == NULL || axis != governor_yaw)
    {
        return;
    }
    // 撤销继续推向饱和方向的增量，再以有限步长跟踪最终电流
    const float delta_a = axis->output_current_a - candidate_a;
    if (delta_a != 0.0f)
    {
        if ((axis->speed_pid.i - previous_speed_integral) * delta_a < 0.0f)
        {
            axis->speed_pid.i = previous_speed_integral;
        }
        axis->speed_pid.i = GimbalGovernor_BackCalculate(axis->speed_pid.i, axis->applied_speed_pid.i,
            axis->applied_speed_pid.i_limit, delta_a, period_s);
        if ((axis->angle_pid.i - previous_angle_integral) * delta_a < 0.0f)
        {
            axis->angle_pid.i = previous_angle_integral;
        }
    }
    if (GimbalGovernor_IsLatched())
    {
        axis->angle_pid.i = previous_angle_integral;
        axis->target_angle_deg = governor_held_angle_deg;
    }
    governor_previous_current_a = axis->output_current_a;
}

/**
 * @brief 计算正向电流上界的连续分段曲线
 *
 * @param[in] residual_rad_s 已校验的转速残差，单位 rad/s
 * @param[in] max_current_a 已校验的电流绝对值上限，单位 A
 * @return 正向电流上界，单位 A，超过硬阈值后为反向制动电流
 */
static float GimbalGovernor_Upper(float residual_rad_s, float max_current_a)
{
    // 两段无记忆斜坡分别削减驱动和增加制动，各边界连续且不引入运行状态
    const float drive = fminf(1.0f, fmaxf(0.0f, (GIMBAL_GOVERNOR_HARD_RAD_S - residual_rad_s) /
                                       (GIMBAL_GOVERNOR_HARD_RAD_S - GIMBAL_GOVERNOR_SOFT_RAD_S)));
    const float brake = fminf(1.0f, fmaxf(0.0f, (residual_rad_s - GIMBAL_GOVERNOR_HARD_RAD_S) /
                                       (GIMBAL_GOVERNOR_BRAKE_RAD_S - GIMBAL_GOVERNOR_HARD_RAD_S)));
    return max_current_a * drive - fminf(GIMBAL_GOVERNOR_BRAKE_CURRENT_A, max_current_a) * brake;
}

/**
 * @brief 按同坐标转速残差夹紧电流，接口参数及返回值见头文件
 *
 * @param[in] motor_speed_rad_s 编码器相对转速，单位 rad/s
 * @param[in] imu_speed_rad_s 上部惯性 yaw 转速，单位 rad/s
 * @param[in] chassis_speed_rad_s 独立实测底盘转速，单位 rad/s
 * @param[in] requested_current_a 候选电流，单位 A
 * @param[in] max_current_a 电流绝对值上限，单位 A，须为有限正值
 * @return 包络内电流，非法输入或溢出时返回非数值
 */
float GimbalGovernor_LimitCurrent(float motor_speed_rad_s, float imu_speed_rad_s, float chassis_speed_rad_s,
                                  float requested_current_a, float max_current_a)
{
    const float residual_rad_s = motor_speed_rad_s - imu_speed_rad_s + chassis_speed_rad_s;
    // 拒绝非法数据，不能用旧转速或数学库的非数值吞并行为继续制动
    if (!isfinite(motor_speed_rad_s) || !isfinite(imu_speed_rad_s) || !isfinite(chassis_speed_rad_s) ||
        !isfinite(residual_rad_s) || !isfinite(requested_current_a) ||
        !isfinite(max_current_a) || max_current_a <= 0.0f)
    {
        return NAN;
    }

    // 下界由上界镜像生成，保留包络内更强的已有反向制动
    const float upper_a = GimbalGovernor_Upper(residual_rad_s, max_current_a);
    const float lower_a = -GimbalGovernor_Upper(-residual_rad_s, max_current_a);
    return fminf(upper_a, fmaxf(lower_a, requested_current_a));
}

/**
 * @brief 限制积分反算的跟踪比例和每秒电流贡献变化
 *
 * @param[in] integral 受限方向增量撤销后的误差积分
 * @param[in] integral_gain 积分增益，须为有限非负值
 * @param[in] integral_limit 误差积分绝对值上限，须为有限非负值
 * @param[in] current_delta_a 实际电流减候选电流，单位 A
 * @param[in] period_s 实际周期，单位 s，范围 (0, 0.1]
 * @return 修正后的误差积分，非法输入返回非数值
 */
float GimbalGovernor_BackCalculate(float integral, float integral_gain, float integral_limit,
                                   float current_delta_a, float period_s)
{
    // 零增益不参与反算，异常输入交回模块故障路径处理
    if (!isfinite(integral) || !isfinite(integral_gain) || integral_gain < 0.0f ||
        !isfinite(integral_limit) || integral_limit < 0.0f || !isfinite(current_delta_a) ||
        !isfinite(period_s) || period_s <= 0.0f || period_s > 0.1f)
    {
        return NAN;
    }
    if (integral_gain <= GIMBAL_GOVERNOR_MIN_INTEGRAL_GAIN)
    {
        return integral;
    }

    // 先在电流贡献域限步长，再换算到误差积分，保留既有积分而不直接清零
    const float tracking = fminf(1.0f, period_s / GIMBAL_GOVERNOR_TRACKING_TIME_S);
    const float max_step_a = GIMBAL_GOVERNOR_INTEGRAL_SLEW_A_S * period_s;
    const float delta_a = fminf(max_step_a, fmaxf(-max_step_a, current_delta_a * tracking));
    return fminf(integral_limit, fmaxf(-integral_limit, integral + delta_a / integral_gain));
}
