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
 * - STOP、RESTART、STOP_TIME：低于 0.2 rad/s 持续 0.1 s 进入目标跟随，高于 0.4 rad/s 重新制动
 * - DAMPING：阻尼系数 0.15 A/(rad/s)，制动上限沿用 0.3 A
 * - RECOVERY_TIME：人工起控及未锁存的失联恢复后，用 0.2 s 渐进开放速度和电流权限
 * - HOLD_CURRENT：目标跟随阶段的电流上限 0.08 A，不主动追赶悬空前的姿态
 * - MATCH、MATCH_TIME：残差小于 0.15 rad/s 且同向运动差小于上部转速的 25%，持续 0.2 s 后退出低速监护
 * - MOVING、FOLLOW_TIME：运动确认下限 0.1 rad/s，残差低于 0.15 rad/s 持续 0.3 s 后捕获当前姿态
 * - RESTORE_TIME：以 0.5 s 恢复速度、电流及原 PID 输出权限，随后继续以低速残差包络监护
 * - 静止也能恢复闭环及配置电流权限，只有运动反馈一致持续 0.2 s 才退出低速监护
 * - RESIDUAL_FILTER_TIME：恢复判据和低速包络使用 0.04 s 一阶残差滤波，兼顾整数 rpm 反馈与异步采样
 * - RELAPSE_TIME：滤波残差超过 0.2 rad/s 持续 0.04 s 才退回跟随，超过 0.4 rad/s 立即重新制动
 * - 原始残差达到 1 rad/s 时立即制动，不等待滤波或持续确认，正常阶段的异常检测仍使用原始残差
 * - CURRENT_SLEW：仅起控、制动及渐进恢复限制电流变化 2 A/s，正常与监护闭环不额外拖慢遥控
 * - 低速包络在残差不超过 0.2 rad/s 时不叠加阻尼，避免量化差异持续改写正常 PID 电流及积分
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
#define GIMBAL_GOVERNOR_HOLD_CURRENT_A (0.08f)
#define GIMBAL_GOVERNOR_MATCH_RAD_S (0.15f)
#define GIMBAL_GOVERNOR_MATCH_RATIO (0.25f)
#define GIMBAL_GOVERNOR_MATCH_TIME_S (0.2f)
#define GIMBAL_GOVERNOR_MOVING_RAD_S (0.1f)
#define GIMBAL_GOVERNOR_FOLLOW_TIME_S (0.3f)
#define GIMBAL_GOVERNOR_RESTORE_TIME_S (0.5f)
#define GIMBAL_GOVERNOR_RESIDUAL_FILTER_TIME_S (0.04f)
#define GIMBAL_GOVERNOR_RELAPSE_TIME_S (0.04f)

// 单一云台任务拥有全部保护上下文，不供中断或多个云台实例并发访问
typedef enum
{
    GIMBAL_GOVERNOR_RECOVERING = 0,
    GIMBAL_GOVERNOR_MONITOR,
    GIMBAL_GOVERNOR_BRAKING,
    GIMBAL_GOVERNOR_HOLD,
    GIMBAL_GOVERNOR_RESTORING,
    GIMBAL_GOVERNOR_GUARDED,
} GimbalGovernor_State_t;

static Gimbal_Axis_t *governor_yaw;
static GimbalGovernor_State_t governor_state;
static float governor_detect_s;
static float governor_stop_s;
static float governor_recovery_s;
static float governor_recovery_angle_deg;
static float governor_previous_current_a;
static float governor_match_s;
static float governor_follow_s;
static float governor_filtered_residual_rad_s;
static float governor_relapse_s;

/**
 * @brief 判断是否需要跟随或捕获当前姿态并暂停外部 yaw 命令
 *
 * @return 制动、目标跟随或渐进恢复时返回 true，监护闭环允许新命令
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
    governor_recovery_angle_deg = NAN;
    governor_previous_current_a = 0.0f;
    governor_match_s = governor_follow_s = 0.0f;
    governor_filtered_residual_rad_s = NAN;
    governor_relapse_s = 0.0f;
}

/**
 * @brief 检查编码器与上部惯性运动是否持续一致
 *
 * @param[in] residual_rad_s 已校验的速度残差，单位 rad/s
 * @param[in] imu_speed_rad_s 已校验的上部 yaw 转速，单位 rad/s
 * @param[in] period_s 已校验的周期，单位 s
 * @return 连续满足运动一致性时返回 true，静止不会解除低速监护
 */
static bool GimbalGovernor_CanRestore(float residual_rad_s, float imu_speed_rad_s, float period_s)
{
    // 静止本身不能证明底座获得支撑，运动确认必须同时比较两种反馈
    const bool matched = fabsf(residual_rad_s) < GIMBAL_GOVERNOR_RESTART_RAD_S &&
        fabsf(governor_filtered_residual_rad_s) < GIMBAL_GOVERNOR_MATCH_RAD_S &&
        fabsf(imu_speed_rad_s) > GIMBAL_GOVERNOR_MOVING_RAD_S &&
        governor_yaw->motor_speed_rad_s * imu_speed_rad_s > 0.0f &&
        fabsf(governor_filtered_residual_rad_s) < GIMBAL_GOVERNOR_MATCH_RATIO * fabsf(imu_speed_rad_s);
    governor_match_s = matched ? governor_match_s + period_s : 0.0f;
    return governor_match_s >= GIMBAL_GOVERNOR_MATCH_TIME_S;
}

/**
 * @brief 集中推进异常检测、制动、目标跟随、渐进恢复及运动监护
 *
 * @param[in] residual_rad_s 有限的同坐标速度残差，单位 rad/s
 * @param[in] imu_speed_rad_s 有限的上部 yaw 转速，单位 rad/s
 * @param[in] period_s 已校验的周期，单位 s
 * @return 状态合法时返回 true，非法状态锁定保持并返回 false
 */
static bool GimbalGovernor_Advance(float residual_rad_s, float imu_speed_rad_s, float period_s)
{
    const float raw_abs_rad_s = fabsf(residual_rad_s);
    const float filtered_abs_rad_s = fabsf(governor_filtered_residual_rad_s);
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
                governor_stop_s = 0.0f;
                governor_match_s = governor_follow_s = 0.0f;
                governor_relapse_s = 0.0f;
                governor_state = GIMBAL_GOVERNOR_BRAKING;
            }
            else if (governor_recovery_s >= GIMBAL_GOVERNOR_RECOVERY_TIME_S)
            {
                governor_state = GIMBAL_GOVERNOR_MONITOR;
            }
            break;

        case GIMBAL_GOVERNOR_BRAKING:
            governor_stop_s = filtered_abs_rad_s < GIMBAL_GOVERNOR_STOP_RAD_S &&
                raw_abs_rad_s < GIMBAL_GOVERNOR_RESTART_RAD_S ?
                governor_stop_s + period_s : 0.0f;
            if (governor_stop_s >= GIMBAL_GOVERNOR_STOP_TIME_S)
            {
                governor_state = GIMBAL_GOVERNOR_HOLD;
                governor_match_s = governor_follow_s = 0.0f;
            }
            break;

        case GIMBAL_GOVERNOR_HOLD:
            // 不要求先产生运动，安静窗口后捕获当前姿态并试行恢复，低速保护始终保留
            governor_follow_s = filtered_abs_rad_s < GIMBAL_GOVERNOR_MATCH_RAD_S ?
                governor_follow_s + period_s : 0.0f;
            if (raw_abs_rad_s >= GIMBAL_GOVERNOR_DETECT_RAD_S ||
                filtered_abs_rad_s > GIMBAL_GOVERNOR_RESTART_RAD_S)
            {
                governor_stop_s = 0.0f;
                governor_state = GIMBAL_GOVERNOR_BRAKING;
            }
            else if (governor_follow_s >= GIMBAL_GOVERNOR_FOLLOW_TIME_S)
            {
                governor_recovery_s = 0.0f;
                governor_relapse_s = 0.0f;
                governor_state = GIMBAL_GOVERNOR_RESTORING;
            }
            break;

        case GIMBAL_GOVERNOR_RESTORING:
        case GIMBAL_GOVERNOR_GUARDED:
            // 短时采样差不能撤销姿态目标，大残差与持续底座运动仍先于恢复处理
            governor_relapse_s = filtered_abs_rad_s >= GIMBAL_GOVERNOR_STOP_RAD_S ?
                fminf(GIMBAL_GOVERNOR_RELAPSE_TIME_S, governor_relapse_s + period_s) : 0.0f;
            if (raw_abs_rad_s >= GIMBAL_GOVERNOR_DETECT_RAD_S ||
                filtered_abs_rad_s > GIMBAL_GOVERNOR_RESTART_RAD_S ||
                governor_relapse_s >= GIMBAL_GOVERNOR_RELAPSE_TIME_S)
            {
                governor_match_s = governor_follow_s = governor_stop_s = 0.0f;
                governor_relapse_s = 0.0f;
                governor_state = raw_abs_rad_s >= GIMBAL_GOVERNOR_DETECT_RAD_S ||
                    filtered_abs_rad_s > GIMBAL_GOVERNOR_RESTART_RAD_S ?
                    GIMBAL_GOVERNOR_BRAKING : GIMBAL_GOVERNOR_HOLD;
            }
            else if (governor_state == GIMBAL_GOVERNOR_RESTORING)
            {
                governor_recovery_s = fminf(GIMBAL_GOVERNOR_RESTORE_TIME_S, governor_recovery_s + period_s);
                if (governor_recovery_s >= GIMBAL_GOVERNOR_RESTORE_TIME_S)
                {
                    governor_match_s = 0.0f;
                    governor_state = GIMBAL_GOVERNOR_GUARDED;
                }
            }
            else if (GimbalGovernor_CanRestore(residual_rad_s, imu_speed_rad_s, period_s))
            {
                governor_detect_s = 0.0f;
                governor_state = GIMBAL_GOVERNOR_MONITOR;
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
 * @brief 校验本轮输入并在保护后跟随当前 yaw，恢复时捕获新目标，保留 pitch
 *
 * @param[in,out] gimbal 已绑定的唯一云台实例
 * @param[in] mode 当前控制模式
 * @param[in,out] setpoint 模块持有的 yaw 设定值，非法值不得被保护覆盖而隐藏
 * @param[in] period_s 实际控制周期，单位 s
 * @return 可继续原单轴控制路径时返回 true，否则由调用方故障停机
 */
bool GimbalGovernor_PrepareYaw(Gimbal_t *gimbal, Gimbal_Mode_t mode,
                               Gimbal_AxisSetpoint_t *setpoint, float period_s)
{
    if (gimbal == NULL || setpoint == NULL || governor_yaw != &gimbal->yaw)
    {
        return false;
    }
    // 明确失能立即复位保护，电机失联仍交给原有轴状态机零输出
    if (mode == GIMBAL_MODE_RELAX || !gimbal->config.yaw.enabled)
    {
        GimbalGovernor_InitYaw(&gimbal->yaw);
        return true;
    }
    if ((mode != GIMBAL_MODE_ANGLE && mode != GIMBAL_MODE_SPEED) ||
        !isfinite(period_s) || period_s <= 0.0f || period_s > 0.1f ||
        !isfinite(setpoint->target_speed_rad_s) ||
        (mode == GIMBAL_MODE_ANGLE && !isfinite(setpoint->target_angle_deg)))
    {
        return false;
    }
    // 旧反馈不得用于主动制动，等待期间清除计时与电流历史，但保留已有锁存
    if (gimbal->imu_state != GIMBAL_IMU_READY || gimbal->yaw.state == GIMBAL_AXIS_FAULT ||
        !isfinite(gimbal->yaw.motor_speed_rad_s) || !isfinite(gimbal->yaw.speed_rad_s) ||
        !isfinite(gimbal->yaw.mechanical_angle_deg) || !isfinite(gimbal->yaw.angle_deg))
    {
        governor_detect_s = governor_stop_s = governor_previous_current_a = 0.0f;
        governor_match_s = governor_follow_s = 0.0f;
        governor_filtered_residual_rad_s = NAN;
        governor_relapse_s = 0.0f;
        if (governor_state == GIMBAL_GOVERNOR_RESTORING || governor_state == GIMBAL_GOVERNOR_GUARDED)
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
    if (!isfinite(residual_rad_s))
    {
        return false;
    }
    // 只平滑运动保护的残差，不改变两环反馈、用户 PID 或真实故障的数据校验
    const float weight = period_s / (GIMBAL_GOVERNOR_RESIDUAL_FILTER_TIME_S + period_s);
    governor_filtered_residual_rad_s = isfinite(governor_filtered_residual_rad_s) ?
        (1.0f - weight) * governor_filtered_residual_rad_s + weight * residual_rad_s : residual_rad_s;
    const bool was_restoring = governor_state == GIMBAL_GOVERNOR_RESTORING;
    if (governor_state == GIMBAL_GOVERNOR_BRAKING || governor_state == GIMBAL_GOVERNOR_HOLD)
    {
        governor_recovery_angle_deg = gimbal->yaw.angle_deg;
    }
    if (!isfinite(governor_filtered_residual_rad_s) ||
        !GimbalGovernor_Advance(residual_rad_s, gimbal->yaw.speed_rad_s, period_s))
    {
        return false;
    }
    // 制动及跟随阶段撤销旧目标，恢复阶段保持入口姿态，交接周期也不接受积累的外部目标
    if (governor_state == GIMBAL_GOVERNOR_BRAKING || governor_state == GIMBAL_GOVERNOR_HOLD)
    {
        governor_recovery_angle_deg = gimbal->yaw.angle_deg;
    }
    if (GimbalGovernor_IsLatched() || was_restoring)
    {
        setpoint->target_angle_deg = governor_recovery_angle_deg;
        setpoint->target_speed_rad_s = 0.0f;
    }
    return true;
}

/**
 * @brief 制动及跟随阶段暂停纠偏，捕获新目标后平滑恢复速度权限
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
        case GIMBAL_GOVERNOR_GUARDED:
            return target_speed_rad_s;
        case GIMBAL_GOVERNOR_BRAKING:
        case GIMBAL_GOVERNOR_HOLD:
            return 0.0f;
        case GIMBAL_GOVERNOR_RESTORING:
            return target_speed_rad_s * governor_recovery_s / GIMBAL_GOVERNOR_RESTORE_TIME_S;
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
    // 残差从停稳阈值增至重新制动阈值时连续削减同向电流，包络内的正常 PID 电流保持不变
    const float width_rad_s = GIMBAL_GOVERNOR_RESTART_RAD_S - GIMBAL_GOVERNOR_STOP_RAD_S;
    const float upper = max_current_a * fminf(1.0f, fmaxf(0.0f,
        (GIMBAL_GOVERNOR_RESTART_RAD_S - residual_rad_s) / width_rad_s));
    const float lower = -max_current_a * fminf(1.0f, fmaxf(0.0f,
        (GIMBAL_GOVERNOR_RESTART_RAD_S + residual_rad_s) / width_rad_s));
    return fminf(upper, fmaxf(lower, fminf(max_current_a, fmaxf(-max_current_a, requested_a))));
}

/**
 * @brief 制动及目标跟随使用阻尼，恢复原 PID 并持续监护低速残差
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
    const float holding_residual_rad_s = isfinite(governor_filtered_residual_rad_s) ?
        governor_filtered_residual_rad_s : residual_rad_s;
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
        case GIMBAL_GOVERNOR_GUARDED:
        {
            if (governor_yaw == NULL)
            {
                return NAN;
            }
            // 跟随时不主动纠偏，捕获新目标后恢复完整 PID，避免仅用比例项导致低出力锁死
            const float blend = governor_state == GIMBAL_GOVERNOR_HOLD ? 0.0f :
                fminf(1.0f, governor_recovery_s / GIMBAL_GOVERNOR_RESTORE_TIME_S);
            const float hold_limit_a = fminf(GIMBAL_GOVERNOR_HOLD_CURRENT_A, max_current_a);
            permitted_a = hold_limit_a + blend * (max_current_a - hold_limit_a);
            current_a *= blend;
            // 低于停稳阈值不额外出力，正常采样差不得持续反算成新的积分偏置
            const float excess_rad_s = copysignf(fmaxf(0.0f,
                fabsf(holding_residual_rad_s) - GIMBAL_GOVERNOR_STOP_RAD_S), holding_residual_rad_s);
            current_a -= GIMBAL_GOVERNOR_DAMPING * excess_rad_s;
            current_a = GimbalGovernor_LimitHolding(holding_residual_rad_s, current_a, permitted_a);
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
    // 正常及监护闭环保持原 PID 响应，渐进恢复已在开放命令前完成，不再常驻电流斜坡
    if (governor_state != GIMBAL_GOVERNOR_MONITOR && governor_state != GIMBAL_GOVERNOR_GUARDED)
    {
        current_a = fminf(previous_a + step_a, fmaxf(previous_a - step_a, current_a));
    }
    // 配置电流上限下降时立即遵守新上限，不能被历史电流绕过
    current_a = fminf(permitted_a, fmaxf(-permitted_a, current_a));
    if (governor_state == GIMBAL_GOVERNOR_HOLD || governor_state == GIMBAL_GOVERNOR_RESTORING ||
        governor_state == GIMBAL_GOVERNOR_GUARDED)
    {
        // 斜率历史不得绕过低速电流包络，此处只夹紧，不重复叠加阻尼
        current_a = GimbalGovernor_LimitHolding(holding_residual_rad_s, current_a, permitted_a);
    }
    return GimbalGovernor_IsLatched() || governor_state == GIMBAL_GOVERNOR_GUARDED ? current_a :
        GimbalGovernor_LimitCurrent(motor_speed_rad_s, imu_speed_rad_s, 0.0f, current_a, max_current_a);
}

/**
 * @brief 将实际执行电流用于反算，跟随及渐进恢复期间冻结角度积分
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
