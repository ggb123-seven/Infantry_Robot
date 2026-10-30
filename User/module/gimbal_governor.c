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
 * 独立底盘项仅接纳标定后的四轮观测，旧接口仍显式取零，倾斜及不同步采样误差仍需实测
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

/*
 * 四轮联合识别的初始观测约束，尚待实物录波确认：
 * - 源帧龄上限 20 毫秒，四轮时间跨度 4 毫秒，与云台反馈总时间跨度 10 毫秒
 * - 有效源时间持续推进 20 毫秒后才使用观测，重复快照不能累计确认时间
 * - 误差余量为 0.15 rad/s 加运动幅值的 10%，并计入轮速角加速度与源时间差，最大 0.5 rad/s
 * - 独立转速及请求上限沿用惯性反馈的 40 rad/s 数据边界
 * - 可疑阶段沿用 0.08 A 受限电流，持续确认后撤驱，禁止仅凭打滑残差主动反向制动
 */
#define GIMBAL_CHASSIS_MAX_AGE_US (20000U)
#define GIMBAL_CHASSIS_WHEEL_SKEW_US (4000U)
#define GIMBAL_CHASSIS_ALIGNMENT_US (10000U)
#define GIMBAL_CHASSIS_CONFIRM_S (0.02f)
#define GIMBAL_CHASSIS_ERROR_RAD_S (0.15f)
#define GIMBAL_CHASSIS_ERROR_RATIO (0.10f)
#define GIMBAL_CHASSIS_MAX_ERROR_RAD_S (0.5f)
#define GIMBAL_CHASSIS_MAX_SPEED_RAD_S (40.0f)

// 单一云台任务拥有全部保护上下文，不供中断或多个云台实例并发访问
typedef enum
{
    GIMBAL_GOVERNOR_RECOVERING = 0,
    GIMBAL_GOVERNOR_MONITOR,
    GIMBAL_GOVERNOR_BRAKING,
    GIMBAL_GOVERNOR_HOLD,
    GIMBAL_GOVERNOR_RESTORING,
    GIMBAL_GOVERNOR_GUARDED,
    GIMBAL_GOVERNOR_SUSPECT,
} GimbalGovernor_State_t;

// 观测状态仅描述独立数据是否可用，异常保护阶段仍由原保护状态机唯一持有
typedef enum
{
    GIMBAL_GOVERNOR_OBSERVATION_LEGACY = 0,
    GIMBAL_GOVERNOR_OBSERVATION_WAITING,
    GIMBAL_GOVERNOR_OBSERVATION_READY,
    GIMBAL_GOVERNOR_OBSERVATION_UNAVAILABLE,
} GimbalGovernor_ObservationState_t;

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
static GimbalGovernor_ObservationState_t governor_observation_state;
static float governor_chassis_speed_rad_s;
static float governor_residual_margin_rad_s;
static float governor_observation_confirm_s;
static float governor_previous_chassis_speed_rad_s;
static float governor_chassis_acceleration_rad_s2;
static uint64_t governor_chassis_time_us;

/**
 * @brief 判断是否需要跟随或捕获当前姿态并暂停外部 yaw 命令
 *
 * @return 可疑、制动、目标跟随或渐进恢复时返回 true，监护闭环允许新命令
 */
static bool GimbalGovernor_IsLatched(void)
{
    return governor_state == GIMBAL_GOVERNOR_BRAKING || governor_state == GIMBAL_GOVERNOR_HOLD ||
           governor_state == GIMBAL_GOVERNOR_RESTORING || governor_state == GIMBAL_GOVERNOR_SUSPECT;
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
    governor_observation_state = GIMBAL_GOVERNOR_OBSERVATION_LEGACY;
    governor_chassis_speed_rad_s = governor_residual_margin_rad_s = 0.0f;
    governor_observation_confirm_s = governor_previous_chassis_speed_rad_s = 0.0f;
    governor_chassis_acceleration_rad_s2 = 0.0f;
    governor_chassis_time_us = 0U;
}

/**
 * @brief 校验并确认独立观测，不以命令或同源底座估计替代四轮测量
 *
 * @param[in] gimbal 已校验本周期反馈的云台实例
 * @param[in] chassis 独立四轮观测，为空时兼容原保护
 * @param[in] now_us 消费反馈后的单调时间，单位微秒
 * @return 无返回值，结果保存在单任务拥有的观测上下文
 */
static void GimbalGovernor_UpdateChassisObservation(const Gimbal_t *gimbal,
                                                    const Gimbal_ChassisFeedback_t *chassis, uint64_t now_us)
{
    const GimbalGovernor_ObservationState_t previous = governor_observation_state;
    if (chassis == NULL)
    {
        governor_observation_state = GIMBAL_GOVERNOR_OBSERVATION_LEGACY;
        governor_chassis_speed_rad_s = governor_residual_margin_rad_s = 0.0f;
        governor_observation_confirm_s = 0.0f;
        governor_chassis_time_us = 0U;
        governor_chassis_acceleration_rad_s2 = 0.0f;
        if (previous != GIMBAL_GOVERNOR_OBSERVATION_LEGACY)
        {
            governor_filtered_residual_rad_s = NAN;
            governor_detect_s = governor_match_s = governor_follow_s = governor_relapse_s = 0.0f;
        }
        return;
    }

    // 无效观测不保留补偿，也不刷新源时间；四轮和两种云台反馈须处于同一短时间窗
    const uint64_t oldest_us = chassis->oldest_sample_time_us;
    const uint64_t newest_us = chassis->newest_sample_time_us;
    uint64_t first_us = oldest_us;
    uint64_t last_us = newest_us;
    first_us = gimbal->yaw.feedback_time_us < first_us ? gimbal->yaw.feedback_time_us : first_us;
    first_us = gimbal->imu.sample_time_us < first_us ? gimbal->imu.sample_time_us : first_us;
    last_us = gimbal->yaw.feedback_time_us > last_us ? gimbal->yaw.feedback_time_us : last_us;
    last_us = gimbal->imu.sample_time_us > last_us ? gimbal->imu.sample_time_us : last_us;
    const bool valid = chassis->valid && (unsigned)chassis->mode < GIMBAL_CHASSIS_MOTION_UNKNOWN &&
        oldest_us <= newest_us && last_us <= now_us &&
        now_us - oldest_us <= GIMBAL_CHASSIS_MAX_AGE_US &&
        newest_us - oldest_us <= GIMBAL_CHASSIS_WHEEL_SKEW_US &&
        last_us - first_us <= GIMBAL_CHASSIS_ALIGNMENT_US &&
        isfinite(chassis->measured_speed_rad_s) && isfinite(chassis->requested_speed_rad_s) &&
        fabsf(chassis->measured_speed_rad_s) <= GIMBAL_CHASSIS_MAX_SPEED_RAD_S &&
        fabsf(chassis->requested_speed_rad_s) <= GIMBAL_CHASSIS_MAX_SPEED_RAD_S;
    const uint64_t source_us = valid ? oldest_us + (newest_us - oldest_us) / 2U : 0U;
    if (!valid || (governor_chassis_time_us != 0U && source_us < governor_chassis_time_us))
    {
        governor_observation_state = GIMBAL_GOVERNOR_OBSERVATION_UNAVAILABLE;
        governor_chassis_speed_rad_s = governor_residual_margin_rad_s = 0.0f;
        governor_observation_confirm_s = 0.0f;
        governor_chassis_time_us = 0U;
        governor_chassis_acceleration_rad_s2 = 0.0f;
        return;
    }

    // 仅源反馈推进才能累计确认；误差余量包含标度误差与当前时间错位产生的动态误差
    const float observed_s = governor_chassis_time_us != 0U ?
        (float)(source_us - governor_chassis_time_us) * 1.0e-6f : 0.0f;
    if (observed_s > (float)GIMBAL_CHASSIS_MAX_AGE_US * 1.0e-6f)
    {
        governor_observation_confirm_s = 0.0f;
        governor_chassis_acceleration_rad_s2 = 0.0f;
    }
    else if (observed_s > 0.0f)
    {
        governor_chassis_acceleration_rad_s2 =
            (chassis->measured_speed_rad_s - governor_previous_chassis_speed_rad_s) / observed_s;
    }
    governor_observation_confirm_s = fminf(GIMBAL_CHASSIS_CONFIRM_S,
        governor_observation_confirm_s +
        (observed_s <= (float)GIMBAL_CHASSIS_MAX_AGE_US * 1.0e-6f ? observed_s : 0.0f));
    governor_chassis_speed_rad_s = chassis->measured_speed_rad_s;
    const float magnitude_rad_s = fmaxf(fabsf(chassis->requested_speed_rad_s),
        fmaxf(fabsf(governor_chassis_speed_rad_s), fabsf(gimbal->yaw.speed_rad_s - gimbal->yaw.motor_speed_rad_s)));
    governor_residual_margin_rad_s = fminf(GIMBAL_CHASSIS_MAX_ERROR_RAD_S,
        GIMBAL_CHASSIS_ERROR_RAD_S + GIMBAL_CHASSIS_ERROR_RATIO * magnitude_rad_s +
        fabsf(governor_chassis_acceleration_rad_s2) * (float)(last_us - first_us) * 1.0e-6f);
    governor_previous_chassis_speed_rad_s = chassis->measured_speed_rad_s;
    governor_chassis_time_us = source_us;
    governor_observation_state = governor_observation_confirm_s >= GIMBAL_CHASSIS_CONFIRM_S ?
        GIMBAL_GOVERNOR_OBSERVATION_READY : GIMBAL_GOVERNOR_OBSERVATION_WAITING;
}

/**
 * @brief 在联合识别时扣除经过上限约束的观测误差余量
 *
 * @param[in] residual_rad_s 四轮补偿后的有限残差，单位 rad/s
 * @return 用于低速异常状态机的有效残差，旧接口保持原值
 */
static float GimbalGovernor_EffectiveResidual(float residual_rad_s)
{
    return copysignf(fmaxf(0.0f, fabsf(residual_rad_s) - governor_residual_margin_rad_s), residual_rad_s);
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
    const float relative_rad_s = imu_speed_rad_s - governor_chassis_speed_rad_s;
    const float moving_rad_s = fmaxf(fabsf(relative_rad_s), fabsf(governor_chassis_speed_rad_s));
    const bool direction_matched = governor_observation_state == GIMBAL_GOVERNOR_OBSERVATION_READY ?
        fabsf(relative_rad_s) < GIMBAL_GOVERNOR_MOVING_RAD_S ||
        governor_yaw->motor_speed_rad_s * relative_rad_s > 0.0f :
        governor_yaw->motor_speed_rad_s * imu_speed_rad_s > 0.0f;
    const bool matched = fabsf(residual_rad_s) < GIMBAL_GOVERNOR_RESTART_RAD_S &&
        fabsf(governor_filtered_residual_rad_s) < GIMBAL_GOVERNOR_MATCH_RAD_S &&
        moving_rad_s > GIMBAL_GOVERNOR_MOVING_RAD_S && direction_matched &&
        fabsf(governor_filtered_residual_rad_s) < GIMBAL_GOVERNOR_MATCH_RATIO * moving_rad_s;
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
        case GIMBAL_GOVERNOR_SUSPECT:
        {
            const bool was_suspect = governor_state == GIMBAL_GOVERNOR_SUSPECT;
            governor_recovery_s = fminf(GIMBAL_GOVERNOR_RECOVERY_TIME_S, governor_recovery_s + period_s);
            const bool inconsistent = governor_observation_state == GIMBAL_GOVERNOR_OBSERVATION_READY ?
                raw_abs_rad_s > GIMBAL_GOVERNOR_STOP_RAD_S :
                raw_abs_rad_s > GIMBAL_GOVERNOR_DETECT_RAD_S &&
                fabsf(imu_speed_rad_s) < GIMBAL_GOVERNOR_IMU_STILL_RAD_S;
            if (inconsistent)
            {
                if (governor_detect_s * residual_rad_s < 0.0f)
                {
                    governor_detect_s = 0.0f;
                }
                governor_detect_s += copysignf(period_s, residual_rad_s);
                if (governor_observation_state == GIMBAL_GOVERNOR_OBSERVATION_READY)
                {
                    governor_state = GIMBAL_GOVERNOR_SUSPECT;
                }
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
            else if (!inconsistent && was_suspect)
            {
                governor_state = GIMBAL_GOVERNOR_RECOVERING;
                governor_recovery_s = 0.0f;
            }
            else if (governor_state != GIMBAL_GOVERNOR_SUSPECT &&
                governor_recovery_s >= GIMBAL_GOVERNOR_RECOVERY_TIME_S)
            {
                governor_state = GIMBAL_GOVERNOR_MONITOR;
            }
            break;
        }

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
 * @brief 校验独立观测并推进保护，观测不足只撤销 yaw 推动，真实故障路径保留
 *
 * @param[in,out] gimbal 已绑定的唯一云台实例
 * @param[in] mode 本周期控制模式
 * @param[in,out] setpoint 模块持有的 yaw 目标，非法目标不得被捕获覆盖
 * @param[in] chassis 独立四轮观测，可为空，为空时兼容原保护
 * @param[in] now_us 消费反馈后的单调时间，单位微秒
 * @param[in] period_s 已校验周期，单位秒
 * @param[in] enabled 本周期配置快照中的 yaw 使能值
 * @return 运动保护计算有效时为真，浮点运算异常时为假
 */
bool GimbalGovernor_PrepareYawWithChassis(Gimbal_t *gimbal, Gimbal_Mode_t mode, Gimbal_AxisSetpoint_t *setpoint,
                                        const Gimbal_ChassisFeedback_t *chassis, uint64_t now_us, float period_s,
                                        bool enabled)
{
    // 明确失能立即复位保护，电机失联仍交给原有轴状态机零输出
    if (mode == GIMBAL_MODE_RELAX || !enabled)
    {
        GimbalGovernor_InitYaw(&gimbal->yaw);
        return true;
    }
    // 旧反馈不得用于主动制动，等待期间清除计时与电流历史，但保留已有锁存
    if (gimbal->imu_state != GIMBAL_IMU_READY || gimbal->yaw.state == GIMBAL_AXIS_FAULT ||
        gimbal->yaw.feedback_time_us == UINT64_MAX)
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

    // 补偿只来自独立四轮观测，模式切换不会自动解除既有异常保护
    const GimbalGovernor_ObservationState_t previous_observation = governor_observation_state;
    GimbalGovernor_UpdateChassisObservation(gimbal, chassis, now_us);
    const bool observation_waiting = governor_observation_state == GIMBAL_GOVERNOR_OBSERVATION_WAITING ||
        governor_observation_state == GIMBAL_GOVERNOR_OBSERVATION_UNAVAILABLE;
    if (observation_waiting || previous_observation != governor_observation_state)
    {
        governor_filtered_residual_rad_s = NAN;
        governor_detect_s = governor_stop_s = governor_match_s = governor_follow_s = governor_relapse_s = 0.0f;
        governor_recovery_s = 0.0f;
        if (!GimbalGovernor_IsLatched())
        {
            governor_state = GIMBAL_GOVERNOR_RECOVERING;
        }
    }
    if (observation_waiting)
    {
        governor_previous_current_a = 0.0f;
        governor_recovery_angle_deg = gimbal->yaw.angle_deg;
        setpoint->target_angle_deg = gimbal->yaw.angle_deg;
        setpoint->target_speed_rad_s = 0.0f;
        return true;
    }
    const float compensated_rad_s = gimbal->yaw.motor_speed_rad_s - gimbal->yaw.speed_rad_s +
        governor_chassis_speed_rad_s;
    const float residual_rad_s = GimbalGovernor_EffectiveResidual(compensated_rad_s);
    if (!isfinite(compensated_rad_s) || !isfinite(residual_rad_s))
    {
        return false;
    }
    // 只平滑运动保护的残差，不改变两环反馈、用户 PID 或真实故障的数据校验
    const float weight = period_s / (GIMBAL_GOVERNOR_RESIDUAL_FILTER_TIME_S + period_s);
    governor_filtered_residual_rad_s = isfinite(governor_filtered_residual_rad_s) ?
        (1.0f - weight) * governor_filtered_residual_rad_s + weight * residual_rad_s : residual_rad_s;
    const bool was_restoring = governor_state == GIMBAL_GOVERNOR_RESTORING;
    const bool was_suspect = governor_state == GIMBAL_GOVERNOR_SUSPECT;
    if (governor_state == GIMBAL_GOVERNOR_BRAKING || governor_state == GIMBAL_GOVERNOR_HOLD || was_suspect)
    {
        governor_recovery_angle_deg = gimbal->yaw.angle_deg;
    }
    if (!isfinite(governor_filtered_residual_rad_s) ||
        !GimbalGovernor_Advance(residual_rad_s, gimbal->yaw.speed_rad_s, period_s))
    {
        return false;
    }
    // 制动及跟随阶段撤销旧目标，恢复阶段保持入口姿态，交接周期也不接受积累的外部目标
    if (governor_state == GIMBAL_GOVERNOR_BRAKING || governor_state == GIMBAL_GOVERNOR_HOLD ||
        governor_state == GIMBAL_GOVERNOR_SUSPECT)
    {
        governor_recovery_angle_deg = gimbal->yaw.angle_deg;
    }
    if (GimbalGovernor_IsLatched() || was_restoring || was_suspect)
    {
        setpoint->target_angle_deg = governor_recovery_angle_deg;
        setpoint->target_speed_rad_s = 0.0f;
    }
    return true;
}

bool GimbalGovernor_IsTracking(void)
{
    // 渐进恢复已持有捕获目标，滑模必须追踪该目标，不能继续逐周期跟随当前姿态
    return governor_state == GIMBAL_GOVERNOR_BRAKING ||
           governor_state == GIMBAL_GOVERNOR_HOLD ||
           governor_state == GIMBAL_GOVERNOR_SUSPECT ||
           governor_observation_state == GIMBAL_GOVERNOR_OBSERVATION_WAITING ||
           governor_observation_state == GIMBAL_GOVERNOR_OBSERVATION_UNAVAILABLE;
}

void GimbalGovernor_CommitCurrent(float current_a)
{
    governor_previous_current_a = current_a;
}

/**
 * @brief 制动及跟随阶段暂停纠偏，捕获新目标后平滑恢复速度权限
 *
 * @param[in] target_speed_rad_s 有限的速度请求，单位 rad/s
 * @return 允许的速度请求，非法状态返回非数值
 */
float GimbalGovernor_LimitSpeed(float target_speed_rad_s)
{
    if (governor_observation_state == GIMBAL_GOVERNOR_OBSERVATION_WAITING ||
        governor_observation_state == GIMBAL_GOVERNOR_OBSERVATION_UNAVAILABLE)
    {
        return 0.0f;
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
        case GIMBAL_GOVERNOR_SUSPECT:
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
    float current_a = GimbalGovernor_LimitCurrent(motor_speed_rad_s, imu_speed_rad_s, governor_chassis_speed_rad_s,
                                                 requested_current_a, max_current_a);
    if (governor_observation_state == GIMBAL_GOVERNOR_OBSERVATION_WAITING ||
        governor_observation_state == GIMBAL_GOVERNOR_OBSERVATION_UNAVAILABLE)
    {
        return 0.0f;
    }
    const bool independent = governor_observation_state == GIMBAL_GOVERNOR_OBSERVATION_READY;
    const float residual_rad_s =
        GimbalGovernor_EffectiveResidual(motor_speed_rad_s - imu_speed_rad_s + governor_chassis_speed_rad_s);
    const float holding_residual_rad_s = isfinite(governor_filtered_residual_rad_s) ?
        governor_filtered_residual_rad_s : residual_rad_s;
    const float step_a = GIMBAL_GOVERNOR_CURRENT_SLEW_A_S * period_s;
    float previous_a = governor_previous_current_a;
    float permitted_a = max_current_a;
    switch (governor_state)
    {
        case GIMBAL_GOVERNOR_MONITOR:
            break;
        case GIMBAL_GOVERNOR_SUSPECT:
            permitted_a = fminf(GIMBAL_GOVERNOR_HOLD_CURRENT_A, max_current_a);
            current_a = fminf(permitted_a, fmaxf(-permitted_a, current_a));
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
            // 四轮异常可能来自空转或打滑，只撤驱，不能按不可信轮速强迫底座匹配
            current_a = independent ? 0.0f :
                fminf(limit_a, fmaxf(-limit_a, -GIMBAL_GOVERNOR_DAMPING * residual_rad_s));
            // 进入制动或运动换向时先撤销同向推动，不能为保持斜率继续加速
            if (independent || previous_a * residual_rad_s >= 0.0f)
            {
                previous_a = 0.0f;
            }
            break;
        }
        case GIMBAL_GOVERNOR_HOLD:
        case GIMBAL_GOVERNOR_RESTORING:
        case GIMBAL_GOVERNOR_GUARDED:
        {
            // 跟随时不主动纠偏，捕获新目标后恢复控制器电流，避免恢复阶段长期压低出力
            const float blend = governor_state == GIMBAL_GOVERNOR_HOLD ? 0.0f :
                fminf(1.0f, governor_recovery_s / GIMBAL_GOVERNOR_RESTORE_TIME_S);
            const float hold_limit_a = fminf(GIMBAL_GOVERNOR_HOLD_CURRENT_A, max_current_a);
            permitted_a = hold_limit_a + blend * (max_current_a - hold_limit_a);
            current_a *= blend;
            // 低于停稳阈值不额外出力，正常采样差不得持续反算成新的积分偏置
            const float excess_rad_s = copysignf(fmaxf(0.0f,
                fabsf(holding_residual_rad_s) - GIMBAL_GOVERNOR_STOP_RAD_S), holding_residual_rad_s);
            if (!independent)
            {
                current_a -= GIMBAL_GOVERNOR_DAMPING * excess_rad_s;
            }
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
    // 联合残差只撤销或收紧原请求，不凭轮速偏差生成反向电流
    current_a = GimbalGovernor_IsLatched() || governor_state == GIMBAL_GOVERNOR_GUARDED ? current_a :
        GimbalGovernor_LimitCurrent(motor_speed_rad_s, imu_speed_rad_s, governor_chassis_speed_rad_s,
                                    current_a, max_current_a);
    return independent && current_a * requested_current_a <= 0.0f ? 0.0f : current_a;
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
    if (GimbalGovernor_IsLatched() || governor_observation_state == GIMBAL_GOVERNOR_OBSERVATION_WAITING ||
        governor_observation_state == GIMBAL_GOVERNOR_OBSERVATION_UNAVAILABLE)
    {
        axis->angle_pid.i = previous_angle_integral;
    }
    GimbalGovernor_CommitCurrent(axis->output_current_a);
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
 * @return 包络内电流，调用方保证输入已在控制入口和反馈边界校验
 */
float GimbalGovernor_LimitCurrent(float motor_speed_rad_s, float imu_speed_rad_s, float chassis_speed_rad_s,
                                  float requested_current_a, float max_current_a)
{
    const float residual_rad_s = motor_speed_rad_s - imu_speed_rad_s + chassis_speed_rad_s;
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
 * @return 修正后的误差积分，调用方保证参数来自本周期有效快照
 */
float GimbalGovernor_BackCalculate(float integral, float integral_gain, float integral_limit,
                                   float current_delta_a, float period_s)
{
    // 零积分增益时没有可反算的电流通道，不执行除法
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
