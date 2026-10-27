#include "module/gimbal_governor.h"

#include <math.h>

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
