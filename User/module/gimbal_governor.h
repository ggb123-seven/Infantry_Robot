#pragma once

#ifdef __cplusplus
extern "C"
{
#endif

/**
 * @brief 按编码器与惯性转速残差连续收紧电流并施加反向制动
 *
 * @param[in] motor_speed_rad_s 模块坐标下编码器相对转速，单位 rad/s，须有限
 * @param[in] imu_speed_rad_s 上部惯性 yaw 转速，单位 rad/s，须与编码器同向且有限
 * @param[in] chassis_speed_rad_s 独立实测底盘转速，单位 rad/s，须同向且有限，本轮调用方传 0
 * @param[in] requested_current_a 候选电流，单位 A，须有限
 * @param[in] max_current_a 电流绝对值上限，单位 A，须为有限正值
 * @return 包络内的电流，单位 A，输入或残差非法时返回非数值供调用方故障停机
 */
float GimbalGovernor_LimitCurrent(float motor_speed_rad_s, float imu_speed_rad_s, float chassis_speed_rad_s,
                                  float requested_current_a, float max_current_a);

/**
 * @brief 将电流饱和差以限制步长的反算反馈到速度环误差积分
 *
 * @param[in] integral 撤销受限方向当周期增量后的误差积分，须有限
 * @param[in] integral_gain 积分增益，单位 A 每误差积分，须为有限非负值，接近零时保留积分
 * @param[in] integral_limit 误差积分绝对值上限，须为有限非负值
 * @param[in] current_delta_a 实际电流减候选电流，单位 A，须有限
 * @param[in] period_s 控制周期，单位 s，须在 (0, 0.1] 内
 * @return 经反算及积分限幅后的误差积分，非法输入返回非数值
 */
float GimbalGovernor_BackCalculate(float integral, float integral_gain, float integral_limit,
                                   float current_delta_a, float period_s);

#ifdef __cplusplus
}
#endif
