#pragma once

#include "module/gimbal.h"

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

/**
 * @brief 绑定本工程唯一 yaw 轴并初始化保护，禁止并发用于多个云台实例
 *
 * @param[in,out] axis 已初始化的 yaw 轴，生命周期须覆盖云台任务
 * @return 无返回值
 */
void GimbalGovernor_InitYaw(Gimbal_Axis_t *axis);

/**
 * @brief 推进异常运动保护，保护期间冻结 yaw 命令并保留原姿态目标
 *
 * @param[in,out] gimbal 云台实例，yaw 须与初始化时绑定的轴一致
 * @param[in,out] command 当前命令，保护时只覆盖 yaw 目标及前馈，不影响 pitch
 * @param[in] period_s 实际周期，单位 s，运行时范围为 (0, 0.1]
 * @return 参数、绑定或运行周期非法时返回 false，由调用方按原故障路径停机
 */
bool GimbalGovernor_PrepareYaw(Gimbal_t *gimbal, Gimbal_Command_t *command, float period_s);

/**
 * @brief 限制 yaw 速度请求，制动与保持时返回零，重新起控时渐进恢复
 *
 * @param[in] target_speed_rad_s 已校验的速度请求，单位 rad/s
 * @return 本周期允许的速度请求，单位 rad/s
 */
float GimbalGovernor_LimitSpeed(float target_speed_rad_s);

/**
 * @brief 生成 yaw 保护电流并保留高速包络，机械边界仍由调用方最后执行
 *
 * @param[in] motor_speed_rad_s 编码器相对转速，单位 rad/s，须有限
 * @param[in] imu_speed_rad_s 上部惯性 yaw 转速，单位 rad/s，须有限
 * @param[in] requested_current_a 速度环限流后的候选电流，单位 A，须有限
 * @param[in] max_current_a 配置电流上限，单位 A，须为有限正值
 * @param[in] period_s 已校验的控制周期，单位 s，范围 (0, 0.1]
 * @return 允许的电流，单位 A，非法数据返回非数值以触发原故障处理
 */
float GimbalGovernor_ControlCurrent(float motor_speed_rad_s, float imu_speed_rad_s, float requested_current_a,
                                    float max_current_a, float period_s);

/**
 * @brief 按机械边界后的实际电流反算积分，保护期间冻结角度积分
 *
 * @param[in,out] axis 本周期完成电流计算的 yaw 轴
 * @param[in] candidate_a 包络及保护处理前的候选电流，单位 A
 * @param[in] previous_speed_integral 速度环计算前的误差积分
 * @param[in] previous_angle_integral 角度环计算前的误差积分
 * @param[in] period_s 已校验的控制周期，单位 s
 * @return 无返回值，非法反算结果由调用方有限性检查转入故障
 */
void GimbalGovernor_TrackCurrent(Gimbal_Axis_t *axis, float candidate_a, float previous_speed_integral,
                                 float previous_angle_integral, float period_s);

#ifdef __cplusplus
}
#endif
