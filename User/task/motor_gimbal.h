#pragma once

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>

#include "module/gimbal.h"

/*
 * 云台调试入口，均由云台任务串行消费，volatile 只保证调试器可见性，不提供多字段原子快照：
 * - gimbal：云台实例，可直接观察两轴状态，并修改 gimbal.config 在线调参或改零点
 *   imu_state 为 2 时惯性反馈就绪，angle_deg/speed_rad_s 为惯性量，mechanical_angle_deg 为编码器机械角
 * - gimbal_command：云台命令，默认 RELAX；写入 GIMBAL_MODE_SPEED 或 GIMBAL_MODE_ANGLE 后开始闭环
 *   轴进入 FAULT 后须先把 mode 写回 RELAX 解除锁存，确认机械位置后再重新使能
 *   首次进入 ANGLE 时目标预置为当前姿态且清零前馈，之后再修改目标角
 * - gimbal_sine_test_enabled：默认 false，闭环后以当前 yaw 为中心生成振幅 30°、相位速度 6 rad/s 的正弦轨迹
 * - gimbal_sine_feedforward_enabled：默认 true，控制正弦测试是否同时提供解析角速度前馈
 */
extern Gimbal_t gimbal;
extern Gimbal_Command_t gimbal_command;
extern volatile bool gimbal_sine_test_enabled;
extern volatile bool gimbal_sine_feedforward_enabled;

/**
 * @brief 初始化云台模块并周期执行反馈读取、控制计算和电流发布
 *
 * @param[in] argument 任务参数，本任务不使用
 * @return 本任务不会返回
 */
void Task_motor_gimbal(void *argument);

#ifdef __cplusplus
}
#endif
