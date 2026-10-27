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
 * - gimbal_command：遥控生成的云台命令，每周期覆盖 mode 和前馈，调试器修改不作为使能入口
 *   右拨杆中位为 RELAX，上位为 ANGLE，下位或非法位置为 RELAX
 *   上电、遥控离线超过 100 毫秒、输入异常或锁存故障后，须先右拨杆回中，再拨上重新使能
 *   单轴电机反馈超时只停止该轴，保持上位时有效反馈恢复即可自动起控，目标重置为当前姿态
 *   首次进入 ANGLE 时捕获当前姿态，右摇杆左右控制 yaw、上下控制 pitch，回中保持目标角
 *   两轴满杆目标角速度均为 6 rad/s，中心死区 5%，方向参数在 motor_gimbal.c 中集中定义
 * - gimbal_sine_test_enabled：默认 false，闭环后以当前 yaw 为中心生成振幅 30°、相位速度 6 rad/s 的正弦轨迹
 * - gimbal_sine_feedforward_enabled：默认 true，控制正弦测试是否同时提供解析角速度前馈
 *   正弦测试会覆盖遥控 yaw 目标，仅用于主动开启的调试实验，同样受右拨杆和失联停机约束
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
