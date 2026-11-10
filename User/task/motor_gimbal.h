#pragma once

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>

#include "module/gimbal.h"

// 置零时编译移除正弦测试及其调试变量，默认保留现有调试能力
#ifndef MOTOR_GIMBAL_ENABLE_SINE_TEST
#define MOTOR_GIMBAL_ENABLE_SINE_TEST (1)
#endif

/*
 * 云台调试入口由云台任务串行消费，volatile 仅保证调试器可见性，不提供多字段原子快照
 * - gimbal：观察两轴状态，在线修改 gimbal.config；imu_state=2 表示惯性反馈就绪
 * - gimbal.config.yaw.enabled、gimbal.config.pitch.enabled：分别使能 yaw、pitch，默认均为 true
 * - gimbal_command：遥控每周期更新，调试模式由 EmberProbe 写入；遥控在线默认两轴 ANGLE，失联或异常时 RELAX
 * - gimbal_debug_enabled：默认 false；置 true 后跳过遥控输入，由 EmberProbe 控制 gimbal_command，mode 取 0/1/2
 *   分别表示停机、速度环、角度与速度串级，停机时写 mode=0 或关闭此开关
 *   速度目标写 rate_rad_s，单位 rad/s；固定角度将 source 设为 1、前馈置零后写 angle_deg
 * - gimbal_remote_speed_loop_enabled、gimbal_remote_angle_loop_enabled：默认均为 true，仅遥控链路生效
 *   speed=false 时两轴撤驱，angle=false 时保留速度环；手动调试、失联和故障待机不受其覆盖
 *   单轴速度环测试：保持 gimbal_debug_enabled=false，目标轴 enabled=true、另一轴 enabled=false，speed=true、angle=false
 *   比赛默认两轴 enabled 和两个遥控环路均为 true，右摇杆左右控制 yaw、上下控制 pitch
 * - gimbal_sine_test_enabled、gimbal_sine_feedforward_enabled：正弦测试及其角速度前馈，默认 false/true
 */
extern Gimbal_t gimbal;
extern Gimbal_Command_t gimbal_command;
extern volatile bool gimbal_debug_enabled;
extern volatile bool gimbal_remote_speed_loop_enabled;
extern volatile bool gimbal_remote_angle_loop_enabled;
#if MOTOR_GIMBAL_ENABLE_SINE_TEST
extern volatile bool gimbal_sine_test_enabled;
extern volatile bool gimbal_sine_feedforward_enabled;
#endif

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
