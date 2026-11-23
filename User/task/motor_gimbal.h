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
 * 云台调试入口，均由云台任务串行消费，volatile 只保证调试器可见性，不提供多字段原子快照：
 * - gimbal：云台实例，可直接观察两轴状态，并修改 gimbal.config 在线调参或改零点
 *   imu_state 为 2 时惯性反馈就绪，angle_deg/speed_rad_s 为惯性量，mechanical_angle_deg 为编码器机械角
 * - gimbal_command：云台任务生成的输入命令，遥控命令每周期重写，调试命令使能时由调试值覆盖 yaw
 *   右拨杆中位为 RELAX，上位为 ANGLE，下位或非法位置为 RELAX
 *   上电、遥控离线超过 100 毫秒、输入异常或锁存故障后，须先右拨杆回中，再拨上重新使能
 *   单轴电机反馈超时只停止该轴，保持上位时有效反馈恢复即可自动起控，目标重置为当前姿态
 *   首次进入 ANGLE 时捕获当前姿态，右摇杆左右控制 yaw、上下控制 pitch，回中保持目标角
 *   两轴满杆目标角速度均为 6 rad/s，中心死区 5%，方向参数在 module/cmd.c 中集中定义
 * - gimbal_debug_yaw_target_enabled：默认 false，置 true 后跳过遥控使能并以绝对角度控制 yaw，输出发布失败时自动关闭
 * - gimbal_debug_yaw_target_deg：调试目标角度，单位度，使用 INS 累计 yaw 坐标，可在运行中修改
 *   开启调试前先将目标设为当前 gimbal.yaw.angle_deg，避免起控时产生大角度跳转
 * - gimbal_debug_yaw_speed_enabled：默认 false，置 true 后强制 SPEED 模式以单独整定速度内环，优先级高于角度调试
 * - gimbal_debug_yaw_speed_target_rad_s：速度内环目标，单位 rad/s，符号与 gimbal.yaw.speed_rad_s 相同
 *   该模式跳过滑模外环，速度目标按 max_speed_rad_s 和机械行程限幅，非有限值使 yaw 轴进入故障锁存
 *   两轴共用模式，pitch 同步进入速度环且目标为零速，测试须短时小幅并确认 pitch 行程与线束
 * - gimbal_debug_pitch_target_enabled：默认 false，置 true 后以绝对角度驱动 pitch，yaw 保留遥控输入
 * - gimbal_debug_pitch_target_deg：pitch 调试目标角度，单位度，使用 INS 俯仰角坐标，可在运行中修改
 *   开启前先设为当前 gimbal.pitch.angle_deg，避免起控时产生大角度跳转；机械限位仍由云台模块处理
 * - gimbal_remote_speed_loop_enabled：默认 true，遥控链路速度环使能，关闭时遥控云台整体待机
 * - gimbal_remote_angle_loop_enabled：默认 true，遥控链路角度环使能，关闭时角度模式降为速度模式
 * - gimbal_sine_test_enabled：默认 false，闭环后以当前 yaw 为中心生成振幅 30°、相位速度 6 rad/s 的正弦轨迹
 * - gimbal_sine_feedforward_enabled：默认 true，控制正弦测试是否同时提供解析角速度前馈
 *   正弦测试仍受失联停机约束，调试命令使能后由调试命令覆盖，编译开关关闭时不提供这两个变量
 * 调试命令只绕过遥控器使能，反馈、配置、故障状态、机械边界和输出闸门仍由云台模块处理
 */
extern Gimbal_t gimbal;
extern Gimbal_Command_t gimbal_command;
extern volatile bool gimbal_debug_yaw_target_enabled;
extern volatile float gimbal_debug_yaw_target_deg;
extern volatile bool gimbal_debug_yaw_speed_enabled;
extern volatile float gimbal_debug_yaw_speed_target_rad_s;
extern volatile bool gimbal_debug_pitch_target_enabled;
extern volatile float gimbal_debug_pitch_target_deg;
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
