/*
 * 云台任务接口
 *
 * 对外暴露云台实例、命令和调试开关，供在线观测与手动控制
 */

#pragma once

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>

#include "module/gimbal.h"

/*
 * 调试说明：gimbal_debug_enabled 为 true 时由 EmberProbe 控制 gimbal_command
 * mode 取 0 停机、1 速度环、2 角度与速度串级
 * 速度目标写 rate_rad_s，固定角度写 angle_deg；遥控在线默认两轴 ANGLE，失联或异常时 RELAX
 */
/* 云台实例，config 可在运行中修改 */
extern Gimbal_t gimbal;
/* 遥控每周期更新，调试模式由 EmberProbe 写入 */
extern Gimbal_Command_t gimbal_command;
/* 为 true 时跳过遥控输入，由 gimbal_command 直接控制 */
extern volatile bool gimbal_debug_enabled;
/* 遥控速度环使能，仅遥控链路生效 */
extern volatile bool gimbal_remote_speed_loop_enabled;
/* 遥控角度环使能，仅遥控链路生效 */
extern volatile bool gimbal_remote_angle_loop_enabled;
#if MOTOR_GIMBAL_ENABLE_SINE_TEST
/* 正弦测试及其角速度前馈使能 */
extern volatile bool gimbal_sine_test_enabled;
extern volatile bool gimbal_sine_feedforward_enabled;
#endif

/* Exported functions ------------------------------------------------------- */
/**
 * @brief 初始化云台模块并周期执行反馈读取、控制计算和电流发布
 *
 * @param argument 任务参数，本任务不使用
 * @return 无返回值
 */
void Task_motor_gimbal(void *argument);

#ifdef __cplusplus
}
#endif
