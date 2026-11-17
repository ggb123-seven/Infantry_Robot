/*
 * 底盘任务接口
 *
 * 对外暴露底盘实例、运动请求和架空测速调试变量
 */

#pragma once

#ifdef __cplusplus
extern "C"
{
#endif

#include "module/chassis.h"
#include "module/fault_detect.h"

/* 底盘实例，config 可在运行中修改，运行状态由本任务独占写入 */
extern Chassis_t chassis;
/* 每周期由 DR16 覆盖的运动请求，仅供观测当前控制输入 */
extern Chassis_Command_t chassis_command;
/* 本周期设备与底盘控制故障汇总 */
extern FaultDetect_Snapshot_t fault_detect_snapshot;
/* 架空测速使能，启动为零，仅值为 1 时接管遥控有效自旋档 */
extern volatile uint32_t chassis_debug_speed_enabled;
/* 四轮同向输出轴转速，单位 rpm，启动为零 */
extern volatile float chassis_debug_speed_rpm;

/* Exported functions ------------------------------------------------------- */
/**
 * @brief 初始化底盘模块并周期执行反馈读取、控制计算和电流发布
 *
 * @param argument 任务参数，本任务不使用
 * @return 无返回值
 */
void Task_motor_chassis(void *argument);

#ifdef __cplusplus
}
#endif
