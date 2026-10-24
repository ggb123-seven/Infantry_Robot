#pragma once

#ifdef __cplusplus
extern "C"
{
#endif

#include "module/chassis.h"
#include "module/fault_detect.h"

/*
 * 底盘调试入口，由 Task_motor_chassis 独占运行状态写入：
 * - chassis.config.wheel_speed_pid：调试器在线修改四轮共用 PID 参数
 * - chassis.wheel[0~3]：电调 ID1~4 实测、目标、输出电流及单轮状态
 * - chassis_command：每周期由 DR16 覆盖，仅供观测当前控制请求
 * - fault_detect_snapshot：本周期设备与底盘控制故障汇总
 */
extern Chassis_t chassis;
extern Chassis_Command_t chassis_command;
extern FaultDetect_Snapshot_t fault_detect_snapshot;

/**
 * @brief 初始化底盘模块并周期执行反馈读取、控制计算和电流发布
 *
 * @param[in] argument 任务参数，本任务不使用
 * @return 本任务不会返回
 */
void Task_motor_chassis(void *argument);

#ifdef __cplusplus
}
#endif
