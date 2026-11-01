#pragma once

#ifdef __cplusplus
extern "C"
{
#endif

#include "device/dr16.h"
#include "module/feeder.h"

/*
 * 拨弹调试入口：feeder 当前仅运行速度环，config 可调，速度目标由遥控命令生成
 * feeder.config 为可写副本，max_speed_rpm 既是输出轴转速上限，也是归一化比例 1 对应的转速
 * feeder.speed_pid.target_speed_rpm 为命令比例换算并限幅后的目标，measured_speed_rpm 为实测转速，反馈无效时为 NaN
 * feeder.speed_pid.controller 保存 PID 控制历史，调参仍使用 feeder.config.speed_pid，其他字段由任务更新
 * feeder.cycle_config 为本周期快照，两环长期引用该快照，速度 PID 输入输出轴 rpm、输出 A
 * measured_angle_deg、target_angle_deg 为实际和目标角度观察值，内部位置运算使用弧度
 * measured_speed_deg_s、target_speed_deg_s 为角速度观察值，前馈系数在 config 内修改
 * trigger_remote 为最近消费的遥控快照，仅用于观测，不作为跨任务裸全局命令入口
 * feeder_command 为命令层本周期的拨弹命令，右拨杆下位且遥控新鲜时才可能进入运行档
 * task_runtime.msgq.trigger_command 传递 DR16_State_t，空邮箱保留缓存，源时间超时后撤驱
 */
extern Feeder_t feeder;
extern DR16_State_t trigger_remote;
extern Feeder_Command_t feeder_command;

/**
 * @brief 周期编排拨弹遥控、反馈、速度控制与电流邮箱发布
 *
 * @param[in] argument 任务参数，本任务不使用
 * @return 本任务不会返回
 */
void Task_motor_trigger(void *argument);

#ifdef __cplusplus
}
#endif
