#pragma once

#ifdef __cplusplus
extern "C"
{
#endif

#include "device/dr16.h"
#include "module/feeder.h"

/*
 * 拨弹调试入口：feeder 运行逐齿角度外环与速度内环串级，config 可调，进给节拍由遥控命令生成
 * feeder.config 为可写副本，max_speed_rpm 为输出轴转速上限，feed_hz 为满倍率下的每秒进给齿数
 * feeder.cycle_config 为本周期集中校验的持久快照，position_pid 输入弧度输出 rad/s，speed_pid 输入 rpm 输出 A
 * feeder.state 为唯一运行状态，0 待机、1 串级运行、2 反馈离线、3 故障锁存，仅运行状态输出电流
 * feeder.target_angle_rad、measured_angle_rad 为角度外环目标与实际角度，进给倍率与 feed_hz 共同决定推进节拍
 * feeder.target_angle_deg、measured_angle_deg 为同两组角度的度数镜像，仅用于观察
 * feeder.target_speed_rpm 为角度外环输出换算并限幅后的速度目标，measured_speed_rpm 为实测转速
 * feeder.measured_speed_rad_s 为角度外环微分输入，feeder.phase_s 为距下一齿的累计秒数
 * feeder.output_current_a 为限幅后的候选电流，非运行状态由输出闸门归零
 * 反馈无效或超时超过 10 ms 时角度与转速观察量均为非数值，恢复后按当前齿位重新捕获目标
 * feeder_command 为命令层本周期的拨弹命令，右拨杆下位且遥控新鲜时才可能进入运行档
 * trigger_remote 为最近消费的遥控快照，仅用于观测，不作为跨任务裸全局命令入口
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
