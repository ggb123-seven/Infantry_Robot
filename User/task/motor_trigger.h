#pragma once

#ifdef __cplusplus
extern "C"
{
#endif

#include "device/dr16.h"
#include "module/feeder.h"

/*
 * 拨弹调试入口：feeder 连发走速度环、单发走逐齿到位串级，config 可调，运行使能由遥控命令生成
 * feeder.config 为可写副本，max_speed_rpm 为输出轴转速上限，feed_hz 限制单发重复的启动频率
 * feeder.config.position_pid 为角度外环参数，输入弧度、输出 rad/s；speed_pid 为速度内环参数，输入 rpm、输出 A
 * feeder_fire_mode 仅观测命令层采用的模式：0 连发速度环、1 逐发串级，不再接受 EmberProbe 模式写入
 * 右拨杆下位后左拨杆切上位选择单发，须先右摇杆回中再向上推到端部，长推到位后重复，松杆完成当前齿
 * feeder_angle_target_deg 仅观测逐发目标角，不再接收手动目标；每齿由 tooth_count 决定，当前为 45 度
 * 到位为输出轴误差不超过 0.5 度且绝对转速不超过 1 rpm 持续 20 毫秒，执行超过 1 秒锁存故障
 * feeder.state 为唯一运行状态，0 待机、1 运行、2 反馈离线、3 故障锁存，仅运行状态输出电流
 * feeder.target_angle_deg 为本周期采用的角度目标，measured_angle_deg 为输出轴单圈角度，两者单位均为度
 * feeder.target_speed_rpm 为速度内环目标，串级时来自角度外环输出，纯速度环时来自摇杆倍率映射
 * feeder.measured_speed_rpm 为实测输出轴转速，单位 rpm
 * feeder.output_current_a 为限幅后的候选电流，非运行状态由输出闸门归零
 * 角度不连续或超时超过 10 ms 时角度观察量为非数值，帧超时转速为非数值，单发恢复后捕获当前位置并等待回中
 * feeder_command 为命令层本周期的拨弹命令，右拨杆下位且遥控新鲜时才可能进入运行档
 * trigger_remote 为最近消费的遥控快照，仅用于观测，不作为跨任务裸全局命令入口
 * task_runtime.msgq.trigger_command 传递 DR16_State_t，空邮箱保留缓存，源时间超时后撤驱
 * 周期快照、反馈源时间和 PID 历史保留在模块内部，不作为调试参数入口
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
