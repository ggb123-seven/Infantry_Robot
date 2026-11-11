#pragma once

#ifdef __cplusplus
extern "C"
{
#endif

#include "device/dr16.h"
#include "module/feeder.h"

/*
 * 拨弹调试入口：feeder.config 可调，遥控命令生成运行状态，环路可由下方两个使能量在线选择
 * feeder.config 为可写副本，max_speed_rpm 为输出轴转速上限，feed_hz 限制五连发组内的启动频率
 * feeder_angle_loop_enabled 默认 true，feeder_speed_loop_enabled 默认 false
 * 单发和五连发在角度环开、速度环关时由角度 PID 直接输出 A；两环同开时恢复位置 rad/s 到速度 rpm 再到电流 A 的串级
 * 角度 PID 参数的输出单位随模式变化，独立角度模式有效限幅为 min(out_limit, max_current_a)，串级限幅仍按 rad/s 配置
 * speed_pid 输入 rpm、输出 A；连续进给要求速度环使能，速度环关闭时连续档撤驱
 * feeder_remote_fire_mode 为可写遥控模式：0 持续连发、1 单发、2 五连发，默认 2，切换时取消旧组并重新计发
 * feeder_fire_mode 仅观测本周期采用的模式，不能作为调试写入接口
 * 右拨杆上位安全、中位备弹、下位开火，与左拨杆及右摇杆解耦
 * 单发和五连发完成后撤驱且持续下位不重复，回中取消剩余并撤驱，下次下位重新计发
 * feeder_angle_target_deg 仅观测逐发目标角，不再接收手动目标；每齿由 tooth_count 决定，当前为 45 度
 * 到位为输出轴误差不超过 0.5 度且绝对转速不超过 1 rpm 持续 20 毫秒，未到位时持续追踪目标
 * feeder.state 为唯一运行状态，0 待机、1 运行、2 反馈离线、3 故障锁存，仅运行状态输出电流
 * feeder.target_angle_deg 为本周期采用的角度目标，measured_angle_deg 为输出轴单圈角度，两者单位均为度
 * feeder.target_speed_rpm 为速度内环目标，单角度环固定为零，串级与纯速度环分别来自外环换算和命令倍率
 * feeder.measured_speed_rpm 为实测输出轴转速，单位 rpm
 * feeder.output_current_a 为限幅后的候选电流，非运行状态由输出闸门归零
 * 角度无效或反馈超过 10 毫秒未更新时撤驱，未完成的组恢复后从当前位置重新计发，完成状态持续保留
 * feeder_command 为命令层本周期的拨弹命令，右拨杆中位或下位且遥控新鲜时才可能进入运行档
 * trigger_remote 为最近消费的遥控快照，仅用于观测，不作为跨任务裸全局命令入口
 * task_runtime.msgq.trigger_command 传递 DR16_State_t，空邮箱保留缓存，源时间超时后撤驱
 * 周期快照、反馈源时间和 PID 历史保留在模块内部，不作为调试参数入口
 */
extern volatile uint8_t feeder_remote_fire_mode;
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
