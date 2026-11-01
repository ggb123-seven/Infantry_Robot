#pragma once

#ifdef __cplusplus
extern "C"
{
#endif

#include "module/feeder.h"

/*
 * 拨弹调试入口：feeder 当前仅运行速度环，config 与 requested_speed_rpm 可写，其他字段由拨弹任务更新
 * feeder.requested_speed_rpm 为直接速度请求，单位输出轴 rpm，上电为零，右拨杆下位使能
 * feeder.target_speed_rpm 为限幅后目标，feeder.measured_speed_rpm 为实测转速，反馈无效时为 NaN
 * feeder.cycle_config 为本周期快照，两环长期引用该快照，速度 PID 输入输出轴 rpm、输出 A
 * measured_angle_deg、target_angle_deg 为实际和目标角度观察值，内部位置运算使用弧度
 * measured_speed_deg_s、target_speed_deg_s 为角速度观察值，前馈系数在 config 内修改
 * trigger_remote 为最近消费的遥控快照，仅用于观测，不作为跨任务裸全局命令入口
 * task_runtime.msgq.trigger_command 传递 DR16_State_t，空邮箱保留缓存，源时间超时后撤驱
 */
extern Feeder_t feeder;
extern DR16_State_t trigger_remote;

/**
 * @brief 周期编排拨弹遥控、反馈、速度调试与电流邮箱发布
 *
 * @param[in] argument 任务参数，本任务不使用
 * @return 本任务不会返回
 */
void Task_motor_trigger(void *argument);

#ifdef __cplusplus
}
#endif
