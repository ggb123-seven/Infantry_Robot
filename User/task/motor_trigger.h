#pragma once

#ifdef __cplusplus
extern "C"
{
#endif

#include "module/feeder.h"

/*
 * 拨弹调试入口：feeder 为逐齿位置环实例，config 可调，其他字段由拨弹任务更新
 * feeder.cycle_config 为本周期快照，位置 PID 长期引用该快照，不读取配置源文件
 * trigger_remote 为最近消费的遥控快照，仅用于观测，不作为跨任务裸全局命令入口
 * task_runtime.msgq.trigger_command 传递 DR16_State_t，空邮箱保留缓存，源时间超时后撤驱
 */
extern Feeder_t feeder;
extern DR16_State_t trigger_remote;

/**
 * @brief 周期编排拨弹遥控、反馈、逐齿位置控制与电流邮箱发布
 *
 * @param[in] argument 任务参数，本任务不使用
 * @return 本任务不会返回
 */
void Task_motor_trigger(void *argument);

#ifdef __cplusplus
}
#endif
