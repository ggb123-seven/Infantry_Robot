#pragma once

#ifdef __cplusplus
extern "C"
{
#endif

#include "module/trigger.h"

/*
 * 拨弹调试入口：trigger 为模块实例，config 可调、其他字段由拨弹任务更新
 * trigger_command 为本周期已消费的命令快照，不作为跨任务裸全局命令入口
 * 外部命令必须周期写入 task_runtime.msgq.trigger_command，缺新命令时失能
 */
extern Trigger_t trigger;
extern Trigger_Command_t trigger_command;

/**
 * @brief 周期编排拨弹反馈、速度控制与电流邮箱发布
 *
 * @param[in] argument 任务参数，本任务不使用
 * @return 本任务不会返回
 */
void Task_motor_trigger(void *argument);

#ifdef __cplusplus
}
#endif
