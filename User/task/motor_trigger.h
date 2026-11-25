/*
 * 拨弹任务接口
 *
 * 对外暴露拨弹实例、遥控快照与进给模式入口
 */

#pragma once

#ifdef __cplusplus
extern "C"
{
#endif

#include "device/dr16.h"
#include "module/feeder.h"

/*
 * 右拨杆上位安全、中位备弹、下位开火，与左拨杆及右摇杆解耦
 * 单发和五连发在角度环开、速度环关时由角度 PID 直接输出电流
 * 单发和五连发完成后撤驱，回中取消剩余并撤驱，下次下位重新计发
 */
/* 遥控进给模式：0 连发、1 单发、2 五连发 */
extern volatile uint8_t feeder_remote_fire_mode;
/* 拨弹实例，config 可在运行中修改，其余字段由本任务更新 */
extern Feeder_t feeder;
/* 最近消费的遥控快照，仅用于观测 */
extern DR16_State_t trigger_remote;
/* 本周期拨弹命令 */
extern Feeder_Command_t feeder_command;

/* Exported functions ------------------------------------------------------- */
/**
 * @brief 周期编排拨弹遥控、反馈、速度控制与电流邮箱发布
 *
 * @param argument 任务参数，本任务不使用
 * @return 无返回值
 */
void Task_motor_trigger(void *argument);

#ifdef __cplusplus
}
#endif
