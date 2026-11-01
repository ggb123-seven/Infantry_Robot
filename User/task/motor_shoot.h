#pragma once

#ifdef __cplusplus
extern "C"
{
#endif

#include "module/shoot.h"

/*
 * 摩擦轮调试入口：shoot 为双轮模块实例，config 可调、其他字段由摩擦轮任务更新
 * shoot.config.speed_pid 使用转子 rpm 和电流 A，初始增益沿用底盘，输出限幅保留摩擦轮电流边界
 * shoot_speed_feedforward_gain_a_per_rpm 为目标逐周期差分前馈系数，零表示关闭
 * shoot_command 为本周期已消费的命令快照，不作为跨任务裸全局命令入口
 * 外部命令必须周期写入 task_runtime.msgq.shoot_command，缺新命令时两轮失能
 */
extern Shoot_t shoot;
extern Shoot_Command_t shoot_command;

/**
 * @brief 周期编排双摩擦轮反馈、速度控制与电流邮箱发布
 *
 * @param[in] argument 任务参数，本任务不使用
 * @return 本任务不会返回
 */
void Task_motor_shoot(void *argument);

#ifdef __cplusplus
}
#endif
