/*
 * 双摩擦轮任务接口
 *
 * 对外暴露摩擦轮实例与命令，周期任务消费反馈并发布电流
 */

#pragma once

#ifdef __cplusplus
extern "C"
{
#endif

#include "module/shoot.h"

/*
 * shoot.config 可在运行中修改，命令邮箱超过 100 ms 无新命令时两轮失能
 * 本任务只负责周期编排，电流输出由模块唯一闸门决定
 */
/* 双摩擦轮实例，config 可在运行中修改，其余字段由本任务更新 */
extern Shoot_t shoot;
/* 本周期实际提交的摩擦轮命令，不作为跨任务裸全局入口 */
extern Shoot_Command_t shoot_command;

/* Exported functions ------------------------------------------------------- */
/**
 * @brief 周期编排双摩擦轮反馈、速度控制与电流邮箱发布
 *
 * @param argument 任务参数，本任务不使用
 * @return 无返回值
 */
void Task_motor_shoot(void *argument);

#ifdef __cplusplus
}
#endif
