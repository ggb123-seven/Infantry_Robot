#pragma once

#include <stdbool.h>

#ifdef __cplusplus
extern "C"
{
#endif

/**
 * @brief 周期消费 yaw 反馈、调用速度环模块并发布电流命令
 *
 * @param[in] argument 任务参数，本任务不使用
 * @return 本任务不会返回
 */
void Task_motor_gimbal(void *argument);

/**
 * @brief 初始化云台 yaw 速度环并校验内核节拍
 *
 * @return 控制器初始化和周期校验成功时返回 true，否则返回 false
 */
bool Task_motor_gimbal_Init(void);

#ifdef __cplusplus
}
#endif
