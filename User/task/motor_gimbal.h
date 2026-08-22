#pragma once

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>

/**
 * @brief 初始化云台 GM6020 电流测试任务
 *
 * @return 初始化成功返回 true
 */
bool Task_motor_gimbal_Init(void);

/**
 * @brief 周期读取 GM6020 反馈并发布电流命令
 *
 * @param[in] argument 任务参数，本任务不使用
 * @return 本任务不会返回
 */
void Task_motor_gimbal(void *argument);

#ifdef __cplusplus
}
#endif
