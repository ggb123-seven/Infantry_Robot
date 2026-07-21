#pragma once

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>

/**
 * @brief 初始化底盘任务私有速度控制器
 *
 * @return 四路速度控制器可运行时返回 true，否则返回 false
 */
bool Task_motor_chassis_Init(void);

/**
 * @brief 运行四个 M3508 电机速度控制任务
 *
 * @param[in] argument 任务参数，本任务不使用
 * @return 本任务不会返回
 */
void Task_motor_chassis(void *argument);

#ifdef __cplusplus
}
#endif
