#pragma once

#include <stdbool.h>

#ifdef __cplusplus
extern "C"
{
#endif

/**
 * @brief 启动并周期调度云台模块，反馈处理和电流发布由模块完成
 *
 * @param[in] argument 任务参数，本任务不使用
 * @return 本任务不会返回
 */
void Task_motor_gimbal(void *argument);

/**
 * @brief 初始化云台 yaw 角度与速度串级控制并校验内核节拍
 *
 * @return 控制器初始化和周期校验成功时返回 true，否则返回 false
 */
bool Task_motor_gimbal_Init(void);

#ifdef __cplusplus
}
#endif
