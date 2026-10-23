#pragma once

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

#ifdef __cplusplus
}
#endif
