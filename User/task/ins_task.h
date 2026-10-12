#pragma once

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>

#include "module/ins.h"

extern INS_t INS;

/**
 * @brief 初始化 INS 任务使用的 BMI088、姿态滤波器和温控模块
 *
 * @return INS 模块进入运行状态时返回 true，否则返回 false
 */
bool INS_Task_Init(void);

/**
 * @brief 以 1000 Hz 采集 BMI088 并更新 INS 姿态解算
 *
 * @param[in] argument 任务参数，本任务不使用
 * @return 本任务不会返回
 */
void INS_Task(void *argument);

#ifdef __cplusplus
}
#endif
