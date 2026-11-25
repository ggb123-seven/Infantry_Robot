#pragma once

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>

#include "device/can_devices.h"

// CAN 设备观测变量：初始化结果、四路反馈和电流提交状态，由 Task_can 每周期写入
extern CANDevices_Snapshot_t can_devices_snapshot;

/**
 * @brief 初始化 CAN 设备集合
 *
 * @return CAN 总线可运行时返回 true，否则返回 false
 */
bool Task_can_Init(void);

/**
 * @brief 运行 CAN 设备反馈采集和电流命令发送任务
 *
 * @param[in] argument 任务参数，本任务不使用
 * @return 本任务不会返回
 */
void Task_can(void *argument);

#ifdef __cplusplus
}
#endif
