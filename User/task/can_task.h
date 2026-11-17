/*
 * CAN 任务接口
 *
 * 对外暴露设备观测快照以及初始化与周期任务入口
 */

#pragma once

#ifdef __cplusplus
extern "C"
{
#endif

/* Includes ----------------------------------------------------------------- */
#include <stdbool.h>

#include "device/can_devices.h"

/* CAN 设备观测快照，由 Task_can 每周期写入 */
extern CANDevices_Snapshot_t can_devices_snapshot;

/* Exported functions ------------------------------------------------------- */
/**
 * @brief 初始化 CAN 设备集合
 *
 * @return bool 总线可运行时返回 true，否则返回 false
 */
bool Task_can_Init(void);

/**
 * @brief 运行 CAN 设备反馈采集和电流命令发送任务
 *
 * @param argument 任务参数，本任务不使用
 * @return 无返回值
 */
void Task_can(void *argument);

#ifdef __cplusplus
}
#endif
