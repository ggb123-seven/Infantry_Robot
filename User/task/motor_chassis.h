#pragma once

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>

#include "module/chassis.h"
#include "module/fault_detect.h"

/*
 * 底盘在线调参：
 * - scale_rpm：归一化轮速尺度，单位 rpm，模块内限制到 CHASSIS_SPEED_LIMIT_RPM
 * - pid：ID1~3 公共速度 PID 参数，ID4 使用 chassis.h 固化参数
 */
typedef struct
{
    float scale_rpm;
    Chassis_PidTune_t pid;
} MotorChassis_Tune_t;

/*
 * 底盘调试观测变量，由 Task_motor_chassis 每周期写入：
 * - motor_chassis_tune：在线调参，调试器可直接修改
 * - chassis_snapshot：运动学目标、反馈转速、电流和逐路控制状态
 * - fault_detect_snapshot：本周期故障检测结果
 */
extern volatile MotorChassis_Tune_t motor_chassis_tune;
extern Chassis_Snapshot_t chassis_snapshot;
extern FaultDetect_Snapshot_t fault_detect_snapshot;

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
