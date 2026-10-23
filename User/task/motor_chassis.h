#pragma once

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>

#include "module/chassis.h"
#include "module/fault_detect.h"

/*
 * 底盘在线调参与转速观测，数组下标 0~3 对应电调 ID1~4：
 * - actual_speed_rpm：四路输出轴反馈转速，单位 rpm，由任务每周期刷新，仅供观测
 * - target_speed_rpm：四路限幅和缓启动后实际进入速度环的输出轴目标转速，单位 rpm，仅供观测
 * - pid：ID1~3 公共速度 PID 参数，ID4 使用 chassis.h 固化参数
 */
typedef struct
{
    float actual_speed_rpm[CHASSIS_MOTOR_COUNT];
    float target_speed_rpm[CHASSIS_MOTOR_COUNT];
    Chassis_PidTune_t pid;
} MotorChassis_Tune_t;

/*
 * 底盘调试观测变量，由 Task_motor_chassis 每周期写入：
 * - motor_chassis_watch：调试器可修改 PID 参数，反馈与目标转速由任务刷新
 * - chassis_snapshot：运动学目标、反馈转速、电流和逐路控制状态
 * - fault_detect_snapshot：本周期故障检测结果
 */
extern volatile MotorChassis_Tune_t motor_chassis_watch;
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
