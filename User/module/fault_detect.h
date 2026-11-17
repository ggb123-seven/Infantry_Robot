/*
 * 故障检测模块对外接口
 *
 * 汇合 CAN 设备快照、底盘实例与 DR16 状态，按位域生成系统级、逐电机和链路故障快照
 */

#pragma once

#ifdef __cplusplus
extern "C"
{
#endif

/* Includes ----------------------------------------------------------------- */
#include "device/can_devices.h"
#include "device/dr16.h"
#include "module/chassis.h"

#include <stdbool.h>
#include <stdint.h>

/* Exported constants ------------------------------------------------------- */
#define FAULT_DETECT_MOTOR_COUNT (CHASSIS_WHEEL_COUNT) /* 参与故障检测的电机数量 */

/* Exported types ----------------------------------------------------------- */
/* 系统级故障位 */
typedef enum
{
    FAULT_DETECT_SYSTEM_NONE = 0U,                  /* 无系统级故障 */
    FAULT_DETECT_SYSTEM_CAN_INIT = 1U << 0,         /* CAN 初始化或设备注册聚合异常 */
    FAULT_DETECT_SYSTEM_CAN_FEEDBACK = 1U << 1,     /* 本周期 CAN 反馈聚合异常 */
    FAULT_DETECT_SYSTEM_CAN_OUTPUT = 1U << 2,       /* 本周期电流命令写入聚合异常 */
    FAULT_DETECT_SYSTEM_CAN_TX = 1U << 3,           /* 本周期统一控制帧发送异常 */
    FAULT_DETECT_SYSTEM_CHASSIS_INIT = 1U << 4,     /* 底盘实例尚未初始化成功 */
    FAULT_DETECT_SYSTEM_CHASSIS_CONTROL = 1U << 5,  /* 非待机模式下至少一轮反馈离线 */
    FAULT_DETECT_SYSTEM_CONFIG = 1U << 6,           /* 本周期底盘配置校验失败 */
    FAULT_DETECT_SYSTEM_FEEDBACK_MISSING = 1U << 7, /* 本周期无可诊断的 CAN 反馈快照 */
    FAULT_DETECT_SYSTEM_MOTOR = 1U << 8,            /* 至少一路电机存在故障位 */
} FaultDetect_SystemFaultFlags_t;

/* 单电机故障位 */
typedef enum
{
    FAULT_DETECT_MOTOR_NONE = 0U,              /* 无电机故障 */
    FAULT_DETECT_MOTOR_REGISTER = 1U << 0,     /* C620 注册失败或设备实例不可用 */
    FAULT_DETECT_MOTOR_OFFLINE = 1U << 1,      /* 电机在线标志为 false */
    FAULT_DETECT_MOTOR_FEEDBACK = 1U << 2,     /* 反馈更新失败或缺少 CAN 快照 */
    FAULT_DETECT_MOTOR_CURRENT = 1U << 3,      /* 电流命令校验、写入或归零异常 */
    FAULT_DETECT_MOTOR_CONTROL_INIT = 1U << 4, /* 底盘实例尚未初始化成功 */
    FAULT_DETECT_MOTOR_CONTROL = 1U << 5,      /* 非待机模式下对应轮离线 */
    FAULT_DETECT_MOTOR_CONFIG = 1U << 6,       /* 本周期底盘配置校验失败 */
    FAULT_DETECT_MOTOR_VALUE = 1U << 7,        /* 仅由 CAN 侧非法电流命令判定 */
} FaultDetect_MotorFaultFlags_t;

/* DR16 故障位 */
typedef enum
{
    FAULT_DETECT_DR16_NONE = 0U,         /* 无链路故障 */
    FAULT_DETECT_DR16_OFFLINE = 1U << 0, /* 接收器当前不在线 */
    FAULT_DETECT_DR16_STREAM = 1U << 1,  /* UART 字节流当前未运行 */
    FAULT_DETECT_DR16_DATA = 1U << 2,    /* 状态缺失或在线状态与帧序号不一致 */
} FaultDetect_DR16FaultFlags_t;

/* DR16 故障检测快照 */
typedef struct
{
    bool evaluated;       /* 至少完成过一次诊断更新 */
    bool has_fault;       /* 存在任意 DR16 故障位 */
    uint32_t fault_flags; /* 当前 DR16 故障位 */
} FaultDetect_DR16Snapshot_t;

/* 故障检测快照 */
typedef struct
{
    bool evaluated;                                       /* 至少完成过一次诊断更新 */
    bool has_fault;                                       /* 系统级或任一电机存在故障位 */
    uint32_t system_fault_flags;                          /* 系统级聚合故障位 */
    uint32_t motor_fault_flags[FAULT_DETECT_MOTOR_COUNT]; /* 逐电机故障位，对应 C620 电调 ID 1~4 */
    uint8_t motor_fault_count;                            /* 存在故障位的电机数量 */
    uint8_t offline_motor_count;                          /* 离线电机数量 */
    uint8_t feedback_fault_count;                         /* 反馈类故障电机数量 */
    uint8_t config_fault_count;                           /* 配置类故障电机数量 */
    uint8_t value_fault_count;                            /* 非法数值故障电机数量 */
} FaultDetect_Snapshot_t;

/* Exported functions ------------------------------------------------------- */
/**
 * @brief 汇总底盘 CAN 设备快照与控制实例中的故障状态
 *
 * @param can_snapshot 本周期 CAN 设备快照
 * @param chassis 本周期底盘控制实例
 * @param fault_snapshot 待写入的故障检测快照
 * @return 无返回值
 */
void FaultDetect_UpdateMotorChassis(const CANDevices_Snapshot_t *can_snapshot,
                                    const Chassis_t *chassis,
                                    FaultDetect_Snapshot_t *fault_snapshot);

/**
 * @brief 汇总 DR16 在线状态与 UART 字节流状态
 *
 * @param dr16_state DR16 接收器状态快照
 * @param stream_running UART 字节流当前运行时为 true
 * @param fault_snapshot 待写入的 DR16 故障检测快照
 * @return 无返回值
 */
void FaultDetect_UpdateDR16(const DR16_State_t *dr16_state, bool stream_running,
                            FaultDetect_DR16Snapshot_t *fault_snapshot);

#ifdef __cplusplus
}
#endif
