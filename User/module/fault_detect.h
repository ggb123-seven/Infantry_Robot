#pragma once

#ifdef __cplusplus
extern "C"
{
#endif

#include "device/can_devices.h"
#include "device/dr16.h"
#include "module/chassis.h"

#include <stdbool.h>
#include <stdint.h>

#define FAULT_DETECT_MOTOR_COUNT (CHASSIS_MOTOR_COUNT)

/*
 * 故障检测系统级故障位：
 * - CAN_INIT：CAN 总线初始化或固定设备注册聚合状态异常。
 * - CAN_FEEDBACK：本周期 CAN 反馈聚合状态异常。
 * - CAN_OUTPUT：本周期电流命令写入聚合状态异常。
 * - CAN_TX：本周期统一 CAN 控制帧发送异常。
 * - CHASSIS_INIT：底盘速度控制器初始化聚合状态异常。
 * - CHASSIS_CONTROL：本周期底盘四路控制聚合状态异常。
 * - CONFIG：底盘固定参数或在线控制参数异常。
 * - FEEDBACK_MISSING：本周期没有取得可诊断的 CAN 反馈快照。
 * - MOTOR：至少一路电机存在故障位。
 */
typedef enum
{
    FAULT_DETECT_SYSTEM_NONE = 0U,
    FAULT_DETECT_SYSTEM_CAN_INIT = 1U << 0,
    FAULT_DETECT_SYSTEM_CAN_FEEDBACK = 1U << 1,
    FAULT_DETECT_SYSTEM_CAN_OUTPUT = 1U << 2,
    FAULT_DETECT_SYSTEM_CAN_TX = 1U << 3,
    FAULT_DETECT_SYSTEM_CHASSIS_INIT = 1U << 4,
    FAULT_DETECT_SYSTEM_CHASSIS_CONTROL = 1U << 5,
    FAULT_DETECT_SYSTEM_CONFIG = 1U << 6,
    FAULT_DETECT_SYSTEM_FEEDBACK_MISSING = 1U << 7,
    FAULT_DETECT_SYSTEM_MOTOR = 1U << 8,
} FaultDetect_SystemFaultFlags_t;

/*
 * 故障检测单电机故障位：
 * - REGISTER：对应 C620/M3508 注册失败或设备实例不可用。
 * - OFFLINE：对应电机在线标志为 false。
 * - FEEDBACK：对应电机本周期反馈更新失败或缺少 CAN 快照。
 * - CURRENT：对应电机电流命令校验、缓存写入或安全归零路径异常。
 * - CONTROL_INIT：对应速度控制器初始化失败。
 * - CONTROL：对应速度控制器本周期运行异常，不包含主动安全禁用。
 * - CONFIG：对应速度控制器固定参数或在线 PID 参数异常。
 * - VALUE：对应反馈、命令或控制周期存在非有限值等非法数值。
 */
typedef enum
{
    FAULT_DETECT_MOTOR_NONE = 0U,
    FAULT_DETECT_MOTOR_REGISTER = 1U << 0,
    FAULT_DETECT_MOTOR_OFFLINE = 1U << 1,
    FAULT_DETECT_MOTOR_FEEDBACK = 1U << 2,
    FAULT_DETECT_MOTOR_CURRENT = 1U << 3,
    FAULT_DETECT_MOTOR_CONTROL_INIT = 1U << 4,
    FAULT_DETECT_MOTOR_CONTROL = 1U << 5,
    FAULT_DETECT_MOTOR_CONFIG = 1U << 6,
    FAULT_DETECT_MOTOR_VALUE = 1U << 7,
} FaultDetect_MotorFaultFlags_t;

/*
 * DR16 故障位：
 * - OFFLINE：接收器当前没有处于在线状态
 * - STREAM：UART 字节流当前没有运行
 * - DATA：DR16 状态缺失或在线状态与合法帧序号不一致
 */
typedef enum
{
    FAULT_DETECT_DR16_NONE = 0U,
    FAULT_DETECT_DR16_OFFLINE = 1U << 0,
    FAULT_DETECT_DR16_STREAM = 1U << 1,
    FAULT_DETECT_DR16_DATA = 1U << 2,
} FaultDetect_DR16FaultFlags_t;

/*
 * DR16 故障检测快照：
 * - evaluated：至少完成过一次诊断更新时为 true
 * - has_fault：存在任意 DR16 故障位时为 true
 * - fault_flags：当前 DR16 故障位，取值见 FaultDetect_DR16FaultFlags_t
 */
typedef struct
{
    bool evaluated;
    bool has_fault;
    uint32_t fault_flags;
} FaultDetect_DR16Snapshot_t;

/*
 * 故障检测快照：
 * - evaluated：至少完成过一次诊断更新时为 true。
 * - has_fault：系统级或任一电机存在故障位时为 true。
 * - system_fault_flags：系统级聚合故障位，取值见 FaultDetect_SystemFaultFlags_t。
 * - motor_fault_flags[0~3]：逐电机故障位，依次对应 C620 电调 ID 1~4。
 * - motor_fault_count：存在任意故障位的电机数量。
 * - offline_motor_count：被判定为离线的电机数量。
 * - feedback_fault_count：反馈缺失、反馈更新失败或反馈非法的电机数量。
 * - config_fault_count：固定配置或在线控制参数异常的电机数量。
 * - value_fault_count：反馈、命令或控制周期非法数值的电机数量。
 */
typedef struct
{
    bool evaluated;
    bool has_fault;
    uint32_t system_fault_flags;
    uint32_t motor_fault_flags[FAULT_DETECT_MOTOR_COUNT];
    uint8_t motor_fault_count;
    uint8_t offline_motor_count;
    uint8_t feedback_fault_count;
    uint8_t config_fault_count;
    uint8_t value_fault_count;
} FaultDetect_Snapshot_t;

/**
 * @brief 汇总底盘 CAN 设备与速度控制快照中的故障状态
 *
 * @param[in] can_snapshot 本周期 CAN 设备快照，允许为 NULL
 * @param[in] chassis_snapshot 本周期底盘控制快照，允许为 NULL
 * @param[out] fault_snapshot 待写入的故障检测快照
 * @return 无返回值
 */
void FaultDetect_UpdateMotorChassis(const CANDevices_Snapshot_t *can_snapshot,
                                    const Chassis_Snapshot_t *chassis_snapshot,
                                    FaultDetect_Snapshot_t *fault_snapshot);

/**
 * @brief 汇总 DR16 在线状态与 UART 字节流状态
 *
 * @param[in] dr16_state DR16 接收器状态快照，允许为 NULL
 * @param[in] stream_running UART 字节流当前运行时为 true
 * @param[out] fault_snapshot 待写入的 DR16 故障检测快照
 * @return 无返回值
 */
void FaultDetect_UpdateDR16(const DR16_State_t *dr16_state, bool stream_running,
                            FaultDetect_DR16Snapshot_t *fault_snapshot);

#ifdef __cplusplus
}
#endif
