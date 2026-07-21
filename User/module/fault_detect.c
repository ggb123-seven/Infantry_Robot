#include "module/fault_detect.h"

#include <stddef.h>

_Static_assert(FAULT_DETECT_MOTOR_COUNT == CAN_DEVICES_CHASSIS_MOTOR_COUNT,
               "故障检测与 CAN 设备数量必须一致");
_Static_assert(FAULT_DETECT_MOTOR_COUNT == CHASSIS_MOTOR_COUNT, "故障检测与底盘控制电机数量必须一致");

static void FaultDetect_ResetSnapshot(FaultDetect_Snapshot_t *fault_snapshot);
static uint32_t FaultDetect_EvaluateCanSystem(const CANDevices_Snapshot_t *can_snapshot);
static uint32_t FaultDetect_EvaluateChassisSystem(const Chassis_Snapshot_t *chassis_snapshot);
static uint32_t FaultDetect_EvaluateCanMotor(const CANDevices_Snapshot_t *can_snapshot, uint32_t motor_index);
static uint32_t FaultDetect_EvaluateChassisMotor(const Chassis_Snapshot_t *chassis_snapshot, uint32_t motor_index);
static void FaultDetect_UpdateMotorCounters(FaultDetect_Snapshot_t *fault_snapshot, uint32_t motor_fault_flags);

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
                                    FaultDetect_Snapshot_t *fault_snapshot)
{
    if (fault_snapshot == NULL)
    {
        return;
    }

    // 每周期从空快照重新汇总，避免历史故障在故障源恢复后残留
    FaultDetect_ResetSnapshot(fault_snapshot);
    fault_snapshot->evaluated = true;
    fault_snapshot->system_fault_flags = FaultDetect_EvaluateCanSystem(can_snapshot) |
                                         FaultDetect_EvaluateChassisSystem(chassis_snapshot);

    // 逐电机合并设备、反馈、输出和控制状态，供调试器直接定位故障槽位
    for (uint32_t motor_index = 0U; motor_index < FAULT_DETECT_MOTOR_COUNT; motor_index++)
    {
        const uint32_t motor_fault_flags = FaultDetect_EvaluateCanMotor(can_snapshot, motor_index) |
                                           FaultDetect_EvaluateChassisMotor(chassis_snapshot, motor_index);
        fault_snapshot->motor_fault_flags[motor_index] = motor_fault_flags;
        FaultDetect_UpdateMotorCounters(fault_snapshot, motor_fault_flags);
    }

    if (fault_snapshot->motor_fault_count > 0U)
    {
        fault_snapshot->system_fault_flags |= FAULT_DETECT_SYSTEM_MOTOR;
    }
    fault_snapshot->has_fault = fault_snapshot->system_fault_flags != FAULT_DETECT_SYSTEM_NONE;
}

/**
 * @brief 恢复故障检测快照的安全默认值
 *
 * @param[out] fault_snapshot 待复位的故障检测快照
 * @return 无返回值
 */
static void FaultDetect_ResetSnapshot(FaultDetect_Snapshot_t *fault_snapshot)
{
    *fault_snapshot = (FaultDetect_Snapshot_t)
    {
        0,
    };
}

/**
 * @brief 汇总 CAN 设备集合的系统级故障位
 *
 * @param[in] can_snapshot 本周期 CAN 设备快照，允许为 NULL
 * @return 系统级 CAN 故障位
 */
static uint32_t FaultDetect_EvaluateCanSystem(const CANDevices_Snapshot_t *can_snapshot)
{
    if (can_snapshot == NULL)
    {
        return FAULT_DETECT_SYSTEM_FEEDBACK_MISSING;
    }

    uint32_t fault_flags = FAULT_DETECT_SYSTEM_NONE;
    if (!can_snapshot->initialized || can_snapshot->init_status != CAN_DEVICES_OK)
    {
        fault_flags |= FAULT_DETECT_SYSTEM_CAN_INIT;
    }
    if (can_snapshot->feedback_status != CAN_DEVICES_OK)
    {
        fault_flags |= FAULT_DETECT_SYSTEM_CAN_FEEDBACK;
    }
    if (can_snapshot->output_status != CAN_DEVICES_OK)
    {
        fault_flags |= FAULT_DETECT_SYSTEM_CAN_OUTPUT;
    }
    if (can_snapshot->tx_status != CAN_DEVICES_OK)
    {
        fault_flags |= FAULT_DETECT_SYSTEM_CAN_TX;
    }
    return fault_flags;
}

/**
 * @brief 汇总底盘速度控制的系统级故障位
 *
 * @param[in] chassis_snapshot 本周期底盘控制快照，允许为 NULL
 * @return 系统级底盘控制故障位
 */
static uint32_t FaultDetect_EvaluateChassisSystem(const Chassis_Snapshot_t *chassis_snapshot)
{
    if (chassis_snapshot == NULL)
    {
        return FAULT_DETECT_SYSTEM_CHASSIS_INIT | FAULT_DETECT_SYSTEM_CHASSIS_CONTROL;
    }

    uint32_t fault_flags = FAULT_DETECT_SYSTEM_NONE;
    if (!chassis_snapshot->initialized || chassis_snapshot->control_init_status != CHASSIS_OK)
    {
        fault_flags |= FAULT_DETECT_SYSTEM_CHASSIS_INIT;
    }
    if (chassis_snapshot->chassis_status != CHASSIS_OK &&
        chassis_snapshot->chassis_status != CHASSIS_NOT_INITIALIZED)
    {
        fault_flags |= FAULT_DETECT_SYSTEM_CHASSIS_CONTROL;
    }
    if (chassis_snapshot->control_init_status == CHASSIS_CONFIG_ERROR ||
        chassis_snapshot->chassis_status == CHASSIS_CONFIG_ERROR)
    {
        fault_flags |= FAULT_DETECT_SYSTEM_CONFIG;
    }
    return fault_flags;
}

/**
 * @brief 汇总单路 CAN 设备、反馈和输出故障位
 *
 * @param[in] can_snapshot 本周期 CAN 设备快照，允许为 NULL
 * @param[in] motor_index 电机索引，范围 0~3
 * @return 单电机 CAN 相关故障位
 */
static uint32_t FaultDetect_EvaluateCanMotor(const CANDevices_Snapshot_t *can_snapshot, uint32_t motor_index)
{
    if (can_snapshot == NULL)
    {
        return FAULT_DETECT_MOTOR_FEEDBACK;
    }

    uint32_t fault_flags = FAULT_DETECT_MOTOR_NONE;
    if (can_snapshot->register_status[motor_index] != CAN_DEVICES_OK)
    {
        fault_flags |= FAULT_DETECT_MOTOR_REGISTER;
    }
    if (!can_snapshot->motor_online[motor_index])
    {
        fault_flags |= FAULT_DETECT_MOTOR_OFFLINE;
    }
    if (can_snapshot->feedback_update_status[motor_index] != CAN_DEVICES_OK)
    {
        fault_flags |= FAULT_DETECT_MOTOR_FEEDBACK;
    }
    if (can_snapshot->current_set_status[motor_index] != CAN_DEVICES_OK)
    {
        fault_flags |= FAULT_DETECT_MOTOR_CURRENT;
    }
    if (can_snapshot->current_set_status[motor_index] == CAN_DEVICES_INVALID_CURRENT)
    {
        fault_flags |= FAULT_DETECT_MOTOR_VALUE;
    }
    return fault_flags;
}

/**
 * @brief 汇总单路底盘速度控制故障位
 *
 * @param[in] chassis_snapshot 本周期底盘控制快照，允许为 NULL
 * @param[in] motor_index 电机索引，范围 0~3
 * @return 单电机控制相关故障位
 */
static uint32_t FaultDetect_EvaluateChassisMotor(const Chassis_Snapshot_t *chassis_snapshot, uint32_t motor_index)
{
    if (chassis_snapshot == NULL)
    {
        return FAULT_DETECT_MOTOR_CONTROL_INIT | FAULT_DETECT_MOTOR_CONTROL;
    }

    uint32_t fault_flags = FAULT_DETECT_MOTOR_NONE;
    if (chassis_snapshot->motor_init_status[motor_index] != CHASSIS_MOTOR_OK)
    {
        fault_flags |= FAULT_DETECT_MOTOR_CONTROL_INIT;
    }
    if (chassis_snapshot->motor_init_status[motor_index] == CHASSIS_MOTOR_CONFIG_ERROR)
    {
        fault_flags |= FAULT_DETECT_MOTOR_CONFIG;
    }
    if (chassis_snapshot->chassis_status == CHASSIS_NOT_INITIALIZED)
    {
        return fault_flags;
    }

    const Chassis_MotorStatus_t control_status = chassis_snapshot->control_status[motor_index];
    if (control_status != CHASSIS_MOTOR_OK && control_status != CHASSIS_MOTOR_DISABLED)
    {
        fault_flags |= FAULT_DETECT_MOTOR_CONTROL;
    }
    if (control_status == CHASSIS_MOTOR_CONFIG_ERROR)
    {
        fault_flags |= FAULT_DETECT_MOTOR_CONFIG;
    }
    if (control_status == CHASSIS_MOTOR_INVALID_VALUE)
    {
        fault_flags |= FAULT_DETECT_MOTOR_VALUE;
    }
    return fault_flags;
}

/**
 * @brief 根据单电机故障位更新故障计数
 *
 * @param[in,out] fault_snapshot 待更新的故障检测快照
 * @param[in] motor_fault_flags 单电机故障位
 * @return 无返回值
 */
static void FaultDetect_UpdateMotorCounters(FaultDetect_Snapshot_t *fault_snapshot, uint32_t motor_fault_flags)
{
    if (motor_fault_flags == FAULT_DETECT_MOTOR_NONE)
    {
        return;
    }

    fault_snapshot->motor_fault_count++;
    if ((motor_fault_flags & FAULT_DETECT_MOTOR_OFFLINE) != 0U)
    {
        fault_snapshot->offline_motor_count++;
    }
    if ((motor_fault_flags & (FAULT_DETECT_MOTOR_FEEDBACK | FAULT_DETECT_MOTOR_VALUE)) != 0U)
    {
        fault_snapshot->feedback_fault_count++;
    }
    if ((motor_fault_flags & FAULT_DETECT_MOTOR_CONFIG) != 0U)
    {
        fault_snapshot->config_fault_count++;
    }
    if ((motor_fault_flags & FAULT_DETECT_MOTOR_VALUE) != 0U)
    {
        fault_snapshot->value_fault_count++;
    }
}
