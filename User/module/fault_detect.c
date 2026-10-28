#include "module/fault_detect.h"

#include <stddef.h>

_Static_assert(FAULT_DETECT_MOTOR_COUNT == CAN_DEVICES_CHASSIS_MOTOR_COUNT, "故障检测与 CAN 设备数量必须一致");
_Static_assert(FAULT_DETECT_MOTOR_COUNT == CHASSIS_WHEEL_COUNT, "故障检测与底盘控制电机数量必须一致");

static uint32_t FaultDetect_EvaluateCanSystem(const CANDevices_Snapshot_t *can_snapshot);
static uint32_t FaultDetect_EvaluateChassisSystem(const Chassis_t *chassis);
static uint32_t FaultDetect_EvaluateCanMotor(const CANDevices_Snapshot_t *can_snapshot, uint32_t motor_index);
static uint32_t FaultDetect_EvaluateChassisMotor(const Chassis_t *chassis, uint32_t motor_index);
static void FaultDetect_UpdateMotorCounters(FaultDetect_Snapshot_t *fault_snapshot, uint32_t motor_fault_flags);

/**
 * @brief 汇总底盘 CAN 设备快照与控制实例中的故障状态
 *
 * @param[in] can_snapshot 本周期 CAN 设备快照，允许为 NULL
 * @param[in] chassis 本周期底盘控制实例，允许为 NULL
 * @param[out] fault_snapshot 待写入的故障检测快照
 * @return 无返回值
 */
void FaultDetect_UpdateMotorChassis(const CANDevices_Snapshot_t *can_snapshot,
                                    const Chassis_t *chassis,
                                    FaultDetect_Snapshot_t *fault_snapshot)
{
    if (fault_snapshot == NULL)
    {
        return;
    }

    // 每周期从空快照重新汇总，避免历史故障在故障源恢复后残留
    *fault_snapshot = (FaultDetect_Snapshot_t)
    {
        .evaluated = true,
    };
    fault_snapshot->system_fault_flags = FaultDetect_EvaluateCanSystem(can_snapshot) |
                                         FaultDetect_EvaluateChassisSystem(chassis);

    // 逐电机合并设备、反馈、输出和控制状态，供调试器直接定位故障槽位
    for (uint32_t motor_index = 0U; motor_index < FAULT_DETECT_MOTOR_COUNT; motor_index++)
    {
        const uint32_t motor_fault_flags = FaultDetect_EvaluateCanMotor(can_snapshot, motor_index) |
                                           FaultDetect_EvaluateChassisMotor(chassis, motor_index);
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
 * @brief 汇总 DR16 在线状态与 UART 字节流状态
 *
 * @param[in] dr16_state DR16 接收器状态快照，允许为 NULL
 * @param[in] stream_running UART 字节流当前运行时为 true
 * @param[out] fault_snapshot 待写入的 DR16 故障检测快照
 * @return 无返回值
 */
void FaultDetect_UpdateDR16(const DR16_State_t *dr16_state, bool stream_running,
                            FaultDetect_DR16Snapshot_t *fault_snapshot)
{
    if (fault_snapshot == NULL)
    {
        return;
    }

    // 每周期从空快照重新判断，使链路和设备恢复后能够立即清除当前故障
    *fault_snapshot = (FaultDetect_DR16Snapshot_t)
    {
        .evaluated = true,
    };
    if (dr16_state == NULL)
    {
        fault_snapshot->fault_flags = FAULT_DETECT_DR16_OFFLINE | FAULT_DETECT_DR16_DATA;
    }
    else
    {
        if (!dr16_state->header.online)
        {
            fault_snapshot->fault_flags |= FAULT_DETECT_DR16_OFFLINE;
        }
        if (dr16_state->header.online && dr16_state->valid_frame_sequence == 0U)
        {
            fault_snapshot->fault_flags |= FAULT_DETECT_DR16_DATA;
        }
    }
    if (!stream_running)
    {
        fault_snapshot->fault_flags |= FAULT_DETECT_DR16_STREAM;
    }
    fault_snapshot->has_fault = fault_snapshot->fault_flags != FAULT_DETECT_DR16_NONE;
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
 * @param[in] chassis 本周期底盘控制实例，允许为 NULL
 * @return 系统级底盘控制故障位
 */
static uint32_t FaultDetect_EvaluateChassisSystem(const Chassis_t *chassis)
{
    if (chassis == NULL)
    {
        return FAULT_DETECT_SYSTEM_CHASSIS_INIT | FAULT_DETECT_SYSTEM_CONFIG;
    }

    uint32_t fault_flags = FAULT_DETECT_SYSTEM_NONE;
    if (!chassis->initialized)
    {
        fault_flags |= FAULT_DETECT_SYSTEM_CHASSIS_INIT;
    }
    if (!chassis->config_valid)
    {
        fault_flags |= FAULT_DETECT_SYSTEM_CONFIG;
    }
    for (uint32_t i = 0U; i < CHASSIS_WHEEL_COUNT; i++)
    {
        if (chassis->mode != CHASSIS_MODE_RELAX && chassis->wheel[i].state == CHASSIS_WHEEL_OFFLINE)
        {
            fault_flags |= FAULT_DETECT_SYSTEM_CHASSIS_CONTROL;
        }
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
 * @param[in] chassis 本周期底盘控制实例，允许为 NULL
 * @param[in] motor_index 电机索引，范围 0~3
 * @return 单电机控制相关故障位
 */
static uint32_t FaultDetect_EvaluateChassisMotor(const Chassis_t *chassis, uint32_t motor_index)
{
    if (chassis == NULL)
    {
        return FAULT_DETECT_MOTOR_CONTROL_INIT | FAULT_DETECT_MOTOR_CONFIG;
    }

    uint32_t fault_flags = FAULT_DETECT_MOTOR_NONE;
    if (!chassis->initialized)
    {
        fault_flags |= FAULT_DETECT_MOTOR_CONTROL_INIT;
    }
    if (!chassis->config_valid)
    {
        fault_flags |= FAULT_DETECT_MOTOR_CONFIG;
    }
    if (chassis->mode != CHASSIS_MODE_RELAX && chassis->wheel[motor_index].state == CHASSIS_WHEEL_OFFLINE)
    {
        fault_flags |= FAULT_DETECT_MOTOR_CONTROL;
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
