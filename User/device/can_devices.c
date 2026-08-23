#include "device/can_devices.h"

#include "bsp/can.h"
#include "device/device.h"
#include "device/motor_rm.h"

#include <math.h>
#include <stddef.h>

/* GM6020 协议反馈速度为 rpm，速度环使用 rad/s，换算系数为 2π/60 */
#define CAN_DEVICES_GM6020_RPM_TO_RAD_S (0.10471975511965977F)

/*
 * 四个 M3508 的固定 CAN 设备参数：
 * - 全部使用 CAN1，C620 电调 ID 依次为 1~4，对应反馈标准帧 ID 0x201~0x204
 * - 电机型号均为 M3508，启用 3591/187 减速箱换算，当前安装方向均不反向
 * - 最终安装方向仍需通过低速板上测试确认
 * @datasheet RoboMaster C620 用户手册“CAN 通信协议”章节
 */
static MOTOR_RM_Param_t can_devices_chassis_motor_param[CAN_DEVICES_CHASSIS_MOTOR_COUNT] =
{
    {
        .can = BSP_CAN_1,
        .id = 0x201U,
        .module = MOTOR_M3508,
        .reverse = false,
        .gear = true,
    },
    {
        .can = BSP_CAN_1,
        .id = 0x202U,
        .module = MOTOR_M3508,
        .reverse = false,
        .gear = true,
    },
    {
        .can = BSP_CAN_1,
        .id = 0x203U,
        .module = MOTOR_M3508,
        .reverse = false,
        .gear = true,
    },
    {
        .can = BSP_CAN_1,
        .id = 0x204U,
        .module = MOTOR_M3508,
        .reverse = false,
        .gear = true,
    },
};

/*
 * GM6020 固定 CAN 设备参数：
 * - 使用 CAN1，电调 ID 为 5，对应反馈标准帧 ID 0x209
 * - 电机型号为 GM6020，不启用减速箱换算，当前安装方向不反向
 * - 本阶段只注册和读取反馈，不写入电流缓存，不发送 0x2FF 控制帧
 * @datasheet RoboMaster GM6020 用户手册“CAN 通信协议”章节
 */
static MOTOR_RM_Param_t can_devices_gm6020_param =
{
    .can = BSP_CAN_1,
    .id = 0x209U,
    .module = MOTOR_GM6020,
    .reverse = false,
    .gear = false,
};

/*
 * 整车 CAN 设备集合私有状态：
 * - initialized：CAN1 初始化成功后为 true
 * - init_status：CAN1 初始化、四路 M3508 和一路 GM6020 注册的聚合结果
 * - register_status[0~3]、feedback_update_status[0~3]：四个 M3508 的注册和反馈更新结果
 * - chassis_motor[0~3]：四个已注册 M3508 实例，注册失败时对应项为 NULL
 * - gm6020_register_status、gm6020_feedback_update_status：GM6020 的注册和反馈更新结果
 * - gm6020：已注册 GM6020 实例，注册失败时为 NULL
 * - sequence：反馈更新周期序号
 */
typedef struct
{
    bool initialized;
    int8_t init_status;
    int8_t register_status[CAN_DEVICES_CHASSIS_MOTOR_COUNT];
    int8_t feedback_update_status[CAN_DEVICES_CHASSIS_MOTOR_COUNT];
    MOTOR_RM_t *chassis_motor[CAN_DEVICES_CHASSIS_MOTOR_COUNT];
    int8_t gm6020_register_status;
    int8_t gm6020_feedback_update_status;
    MOTOR_RM_t *gm6020;
    uint32_t sequence;
} CANDevices_State_t;

static CANDevices_State_t can_devices_state;

static void CANDevices_ResetState(void);
static void CANDevices_ResetSnapshot(CANDevices_Snapshot_t *snapshot);
static int8_t CANDevices_WriteCurrent(uint32_t motor_index, float requested_current_a, float *applied_current_a);
static int8_t CANDevices_WriteGimbalCurrent(float requested_current_a, float *applied_current_a);

/**
 * @brief 初始化 CAN1 并注册四个底盘 M3508 和一个 GM6020
 *
 * @param[out] snapshot CAN 设备集合初始化结果快照
 * @return 全部设备注册成功返回 CAN_DEVICES_OK，部分失败或总线不可用时返回对应状态码
 */
int8_t CANDevices_Init(CANDevices_Snapshot_t *snapshot)
{
    if (snapshot == NULL)
    {
        return CAN_DEVICES_NULL_ERROR;
    }

    // 先清除跨周期设备状态，保证任一初始化失败路径都保持安全默认值
    CANDevices_ResetState();
    CANDevices_ResetSnapshot(snapshot);

    // CAN1 初始化成功后才注册固定设备，避免在不可用总线上创建半初始化实例
    if (BSP_CAN_Init() != BSP_OK)
    {
        can_devices_state.init_status = CAN_DEVICES_CAN_INIT_ERROR;
        CANDevices_ResetSnapshot(snapshot);
        return CAN_DEVICES_CAN_INIT_ERROR;
    }
    can_devices_state.initialized = true;

    // 逐路注册实际存在的四个 M3508，单路失败不阻止其他设备完成注册
    bool all_registered = true;
    for (uint32_t motor_index = 0U; motor_index < CAN_DEVICES_CHASSIS_MOTOR_COUNT; motor_index++)
    {
        MOTOR_RM_Param_t *motor_param = &can_devices_chassis_motor_param[motor_index];
        can_devices_state.register_status[motor_index] = MOTOR_RM_Register(motor_param);
        if (can_devices_state.register_status[motor_index] == DEVICE_OK)
        {
            can_devices_state.chassis_motor[motor_index] = MOTOR_RM_GetMotor(motor_param);
            if (can_devices_state.chassis_motor[motor_index] == NULL)
            {
                can_devices_state.register_status[motor_index] = DEVICE_ERR_NO_DEV;
            }
        }
        if (can_devices_state.register_status[motor_index] != DEVICE_OK)
        {
            all_registered = false;
        }
    }

    // 独立注册 CAN1 上电调 ID 5 的 GM6020，不扩展底盘四电机槽位
    can_devices_state.gm6020_register_status = MOTOR_RM_Register(&can_devices_gm6020_param);
    if (can_devices_state.gm6020_register_status == DEVICE_OK)
    {
        can_devices_state.gm6020 = MOTOR_RM_GetMotor(&can_devices_gm6020_param);
        if (can_devices_state.gm6020 == NULL)
        {
            can_devices_state.gm6020_register_status = DEVICE_ERR_NO_DEV;
        }
    }
    if (can_devices_state.gm6020_register_status != DEVICE_OK)
    {
        all_registered = false;
    }

    can_devices_state.init_status = all_registered ? CAN_DEVICES_OK : CAN_DEVICES_ERROR;
    CANDevices_ResetSnapshot(snapshot);
    return can_devices_state.init_status;
}

/**
 * @brief 更新四个底盘 M3508 和一个 GM6020 的最新反馈
 *
 * @param[out] snapshot 本周期 CAN 设备反馈快照
 * @return 五路均取得新反馈返回 CAN_DEVICES_OK，否则返回 CAN_DEVICES_ERROR
 */
int8_t CANDevices_UpdateFeedback(CANDevices_Snapshot_t *snapshot)
{
    if (snapshot == NULL)
    {
        return CAN_DEVICES_NULL_ERROR;
    }

    // 每个反馈周期重新建立快照，禁止沿用上一周期的成功状态
    CANDevices_ResetSnapshot(snapshot);
    if (!can_devices_state.initialized)
    {
        return CAN_DEVICES_NOT_INITIALIZED;
    }

    can_devices_state.sequence++;
    snapshot->sequence = can_devices_state.sequence;

    // 更新四路反馈并复制物理量，注册失败和无新反馈路径分别保留故障状态
    bool all_updated = true;
    for (uint32_t motor_index = 0U; motor_index < CAN_DEVICES_CHASSIS_MOTOR_COUNT; motor_index++)
    {
        MOTOR_RM_t *motor = can_devices_state.chassis_motor[motor_index];
        if (motor == NULL)
        {
            can_devices_state.feedback_update_status[motor_index] = DEVICE_ERR_NO_DEV;
            snapshot->feedback_update_status[motor_index] = DEVICE_ERR_NO_DEV;
            all_updated = false;
            continue;
        }

        can_devices_state.feedback_update_status[motor_index] =
            MOTOR_RM_Update(&can_devices_chassis_motor_param[motor_index]);
        snapshot->feedback_update_status[motor_index] = can_devices_state.feedback_update_status[motor_index];
        snapshot->motor_online[motor_index] = motor->motor.header.online;
        snapshot->actual_speed_rpm[motor_index] = motor->feedback.rotor_speed;
        snapshot->temperature_c[motor_index] = motor->feedback.temp;
        if (snapshot->feedback_update_status[motor_index] != DEVICE_OK)
        {
            all_updated = false;
        }
    }

    // 独立更新 GM6020 反馈，仅向快照发布物理量与在线状态
    if (can_devices_state.gm6020 == NULL)
    {
        can_devices_state.gm6020_feedback_update_status = DEVICE_ERR_NO_DEV;
        snapshot->gm6020_feedback_update_status = DEVICE_ERR_NO_DEV;
        all_updated = false;
    }
    else
    {
        can_devices_state.gm6020_feedback_update_status = MOTOR_RM_Update(&can_devices_gm6020_param);
        snapshot->gm6020_feedback_update_status = can_devices_state.gm6020_feedback_update_status;
        snapshot->gm6020_online = can_devices_state.gm6020->motor.header.online;
        snapshot->gm6020_angle_rad = can_devices_state.gm6020->feedback.rotor_abs_angle;
        snapshot->gm6020_speed_rpm = can_devices_state.gm6020->feedback.rotor_speed;
        snapshot->gm6020_speed_rad_s = snapshot->gm6020_speed_rpm * CAN_DEVICES_GM6020_RPM_TO_RAD_S;
        snapshot->gm6020_raw_current_lsb = can_devices_state.gm6020->motor.raw_feedback.raw_current;
        snapshot->gm6020_torque_current_a = can_devices_state.gm6020->feedback.torque_current;
        snapshot->gm6020_temperature_c = can_devices_state.gm6020->feedback.temp;
        if (snapshot->gm6020_feedback_update_status != DEVICE_OK)
        {
            all_updated = false;
        }
    }

    snapshot->feedback_status = all_updated ? CAN_DEVICES_OK : CAN_DEVICES_ERROR;
    return snapshot->feedback_status;
}

/**
 * @brief 校验并写入四路电流后统一发送一次 RM 控制帧
 *
 * @param[in] command 本周期四路电流命令快照，允许为 NULL
 * @param[in,out] snapshot 本周期 CAN 设备反馈与输出结果快照
 * @return 四路命令均有效且统一发送成功返回 CAN_DEVICES_OK，否则返回 CAN_DEVICES_ERROR
 */
int8_t CANDevices_ApplyCurrent(const CANDevices_Command_t *command, CANDevices_Snapshot_t *snapshot)
{
    if (snapshot == NULL)
    {
        return CAN_DEVICES_NULL_ERROR;
    }
    if (!can_devices_state.initialized)
    {
        CANDevices_ResetSnapshot(snapshot);
        return CAN_DEVICES_NOT_INITIALIZED;
    }

    // 先清除本周期输出结果，空命令和无效命令均不得复用旧电流缓存状态
    snapshot->output_status = CAN_DEVICES_ERROR;
    snapshot->tx_status = CAN_DEVICES_DEVICE_UNAVAILABLE;
    snapshot->applied_command_sequence = command == NULL ? 0U : command->sequence;
    bool all_current_set = command != NULL;
    for (uint32_t motor_index = 0U; motor_index < CAN_DEVICES_CHASSIS_MOTOR_COUNT; motor_index++)
    {
        snapshot->current_set_status[motor_index] = CAN_DEVICES_DEVICE_UNAVAILABLE;
        snapshot->applied_current_a[motor_index] = 0.0F;
        if (can_devices_state.chassis_motor[motor_index] == NULL)
        {
            all_current_set = false;
            continue;
        }

        float requested_current_a = command == NULL ? 0.0F : command->current_a[motor_index];
        int8_t command_status = CAN_DEVICES_OK;
        if (!isfinite(requested_current_a))
        {
            requested_current_a = 0.0F;
            command_status = CAN_DEVICES_INVALID_CURRENT;
        }
        else if ((can_devices_state.feedback_update_status[motor_index] != DEVICE_OK ||
                  !can_devices_state.chassis_motor[motor_index]->motor.header.online) &&
                 requested_current_a != 0.0F)
        {
            requested_current_a = 0.0F;
            command_status = CAN_DEVICES_DEVICE_UNAVAILABLE;
        }

        // 写入经过安全约束的电流，失败时再次用零电流覆盖对应槽位
        snapshot->current_set_status[motor_index] =
            CANDevices_WriteCurrent(motor_index, requested_current_a, &snapshot->applied_current_a[motor_index]);
        if (snapshot->current_set_status[motor_index] == DEVICE_OK && command_status != CAN_DEVICES_OK)
        {
            snapshot->current_set_status[motor_index] = command_status;
        }
        if (snapshot->current_set_status[motor_index] != DEVICE_OK)
        {
            all_current_set = false;
        }
    }

    // 四路槽位全部处理后只发送一次 0x200 控制帧，保证同周期命令一致提交
    snapshot->tx_status = MOTOR_RM_FlushGroup(&can_devices_chassis_motor_param[0]);
    snapshot->output_status = all_current_set && snapshot->tx_status == DEVICE_OK ? CAN_DEVICES_OK : CAN_DEVICES_ERROR;
    return snapshot->output_status;
}

/**
 * @brief 校验并提交 GM6020 独立电流命令后发送对应控制帧
 *
 * @param[in] command 云台电流命令快照，允许为 NULL
 * @param[in,out] snapshot CAN 设备反馈与云台输出结果快照
 * @return 命令有效且 GM6020 控制帧发送成功时返回 CAN_DEVICES_OK，否则返回对应状态码
 */
int8_t CANDevices_ApplyGimbalCurrent(const CANDevices_GimbalCommand_t *command,
                                     CANDevices_Snapshot_t *snapshot)
{
    if (snapshot == NULL)
    {
        return CAN_DEVICES_NULL_ERROR;
    }
    if (!can_devices_state.initialized)
    {
        CANDevices_ResetSnapshot(snapshot);
        return CAN_DEVICES_NOT_INITIALIZED;
    }

    snapshot->gm6020_current_set_status = CAN_DEVICES_DEVICE_UNAVAILABLE;
    snapshot->gm6020_tx_status = CAN_DEVICES_DEVICE_UNAVAILABLE;
    snapshot->gm6020_applied_current_a = 0.0F;
    snapshot->gm6020_applied_command_sequence = command == NULL ? 0U : command->sequence;

    if (can_devices_state.gm6020 == NULL)
    {
        return CAN_DEVICES_DEVICE_UNAVAILABLE;
    }

    float requested_current_a = command == NULL ? 0.0F : command->current_a;
    int8_t command_status = CAN_DEVICES_OK;
    if (!isfinite(requested_current_a))
    {
        requested_current_a = 0.0F;
        command_status = CAN_DEVICES_INVALID_CURRENT;
    }
    else if (requested_current_a < -CAN_DEVICES_GM6020_CURRENT_LIMIT_A ||
             requested_current_a > CAN_DEVICES_GM6020_CURRENT_LIMIT_A)
    {
        requested_current_a = 0.0F;
        command_status = CAN_DEVICES_INVALID_CURRENT;
    }
    else if ((can_devices_state.gm6020_feedback_update_status != DEVICE_OK ||
              !can_devices_state.gm6020->motor.header.online) &&
             requested_current_a != 0.0F)
    {
        requested_current_a = 0.0F;
        command_status = CAN_DEVICES_DEVICE_UNAVAILABLE;
    }

    snapshot->gm6020_current_set_status =
        CANDevices_WriteGimbalCurrent(requested_current_a, &snapshot->gm6020_applied_current_a);
    if (snapshot->gm6020_current_set_status == DEVICE_OK && command_status != CAN_DEVICES_OK)
    {
        snapshot->gm6020_current_set_status = command_status;
    }
    snapshot->gm6020_tx_status = MOTOR_RM_FlushGroup(&can_devices_gm6020_param);
    if (snapshot->gm6020_current_set_status != DEVICE_OK || snapshot->gm6020_tx_status != DEVICE_OK)
    {
        return CAN_DEVICES_ERROR;
    }
    return CAN_DEVICES_OK;
}

/**
 * @brief 恢复整车 CAN 设备集合的安全默认状态
 *
 * @return 无返回值
 */
static void CANDevices_ResetState(void)
{
    can_devices_state = (CANDevices_State_t)
    {
        0,
    };
    can_devices_state.init_status = CAN_DEVICES_NOT_INITIALIZED;
    for (uint32_t motor_index = 0U; motor_index < CAN_DEVICES_CHASSIS_MOTOR_COUNT; motor_index++)
    {
        can_devices_state.register_status[motor_index] = DEVICE_ERR_NO_DEV;
        can_devices_state.feedback_update_status[motor_index] = DEVICE_ERR_NO_DEV;
    }
    can_devices_state.gm6020_register_status = DEVICE_ERR_NO_DEV;
    can_devices_state.gm6020_feedback_update_status = DEVICE_ERR_NO_DEV;
}

/**
 * @brief 使用设备集合私有状态建立安全一致的公开快照
 *
 * @param[out] snapshot 待复位的 CAN 设备集合快照
 * @return 无返回值
 */
static void CANDevices_ResetSnapshot(CANDevices_Snapshot_t *snapshot)
{
    *snapshot = (CANDevices_Snapshot_t)
    {
        0,
    };
    snapshot->initialized = can_devices_state.initialized;
    snapshot->init_status = can_devices_state.init_status;
    snapshot->feedback_status = CAN_DEVICES_NOT_INITIALIZED;
    snapshot->output_status = CAN_DEVICES_NOT_INITIALIZED;
    snapshot->tx_status = CAN_DEVICES_NOT_INITIALIZED;
    snapshot->gm6020_current_set_status = CAN_DEVICES_DEVICE_UNAVAILABLE;
    snapshot->gm6020_tx_status = CAN_DEVICES_DEVICE_UNAVAILABLE;
    snapshot->gm6020_applied_current_a = 0.0F;
    snapshot->gm6020_applied_command_sequence = 0U;
    snapshot->sequence = can_devices_state.sequence;
    snapshot->gm6020_register_status = can_devices_state.gm6020_register_status;
    snapshot->gm6020_feedback_update_status = CAN_DEVICES_DEVICE_UNAVAILABLE;
    for (uint32_t motor_index = 0U; motor_index < CAN_DEVICES_CHASSIS_MOTOR_COUNT; motor_index++)
    {
        snapshot->register_status[motor_index] = can_devices_state.register_status[motor_index];
        snapshot->feedback_update_status[motor_index] = CAN_DEVICES_DEVICE_UNAVAILABLE;
        snapshot->current_set_status[motor_index] = CAN_DEVICES_DEVICE_UNAVAILABLE;
    }
}

/**
 * @brief 写入单路安全电流，并在失败时用零电流覆盖旧命令
 *
 * @param[in] motor_index 底盘电机索引，范围 0~3
 * @param[in] requested_current_a 待写入的转子侧电流，单位 A
 * @param[out] applied_current_a 实际保留在发送缓存中的转子侧电流，单位 A
 * @return 写入成功返回 DEVICE_OK，否则返回 RM 电机驱动状态码
 */
static int8_t CANDevices_WriteCurrent(uint32_t motor_index, float requested_current_a, float *applied_current_a)
{
    MOTOR_RM_Param_t *motor_param = &can_devices_chassis_motor_param[motor_index];
    const int8_t current_status = MOTOR_RM_SetTorqueCurrent(motor_param, requested_current_a);
    if (current_status == DEVICE_OK)
    {
        *applied_current_a = requested_current_a;
        return DEVICE_OK;
    }

    const int8_t zero_status = MOTOR_RM_SetTorqueCurrent(motor_param, 0.0F);
    *applied_current_a = 0.0F;
    return zero_status == DEVICE_OK ? current_status : zero_status;
}

/**
 * @brief 写入 GM6020 转子侧电流并在失败时覆盖为零电流
 *
 * @param[in] requested_current_a 待写入的转子侧电流，单位 A
 * @param[out] applied_current_a 实际保留在发送缓存中的电流，单位 A
 * @return 写入成功返回 DEVICE_OK，否则返回设备层错误码
 */
static int8_t CANDevices_WriteGimbalCurrent(float requested_current_a, float *applied_current_a)
{
    const int8_t current_status = MOTOR_RM_SetTorqueCurrent(&can_devices_gm6020_param, requested_current_a);
    if (current_status == DEVICE_OK)
    {
        *applied_current_a = requested_current_a;
        return DEVICE_OK;
    }

    const int8_t zero_status = MOTOR_RM_SetTorqueCurrent(&can_devices_gm6020_param, 0.0F);
    *applied_current_a = 0.0F;
    return zero_status == DEVICE_OK ? current_status : zero_status;
}
