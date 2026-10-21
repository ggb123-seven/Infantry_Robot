#include "device/can_devices.h"

#include "bsp/can.h"
#include "device/device.h"
#include "device/motor_rm.h"

#include <math.h>
#include <stddef.h>

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
 * 整车 CAN 设备集合私有状态：
 * - initialized：CAN1 初始化成功后为 true
 * - init_status：CAN1 初始化、四路 M3508 注册的聚合结果
 * - register_status[0~3]、feedback_update_status[0~3]：四个 M3508 的注册和反馈更新结果
 * - chassis_motor[0~3]：四个已注册 M3508 实例，注册失败时对应项为 NULL
 * - sequence：反馈更新周期序号
 */
typedef struct
{
    bool initialized;
    int8_t init_status;
    int8_t register_status[CAN_DEVICES_CHASSIS_MOTOR_COUNT];
    int8_t feedback_update_status[CAN_DEVICES_CHASSIS_MOTOR_COUNT];
    MOTOR_RM_t *chassis_motor[CAN_DEVICES_CHASSIS_MOTOR_COUNT];
    uint32_t sequence;
} CANDevices_State_t;

static CANDevices_State_t can_devices_state;

/*
 * CAN2 云台固定参数：索引 0 为 ID5 yaw，索引 1 为 ID6 pitch
 * 反馈标准帧分别为 0x209、0x20A，电流控制共用 0x2FE，方向暂不反转，无外置减速箱
 * @datasheet RoboMaster GM6020 使用说明 20231013 的 CAN 电流控制协议
 */
static MOTOR_RM_Param_t can_devices_gimbal_param[CAN_DEVICES_GIMBAL_MOTOR_COUNT] =
{
    {
        .can = BSP_CAN_2,
        .id = 0x209U,
        .module = MOTOR_GM6020,
        .reverse = false,
        .gear = false,
    },
    {
        .can = BSP_CAN_2,
        .id = 0x20AU,
        .module = MOTOR_GM6020,
        .reverse = false,
        .gear = false,
    },
};

static bool can_devices_gimbal_registered[CAN_DEVICES_GIMBAL_MOTOR_COUNT];

static void CANDevices_ResetState(void);
static void CANDevices_ResetSnapshot(CANDevices_Snapshot_t *snapshot);
static int8_t CANDevices_WriteCurrent(uint32_t motor_index, float requested_current_a, float *applied_current_a);
static int8_t CANDevices_InitGimbal(void);

/**
 * @brief 初始化 CAN 总线并注册底盘与云台电机
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

    // 云台初始化结果纳入聚合状态，单路失败仍允许已注册的底盘设备运行
    const int8_t gimbal_status = CANDevices_InitGimbal();
    can_devices_state.init_status = all_registered && gimbal_status == CAN_DEVICES_OK ?
                                    CAN_DEVICES_OK : CAN_DEVICES_ERROR;
    CANDevices_ResetSnapshot(snapshot);
    return can_devices_state.init_status;
}

/**
 * @brief 更新四个底盘 M3508 的最新反馈
 *
 * @param[out] snapshot 本周期 CAN 设备反馈快照
 * @return 四路均取得新反馈返回 CAN_DEVICES_OK，否则返回 CAN_DEVICES_ERROR
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
 * @brief 注册 CAN2 两路 GM6020，并预置零电流缓存
 *
 * @return 两路注册与缓存初始化均成功返回 CAN_DEVICES_OK，否则返回 CAN_DEVICES_ERROR
 */
static int8_t CANDevices_InitGimbal(void)
{
    // 总线不可用时不创建设备，防止缺失的 CAN2 配置影响 CAN1 通信
    for (uint8_t axis = 0U; axis < CAN_DEVICES_GIMBAL_MOTOR_COUNT; axis++)
    {
        can_devices_gimbal_registered[axis] = false;
    }
    if (BSP_CAN_GetHandle(BSP_CAN_2) == NULL)
    {
        return CAN_DEVICES_ERROR;
    }

    // 两轴分别注册并预置零电流，等待通信任务提交控制输出
    bool success = true;
    for (uint8_t axis = 0U; axis < CAN_DEVICES_GIMBAL_MOTOR_COUNT; axis++)
    {
        MOTOR_RM_Param_t *param = &can_devices_gimbal_param[axis];
        if (MOTOR_RM_Register(param) != DEVICE_OK || MOTOR_RM_GetMotor(param) == NULL ||
            MOTOR_RM_SetTorqueCurrent(param, 0.0f) != DEVICE_OK)
        {
            success = false;
            continue;
        }
        can_devices_gimbal_registered[axis] = true;
    }
    return success ? CAN_DEVICES_OK : CAN_DEVICES_ERROR;
}

/**
 * @brief 获取单轴新反馈，失败时阻止无效数据进入任务邮箱
 *
 * @param[in] axis 云台轴索引，范围 0 至 1
 * @param[out] feedback 新反馈，速度单位 rpm，时间戳单位微秒，失败时清零
 * @return 成功返回 CAN_DEVICES_OK，否则返回设备错误码
 */
int8_t CANDevices_UpdateGimbalFeedback(uint8_t axis, MOTOR_RM_Feedback_t *feedback)
{
    if (feedback == NULL)
    {
        return CAN_DEVICES_NULL_ERROR;
    }

    // 失败路径清零输出，并拒绝越界或尚未完成注册的设备
    *feedback = (MOTOR_RM_Feedback_t)
    {
        0,
    };
    if (axis >= CAN_DEVICES_GIMBAL_MOTOR_COUNT || !can_devices_gimbal_registered[axis])
    {
        return CAN_DEVICES_DEVICE_UNAVAILABLE;
    }

    // 只发布本周期实际取得的新帧，邮箱消费者使用微秒时间戳判断反馈时效
    MOTOR_RM_Param_t *param = &can_devices_gimbal_param[axis];
    const int8_t status = MOTOR_RM_Update(param);
    if (status != DEVICE_OK)
    {
        return status;
    }
    const MOTOR_RM_t *motor = MOTOR_RM_GetMotor(param);
    if (motor == NULL)
    {
        return CAN_DEVICES_DEVICE_UNAVAILABLE;
    }
    *feedback = motor->feedback;
    return CAN_DEVICES_OK;
}

/**
 * @brief 写入 yaw 电流并清零 pitch，再统一发送云台控制帧
 *
 * @param[in] yaw_current_a yaw 转子侧电流，单位 A，有限值由电机驱动按协议量程限幅
 * @return 两路缓存写入且发送成功返回 CAN_DEVICES_OK，非法输入归零并返回 CAN_DEVICES_ERROR
 */
int8_t CANDevices_ApplyGimbalCurrent(float yaw_current_a)
{
    bool success = isfinite(yaw_current_a);
    MOTOR_RM_Param_t *flush_param = NULL;

    // 非法请求使用零电流，先写完两轴缓存再统一发送，pitch 始终不参与控制
    for (uint8_t axis = 0U; axis < CAN_DEVICES_GIMBAL_MOTOR_COUNT; axis++)
    {
        if (!can_devices_gimbal_registered[axis])
        {
            success = false;
            continue;
        }
        MOTOR_RM_Param_t *param = &can_devices_gimbal_param[axis];
        const float current_a = axis == CAN_DEVICES_GIMBAL_YAW && isfinite(yaw_current_a) ? yaw_current_a : 0.0f;
        if (MOTOR_RM_SetTorqueCurrent(param, current_a) != DEVICE_OK)
        {
            success = false;
        }
        flush_param = param;
    }

    // 共用的 0x2FE 每周期只发送一次，未使用的 ID7 与保留字节由驱动清零
    if (flush_param == NULL || MOTOR_RM_FlushGroup(flush_param) != DEVICE_OK)
    {
        return CAN_DEVICES_ERROR;
    }
    return success ? CAN_DEVICES_OK : CAN_DEVICES_ERROR;
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
    snapshot->sequence = can_devices_state.sequence;
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
