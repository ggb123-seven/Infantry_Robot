#include "module/chassis_can.h"

#include "bsp/can.h"
#include "device/motor_rm.h"

#include <math.h>
#include <stddef.h>

/*
 * 四个 M3508 的固定 CAN 设备参数：
 * - 全部使用 CAN1，C620 电调 ID 依次为 1~4，对应反馈标准帧 ID 0x201~0x204。
 * - 电机型号均为 M3508，启用 3591/187 减速箱换算，当前安装方向均不反向。
 * - 最终安装方向仍需通过低速板上测试确认。
 * @datasheet RoboMaster C620 用户手册“CAN 通信协议”章节
 */
static MOTOR_RM_Param_t chassis_can_motor_param[CHASSIS_CAN_MOTOR_COUNT] =
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

static MOTOR_RM_t *chassis_can_motor[CHASSIS_CAN_MOTOR_COUNT];

static void ChassisCAN_ResetFeedback(ChassisCAN_Feedback_t *feedback);
static void ChassisCAN_ResetOutputStatus(ChassisCAN_OutputStatus_t *status);

/**
 * @brief 初始化 CAN 总线并注册四个 M3508 设备
 *
 * @param[out] context 四电机 CAN 边界上下文
 * @return 全部设备注册成功返回 CHASSIS_CAN_OK，部分注册失败返回 CHASSIS_CAN_ERROR，参数或总线失败返回对应状态码
 */
int8_t ChassisCAN_Init(ChassisCAN_t *context)
{
    if (context == NULL)
    {
        return CHASSIS_CAN_NULL_ERROR;
    }

    // 先清除边界状态和设备引用，保证任何初始化失败路径都不可执行周期收发
    *context = (ChassisCAN_t)
    {
        0,
    };
    for (uint32_t motor_index = 0U; motor_index < CHASSIS_CAN_MOTOR_COUNT; motor_index++)
    {
        context->register_status[motor_index] = CHASSIS_CAN_DEVICE_UNAVAILABLE;
        chassis_can_motor[motor_index] = NULL;
    }

    // CAN 总线初始化成功后才允许设备注册，失败时保持上下文未初始化
    const int8_t can_init_status = BSP_CAN_Init();
    if (can_init_status != BSP_OK)
    {
        return can_init_status;
    }
    context->initialized = true;

    // 注册四个固定设备并保存实例，单路失败不阻止其余设备完成注册
    bool all_registered = true;
    for (uint32_t motor_index = 0U; motor_index < CHASSIS_CAN_MOTOR_COUNT; motor_index++)
    {
        context->register_status[motor_index] = MOTOR_RM_Register(&chassis_can_motor_param[motor_index]);
        if (context->register_status[motor_index] == DEVICE_OK)
        {
            chassis_can_motor[motor_index] = MOTOR_RM_GetMotor(&chassis_can_motor_param[motor_index]);
            if (chassis_can_motor[motor_index] == NULL)
            {
                context->register_status[motor_index] = DEVICE_ERR_NO_DEV;
            }
        }

        if (context->register_status[motor_index] != DEVICE_OK)
        {
            all_registered = false;
        }
    }

    return all_registered ? CHASSIS_CAN_OK : CHASSIS_CAN_ERROR;
}

/**
 * @brief 刷新四个 M3508 反馈并生成安全一致快照
 *
 * @param[in] context 已初始化的四电机 CAN 边界上下文
 * @param[out] feedback 本周期反馈快照
 * @return 四路均收到新反馈返回 CHASSIS_CAN_OK，否则返回对应状态码
 */
int8_t ChassisCAN_ReadFeedback(const ChassisCAN_t *context, ChassisCAN_Feedback_t *feedback)
{
    if (feedback == NULL)
    {
        return CHASSIS_CAN_NULL_ERROR;
    }

    // 在检查上下文前建立离线零反馈，防止失败路径泄漏上一周期数据
    ChassisCAN_ResetFeedback(feedback);
    if (context == NULL)
    {
        return CHASSIS_CAN_NULL_ERROR;
    }
    for (uint32_t motor_index = 0U; motor_index < CHASSIS_CAN_MOTOR_COUNT; motor_index++)
    {
        feedback->register_status[motor_index] = context->register_status[motor_index];
    }
    if (!context->initialized)
    {
        return CHASSIS_CAN_NOT_INITIALIZED;
    }

    // 各路独立刷新并复制设备快照，由调用方结合更新状态决定本周期是否参与控制
    bool all_updated = true;
    for (uint32_t motor_index = 0U; motor_index < CHASSIS_CAN_MOTOR_COUNT; motor_index++)
    {
        if (chassis_can_motor[motor_index] == NULL)
        {
            all_updated = false;
            continue;
        }

        feedback->feedback_update_status[motor_index] = MOTOR_RM_Update(&chassis_can_motor_param[motor_index]);
        feedback->motor_online[motor_index] = chassis_can_motor[motor_index]->motor.header.online;
        feedback->actual_speed_rpm[motor_index] = chassis_can_motor[motor_index]->feedback.rotor_speed;
        feedback->temperature_c[motor_index] = chassis_can_motor[motor_index]->feedback.temp;
        if (feedback->feedback_update_status[motor_index] != DEVICE_OK)
        {
            all_updated = false;
        }
    }

    return all_updated ? CHASSIS_CAN_OK : CHASSIS_CAN_ERROR;
}

/**
 * @brief 写入四路电流槽位并统一发送一次 CAN 控制帧
 *
 * @param[in] context 已初始化的四电机 CAN 边界上下文
 * @param[in] current_command_a 四路转子侧目标电流，单位 A
 * @param[out] status 本周期四路槽位写入和统一发送状态
 * @return 四路写入和统一发送均成功返回 CHASSIS_CAN_OK，否则返回对应状态码
 */
int8_t ChassisCAN_WriteCurrent(const ChassisCAN_t *context,
                               const float current_command_a[CHASSIS_CAN_MOTOR_COUNT],
                               ChassisCAN_OutputStatus_t *status)
{
    if (status == NULL)
    {
        return CHASSIS_CAN_NULL_ERROR;
    }

    // 在检查输入前建立未发送状态，保证非法调用不会沿用上一周期成功结果
    ChassisCAN_ResetOutputStatus(status);
    if (context == NULL || current_command_a == NULL)
    {
        return CHASSIS_CAN_NULL_ERROR;
    }
    if (!context->initialized)
    {
        return CHASSIS_CAN_NOT_INITIALIZED;
    }

    // 依次覆盖四个发送槽位，非法值或写入失败时立即用零电流清除该路旧命令
    bool all_current_set = true;
    for (uint32_t motor_index = 0U; motor_index < CHASSIS_CAN_MOTOR_COUNT; motor_index++)
    {
        if (chassis_can_motor[motor_index] == NULL)
        {
            all_current_set = false;
            continue;
        }

        if (!isfinite(current_command_a[motor_index]))
        {
            status->current_set_status[motor_index] = CHASSIS_CAN_INVALID_CURRENT;
            const int8_t zero_set_status = MOTOR_RM_SetTorqueCurrent(&chassis_can_motor_param[motor_index], 0.0F);
            if (zero_set_status != DEVICE_OK)
            {
                status->current_set_status[motor_index] = zero_set_status;
            }
            all_current_set = false;
            continue;
        }

        status->current_set_status[motor_index] =
            MOTOR_RM_SetTorqueCurrent(&chassis_can_motor_param[motor_index], current_command_a[motor_index]);
        if (status->current_set_status[motor_index] != DEVICE_OK)
        {
            const int8_t zero_set_status = MOTOR_RM_SetTorqueCurrent(&chassis_can_motor_param[motor_index], 0.0F);
            if (zero_set_status != DEVICE_OK)
            {
                status->current_set_status[motor_index] = zero_set_status;
            }
            all_current_set = false;
        }
    }

    // 四路槽位全部处理后只发送一次同组控制帧，保证控制命令属于同一周期快照
    status->can_tx_status = MOTOR_RM_FlushGroup(&chassis_can_motor_param[0]);
    if (status->can_tx_status != DEVICE_OK)
    {
        return CHASSIS_CAN_ERROR;
    }
    return all_current_set ? CHASSIS_CAN_OK : CHASSIS_CAN_ERROR;
}

/**
 * @brief 将反馈快照复位为离线零数据
 *
 * @param[out] feedback 待复位的反馈快照
 * @return 无返回值
 */
static void ChassisCAN_ResetFeedback(ChassisCAN_Feedback_t *feedback)
{
    *feedback = (ChassisCAN_Feedback_t)
    {
        0,
    };
    for (uint32_t motor_index = 0U; motor_index < CHASSIS_CAN_MOTOR_COUNT; motor_index++)
    {
        feedback->feedback_update_status[motor_index] = CHASSIS_CAN_DEVICE_UNAVAILABLE;
        feedback->register_status[motor_index] = CHASSIS_CAN_DEVICE_UNAVAILABLE;
    }
}

/**
 * @brief 将电流提交状态复位为未写入且未发送
 *
 * @param[out] status 待复位的电流提交状态
 * @return 无返回值
 */
static void ChassisCAN_ResetOutputStatus(ChassisCAN_OutputStatus_t *status)
{
    *status = (ChassisCAN_OutputStatus_t)
    {
        0,
    };
    for (uint32_t motor_index = 0U; motor_index < CHASSIS_CAN_MOTOR_COUNT; motor_index++)
    {
        status->current_set_status[motor_index] = CHASSIS_CAN_DEVICE_UNAVAILABLE;
    }
    status->can_tx_status = CHASSIS_CAN_DEVICE_UNAVAILABLE;
}
