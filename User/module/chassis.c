#include "module/chassis.h"

#include "bsp/can.h"
#include "device/motor_rm.h"

#include <math.h>
#include <stddef.h>

_Static_assert(CHASSIS_MOTOR_COUNT == 4U, "底盘固定使用四个 M3508 电机");

/*
 * 底盘模块私有运行状态：
 * - initialized：CAN 总线和四路速度控制器可运行时为 true，允许单个电机注册失败后隔离运行。
 * - can_initialized：CAN 总线初始化成功后为 true。
 * - can_init_status、control_init_status：CAN 设备边界和四路速度控制器的聚合初始化结果。
 * - register_status[0~3]、motor_init_status[0~3]：四个设备和速度控制器的初始化结果。
 * - speed_control[0~3]：四个相互独立的单电机速度控制上下文。
 */
typedef struct
{
    bool initialized;
    bool can_initialized;
    int8_t can_init_status;
    int8_t control_init_status;
    int8_t register_status[CHASSIS_MOTOR_COUNT];
    int8_t motor_init_status[CHASSIS_MOTOR_COUNT];
    MotorSpeedControl_t speed_control[CHASSIS_MOTOR_COUNT];
} Chassis_State_t;

/*
 * 四个 M3508 的固定 CAN 设备参数：
 * - 全部使用 CAN1，C620 电调 ID 依次为 1~4，对应反馈标准帧 ID 0x201~0x204。
 * - 电机型号均为 M3508，启用 3591/187 减速箱换算，当前安装方向均不反向。
 * - 最终安装方向仍需通过低速板上测试确认。
 * @datasheet RoboMaster C620 用户手册“CAN 通信协议”章节
 */
static MOTOR_RM_Param_t chassis_motor_param[CHASSIS_MOTOR_COUNT] =
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

static Chassis_State_t chassis_state;
static MOTOR_RM_t *chassis_motor[CHASSIS_MOTOR_COUNT];

static void Chassis_ResetState(void);
static void Chassis_ResetSnapshot(Chassis_Snapshot_t *snapshot);
static int8_t Chassis_InitCan(void);
static int8_t Chassis_InitControllers(float sample_frequency_hz);
static int8_t Chassis_ReadFeedback(Chassis_Snapshot_t *snapshot);
static int8_t Chassis_Calculate(const Chassis_Input_t *input, Chassis_Snapshot_t *snapshot);
static int8_t Chassis_WriteCurrent(Chassis_Snapshot_t *snapshot);

/**
 * @brief 初始化四个 M3508 的 CAN 设备边界和速度控制器
 *
 * @param[in] sample_frequency_hz 控制采样频率，单位 Hz，必须大于 0
 * @param[out] snapshot 底盘初始化结果快照
 * @return 完全成功返回 CHASSIS_OK，存在隔离故障或初始化失败时返回对应状态码
 */
int8_t Chassis_Init(float sample_frequency_hz, Chassis_Snapshot_t *snapshot)
{
    if (snapshot == NULL)
    {
        return CHASSIS_NULL_ERROR;
    }

    // 先清除全部跨周期状态，保证任一初始化失败路径都不能进入周期控制
    Chassis_ResetState();
    Chassis_ResetSnapshot(snapshot);
    if (!isfinite(sample_frequency_hz) || sample_frequency_hz <= 0.0F)
    {
        chassis_state.control_init_status = CHASSIS_CONFIG_ERROR;
        for (uint32_t motor_index = 0U; motor_index < CHASSIS_MOTOR_COUNT; motor_index++)
        {
            chassis_state.motor_init_status[motor_index] = MOTOR_SPEED_CONTROL_CONFIG_ERROR;
        }
        Chassis_ResetSnapshot(snapshot);
        return CHASSIS_CONFIG_ERROR;
    }

    // 先初始化 CAN 总线和固定设备，硬件边界不可用时不再创建速度控制器
    const int8_t can_status = Chassis_InitCan();
    if (!chassis_state.can_initialized)
    {
        Chassis_ResetSnapshot(snapshot);
        return CHASSIS_CAN_INIT_ERROR;
    }

    // CAN 总线可运行后初始化四路速度控制器，单个设备注册失败由周期控制逐路隔离
    const int8_t control_status = Chassis_InitControllers(sample_frequency_hz);
    chassis_state.initialized = control_status == CHASSIS_OK;
    Chassis_ResetSnapshot(snapshot);
    if (!chassis_state.initialized)
    {
        return CHASSIS_CONTROL_INIT_ERROR;
    }
    return can_status == CHASSIS_OK ? CHASSIS_OK : CHASSIS_ERROR;
}

/**
 * @brief 执行一次反馈采集、四路速度控制和电流统一发送
 *
 * @param[in] input 本周期控制输入快照
 * @param[out] snapshot 本周期反馈、控制和发送结果快照
 * @return 全链路正常返回 CHASSIS_OK，存在隔离故障返回 CHASSIS_ERROR
 */
int8_t Chassis_Run(const Chassis_Input_t *input, Chassis_Snapshot_t *snapshot)
{
    if (snapshot == NULL)
    {
        return CHASSIS_NULL_ERROR;
    }

    // 在检查输入前建立安全快照，非法调用不得沿用上一周期数据
    Chassis_ResetSnapshot(snapshot);
    if (input == NULL)
    {
        return CHASSIS_NULL_ERROR;
    }
    if (!chassis_state.initialized)
    {
        return CHASSIS_NOT_INITIALIZED;
    }

    // 按固定顺序完成反馈、控制和统一发送，任一路异常均在所属阶段归零
    snapshot->feedback_status = Chassis_ReadFeedback(snapshot);
    snapshot->chassis_status = Chassis_Calculate(input, snapshot);
    snapshot->output_status = Chassis_WriteCurrent(snapshot);

    const bool all_succeeded = snapshot->feedback_status == CHASSIS_OK &&
                               snapshot->chassis_status == CHASSIS_OK &&
                               snapshot->output_status == CHASSIS_OK;
    return all_succeeded ? CHASSIS_OK : CHASSIS_ERROR;
}

/**
 * @brief 恢复底盘模块私有状态的安全默认值
 *
 * @return 无返回值
 */
static void Chassis_ResetState(void)
{
    chassis_state = (Chassis_State_t)
    {
        0,
    };
    chassis_state.can_init_status = CHASSIS_NOT_INITIALIZED;
    chassis_state.control_init_status = CHASSIS_NOT_INITIALIZED;
    for (uint32_t motor_index = 0U; motor_index < CHASSIS_MOTOR_COUNT; motor_index++)
    {
        chassis_state.register_status[motor_index] = CHASSIS_DEVICE_UNAVAILABLE;
        chassis_state.motor_init_status[motor_index] = MOTOR_SPEED_CONTROL_INIT_ERROR;
        chassis_motor[motor_index] = NULL;
    }
}

/**
 * @brief 使用模块私有状态建立安全一致的公开快照
 *
 * @param[out] snapshot 待复位的底盘结果快照
 * @return 无返回值
 */
static void Chassis_ResetSnapshot(Chassis_Snapshot_t *snapshot)
{
    *snapshot = (Chassis_Snapshot_t)
    {
        0,
    };
    snapshot->initialized = chassis_state.initialized;
    snapshot->can_init_status = chassis_state.can_init_status;
    snapshot->control_init_status = chassis_state.control_init_status;
    snapshot->feedback_status = CHASSIS_DEVICE_UNAVAILABLE;
    snapshot->chassis_status = CHASSIS_NOT_INITIALIZED;
    snapshot->output_status = CHASSIS_DEVICE_UNAVAILABLE;
    snapshot->can_tx_status = CHASSIS_DEVICE_UNAVAILABLE;
    for (uint32_t motor_index = 0U; motor_index < CHASSIS_MOTOR_COUNT; motor_index++)
    {
        snapshot->register_status[motor_index] = chassis_state.register_status[motor_index];
        snapshot->motor_init_status[motor_index] = chassis_state.motor_init_status[motor_index];
        snapshot->feedback_update_status[motor_index] = CHASSIS_DEVICE_UNAVAILABLE;
        snapshot->control_status[motor_index] = MOTOR_SPEED_CONTROL_INIT_ERROR;
        snapshot->current_set_status[motor_index] = CHASSIS_DEVICE_UNAVAILABLE;
    }
}

/**
 * @brief 初始化 CAN 总线并注册四个固定 M3508 设备
 *
 * @return 全部设备注册成功返回 CHASSIS_OK，部分注册失败返回 CHASSIS_ERROR，总线失败返回对应状态码
 */
static int8_t Chassis_InitCan(void)
{
    // CAN 总线初始化成功后才允许设备注册，失败时保持模块不可运行
    chassis_state.can_init_status = BSP_CAN_Init();
    if (chassis_state.can_init_status != BSP_OK)
    {
        return CHASSIS_CAN_INIT_ERROR;
    }
    chassis_state.can_initialized = true;

    // 注册四个固定设备并保存实例，单路失败不阻止其余设备完成注册
    bool all_registered = true;
    for (uint32_t motor_index = 0U; motor_index < CHASSIS_MOTOR_COUNT; motor_index++)
    {
        chassis_state.register_status[motor_index] = MOTOR_RM_Register(&chassis_motor_param[motor_index]);
        if (chassis_state.register_status[motor_index] == DEVICE_OK)
        {
            chassis_motor[motor_index] = MOTOR_RM_GetMotor(&chassis_motor_param[motor_index]);
            if (chassis_motor[motor_index] == NULL)
            {
                chassis_state.register_status[motor_index] = DEVICE_ERR_NO_DEV;
            }
        }
        if (chassis_state.register_status[motor_index] != DEVICE_OK)
        {
            all_registered = false;
        }
    }

    chassis_state.can_init_status = all_registered ? CHASSIS_OK : CHASSIS_ERROR;
    return chassis_state.can_init_status;
}

/**
 * @brief 初始化四个相互独立的单电机速度控制器
 *
 * @param[in] sample_frequency_hz 控制采样频率，单位 Hz，必须大于 0
 * @return 全部速度控制器初始化成功返回 CHASSIS_OK，否则返回 CHASSIS_CONTROL_INIT_ERROR
 */
static int8_t Chassis_InitControllers(float sample_frequency_hz)
{
    bool all_initialized = true;
    for (uint32_t motor_index = 0U; motor_index < CHASSIS_MOTOR_COUNT; motor_index++)
    {
        chassis_state.motor_init_status[motor_index] =
            MotorSpeedControl_Init(&chassis_state.speed_control[motor_index], sample_frequency_hz);
        if (chassis_state.motor_init_status[motor_index] != MOTOR_SPEED_CONTROL_OK)
        {
            all_initialized = false;
        }
    }

    chassis_state.control_init_status = all_initialized ? CHASSIS_OK : CHASSIS_CONTROL_INIT_ERROR;
    return chassis_state.control_init_status;
}

/**
 * @brief 刷新四个 M3508 反馈并写入本周期快照
 *
 * @param[out] snapshot 本周期底盘结果快照
 * @return 四路均收到新反馈返回 CHASSIS_OK，否则返回 CHASSIS_ERROR
 */
static int8_t Chassis_ReadFeedback(Chassis_Snapshot_t *snapshot)
{
    bool all_updated = true;
    for (uint32_t motor_index = 0U; motor_index < CHASSIS_MOTOR_COUNT; motor_index++)
    {
        if (chassis_motor[motor_index] == NULL)
        {
            all_updated = false;
            continue;
        }

        snapshot->feedback_update_status[motor_index] = MOTOR_RM_Update(&chassis_motor_param[motor_index]);
        snapshot->motor_online[motor_index] = chassis_motor[motor_index]->motor.header.online;
        snapshot->actual_speed_rpm[motor_index] = chassis_motor[motor_index]->feedback.rotor_speed;
        snapshot->temperature_c[motor_index] = chassis_motor[motor_index]->feedback.temp;
        if (snapshot->feedback_update_status[motor_index] != DEVICE_OK)
        {
            all_updated = false;
        }
    }
    return all_updated ? CHASSIS_OK : CHASSIS_ERROR;
}

/**
 * @brief 根据本周期反馈独立计算四路安全电流指令
 *
 * @param[in] input 本周期控制输入快照
 * @param[in,out] snapshot 本周期底盘结果快照
 * @return 全部活动控制器正常返回 CHASSIS_OK，存在控制异常返回 CHASSIS_ERROR
 */
static int8_t Chassis_Calculate(const Chassis_Input_t *input, Chassis_Snapshot_t *snapshot)
{
    bool all_control_valid = true;
    for (uint32_t motor_index = 0U; motor_index < CHASSIS_MOTOR_COUNT; motor_index++)
    {
        MotorSpeedControl_t *speed_control = &chassis_state.speed_control[motor_index];
        const bool feedback_updated = snapshot->feedback_update_status[motor_index] == DEVICE_OK;
        const float actual_speed_rpm = feedback_updated ? snapshot->actual_speed_rpm[motor_index] : 0.0F;
        const int8_t feedback_status = MotorSpeedControl_UpdateFeedback(speed_control, actual_speed_rpm);
        const bool motor_enabled = input->enabled && feedback_updated && snapshot->motor_online[motor_index] &&
                                   feedback_status == MOTOR_SPEED_CONTROL_OK;
        const int8_t control_status =
            MotorSpeedControl_Control(speed_control, input->requested_speed_rpm[motor_index], &input->pid_tune,
                                      motor_enabled, input->control_period_s);
        float current_command_a = 0.0F;
        const int8_t output_status = MotorSpeedControl_DumpOutput(speed_control, &current_command_a);

        // 汇总本路状态，异常反馈或计算结果只清零本路电流
        int8_t published_status = control_status;
        if (feedback_status != MOTOR_SPEED_CONTROL_OK)
        {
            published_status = feedback_status;
            all_control_valid = false;
        }
        else if (output_status != MOTOR_SPEED_CONTROL_OK)
        {
            published_status = output_status;
            all_control_valid = false;
        }
        else if (control_status != MOTOR_SPEED_CONTROL_OK && control_status != MOTOR_SPEED_CONTROL_DISABLED)
        {
            all_control_valid = false;
        }
        if (published_status != MOTOR_SPEED_CONTROL_OK)
        {
            current_command_a = 0.0F;
        }

        // 保存统一结果快照，调用方不再读取控制器内部状态
        snapshot->control_status[motor_index] = published_status;
        snapshot->motor_enabled[motor_index] = speed_control->feedback.enabled;
        snapshot->limited_target_speed_rpm[motor_index] = speed_control->feedback.limited_target_speed_rpm;
        snapshot->ramped_target_speed_rpm[motor_index] = speed_control->feedback.target_speed_rpm;
        snapshot->actual_speed_rpm[motor_index] = speed_control->feedback.actual_speed_rpm;
        snapshot->current_command_a[motor_index] = current_command_a;
    }
    return all_control_valid ? CHASSIS_OK : CHASSIS_ERROR;
}

/**
 * @brief 写入四路电流槽位并统一发送一次 CAN 控制帧
 *
 * @param[in,out] snapshot 本周期底盘结果快照
 * @return 四路写入和统一发送均成功返回 CHASSIS_OK，否则返回 CHASSIS_ERROR
 */
static int8_t Chassis_WriteCurrent(Chassis_Snapshot_t *snapshot)
{
    bool all_current_set = true;
    for (uint32_t motor_index = 0U; motor_index < CHASSIS_MOTOR_COUNT; motor_index++)
    {
        if (chassis_motor[motor_index] == NULL)
        {
            snapshot->current_command_a[motor_index] = 0.0F;
            all_current_set = false;
            continue;
        }

        // 非法值或写入失败时立即用零电流覆盖该路旧命令
        if (!isfinite(snapshot->current_command_a[motor_index]))
        {
            snapshot->current_set_status[motor_index] = CHASSIS_INVALID_CURRENT;
            const int8_t zero_status = MOTOR_RM_SetTorqueCurrent(&chassis_motor_param[motor_index], 0.0F);
            if (zero_status != DEVICE_OK)
            {
                snapshot->current_set_status[motor_index] = zero_status;
            }
            snapshot->current_command_a[motor_index] = 0.0F;
            all_current_set = false;
            continue;
        }

        snapshot->current_set_status[motor_index] =
            MOTOR_RM_SetTorqueCurrent(&chassis_motor_param[motor_index], snapshot->current_command_a[motor_index]);
        if (snapshot->current_set_status[motor_index] != DEVICE_OK)
        {
            const int8_t zero_status = MOTOR_RM_SetTorqueCurrent(&chassis_motor_param[motor_index], 0.0F);
            if (zero_status != DEVICE_OK)
            {
                snapshot->current_set_status[motor_index] = zero_status;
            }
            snapshot->current_command_a[motor_index] = 0.0F;
            all_current_set = false;
        }
    }

    // 四路槽位全部处理后只发送一次同组控制帧，保证控制命令属于同一周期快照
    snapshot->can_tx_status = MOTOR_RM_FlushGroup(&chassis_motor_param[0]);
    if (snapshot->can_tx_status != DEVICE_OK)
    {
        return CHASSIS_ERROR;
    }
    return all_current_set ? CHASSIS_OK : CHASSIS_ERROR;
}
