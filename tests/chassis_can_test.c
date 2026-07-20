#include "module/chassis_can.h"

#include "bsp/can.h"
#include "device/motor_rm.h"

#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

static MOTOR_RM_t mock_motor[CHASSIS_CAN_MOTOR_COUNT];
static int8_t mock_update_status[CHASSIS_CAN_MOTOR_COUNT];
static float mock_set_value[CHASSIS_CAN_MOTOR_COUNT][2];
static uint32_t mock_set_count[CHASSIS_CAN_MOTOR_COUNT];
static uint32_t mock_register_count;
static uint32_t mock_flush_count;
static int32_t mock_register_fail_index;
static int32_t mock_get_fail_index;
static int32_t mock_set_fail_index;
static int8_t mock_can_init_status;
static int8_t mock_flush_status;

static void ChassisCANTest_ResetMocks(void);
static uint32_t ChassisCANTest_GetMotorIndex(const MOTOR_RM_Param_t *param);
static ChassisCAN_t ChassisCANTest_Initialize(void);
static void ChassisCANTest_InitializesFixedMotorSet(void);
static void ChassisCANTest_IsolatesRegistrationFailure(void);
static void ChassisCANTest_IsolatesFeedbackFailures(void);
static void ChassisCANTest_WritesFourSlotsThenFlushesOnce(void);
static void ChassisCANTest_OverwritesFailedSlotWithZero(void);
static void ChassisCANTest_RejectsNonFiniteCurrent(void);
static void ChassisCANTest_RejectsUninitializedAccess(void);
static void ChassisCANTest_StopsWhenCANInitializationFails(void);

/**
 * @brief 运行四电机 CAN 边界主机测试
 *
 * @return 全部断言通过时返回 0
 */
int main(void)
{
    ChassisCANTest_InitializesFixedMotorSet();
    ChassisCANTest_IsolatesRegistrationFailure();
    ChassisCANTest_IsolatesFeedbackFailures();
    ChassisCANTest_WritesFourSlotsThenFlushesOnce();
    ChassisCANTest_OverwritesFailedSlotWithZero();
    ChassisCANTest_RejectsNonFiniteCurrent();
    ChassisCANTest_RejectsUninitializedAccess();
    ChassisCANTest_StopsWhenCANInitializationFails();

    puts("Chassis CAN tests passed");
    return 0;
}

/**
 * @brief 模拟初始化 CAN 总线
 *
 * @return 当前测试配置的 CAN 初始化状态
 */
int8_t BSP_CAN_Init(void)
{
    return mock_can_init_status;
}

/**
 * @brief 模拟注册一个 RM 电机并校验固定参数
 *
 * @param[in] param 待注册电机参数
 * @return 当前测试配置的逐路注册状态
 */
int8_t MOTOR_RM_Register(MOTOR_RM_Param_t *param)
{
    const uint32_t motor_index = ChassisCANTest_GetMotorIndex(param);
    assert(param->can == BSP_CAN_1);
    assert(param->module == MOTOR_M3508);
    assert(!param->reverse);
    assert(param->gear);
    mock_register_count++;
    return (int32_t)motor_index == mock_register_fail_index ? DEVICE_ERR : DEVICE_OK;
}

/**
 * @brief 返回对应 ID 的模拟 RM 电机实例
 *
 * @param[in] param 电机参数
 * @return 正常返回模拟实例，当前测试要求失败时返回空指针
 */
MOTOR_RM_t *MOTOR_RM_GetMotor(MOTOR_RM_Param_t *param)
{
    const uint32_t motor_index = ChassisCANTest_GetMotorIndex(param);
    return (int32_t)motor_index == mock_get_fail_index ? NULL : &mock_motor[motor_index];
}

/**
 * @brief 返回对应电机的模拟反馈更新状态
 *
 * @param[in] param 电机参数
 * @return 当前测试配置的更新状态
 */
int8_t MOTOR_RM_Update(MOTOR_RM_Param_t *param)
{
    return mock_update_status[ChassisCANTest_GetMotorIndex(param)];
}

/**
 * @brief 记录对应电机发送槽位的电流写入
 *
 * @param[in] param 电机参数
 * @param[in] current_a 转子侧目标电流，单位 A
 * @return 首次指定写入可按测试配置返回失败，其余写入返回成功
 */
int8_t MOTOR_RM_SetTorqueCurrent(MOTOR_RM_Param_t *param, float current_a)
{
    const uint32_t motor_index = ChassisCANTest_GetMotorIndex(param);
    assert(mock_set_count[motor_index] < 2U);
    mock_set_value[motor_index][mock_set_count[motor_index]] = current_a;
    mock_set_count[motor_index]++;
    if ((int32_t)motor_index == mock_set_fail_index && mock_set_count[motor_index] == 1U)
    {
        return DEVICE_ERR;
    }
    return DEVICE_OK;
}

/**
 * @brief 记录四电机控制帧统一发送次数
 *
 * @param[in] param 用于定位控制帧组的电机参数
 * @return 当前测试配置的发送状态
 */
int8_t MOTOR_RM_FlushGroup(MOTOR_RM_Param_t *param)
{
    assert(ChassisCANTest_GetMotorIndex(param) == 0U);
    mock_flush_count++;
    return mock_flush_status;
}

/**
 * @brief 清空全部模拟状态并恢复成功默认值
 *
 * @return 无返回值
 */
static void ChassisCANTest_ResetMocks(void)
{
    memset(mock_motor, 0, sizeof(mock_motor));
    memset(mock_set_value, 0, sizeof(mock_set_value));
    memset(mock_set_count, 0, sizeof(mock_set_count));
    mock_register_count = 0U;
    mock_flush_count = 0U;
    mock_register_fail_index = -1;
    mock_get_fail_index = -1;
    mock_set_fail_index = -1;
    mock_can_init_status = BSP_OK;
    mock_flush_status = DEVICE_OK;
    for (uint32_t motor_index = 0U; motor_index < CHASSIS_CAN_MOTOR_COUNT; motor_index++)
    {
        mock_update_status[motor_index] = DEVICE_OK;
    }
}

/**
 * @brief 根据固定反馈 ID 获取测试电机索引
 *
 * @param[in] param 电机参数
 * @return 0~3 的测试电机索引
 */
static uint32_t ChassisCANTest_GetMotorIndex(const MOTOR_RM_Param_t *param)
{
    assert(param != NULL);
    assert(param->id >= 0x201U && param->id <= 0x204U);
    return param->id - 0x201U;
}

/**
 * @brief 使用成功默认值初始化待测 CAN 边界
 *
 * @return 已初始化的边界上下文
 */
static ChassisCAN_t ChassisCANTest_Initialize(void)
{
    ChassisCAN_t context;
    ChassisCANTest_ResetMocks();
    assert(ChassisCAN_Init(&context) == CHASSIS_CAN_OK);
    return context;
}

/**
 * @brief 验证边界固定注册 CAN1 上 ID 1~4 的四个 M3508
 *
 * @return 无返回值
 */
static void ChassisCANTest_InitializesFixedMotorSet(void)
{
    const ChassisCAN_t context = ChassisCANTest_Initialize();
    assert(context.initialized);
    assert(mock_register_count == CHASSIS_CAN_MOTOR_COUNT);
    for (uint32_t motor_index = 0U; motor_index < CHASSIS_CAN_MOTOR_COUNT; motor_index++)
    {
        assert(context.register_status[motor_index] == DEVICE_OK);
    }

    assert(ChassisCAN_Init(NULL) == CHASSIS_CAN_NULL_ERROR);
}

/**
 * @brief 验证单路注册失败后其余设备仍可写槽位并统一发送
 *
 * @return 无返回值
 */
static void ChassisCANTest_IsolatesRegistrationFailure(void)
{
    ChassisCAN_t context;
    const float current_command_a[CHASSIS_CAN_MOTOR_COUNT] =
    {
        1.0F,
        2.0F,
        3.0F,
        4.0F,
    };
    ChassisCAN_OutputStatus_t status;
    ChassisCANTest_ResetMocks();
    mock_register_fail_index = 2;

    assert(ChassisCAN_Init(&context) == CHASSIS_CAN_ERROR);
    assert(context.initialized);
    assert(context.register_status[2] == DEVICE_ERR);
    assert(ChassisCAN_WriteCurrent(&context, current_command_a, &status) == CHASSIS_CAN_ERROR);
    assert(mock_set_count[0] == 1U);
    assert(mock_set_count[1] == 1U);
    assert(mock_set_count[2] == 0U);
    assert(mock_set_count[3] == 1U);
    assert(status.current_set_status[2] == CHASSIS_CAN_DEVICE_UNAVAILABLE);
    assert(mock_flush_count == 1U);
}

/**
 * @brief 验证单路无新帧或离线不会阻止其他反馈形成快照
 *
 * @return 无返回值
 */
static void ChassisCANTest_IsolatesFeedbackFailures(void)
{
    const ChassisCAN_t context = ChassisCANTest_Initialize();
    ChassisCAN_Feedback_t feedback;
    for (uint32_t motor_index = 0U; motor_index < CHASSIS_CAN_MOTOR_COUNT; motor_index++)
    {
        mock_motor[motor_index].motor.header.online = true;
        mock_motor[motor_index].feedback.rotor_speed = 100.0F + (float)motor_index;
        mock_motor[motor_index].feedback.temp = 30.0F + (float)motor_index;
    }
    mock_update_status[1] = DEVICE_ERR;
    mock_update_status[2] = DEVICE_ERR_NO_DEV;
    mock_motor[2].motor.header.online = false;

    assert(ChassisCAN_ReadFeedback(&context, &feedback) == CHASSIS_CAN_ERROR);
    assert(feedback.motor_online[0]);
    assert(feedback.actual_speed_rpm[0] == 100.0F);
    assert(feedback.feedback_update_status[1] == DEVICE_ERR);
    assert(feedback.motor_online[1]);
    assert(feedback.actual_speed_rpm[1] == 101.0F);
    assert(!feedback.motor_online[2]);
    assert(feedback.actual_speed_rpm[2] == 102.0F);
    assert(feedback.temperature_c[2] == 32.0F);
    assert(feedback.motor_online[3]);
    assert(feedback.actual_speed_rpm[3] == 103.0F);
}

/**
 * @brief 验证四路电流各写一次后只统一发送一次控制帧
 *
 * @return 无返回值
 */
static void ChassisCANTest_WritesFourSlotsThenFlushesOnce(void)
{
    const ChassisCAN_t context = ChassisCANTest_Initialize();
    const float current_command_a[CHASSIS_CAN_MOTOR_COUNT] =
    {
        1.0F,
        -2.0F,
        3.0F,
        -4.0F,
    };
    ChassisCAN_OutputStatus_t status;

    assert(ChassisCAN_WriteCurrent(&context, current_command_a, &status) == CHASSIS_CAN_OK);
    for (uint32_t motor_index = 0U; motor_index < CHASSIS_CAN_MOTOR_COUNT; motor_index++)
    {
        assert(mock_set_count[motor_index] == 1U);
        assert(mock_set_value[motor_index][0] == current_command_a[motor_index]);
        assert(status.current_set_status[motor_index] == DEVICE_OK);
    }
    assert(mock_flush_count == 1U);
    assert(status.can_tx_status == DEVICE_OK);
}

/**
 * @brief 验证单路写入失败会在统一发送前用零电流覆盖旧槽位
 *
 * @return 无返回值
 */
static void ChassisCANTest_OverwritesFailedSlotWithZero(void)
{
    const ChassisCAN_t context = ChassisCANTest_Initialize();
    const float current_command_a[CHASSIS_CAN_MOTOR_COUNT] =
    {
        1.0F,
        2.0F,
        3.0F,
        4.0F,
    };
    ChassisCAN_OutputStatus_t status;
    mock_set_fail_index = 1;

    assert(ChassisCAN_WriteCurrent(&context, current_command_a, &status) == CHASSIS_CAN_ERROR);
    assert(mock_set_count[1] == 2U);
    assert(mock_set_value[1][0] == 2.0F);
    assert(mock_set_value[1][1] == 0.0F);
    assert(status.current_set_status[1] == DEVICE_ERR);
    assert(mock_set_count[0] == 1U);
    assert(mock_set_count[2] == 1U);
    assert(mock_set_count[3] == 1U);
    assert(mock_flush_count == 1U);
}

/**
 * @brief 验证非有限电流只写入零值并报告输入错误
 *
 * @return 无返回值
 */
static void ChassisCANTest_RejectsNonFiniteCurrent(void)
{
    const ChassisCAN_t context = ChassisCANTest_Initialize();
    const float current_command_a[CHASSIS_CAN_MOTOR_COUNT] =
    {
        1.0F,
        2.0F,
        NAN,
        4.0F,
    };
    ChassisCAN_OutputStatus_t status;

    assert(ChassisCAN_WriteCurrent(&context, current_command_a, &status) == CHASSIS_CAN_ERROR);
    assert(mock_set_count[2] == 1U);
    assert(mock_set_value[2][0] == 0.0F);
    assert(status.current_set_status[2] == CHASSIS_CAN_INVALID_CURRENT);
    assert(mock_flush_count == 1U);
}

/**
 * @brief 验证未初始化或空输入不会写槽位及发送控制帧
 *
 * @return 无返回值
 */
static void ChassisCANTest_RejectsUninitializedAccess(void)
{
    ChassisCANTest_ResetMocks();
    const ChassisCAN_t context =
    {
        0,
    };
    const float current_command_a[CHASSIS_CAN_MOTOR_COUNT] =
    {
        1.0F,
        2.0F,
        3.0F,
        4.0F,
    };
    ChassisCAN_OutputStatus_t status;
    ChassisCAN_Feedback_t feedback;

    assert(ChassisCAN_WriteCurrent(&context, current_command_a, &status) == CHASSIS_CAN_NOT_INITIALIZED);
    assert(status.can_tx_status == CHASSIS_CAN_DEVICE_UNAVAILABLE);
    assert(mock_flush_count == 0U);
    assert(ChassisCAN_ReadFeedback(&context, &feedback) == CHASSIS_CAN_NOT_INITIALIZED);
    assert(!feedback.motor_online[0]);
    assert(ChassisCAN_WriteCurrent(&context, current_command_a, NULL) == CHASSIS_CAN_NULL_ERROR);
}

/**
 * @brief 验证 CAN 初始化失败时不注册设备且上下文保持不可用
 *
 * @return 无返回值
 */
static void ChassisCANTest_StopsWhenCANInitializationFails(void)
{
    ChassisCAN_t context;
    ChassisCANTest_ResetMocks();
    mock_can_init_status = BSP_ERR;

    assert(ChassisCAN_Init(&context) == BSP_ERR);
    assert(!context.initialized);
    assert(mock_register_count == 0U);
}
