#include "module/chassis.h"

#include <assert.h>
#include <math.h>
#include <stdio.h>

static Chassis_Input_t ChassisTest_CreateValidInput(void);
static void ChassisTest_InitializesFourControllers(void);
static void ChassisTest_ControlsFourMotorsIndependently(void);
static void ChassisTest_IsolatesOfflineMotor(void);
static void ChassisTest_DisablesAllOutputs(void);
static void ChassisTest_IsolatesInvalidFeedback(void);
static void ChassisTest_ReturnsSafeOutputOnInvalidArguments(void);

/**
 * @brief 运行 Chassis 四路组合控制主机测试
 *
 * @return 全部断言通过时返回 0
 */
int main(void)
{
    ChassisTest_InitializesFourControllers();
    ChassisTest_ControlsFourMotorsIndependently();
    ChassisTest_IsolatesOfflineMotor();
    ChassisTest_DisablesAllOutputs();
    ChassisTest_IsolatesInvalidFeedback();
    ChassisTest_ReturnsSafeOutputOnInvalidArguments();

    puts("Chassis tests passed");
    return 0;
}

/**
 * @brief 创建四路在线且启用的合法测试输入
 *
 * @return 合法 Chassis 输入快照
 */
static Chassis_Input_t ChassisTest_CreateValidInput(void)
{
    Chassis_Input_t input =
    {
        .enabled = true,
        .requested_speed_rpm =
        {
            100.0F,
            -100.0F,
            50.0F,
            -50.0F,
        },
        .actual_speed_rpm =
        {
            0.0F,
            0.0F,
            0.0F,
            0.0F,
        },
        .motor_online =
        {
            true,
            true,
            true,
            true,
        },
        .pid_tune =
        {
            .kp = MOTOR_SPEED_PID_KP,
            .ki = MOTOR_SPEED_PID_KI,
            .kd = MOTOR_SPEED_PID_KD,
        },
        .control_period_s = 0.002F,
    };
    return input;
}

/**
 * @brief 验证四个速度控制器全部初始化并记录独立状态
 *
 * @return 无返回值
 */
static void ChassisTest_InitializesFourControllers(void)
{
    Chassis_t chassis;
    assert(Chassis_Init(&chassis, 500.0F) == CHASSIS_OK);
    assert(chassis.initialized);
    for (uint32_t motor_index = 0U; motor_index < CHASSIS_MOTOR_COUNT; motor_index++)
    {
        assert(chassis.init_status[motor_index] == MOTOR_SPEED_CONTROL_OK);
        assert(chassis.speed_control[motor_index].feedback.initialized);
    }

    assert(Chassis_Init(NULL, 500.0F) == CHASSIS_NULL_ERROR);
    assert(Chassis_Init(&chassis, NAN) == CHASSIS_CONFIG_ERROR);
    assert(!chassis.initialized);
}

/**
 * @brief 验证四路目标分别驱动独立速度控制器
 *
 * @return 无返回值
 */
static void ChassisTest_ControlsFourMotorsIndependently(void)
{
    Chassis_t chassis;
    Chassis_Output_t output;
    Chassis_Input_t input = ChassisTest_CreateValidInput();
    assert(Chassis_Init(&chassis, 500.0F) == CHASSIS_OK);

    assert(Chassis_Control(&chassis, &input, &output) == CHASSIS_OK);
    assert(output.control_status[0] == MOTOR_SPEED_CONTROL_OK);
    assert(output.control_status[1] == MOTOR_SPEED_CONTROL_OK);
    assert(output.control_status[2] == MOTOR_SPEED_CONTROL_OK);
    assert(output.control_status[3] == MOTOR_SPEED_CONTROL_OK);
    assert(output.current_command_a[0] > 0.0F);
    assert(output.current_command_a[1] < 0.0F);
    assert(output.current_command_a[2] > 0.0F);
    assert(output.current_command_a[3] < 0.0F);
    assert(output.ramped_target_speed_rpm[0] == -output.ramped_target_speed_rpm[1]);
    assert(output.ramped_target_speed_rpm[2] == -output.ramped_target_speed_rpm[3]);
}

/**
 * @brief 验证单路离线只清零对应电流而不阻止其他控制器运行
 *
 * @return 无返回值
 */
static void ChassisTest_IsolatesOfflineMotor(void)
{
    Chassis_t chassis;
    Chassis_Output_t output;
    Chassis_Input_t input = ChassisTest_CreateValidInput();
    input.motor_online[1] = false;
    assert(Chassis_Init(&chassis, 500.0F) == CHASSIS_OK);

    assert(Chassis_Control(&chassis, &input, &output) == CHASSIS_OK);
    assert(output.control_status[1] == MOTOR_SPEED_CONTROL_DISABLED);
    assert(output.current_command_a[1] == 0.0F);
    assert(!output.motor_enabled[1]);
    assert(output.control_status[0] == MOTOR_SPEED_CONTROL_OK);
    assert(output.current_command_a[0] > 0.0F);
}

/**
 * @brief 验证统一禁用会清零全部输出且不报告组合控制错误
 *
 * @return 无返回值
 */
static void ChassisTest_DisablesAllOutputs(void)
{
    Chassis_t chassis;
    Chassis_Output_t output;
    Chassis_Input_t input = ChassisTest_CreateValidInput();
    input.enabled = false;
    assert(Chassis_Init(&chassis, 500.0F) == CHASSIS_OK);

    assert(Chassis_Control(&chassis, &input, &output) == CHASSIS_OK);
    for (uint32_t motor_index = 0U; motor_index < CHASSIS_MOTOR_COUNT; motor_index++)
    {
        assert(output.control_status[motor_index] == MOTOR_SPEED_CONTROL_DISABLED);
        assert(output.current_command_a[motor_index] == 0.0F);
        assert(!output.motor_enabled[motor_index]);
    }
}

/**
 * @brief 验证单路非法反馈被隔离且其余控制器继续运行
 *
 * @return 无返回值
 */
static void ChassisTest_IsolatesInvalidFeedback(void)
{
    Chassis_t chassis;
    Chassis_Output_t output;
    Chassis_Input_t input = ChassisTest_CreateValidInput();
    input.actual_speed_rpm[2] = NAN;
    assert(Chassis_Init(&chassis, 500.0F) == CHASSIS_OK);

    assert(Chassis_Control(&chassis, &input, &output) == CHASSIS_CONTROL_ERROR);
    assert(output.control_status[2] == MOTOR_SPEED_CONTROL_INVALID_VALUE);
    assert(output.current_command_a[2] == 0.0F);
    assert(output.control_status[0] == MOTOR_SPEED_CONTROL_OK);
    assert(output.current_command_a[0] > 0.0F);
}

/**
 * @brief 验证空指针和未初始化上下文始终返回安全零输出
 *
 * @return 无返回值
 */
static void ChassisTest_ReturnsSafeOutputOnInvalidArguments(void)
{
    Chassis_t chassis =
    {
        0,
    };
    Chassis_Output_t output;
    Chassis_Input_t input = ChassisTest_CreateValidInput();

    assert(Chassis_Control(NULL, &input, &output) == CHASSIS_NULL_ERROR);
    assert(output.current_command_a[0] == 0.0F);
    assert(Chassis_Control(&chassis, &input, &output) == CHASSIS_INIT_ERROR);
    assert(output.control_status[0] == MOTOR_SPEED_CONTROL_INIT_ERROR);
    assert(Chassis_Control(&chassis, &input, NULL) == CHASSIS_NULL_ERROR);
}
