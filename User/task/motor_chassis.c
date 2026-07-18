#include "task/motor_chassis.h"

#include "bsp/can.h"
#include "device/motor_rm.h"
#include "task/motor_speed_control.h"
#include "task/user_task.h"

/*
 * 调试停机确认参数：
 * - MOTOR_DEBUG_STOP_CONFIRM_CYCLES：关闭调试使能后，需要连续成功提交零电流帧的控制周期数。
 */
#define MOTOR_DEBUG_STOP_CONFIRM_CYCLES (5U)

/*
 * 单个 M3508 电机参数：
 * - CAN 总线：CAN1。
 * - C620 电调 ID：1，对应反馈标准帧 ID 0x201。
 * - 电机型号：M3508。
 * - 安装方向：当前不反向，最终方向需通过低速上板测试确认。
 * - 减速箱：启用，设备层将转子转速除以 3591/187 后输出 rpm。
 * @datasheet RoboMaster C620 用户手册“CAN 通信协议”章节
 */
static MOTOR_RM_Param_t motor_3508_param =
{
  .can = BSP_CAN_1,
  .id = 0x201U,
  .module = MOTOR_M3508,
  .reverse = false,
  .gear = true,
};

volatile MotorChassisTune_t g_motor_chassis_tune =
{
  .motor_debug_enable = true,
  .requested_speed_rpm = MOTOR_SPEED_TARGET_RPM,
  .pid_kp = MOTOR_SPEED_PID_KP,
  .pid_ki = MOTOR_SPEED_PID_KI,
  .pid_kd = MOTOR_SPEED_PID_KD,
  .actual_speed_rpm = 0.0F,
};

volatile MotorChassisMonitor_t g_motor_chassis_monitor;

/*
 * 单个 M3508 电机任务变量：
 * - motor_3508：注册后的 M3508 电机实例，初始化失败时为 NULL。
 * - motor_speed_control：速度环主结构体，包含目标、反馈、PID、缓启动状态和输出。
 * - motor_current_command_a：本周期需要下发的转子侧电流指令，单位 A。
 * - debug_stop_zero_tx_count：关闭调试使能后连续成功提交零电流帧的周期数。
 */
#ifdef DEBUG
MOTOR_RM_t *motor_3508;
MotorSpeedControl_t motor_speed_control;
float motor_current_command_a;
#else
static MOTOR_RM_t *motor_3508;
static MotorSpeedControl_t motor_speed_control;
static float motor_current_command_a;
#endif
static uint32_t debug_stop_zero_tx_count;

static void MotorChassis_UpdateOzoneData(int8_t current_set_status, int8_t can_tx_status, bool motor_debug_enable);

/**
 * @brief 运行单个 M3508 电机速度控制任务
 *
 * @param[in] argument 任务参数，本任务不使用
 * @return 本任务不会返回
 */
void Task_motor_chassis(void *argument)
{
  (void)argument;

  const uint32_t delay_tick = osKernelGetTickFreq() / MOTOR_CHASSIS_FREQ;
  const float control_period_s = 1.0F / (float)MOTOR_CHASSIS_FREQ;
  osDelay(MOTOR_CHASSIS_INIT_DELAY);

  BSP_CAN_Init();
  g_motor_chassis_monitor.register_status = MOTOR_RM_Register(&motor_3508_param);
  motor_3508 = MOTOR_RM_GetMotor(&motor_3508_param);
  g_motor_chassis_monitor.control_init_status = MotorSpeedControl_Init(&motor_speed_control, (float)MOTOR_CHASSIS_FREQ);

  uint32_t tick = osKernelGetTickCount();
  while (1)
  {
    int8_t current_set_status = DEVICE_ERR_NO_DEV;
    int8_t can_tx_status = DEVICE_ERR_NO_DEV;

    // 从 CAN 接收缓存更新 M3508 电机反馈
    if (motor_3508 != NULL)
      MOTOR_RM_Update(&motor_3508_param);
    const float actual_speed_rpm = motor_3508 != NULL ? motor_3508->feedback.rotor_speed : 0.0F;
    const bool motor_debug_enable = g_motor_chassis_tune.motor_debug_enable;
    const bool speed_control_enabled = motor_debug_enable && motor_3508 != NULL && motor_3508->motor.header.online;
    const float requested_speed_rpm = g_motor_chassis_tune.requested_speed_rpm;
    const MotorSpeedPidTune_t pid_tune =
    {
      .kp = g_motor_chassis_tune.pid_kp,
      .ki = g_motor_chassis_tune.pid_ki,
      .kd = g_motor_chassis_tune.pid_kd,
    };

    // 将真实输出轴速度写入速度环反馈结构体
    MotorSpeedControl_UpdateFeedback(&motor_speed_control, actual_speed_rpm);
    // 根据目标速度和真实速度计算本周期电流指令
    MotorSpeedControl_Control(&motor_speed_control, requested_speed_rpm, &pid_tune, speed_control_enabled,
                              control_period_s);
    // 从速度环主结构体导出转子侧电流指令
    MotorSpeedControl_DumpOutput(&motor_speed_control, &motor_current_command_a);

    if (motor_3508 != NULL)
    {
      // 将速度环输出写入 M3508 电流发送缓存
      current_set_status = MOTOR_RM_SetTorqueCurrent(&motor_3508_param, motor_current_command_a);
      if (current_set_status == DEVICE_OK)
      {
        // 发送当前 M3508 所属控制组的 CAN 控制帧
        can_tx_status = MOTOR_RM_Ctrl(&motor_3508_param);
      }
    }

    MotorChassis_UpdateOzoneData(current_set_status, can_tx_status, motor_debug_enable);

    tick += delay_tick;
    // 等待至下一个绝对任务周期，避免累计调度漂移
    osDelayUntil(tick);
  }
}

/**
 * @brief 刷新 Ozone 调参与监视数据
 *
 * @param[in] current_set_status 本周期电流指令写入结果
 * @param[in] can_tx_status 本周期 CAN 控制帧发送结果
 * @param[in] motor_debug_enable 本周期采用的电机调试使能状态
 * @return 无返回值
 */
static void MotorChassis_UpdateOzoneData(int8_t current_set_status, int8_t can_tx_status, bool motor_debug_enable)
{
  const bool motor_online = motor_3508 != NULL && motor_3508->motor.header.online;

  if (!motor_debug_enable && motor_current_command_a == 0.0F && current_set_status == DEVICE_OK &&
      can_tx_status == DEVICE_OK)
  {
    if (debug_stop_zero_tx_count < MOTOR_DEBUG_STOP_CONFIRM_CYCLES)
    {
      debug_stop_zero_tx_count++;
    }
  }
  else
  {
    debug_stop_zero_tx_count = 0U;
  }

  g_motor_chassis_monitor.motor_online = motor_online;
  g_motor_chassis_monitor.current_saturated =
      motor_current_command_a >= MOTOR_SPEED_CURRENT_LIMIT_A || motor_current_command_a <= -MOTOR_SPEED_CURRENT_LIMIT_A;
  g_motor_chassis_monitor.debug_stop_ready = debug_stop_zero_tx_count >= MOTOR_DEBUG_STOP_CONFIRM_CYCLES;
  g_motor_chassis_monitor.control_status = motor_speed_control.feedback.status;
  g_motor_chassis_monitor.can_tx_status = can_tx_status;
  g_motor_chassis_monitor.debug_stop_zero_tx_count = debug_stop_zero_tx_count;
  g_motor_chassis_monitor.limited_target_speed_rpm = motor_speed_control.feedback.limited_target_speed_rpm;
  g_motor_chassis_monitor.ramped_target_speed_rpm = motor_speed_control.feedback.target_speed_rpm;
  g_motor_chassis_tune.actual_speed_rpm = motor_speed_control.feedback.actual_speed_rpm;
  g_motor_chassis_monitor.current_command_a = motor_current_command_a;

  if (motor_3508 != NULL)
  {
    g_motor_chassis_monitor.temperature_c = motor_3508->feedback.temp;
  }
  else
  {
    g_motor_chassis_monitor.temperature_c = 0.0F;
  }
}
