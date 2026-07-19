#include "task/motor_chassis.h"

#include "bsp/can.h"
#include "device/motor_rm.h"
#include "task/motor_speed_control.h"
#include "task/user_task.h"

/*
 * 调试停机确认参数：
 * - MOTOR_DEBUG_STOP_CONFIRM_CYCLES：关闭调试使能后，需要连续成功提交四电机零电流帧的控制周期数。
 */
#define MOTOR_DEBUG_STOP_CONFIRM_CYCLES (5U)

/*
 * 四个 M3508 电机参数：
 * - CAN 总线：全部使用 CAN1。
 * - C620 电调 ID：依次为 1~4，对应反馈标准帧 ID 0x201~0x204。
 * - 电机型号：全部为 M3508。
 * - 安装方向：当前全部不反向，最终方向需通过低速上板测试确认。
 * - 减速箱：全部启用，设备层将转子转速除以 3591/187 后输出 rpm。
 * @datasheet RoboMaster C620 用户手册“CAN 通信协议”章节
 */
static MOTOR_RM_Param_t motor_3508_param[MOTOR_CHASSIS_MOTOR_COUNT] =
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

/**
 * @brief 四个 M3508 的在线调试参数初值
 *
 * 上电后默认打开速度环调试，使四个电机使用统一的目标转速和 PID 参数。
 * 实际速度数组由任务周期刷新，其他字段可通过 Ozone 在线调整。
 */
volatile MotorChassisTune_t g_motor_chassis_tune =
{
  .motor_debug_enable = true,
  .requested_speed_rpm =
  {
    MOTOR_SPEED_TARGET_RPM,
    MOTOR_SPEED_TARGET_RPM,
    MOTOR_SPEED_TARGET_RPM,
    MOTOR_SPEED_TARGET_RPM,
  },
  .pid_kp = MOTOR_SPEED_PID_KP,
  .pid_ki = MOTOR_SPEED_PID_KI,
  .pid_kd = MOTOR_SPEED_PID_KD,
  .actual_speed_rpm =
  {
    0.0F,
    0.0F,
    0.0F,
    0.0F,
  },
};

/**
 * @brief 四个 M3508 的运行监视数据初值
 *
 * 任务启动前所有电机均视为未注册、未初始化且未收到反馈；数值反馈清零，
 * 发送状态置为“无设备”，避免把尚未运行的状态误认为有效数据。
 */
volatile MotorChassisMonitor_t g_motor_chassis_monitor =
{
  .motor_online = {false, false, false, false},
  .current_saturated = {false, false, false, false},
  .debug_stop_ready = false,
  .register_status = {DEVICE_ERR_NO_DEV, DEVICE_ERR_NO_DEV, DEVICE_ERR_NO_DEV, DEVICE_ERR_NO_DEV},
  .control_init_status = {MOTOR_SPEED_CONTROL_INIT_ERROR, MOTOR_SPEED_CONTROL_INIT_ERROR,
                          MOTOR_SPEED_CONTROL_INIT_ERROR, MOTOR_SPEED_CONTROL_INIT_ERROR},
  .feedback_update_status = {DEVICE_ERR_NO_DEV, DEVICE_ERR_NO_DEV, DEVICE_ERR_NO_DEV, DEVICE_ERR_NO_DEV},
  .current_set_status = {DEVICE_ERR_NO_DEV, DEVICE_ERR_NO_DEV, DEVICE_ERR_NO_DEV, DEVICE_ERR_NO_DEV},
  .control_status = {MOTOR_SPEED_CONTROL_INIT_ERROR, MOTOR_SPEED_CONTROL_INIT_ERROR, MOTOR_SPEED_CONTROL_INIT_ERROR,
                     MOTOR_SPEED_CONTROL_INIT_ERROR},
  .can_tx_status = DEVICE_ERR_NO_DEV,
  .debug_stop_zero_tx_count = 0U,
  .limited_target_speed_rpm = {0.0F, 0.0F, 0.0F, 0.0F},
  .ramped_target_speed_rpm = {0.0F, 0.0F, 0.0F, 0.0F},
  .current_command_a = {0.0F, 0.0F, 0.0F, 0.0F},
  .temperature_c = {0.0F, 0.0F, 0.0F, 0.0F},
};

/*
 * 四个 M3508 电机任务变量：
 * - motor_3508[0~3]：注册后的 M3508 电机实例，初始化失败时对应元素为 NULL。
 * - motor_speed_control[0~3]：各电机独立的速度环主结构体，包含目标、反馈、PID、缓启动状态和输出。
 * - motor_current_command_a[0~3]：本周期需要下发的转子侧电流指令，单位 A。
 * - debug_stop_zero_tx_count：关闭调试使能后连续成功提交四电机零电流帧的周期数。
 */
#ifdef DEBUG
MOTOR_RM_t *motor_3508[MOTOR_CHASSIS_MOTOR_COUNT];
MotorSpeedControl_t motor_speed_control[MOTOR_CHASSIS_MOTOR_COUNT];
float motor_current_command_a[MOTOR_CHASSIS_MOTOR_COUNT];
#else
static MOTOR_RM_t *motor_3508[MOTOR_CHASSIS_MOTOR_COUNT];
static MotorSpeedControl_t motor_speed_control[MOTOR_CHASSIS_MOTOR_COUNT];
static float motor_current_command_a[MOTOR_CHASSIS_MOTOR_COUNT];
#endif
static uint32_t debug_stop_zero_tx_count;

static void MotorChassis_UpdateOzoneData(const int8_t feedback_update_status[MOTOR_CHASSIS_MOTOR_COUNT],
                                         const int8_t current_set_status[MOTOR_CHASSIS_MOTOR_COUNT],
                                         int8_t can_tx_status, bool motor_debug_enable);

/**
 * @brief 初始化并周期运行四个 M3508 的速度控制链
 *
 * 本任务按“读取反馈、计算速度环、写入电流缓存、发送 CAN 帧”的顺序执行。
 * 调试使能关闭或电机反馈离线时，速度环输出被清零，仍发送零电流控制帧。
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
  for (uint32_t motor_index = 0U; motor_index < MOTOR_CHASSIS_MOTOR_COUNT; motor_index++)
  {
    g_motor_chassis_monitor.register_status[motor_index] = MOTOR_RM_Register(&motor_3508_param[motor_index]);
    motor_3508[motor_index] = MOTOR_RM_GetMotor(&motor_3508_param[motor_index]);
    g_motor_chassis_monitor.control_init_status[motor_index] =
        MotorSpeedControl_Init(&motor_speed_control[motor_index], (float)MOTOR_CHASSIS_FREQ);
  }

  uint32_t tick = osKernelGetTickCount();
  while (1)
  {
    int8_t feedback_update_status[MOTOR_CHASSIS_MOTOR_COUNT];
    int8_t current_set_status[MOTOR_CHASSIS_MOTOR_COUNT];
    const bool motor_debug_enable = g_motor_chassis_tune.motor_debug_enable;
    const MotorSpeedPidTune_t pid_tune =
    {
      .kp = g_motor_chassis_tune.pid_kp,
      .ki = g_motor_chassis_tune.pid_ki,
      .kd = g_motor_chassis_tune.pid_kd,
    };

    for (uint32_t motor_index = 0U; motor_index < MOTOR_CHASSIS_MOTOR_COUNT; motor_index++)
    {
      feedback_update_status[motor_index] = DEVICE_ERR_NO_DEV;
      current_set_status[motor_index] = DEVICE_ERR_NO_DEV;

      // 从 CAN 接收缓存更新对应 M3508 电机反馈
      if (motor_3508[motor_index] != NULL)
      {
        feedback_update_status[motor_index] = MOTOR_RM_Update(&motor_3508_param[motor_index]);
      }

      const float actual_speed_rpm =
          motor_3508[motor_index] != NULL ? motor_3508[motor_index]->feedback.rotor_speed : 0.0F;
      const bool speed_control_enabled =
          motor_debug_enable && motor_3508[motor_index] != NULL && motor_3508[motor_index]->motor.header.online;
      const float requested_speed_rpm = g_motor_chassis_tune.requested_speed_rpm[motor_index];

      // 将对应输出轴真实速度写入独立速度环
      MotorSpeedControl_UpdateFeedback(&motor_speed_control[motor_index], actual_speed_rpm);
      // 根据对应目标速度和真实速度计算本周期电流指令
      MotorSpeedControl_Control(&motor_speed_control[motor_index], requested_speed_rpm, &pid_tune,
                                speed_control_enabled, control_period_s);
      MotorSpeedControl_DumpOutput(&motor_speed_control[motor_index], &motor_current_command_a[motor_index]);

      if (motor_3508[motor_index] != NULL)
      {
        // 将速度环输出写入对应 M3508 电流发送缓存
        current_set_status[motor_index] =
            MOTOR_RM_SetTorqueCurrent(&motor_3508_param[motor_index], motor_current_command_a[motor_index]);
      }
    }

    // 四个电流槽位更新完成后，统一发送一次 0x200 控制帧
    const int8_t can_tx_status = MOTOR_RM_FlushGroup(&motor_3508_param[0]);
    MotorChassis_UpdateOzoneData(feedback_update_status, current_set_status, can_tx_status, motor_debug_enable);

    tick += delay_tick;
    // 等待至下一个绝对任务周期，避免累计调度漂移
    osDelayUntil(tick);
  }
}

/**
 * @brief 汇总本周期结果并刷新 Ozone 四电机诊断数据
 *
 * 除更新每个电机的反馈、控制和电流状态外，还统计连续成功发送零电流帧的周期数，
 * 用于确认关闭调试使能后的安全停机状态。
 *
 * @param[in] feedback_update_status 本周期四电机反馈更新结果
 * @param[in] current_set_status 本周期四电机电流指令写入结果
 * @param[in] can_tx_status 本周期 CAN 控制帧发送结果
 * @param[in] motor_debug_enable 本周期采用的四电机全局调试使能状态
 * @return 无返回值
 */
static void MotorChassis_UpdateOzoneData(const int8_t feedback_update_status[MOTOR_CHASSIS_MOTOR_COUNT],
                                         const int8_t current_set_status[MOTOR_CHASSIS_MOTOR_COUNT],
                                         int8_t can_tx_status, bool motor_debug_enable)
{
  bool all_zero_current_set = !motor_debug_enable && can_tx_status == DEVICE_OK;

  for (uint32_t motor_index = 0U; motor_index < MOTOR_CHASSIS_MOTOR_COUNT; motor_index++)
  {
    const bool motor_online = motor_3508[motor_index] != NULL && motor_3508[motor_index]->motor.header.online;
    if (motor_current_command_a[motor_index] != 0.0F || current_set_status[motor_index] != DEVICE_OK)
    {
      all_zero_current_set = false;
    }

    g_motor_chassis_monitor.motor_online[motor_index] = motor_online;
    g_motor_chassis_monitor.current_saturated[motor_index] =
        motor_current_command_a[motor_index] >= MOTOR_SPEED_CURRENT_LIMIT_A ||
        motor_current_command_a[motor_index] <= -MOTOR_SPEED_CURRENT_LIMIT_A;
    g_motor_chassis_monitor.feedback_update_status[motor_index] = feedback_update_status[motor_index];
    g_motor_chassis_monitor.current_set_status[motor_index] = current_set_status[motor_index];
    g_motor_chassis_monitor.control_status[motor_index] = motor_speed_control[motor_index].feedback.status;
    g_motor_chassis_monitor.limited_target_speed_rpm[motor_index] =
        motor_speed_control[motor_index].feedback.limited_target_speed_rpm;
    g_motor_chassis_monitor.ramped_target_speed_rpm[motor_index] =
        motor_speed_control[motor_index].feedback.target_speed_rpm;
    g_motor_chassis_tune.actual_speed_rpm[motor_index] = motor_speed_control[motor_index].feedback.actual_speed_rpm;
    g_motor_chassis_monitor.current_command_a[motor_index] = motor_current_command_a[motor_index];
    g_motor_chassis_monitor.temperature_c[motor_index] =
        motor_3508[motor_index] != NULL ? motor_3508[motor_index]->feedback.temp : 0.0F;
  }

  if (all_zero_current_set)
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

  g_motor_chassis_monitor.debug_stop_ready = debug_stop_zero_tx_count >= MOTOR_DEBUG_STOP_CONFIRM_CYCLES;
  g_motor_chassis_monitor.can_tx_status = can_tx_status;
  g_motor_chassis_monitor.debug_stop_zero_tx_count = debug_stop_zero_tx_count;
}
