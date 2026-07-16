#include "task/motor_chassis.h"

#include "bsp/can.h"
#include "device/motor_rm.h"
#include "task/motor_speed_control.h"
#include "task/user_task.h"

/*
 * 单个 M3508 电机参数：
 * - CAN 总线：CAN1。
 * - C620 电调 ID：1，对应反馈标准帧 ID 0x201。
 * - 电机型号：M3508。
 * - 安装方向：当前不反向，最终方向需通过低速上板测试确认。
 * - 减速箱：启用，设备层将转子转速除以 3591/187 后输出 rpm。
 * @datasheet RoboMaster C620 用户手册“CAN 通信协议”章节
 */
static MOTOR_RM_Param_t motor_3508_param = {
    .can = BSP_CAN_1,
    .id = 0x201U,
    .module = MOTOR_M3508,
    .reverse = false,
    .gear = true,
};

/*
 * 单个 M3508 电机任务变量：
 * - motor_3508：注册后的 M3508 电机实例，初始化失败时为 NULL。
 * - motor_speed_control：速度环主结构体，包含目标、反馈、PID、缓启动状态和输出。
 * - motor_current_command_a：本周期需要下发的转子侧电流指令，单位 A。
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

/**
 * @brief 运行单个 M3508 电机速度控制任务
 *
 * @param[in] argument 任务参数，本任务不使用
 * @return 本任务不会返回
 */
void Task_motor_chassis(void *argument) {
  (void)argument;

  const uint32_t delay_tick = osKernelGetTickFreq() / MOTOR_CHASSIS_FREQ;
  const float control_period_s = 1.0F / (float)MOTOR_CHASSIS_FREQ;
  osDelay(MOTOR_CHASSIS_INIT_DELAY);

  BSP_CAN_Init();
  MOTOR_RM_Register(&motor_3508_param);
  motor_3508 = MOTOR_RM_GetMotor(&motor_3508_param);
  MotorSpeedControl_Init(&motor_speed_control, (float)MOTOR_CHASSIS_FREQ);

  uint32_t tick = osKernelGetTickCount();
  while (1) {
    // 从 CAN 接收缓存更新 M3508 电机反馈
    if (motor_3508 != NULL)
      MOTOR_RM_Update(&motor_3508_param);
    const float actual_speed_rpm =
        motor_3508 != NULL ? motor_3508->feedback.rotor_speed : 0.0F;
    const bool speed_control_enabled =
        motor_3508 != NULL && motor_3508->motor.header.online;

    // 将真实输出轴速度写入速度环反馈结构体
    MotorSpeedControl_UpdateFeedback(&motor_speed_control, actual_speed_rpm);
    // 根据目标速度和真实速度计算本周期电流指令
    MotorSpeedControl_Control(&motor_speed_control, MOTOR_SPEED_TARGET_RPM,
                              speed_control_enabled, control_period_s);
    // 从速度环主结构体导出转子侧电流指令
    MotorSpeedControl_DumpOutput(&motor_speed_control,
                                 &motor_current_command_a);

    if (motor_3508 != NULL) {
      // 将速度环输出写入 M3508 电流发送缓存
      MOTOR_RM_SetTorqueCurrent(&motor_3508_param, motor_current_command_a);
      // 发送当前 M3508 所属控制组的 CAN 控制帧
      MOTOR_RM_Ctrl(&motor_3508_param);
    }

    tick += delay_tick;
    // 等待至下一个绝对任务周期，避免累计调度漂移
    osDelayUntil(tick);
  }
}
