/*
    motor_chassis Task

*/

/* Includes ----------------------------------------------------------------- */
#include "task/user_task.h"
/* USER INCLUDE BEGIN */
#include "bsp/can.h"
#include "device/device.h"
#include "device/motor_rm.h"
#include "task/motor_chassis.h"
/* USER INCLUDE END */

/* Private typedef ---------------------------------------------------------- */
/* Private define ----------------------------------------------------------- */
/* Private macro ------------------------------------------------------------ */
/* Private variables -------------------------------------------------------- */
/* USER STRUCT BEGIN */
/*
 * 3508 电机使用 CAN1，C620 电调 ID 设置为 1，对应反馈 ID 为 0x201。
 * 任务以 0.8 A 测试电流持续下发控制命令。
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

volatile MotorChassisFeedbackSnapshot_t g_motor_chassis_feedback;
/* USER STRUCT END */

/* Private function --------------------------------------------------------- */
/* USER PRIVATE CODE BEGIN */
/**
 * @brief 发布驱动电机反馈快照
 *
 * @return 无返回值
 */
static void MotorChassis_PublishFeedback(void)
{
  MotorChassisFeedbackSnapshot_t snapshot =
  {
    0
  };

  MOTOR_RM_t *motor = MOTOR_RM_GetMotor(&motor_3508_param);
  if (motor != NULL)
  {
    snapshot.online = motor->motor.header.online;
    snapshot.output_total_angle_rad = motor->feedback.rotor_total_angle;
    snapshot.output_speed_rpm = motor->feedback.rotor_speed;
    snapshot.torque_current_feedback = motor->feedback.torque_current;
    snapshot.temperature_c = motor->feedback.temp;
    snapshot.feedback_timestamp_us = motor->feedback.last_update_time;
  }

  taskENTER_CRITICAL();
  g_motor_chassis_feedback = snapshot;
  taskEXIT_CRITICAL();
}
/* USER PRIVATE CODE END */
/* Exported functions ------------------------------------------------------- */

/**
 * @brief 获取驱动电机反馈快照
 *
 * @param[out] snapshot 用于接收反馈快照的结构体指针
 * @return 成功返回 DEVICE_OK，参数为空时返回 DEVICE_ERR_NULL
 */
int8_t
MotorChassis_GetFeedbackSnapshot(MotorChassisFeedbackSnapshot_t *snapshot)
{
  if (snapshot == NULL)
  {
    return DEVICE_ERR_NULL;
  }

  taskENTER_CRITICAL();
  *snapshot = g_motor_chassis_feedback;
  taskEXIT_CRITICAL();
  return DEVICE_OK;
}

void Task_motor_chassis(void *argument)
{
  (void)argument; /* 未使用argument，消除警告 */

  /* 计算任务运行到指定频率需要等待的tick数 */
  const uint32_t delay_tick = osKernelGetTickFreq() / MOTOR_CHASSIS_FREQ;

  osDelay(MOTOR_CHASSIS_INIT_DELAY); /* 延时一段时间再开启任务 */

  uint32_t tick = osKernelGetTickCount(); /* 控制任务运行频率的计时 */
  /* USER CODE INIT BEGIN */
  // 初始化 CAN BSP 并启动 CAN1 接收与发送功能
  BSP_CAN_Init();

  // 注册 CAN1 上反馈 ID 为 0x201 的 M3508 电机
  MOTOR_RM_Register(&motor_3508_param);

  // 发布电机注册后的初始反馈快照
  MotorChassis_PublishFeedback();
  /* USER CODE INIT END */

  while (1)
  {
    tick += delay_tick; /* 计算下一个唤醒时刻 */
    /* USER CODE BEGIN */
    // 读取最新 CAN 反馈并更新 RM 电机设备数据
    MOTOR_RM_Update(&motor_3508_param);

    // 将本周期电机数据发布为控制反馈快照
    MotorChassis_PublishFeedback();

    // 设置 M3508 的测试目标电流为 0.8 A
    MOTOR_RM_SetTorqueCurrent(&motor_3508_param, 0.8F);

    // 发送当前 RM 电机控制帧
    MOTOR_RM_Ctrl(&motor_3508_param);
    /* USER CODE END */

    // 等待至下一运行周期
    osDelayUntil(tick);
  }
}
