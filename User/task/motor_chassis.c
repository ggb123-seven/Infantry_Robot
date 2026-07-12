/*
    motor_chassis Task

*/

/* Includes ----------------------------------------------------------------- */
#include "task/user_task.h"
/* USER INCLUDE BEGIN */
#include "bsp/can.h"
#include "device/motor_rm.h"
/* USER INCLUDE END */

/* Private typedef ---------------------------------------------------------- */
/* Private define ----------------------------------------------------------- */
/* Private macro ------------------------------------------------------------ */
/* Private variables -------------------------------------------------------- */
/* USER STRUCT BEGIN */
/*
 * 3508 电机使用 CAN1，C620 电调 ID 设置为 1，对应反馈 ID 为 0x201。
 * 任务以 0.5 A 测试电流持续下发控制命令。
 * @datasheet RoboMaster C620 用户手册“CAN 通信协议”章节
 */
static MOTOR_RM_Param_t motor_3508_param = {
    .can = BSP_CAN_1,
    .id = 0x201U,
    .module = MOTOR_M3508,
    .reverse = false,
    .gear = true,
};
/* USER STRUCT END */

/* Private function --------------------------------------------------------- */
/* USER PRIVATE CODE BEGIN */
/* USER PRIVATE CODE END */
/* Exported functions ------------------------------------------------------- */
void Task_motor_chassis(void *argument) {
  (void)argument; /* 未使用argument，消除警告 */

  /* 计算任务运行到指定频率需要等待的tick数 */
  const uint32_t delay_tick = osKernelGetTickFreq() / MOTOR_CHASSIS_FREQ;

  osDelay(MOTOR_CHASSIS_INIT_DELAY); /* 延时一段时间再开启任务 */

  uint32_t tick = osKernelGetTickCount(); /* 控制任务运行频率的计时 */
  /* USER CODE INIT BEGIN */
  BSP_CAN_Init();
  MOTOR_RM_Register(&motor_3508_param);
  /* USER CODE INIT END */

  while (1) {
    tick += delay_tick; /* 计算下一个唤醒时刻 */
    /* USER CODE BEGIN */
    MOTOR_RM_Update(&motor_3508_param);
    MOTOR_RM_SetTorqueCurrent(&motor_3508_param, 0.8F);
    MOTOR_RM_Ctrl(&motor_3508_param);
    /* USER CODE END */
    osDelayUntil(tick); /* 运行结束，等待下一次唤醒 */
  }
}
