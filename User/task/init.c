/*
    Init Task
    任务初始化，创建各个线程任务和消息队列
*/

/* Includes ----------------------------------------------------------------- */
#include "task/user_task.h"

/* USER INCLUDE BEGIN */

/* USER INCLUDE END */

/* Private typedef ---------------------------------------------------------- */
/* Private define ----------------------------------------------------------- */
/* Private macro ------------------------------------------------------------ */
/* Private variables -------------------------------------------------------- */
/* Private function --------------------------------------------------------- */
/* Exported functions ------------------------------------------------------- */

/**
 * @brief 创建业务任务和消息队列
 *
 * @param[in] argument 任务参数，本任务不使用
 * @return 本任务不会返回
 */
void Task_Init(void *argument)
{
  // 标记任务参数未使用
  (void)argument;
  /* USER CODE INIT BEGIN */

  /* USER CODE INIT END */
  osKernelLock(); /* 锁定内核，防止任务切换 */

  /* 创建任务线程 */
  task_runtime.thread.motor_chassis = osThreadNew(Task_motor_chassis, NULL, &attr_motor_chassis);

  // 创建消息队列
  /* USER MESSAGE BEGIN */
  task_runtime.msgq.user_msg = osMessageQueueNew(2u, 10, NULL);
  /* USER MESSAGE END */

  // 解锁内核并允许已创建任务参与调度
  osKernelUnlock();
  // 任务创建完成后结束初始化任务
  osThreadTerminate(osThreadGetId());
}
