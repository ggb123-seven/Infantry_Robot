#pragma once

#ifdef __cplusplus
extern "C"
{
#endif
/* Includes ----------------------------------------------------------------- */
#include "FreeRTOS.h"
#include "task.h"
#include <cmsis_os2.h>

/* USER INCLUDE BEGIN */

/* USER INCLUDE END */
/* Exported constants ------------------------------------------------------- */
/* 任务运行频率 */
#define MOTOR_CHASSIS_FREQ (500U)

/* 任务初始化延时ms */
#define TASK_INIT_DELAY (100u)
#define MOTOR_CHASSIS_INIT_DELAY (0)

/* Exported defines --------------------------------------------------------- */
/* Exported macro ----------------------------------------------------------- */
/* Exported types ----------------------------------------------------------- */

/* 任务运行时结构体 */
typedef struct
{
  /* 各任务，也可以叫做线程 */
  struct
  {
    osThreadId_t motor_chassis;
  } thread;

  /* USER MESSAGE BEGIN */
  struct
  {
    osMessageQueueId_t user_msg; /* 用户自定义任务消息队列 */
  } msgq;
  /* USER MESSAGE END */

  /* 机器人状态 */
  struct
  {
    float battery;  /* 电池电量百分比 */
    float vbat;     /* 电池电压 */
    float cpu_temp; /* CPU温度 */
  } status;

  /* USER CONFIG BEGIN */

  /* USER CONFIG END */

  /* 各任务的stack使用 */
  struct
  {
    UBaseType_t motor_chassis;
  } stack_water_mark;

  /* 各任务运行频率 */
  struct
  {
    float motor_chassis;
  } freq;

  /* 任务最近运行时间 */
  struct
  {
    float motor_chassis;
  } last_up_time;

} Task_Runtime_t;

/* 任务运行时结构体 */
extern Task_Runtime_t task_runtime;

/* 初始化任务句柄 */
extern const osThreadAttr_t attr_init;
extern const osThreadAttr_t attr_motor_chassis;

/* 任务函数声明 */
void Task_Init(void *argument);
void Task_motor_chassis(void *argument);

#ifdef __cplusplus
}
#endif
