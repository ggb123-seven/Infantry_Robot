/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "task/user_task.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

extern void blue_led_task(void *argument);
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
/* Definitions for LED_RED */
osThreadId_t LED_REDHandle;
const osThreadAttr_t LED_RED_attributes = {
  .name = "LED_RED",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for LED_GREEN */
osThreadId_t LED_GREENHandle;
const osThreadAttr_t LED_GREEN_attributes = {
  .name = "LED_GREEN",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
   
osThreadId_t led_blue_handle;
const osThreadAttr_t LED_BLUE_attributes = {
  .name = "LED_BLUE",
  .stack_size = 512,
  .priority = (osPriority_t) osPriorityHigh,
};
/* USER CODE END FunctionPrototypes */

void red_led_task(void *argument);
extern void green_led_task(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
       
  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of LED_RED */
  LED_REDHandle = osThreadNew(red_led_task, NULL, &LED_RED_attributes);

  /* creation of LED_GREEN */
  LED_GREENHandle = osThreadNew(green_led_task, NULL, &LED_GREEN_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  led_blue_handle = osThreadNew(blue_led_task, NULL, &LED_BLUE_attributes);
  (void)osThreadNew(Task_Init, NULL, &attr_init);
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_red_led_task */
/**
  * @brief  Function implementing the LED_RED thread.
  * @param  argument: Not used 
  * @retval None
  */
/* USER CODE END Header_red_led_task */
__weak void red_led_task(void *argument)
{
  /* USER CODE BEGIN red_led_task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END red_led_task */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
     
/* USER CODE END Application */

