/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
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
#include "usart.h"
#include "stdio.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

osMailQId myMail01Handle;
extern uint8_t data[20];
osMailQId myMail02Handle;
extern uint8_t data2[20];
//osMailQId myMail03Handle;
//extern uint8_t data3[20];

/* USER CODE END Variables */
osThreadId Task_IMUHandle;
osThreadId Task_StateMHandle;
osThreadId Task_InitHandle;
osThreadId Task_ControlHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
   
/* USER CODE END FunctionPrototypes */

void Task_IMU_Start(void const * argument);
void Task_StateMachine_Start(void const * argument);
void Task_Init_Start(void const * argument);
void Task_Control_Start(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];
  
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
{
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
  *ppxIdleTaskStackBuffer = &xIdleStack[0];
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
  /* place for user code */
}                   
/* USER CODE END GET_IDLE_TASK_MEMORY */

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
	osMailQDef(myMail01,5,uint16_t);
	myMail01Handle = osMailCreate (osMailQ(myMail01),NULL);
	osMailQDef(myMail02,10,uint8_t);
	myMail02Handle = osMailCreate (osMailQ(myMail02),NULL);
//	osMailQDef(myMail03,6,uint8_t);
//	myMail03Handle = osMailCreate (osMailQ(myMail03),NULL);	
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of Task_IMU */
  osThreadDef(Task_IMU, Task_IMU_Start, osPriorityNormal, 0, 128);
  Task_IMUHandle = osThreadCreate(osThread(Task_IMU), NULL);

  /* definition and creation of Task_StateM */
  osThreadDef(Task_StateM, Task_StateMachine_Start, osPriorityAboveNormal, 0, 128);
  Task_StateMHandle = osThreadCreate(osThread(Task_StateM), NULL);

  /* definition and creation of Task_Init */
  osThreadDef(Task_Init, Task_Init_Start, osPriorityHigh, 0, 128);
  Task_InitHandle = osThreadCreate(osThread(Task_Init), NULL);

  /* definition and creation of Task_Control */
  osThreadDef(Task_Control, Task_Control_Start, osPriorityNormal, 0, 128);
  Task_ControlHandle = osThreadCreate(osThread(Task_Control), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_Task_IMU_Start */
/**
  * @brief  Function implementing the Task_IMU thread.
  * @param  argument: Not used 
  * @retval None
  */
/* USER CODE END Header_Task_IMU_Start */
__weak void Task_IMU_Start(void const * argument)
{
  /* USER CODE BEGIN Task_IMU_Start */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Task_IMU_Start */
}

/* USER CODE BEGIN Header_Task_StateMachine_Start */
/**
* @brief Function implementing the Task_StateM thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Task_StateMachine_Start */
__weak void Task_StateMachine_Start(void const * argument)
{
  /* USER CODE BEGIN Task_StateMachine_Start */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Task_StateMachine_Start */
}

/* USER CODE BEGIN Header_Task_Init_Start */
/**
* @brief Function implementing the Task_Init thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Task_Init_Start */
__weak void Task_Init_Start(void const * argument)
{
  /* USER CODE BEGIN Task_Init_Start */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Task_Init_Start */
}

/* USER CODE BEGIN Header_Task_Control_Start */
/**
* @brief Function implementing the Task_Control thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Task_Control_Start */
__weak void Task_Control_Start(void const * argument)
{
  /* USER CODE BEGIN Task_Control_Start */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Task_Control_Start */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
     
/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
