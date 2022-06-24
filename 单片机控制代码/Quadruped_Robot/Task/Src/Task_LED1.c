#include "Task_init.h"

extern uint8_t data[20];
/**************************
  * @brief  LED1闪烁任务
  * @param  unused
  * @retval void
  * @note   LED1闪烁
  */
void Task_LED1(void *parameters)
{
  TickType_t xLastWakeUpTime;
  xLastWakeUpTime = xTaskGetTickCount();
	while(1)
	{  
		HAL_GPIO_TogglePin((GPIO_TypeDef *)LED1_GPIO_Port, 
		                   (uint16_t)      LED1_Pin);		
		vTaskDelayUntil(&xLastWakeUpTime, 200);
	}
}
