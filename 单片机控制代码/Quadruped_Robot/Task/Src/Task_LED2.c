#include "Task_init.h"

/**************************
  * @brief  LED2闪烁任务
  * @param  unused
  * @retval void
  * @note   LED2闪烁
  */
void Task_LED2(void *parameters)
{
  TickType_t xLastWakeUpTime;
  xLastWakeUpTime = xTaskGetTickCount();
	while(1)
	{
		HAL_GPIO_TogglePin((GPIO_TypeDef *)LED0_GPIO_Port, 
		                   (uint16_t)      LED0_Pin);
    vTaskDelayUntil(&xLastWakeUpTime, 100);
	}
}
