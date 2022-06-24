#include "Task_init.h"

void Bluetooth_Data_Update(void);

/**************************
  * @brief  蓝牙串口接收任务
  * @param  unused
  * @retval void
  * @note   接收蓝牙数据
  */
void Task_Bluetooth(void *parameters)
{
	uint8_t len=0;
	uint8_t CommandValue=COMMANDERR;
	uint32_t NotifyValue;
	
	uint8_t *CommandStr;
	while(1)
	{
		ulTaskNotifyTake(pdTRUE,portMAX_DELAY);	 /*等待蓝牙接收消息*/
    Bluetooth_Data_Update();                 /*蓝牙数据更新*/
		
	}
}
