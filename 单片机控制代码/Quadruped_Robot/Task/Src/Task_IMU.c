#include "Task_init.h"
#include "stdio.h"


static uint8_t IMU_attitude_update(void);		/*更新数据*/
static uint8_t MPU_Getdata(void);
/*----------------------- state Variables -----------------------*/
uint8_t *rdata2;
osEvent retval2;
float a[3]={0},w[3]={0},angle[3]={0},T=0;

/* ---------------------------- Global Variables ---------------------------- */ 
extern UART_HandleTypeDef huart3;
extern osMailQId myMail02Handle;
extern uint8_t data2[20];

/**************************
  * @brief  IMU任务
  * @param  unused
  * @retval void
  * @note   陀螺仪数据接收及解算，5ms更新一次
  */
void Task_IMU_Start(void *parameters)
{
	static int t=0;
	HAL_UART_Receive_IT(&huart3,data2,1);
  TickType_t xLastWakeUpTime;
  xLastWakeUpTime = xTaskGetTickCount();
  while(1)
  {
	  retval2=osMailGet(myMail02Handle,5);
	  if(retval2.status==osEventMail)
	  {
		  MPU_Getdata();
	    IMU_attitude_update();					/*更新数据*/
	  }		  
		if(t>20)
		{
			HAL_GPIO_TogglePin(LED1_GPIO_Port,LED1_Pin);
			t=0;
		}
		t++;
		
	  vTaskDelayUntil(&xLastWakeUpTime, 5);
  }
}

uint8_t MPU_Getdata(void)
{
	/*校验和*/
	uint8_t sum_angle = 0;
	uint8_t sum_gyro  = 0;
	uint8_t sum_acc   = 0;	
	
	rdata2=retval2.value.p;	
	if(*rdata2==0x55)       //检查帧头
	{  	
		switch(*(rdata2+1))
		{
			case 0x51: //加速度包
			sum_acc=0x55+0x51+*(rdata2+2)+*(rdata2+3)+*(rdata2+4)+*(rdata2+5)
			        +*(rdata2+6)+*(rdata2+7)+*(rdata2+8)+*(rdata2+9);
			if(sum_acc==*(rdata2+10))
			{
				a[0] = ((short)(*(rdata2+3)<<8 | *(rdata2+2)))/32768.0*16*9.8;      //X轴加速度
				a[1] = ((short)(*(rdata2+5)<<8 | *(rdata2+4)))/32768.0*16*9.8;      //Y轴加速度
				a[2] = ((short)(*(rdata2+7)<<8 | *(rdata2+6)))/32768.0*16*9.8;      //Z轴加速度
				T    = ((short)(*(rdata2+9)<<8 | *(rdata2+8)))/340.0+36.53; 				//温度
			}
			break;
			
			case 0x52: //角速度包
			sum_gyro=0x55+0x52+*(rdata2+2)+*(rdata2+3)+*(rdata2+4)+*(rdata2+5)
			        +*(rdata2+6)+*(rdata2+7)+*(rdata2+8)+*(rdata2+9);
			if(sum_gyro==*(rdata2+10))
			{
				w[0] = ((short)(*(rdata2+3)<<8 | *(rdata2+2)))/32768.0*2000;      //X轴角速度
				w[1] = ((short)(*(rdata2+5)<<8 | *(rdata2+4)))/32768.0*2000;      //Y轴角速度
				w[2] = ((short)(*(rdata2+7)<<8 | *(rdata2+6)))/32768.0*2000;      //Z轴角速度
				T    = ((short)(*(rdata2+9)<<8 | *(rdata2+8)))/340.0+36.53;      //温度
      }
			break;
              
			case 0x53: //角度包  单位:°
			sum_angle=0x55+0x53+*(rdata2+2)+*(rdata2+3)+*(rdata2+4)+*(rdata2+5)
			        +*(rdata2+6)+*(rdata2+7)+*(rdata2+8)+*(rdata2+9);
			if(sum_angle==*(rdata2+10))
			{			
				angle[0] = ((short)(*(rdata2+3)<<8 | *(rdata2+2)))/32768.0*180;   //X轴滚转角
				angle[1] = ((short)(*(rdata2+5)<<8 | *(rdata2+4)))/32768.0*180;   //Y轴俯仰角
				angle[2] = ((short)(*(rdata2+7)<<8 | *(rdata2+6)))/32768.0*180;   //Z轴偏航角
				T    = ((short)(*(rdata2+9)<<8 | *(rdata2+8)))/340.0+36.53;      //温度
			}
			break;
      default:  break;
		}
		//printf("a[X]:%.2f  a[Y]:%.2f  a[z]:%.2f  w[X]:%.2f  w[Y]:%.2f  w[Z]:%.2f   theta[X]:%.2f  theta[Y]:%.2f  theta[Z]:%.2f\r\n",
		//				a[0],a[1],a[2],w[0],w[1],w[2],angle[0],angle[1],angle[2]);								
	}  
	return 1;	
}


uint8_t IMU_attitude_update(void)
{
	return 1;
}

