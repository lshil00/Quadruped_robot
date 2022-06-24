#include "Task_StateMachine.h"
#include "stdio.h"
/*------------------------ state enum --------------------------*/

/*----------------------- state Variables -----------------------*/
GaitMode_t GaitMode;
MainState_t Mainstate;
uint8_t *rdata;
uint8_t *rdata3;

osEvent retval;
osEvent retval3;	
/* ---------------------------- Global Variables ---------------------------- */ 
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart4;
extern TaskHandle_t TaskHandle_StateMachine;
extern osMailQId myMail01Handle;
extern uint8_t data[20];
//extern osMailQId myMail03Handle;

extern uint8_t Re_buf2[10];
uint8_t flag=0;
extern uint8_t data3[10];
uint8_t distance=0;
/*----------------------- state functions -----------------------*/
static uint8_t StateMachine_Init(void);
static uint8_t MainState_Update(void);
static uint8_t GaitMode_Update(void);
static uint8_t Avoid_Obstacle(void);

/**************************
  * @brief  状态机任务
  * @param  unused
  * @retval void
  * @note   5ms执行一次
  */
void Task_StateMachine_Start(void *parameters)
{	
	  StateMachine_Init();
		TickType_t xLastWakeUpTime;
	  HAL_UART_Receive_IT(&huart2,data,2);
	  HAL_UART_Receive_IT(&huart4,data3,1);
    xLastWakeUpTime = xTaskGetTickCount();
    while (1)
    {
			retval=osMailGet(myMail01Handle,5);
			if(retval.status==osEventMail)
			{
			 // printf("Receiving queue!\n");
			  rdata=retval.value.p;
		  	//printf("var is %s\n",rdata);
			  MainState_Update();
			  GaitMode_Update();
      }
			Avoid_Obstacle();
			vTaskDelayUntil(&xLastWakeUpTime, 10);      /*5ms更新一次*/
		}
}

GaitMode_t GetGaitMode(void)
{
    return GaitMode;
}

MainState_t GetMainState(void)
{
    return Mainstate;
}

/*--------------------static Function--------------------*/
/**
 * @brief  初始化状态机
 * @note   默认所有保护模式
 * @retval 
 */
uint8_t StateMachine_Init(void)
{
    GaitMode = GaitMode_Stand;     //站立
    Mainstate = MainState_Static;  //静止
    return 1;
}

uint8_t MainState_Update(void)
{
	if(*rdata==48)     //rdata[0]="0"
	{
	 switch(*(rdata+1)) 
	 {			
		 case 53:	//rdata[1]="5"
       Mainstate = MainState_Step;
		 break;	
		 case 50://rdata[1]="2"
       Mainstate = MainState_Forward;;
		 break;
		 case 56://rdata[1]="8"
       Mainstate = MainState_Backward;
		 break;		 
		 case 52://rdata[1]="4"
       Mainstate = MainState_Move_to_Left;
		 break;	
		 case 54://rdata[1]="6"
       Mainstate = MainState_Move_to_Right;
		 break;	
		 case 49://rdata[1]="1"
       Mainstate = MainState_Turn_Left;
		 break;		 
		 case 51://rdata[1]="3"
       Mainstate = MainState_Turn_Right;
		 break;	
     case 55:
			 if( GaitMode == GaitMode_Trot ) 
				 flag=1;
       else if( GaitMode == GaitMode_Body_Twist )
				 Mainstate = MainState_IMU;
     default:
       ;
	  }
   }
	 else if(*rdata==49)     //rdata[0]="0"
	 {
		 flag=0;
		 Mainstate=MainState_Static;
	 }
		 return 1;
}

uint8_t GaitMode_Update(void)
{
	if(*rdata==49)     //rdata[0]="1"
	{
		switch(*(rdata+1))
		 {				 
			 case 48:	//rdata[1]="10"
         GaitMode = GaitMode_Walk;
			 break;	
			 case 49://rdata[1]="11"
         GaitMode = GaitMode_Trot;
			 break;
			 case 50://rdata[1]="12"
         GaitMode = GaitMode_Stand;
			 break;		
			 case 51://rdata[1]="13"
         GaitMode = GaitMode_Body_Twist;
			 break;			 

			 default:
				 GaitMode = GaitMode_Stand;
		 }
	}
  return 1;	
}

uint8_t Avoid_Obstacle(void)
{	
	if(GaitMode==GaitMode_Trot && flag==1)
	{
      if(Re_buf2[0]==68)
				distance=100*(Re_buf2[2]-48)+10*(Re_buf2[3]-48)+(Re_buf2[4]-48);
			if(distance<30&&distance>20)
				Mainstate = MainState_Step;
			else if(distance>0&&distance<=20)
				Mainstate = MainState_Backward;
			else if(distance>=30)
				Mainstate = MainState_Forward;
	}	
	//printf("distance=%d \n",distance);
	return 1;		
}
