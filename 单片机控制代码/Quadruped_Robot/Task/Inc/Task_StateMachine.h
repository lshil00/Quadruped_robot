#ifndef _TASK_STATEMACHINE_H__
#define _TASK_STATEMACHINE_H__
#include "sysconfig.h"

/******步态
       站立
       Walk步态
			 Trot步态
			 Crawl步态
***************/
typedef enum
{
    GaitMode_Stand,	
    GaitMode_Walk,
    GaitMode_Trot,
	  GaitMode_Crawl,
	  GaitMode_Body_Twist
}GaitMode_t;

/******主状态
       静止
       前进
			 后退
			 向左平移
			 向右平移
			 向左旋转
			 向右旋转
***************/
typedef enum
{
	  MainState_Static,
		MainState_Step,
    MainState_Forward,
    MainState_Backward,
    MainState_Move_to_Left,
    MainState_Move_to_Right,
    MainState_Turn_Left,
	  MainState_Turn_Right,
	  MainState_IMU,
	  MainState_Avoid_Obstacle
}MainState_t;

void Task_StateMachine_Start(void *parameters);
GaitMode_t GetGaitMode(void);
MainState_t GetMainState(void);
#endif
