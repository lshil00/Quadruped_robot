/*
 * @Description: CAN通讯头文件定义
 * @Author: Mr Zhang
 * @Github: https://github.com/bigfatpaper
 * @Date: 2019-10-13 14:00:36
 * @LastEditors: Mr Zhang
 * @LastEditTime: 2019-10-16 23:58:33
 */

/*编译预处理*/
#ifndef __TASKS_CAN_H
#define __TASKS_CAN_H

/*外部声明预处理*/
#ifdef __TASK_CAN_GLOBALS
#define TASK_CAN_EXT
#else
#define TASK_CAN_EXT extern
#endif
/*外部声明预处理结束*/

/*头文件*/
#include "Task_init.h"
/*包含头文件结束*/

/*宏定义*/
#define CANSEND_1 1                        /*CANSEND发送 1 0x200 2 0x1ff*/
#define CANSEND_2 2
/*宏定义结束*/

/*结构体定义*/

typedef struct                             /*CAN发送结构体*/
{
    uint8_t            CANx;               /* CAN1 or CAN2 发送*/
    uint32_t           stdid;              /* ID: CAN1 0X200  CAN2 0X1FF */
		uint8_t            Data[8];            /* CAN发送数据*/
}CanSend_Type;

/*结构体定义结束*/

/*变量声明*/
TASK_CAN_EXT CanSend_Type CAN_Tx_Msg;      /*CAN发送变量定义*/
/*变量声明结束*/

/*函数声明*/
/*CAN的初始化函数 滤波和使能中断*/
void CAN_Init(CAN_HandleTypeDef *hcan);
void CAN_Recieve(CAN_HandleTypeDef *hcan);

extern int16_t delta_Mechanical_angle(int16_t speed, uint16_t angle_last, uint16_t angle_now);
/*函数声明结束*/

#endif
/*编译预处理结束*/
