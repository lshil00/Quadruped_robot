#ifndef __TASKS_INIT_H__
#define __TASKS_INIT_H__

#include "sysconfig.h"
#include "stdio.h"
#include "usart.h"

#ifdef 	__TASK_INIT_GLOBALS
#define TASK_INIT_EXT
#else
#define TASK_INIT_EXT extern
#endif
TASK_INIT_EXT TaskHandle_t TaskHandle_IMU;

void Task_IMU(void *parameters);

#endif
