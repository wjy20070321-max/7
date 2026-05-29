#ifndef __BSP_SA100_H
#define __BSP_SA100_H

#include "main.h"

/* 声明全局角度变量，供 app_control.c 外部调用 */
extern volatile float Pendulum_Angle;

/* 函数声明 */
void SA100_Init(void);

#endif /* __BSP_SA100_H */
