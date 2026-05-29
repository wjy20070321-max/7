#ifndef __BSP_SENSOR_H
#define __BSP_SENSOR_H

#include "main.h"

/* 8路灰度传感器引脚定义 */
#define TRACK_COUNT  8

#define TRACK1_PORT  GPIOF
#define TRACK1_PIN   GPIO_PIN_12
#define TRACK2_PORT  GPIOE
#define TRACK2_PIN   GPIO_PIN_7
#define TRACK3_PORT  GPIOE
#define TRACK3_PIN   GPIO_PIN_4
#define TRACK4_PORT  GPIOF
#define TRACK4_PIN   GPIO_PIN_2
#define TRACK5_PORT  GPIOG
#define TRACK5_PIN   GPIO_PIN_4
#define TRACK6_PORT  GPIOG
#define TRACK6_PIN   GPIO_PIN_3
#define TRACK7_PORT  GPIOG
#define TRACK7_PIN   GPIO_PIN_0
#define TRACK8_PORT  GPIOF
#define TRACK8_PIN   GPIO_PIN_14

/* 函数声明 */
void Sensor_Init(void);
float Sensor_Get_Track_Error(void);

#endif /* __BSP_SENSOR_H */
