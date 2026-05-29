#ifndef __BSP_SENSOR_H
#define __BSP_SENSOR_H

#include "main.h"

/* 8路灰度传感器引脚定义 */
#define TRACK_COUNT 8

#define TRACK1_PORT GPIOF
#define TRACK1_PIN  GPIO_PIN_12

#define TRACK2_PORT GPIOE
#define TRACK2_PIN  GPIO_PIN_7

#define TRACK3_PORT GPIOE
#define TRACK3_PIN  GPIO_PIN_4

#define TRACK4_PORT GPIOF
#define TRACK4_PIN  GPIO_PIN_2

#define TRACK5_PORT GPIOG
#define TRACK5_PIN  GPIO_PIN_4

#define TRACK6_PORT GPIOG
#define TRACK6_PIN  GPIO_PIN_3

#define TRACK7_PORT GPIOG
#define TRACK7_PIN  GPIO_PIN_0

#define TRACK8_PORT GPIOF
#define TRACK8_PIN  GPIO_PIN_14

/*
 * 黑线有效电平统一入口。
 * 如果实测黑线输出低电平，把这里改成 GPIO_PIN_RESET 即可，
 * 循迹误差和 B 点判断会一起跟着改，不会再前后矛盾。
 */
#define TRACK_LINE_ACTIVE_LEVEL GPIO_PIN_SET

void Sensor_Init(void);
float Sensor_Get_Track_Error(void);
uint8_t Sensor_Is_On_Line(uint8_t index);
uint8_t Sensor_Count_Line_Active(void);
uint8_t Sensor_Count_Center_Line_Active(void);

#endif /* __BSP_SENSOR_H */
