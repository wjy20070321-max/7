#ifndef __BSP_SENSOR_H
#define __BSP_SENSOR_H

#include "main.h"

/* 8路灰度/红外寻迹传感器引脚定义 */
#define TRACK_COUNT 8U

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
 * 黑线有效电平。
 * 你现在任务一寻不了迹，大概率是模块压黑线输出低电平，
 * 所以这版默认用 GPIO_PIN_RESET。
 * 如果你用串口/屏幕看到压黑线时读数反了，再改成 GPIO_PIN_SET。
 */
#define TRACK_BLACK_LEVEL GPIO_PIN_RESET

/*
 * B点是直径4cm黑圆，正常会比1.8cm引导线压到更多传感器。
 * 如果你的传感器间距较大，B点不停，可以把 B_POINT_BLACK_MIN_COUNT 从5改4。
 */
#define B_POINT_BLACK_MIN_COUNT         5U
#define B_POINT_CENTER_BLACK_MIN_COUNT  3U

void Sensor_Init(void);
void Sensor_Reset_Track_State(void);

float Sensor_Get_Track_Error(void);
uint8_t Sensor_Is_On_Line(uint8_t index);
uint8_t Sensor_Count_Black(void);
uint8_t Sensor_Count_Center_Black(void);
uint8_t Sensor_Get_Raw_Bits(void);

/* 兼容上一版函数名 */
uint8_t Sensor_Count_Line_Active(void);
uint8_t Sensor_Count_Center_Line_Active(void);

#endif /* __BSP_SENSOR_H */
