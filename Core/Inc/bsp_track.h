#ifndef __BSP_TRACK_H
#define __BSP_TRACK_H

#include "main.h"

// 8路灰度传感器数据结构体
typedef struct {
    uint8_t ch1;  // 最左侧 (PF12)
    uint8_t ch2;  // 左侧   (PE7)
    uint8_t ch3;  // 左侧   (PE4)
    uint8_t ch4;  // 中心左 (PF2)
    uint8_t ch5;  // 中心右 (PG4)
    uint8_t ch6;  // 右侧   (PG3)
    uint8_t ch7;  // 右侧   (PG0)
    uint8_t ch8;  // 最右侧 (PF14)
} Track_Data_t;

// 函数声明
void BSP_Track_Init(void);
void BSP_Track_Read(Track_Data_t* track_data);

#endif
