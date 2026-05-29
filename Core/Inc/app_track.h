#ifndef __APP_TRACK_H
#define __APP_TRACK_H

#include "main.h"

// 循迹控制参数结构体
typedef struct {
    float error;        // 当前偏差值 (-4.0 到 4.0)
    float last_error;   // 上一次的偏差 (用于计算D项和脱线保护)
    int   base_speed;   // 基础前进速度
    float kp;           // 比例系数 P
    float kd;           // 微分系数 D
} Track_Ctrl_t;

// 暴露给外部的全局控制对象，方便在主循环或通过屏幕、蓝牙调参
extern Track_Ctrl_t TrackCtrl;

// 函数声明
void App_Track_Init(void);
float App_Track_Get_Error(void);
void App_Track_Follow_Line(int base_speed);

#endif
