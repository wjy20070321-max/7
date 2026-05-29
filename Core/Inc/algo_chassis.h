#ifndef __ALGO_CHASSIS_H
#define __ALGO_CHASSIS_H

#include "main.h"

/* 底盘物理结构参数定义 */
typedef struct {
    float wheel_track;      // 左右两个主动轮之间的物理轴距 (单位: mm 或 抽象单位)
    float max_wheel_speed;  // 单轮物理限速 (对应 TB6612 最大 PWM 8399)
} Chassis_Param_t;

/* 函数声明 */
void Chassis_Init(float track, float max_speed);
void Chassis_Inverse_Kinematics(float linear_v, float angular_w, float *out_left_v, float *out_right_v);

#endif /* __ALGO_CHASSIS_H */
