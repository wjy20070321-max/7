#ifndef __ALGO_PID_H
#define __ALGO_PID_H

#include "main.h"

/* PID 结构体定义 */
typedef struct {
    float kp;               // 比例系数
    float ki;               // 积分系数
    float kd;               // 微分系数
    
    float error;            // 当前误差
    float last_error;       // 上一次误差
    float prev_error;       // 上上次误差 (增量式专用)
    
    float integral;         // 误差积分值 (位置式专用)
    float integral_limit;   // 积分限幅值
    float output_limit;     // 输出限幅值
    
    float output;           // PID 计算输出值
} PID_t;

/* 函数声明 */
void PID_Init(PID_t *pid, float kp, float ki, float kd, float int_limit, float out_limit);
float PID_Compute_Position(PID_t *pid, float target, float measured);
float PID_Compute_Incremental(PID_t *pid, float target, float measured);
void PID_Reset(PID_t *pid);

#endif /* __ALGO_PID_H */
