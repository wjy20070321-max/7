#ifndef __ALGO_FILTER_H
#define __ALGO_FILTER_H

#include "main.h"

/* 1. 一阶低通滤波器结构体 */
typedef struct {
    float alpha;        // 滤波系数 (0.0 到 1.0 之间，值越小滤波越强，延迟越大)
    float last_output;  // 上一次的滤波输出
} LowPassFilter_t;

/* 2. 滑动平均滤波器结构体 (窗口大小设为 8) */
#define FILTER_WINDOW_SIZE  8
typedef struct {
    float buffer[FILTER_WINDOW_SIZE]; // 缓存采样数据的数组
    uint8_t index;                    // 当前缓冲区索引
    float sum;                        // 窗口内的当前数据总和
} MovingAverageFilter_t;

/* 函数声明 */
void LPF_Init(LowPassFilter_t *lpf, float alpha);
float LPF_Apply(LowPassFilter_t *lpf, float input);

void MAF_Init(MovingAverageFilter_t *maf);
float MAF_Apply(MovingAverageFilter_t *maf, float input);

#endif /* __ALGO_FILTER_H */
