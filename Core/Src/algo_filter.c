#include "algo_filter.h"
#include <string.h>

/**
  * @brief  初始化一阶低通滤波器
  * @param  alpha: 滤波系数 (推荐 0.1 ~ 0.4)
  */
void LPF_Init(LowPassFilter_t *lpf, float alpha)
{
    lpf->alpha = alpha;
    lpf->last_output = 0.0f;
}

/**
  * @brief  执行一阶低通滤波
  * @param  input: 刚采集到的原始传感器数据
  * @retval 滤波后的平滑数据
  */
float LPF_Apply(LowPassFilter_t *lpf, float input)
{
    // 公式: Y(n) = alpha * X(n) + (1 - alpha) * Y(n-1)
    lpf->last_output = (lpf->alpha * input) + ((1.0f - lpf->alpha) * lpf->last_output);
    return lpf->last_output;
}

/**
  * @brief  初始化滑动平均滤波器
  */
void MAF_Init(MovingAverageFilter_t *maf)
{
    maf->index = 0;
    maf->sum = 0.0f;
    memset(maf->buffer, 0, sizeof(maf->buffer));
}

/**
  * @brief  执行滑动平均滤波
  * @param  input: 新采样的原始数据
  * @retval 当前窗口内所有数据的平均值
  */
float MAF_Apply(MovingAverageFilter_t *maf, float input)
{
    // 1. 减去即将被丢弃的旧数据
    maf->sum -= maf->buffer[maf->index];
    
    // 2. 将新数据存入窗口
    maf->buffer[maf->index] = input;
    
    // 3. 加上新数据
    maf->sum += input;
    
    // 4. 移动滑动指针
    maf->index = (maf->index + 1) % FILTER_WINDOW_SIZE;
    
    // 5. 计算并返回平均值
    return (maf->sum / (float)FILTER_WINDOW_SIZE);
}
