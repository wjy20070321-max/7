#include "bsp_sensor.h"

// 静态数组存放引脚，便于循环读取
static GPIO_TypeDef* Track_Ports[TRACK_COUNT] = {TRACK1_PORT, TRACK2_PORT, TRACK3_PORT, TRACK4_PORT, TRACK5_PORT, TRACK6_PORT, TRACK7_PORT, TRACK8_PORT};
static uint16_t Track_Pins[TRACK_COUNT] = {TRACK1_PIN, TRACK2_PIN, TRACK3_PIN, TRACK4_PIN, TRACK5_PIN, TRACK6_PIN, TRACK7_PIN, TRACK8_PIN};

// 8路灰度的权重：4和5位于中心，向两侧递增
// 压黑线输出高电平(1)还是低电平(0)根据你传感器的硬件逻辑决定，这里假设检测到黑线读取到 1
static const float Track_Weights[TRACK_COUNT] = {-7.0f, -5.0f, -3.0f, -1.0f, 1.0f, 3.0f, 5.0f, 7.0f};

void Sensor_Init(void)
{
    // GPIO初始化已由CubeMX完成
}

/**
  * @brief  加权算法计算寻迹黑线偏差值 (终极融合版)
  * @retval 偏差范围约 -7.0 到 +7.0f。若完全脱轨，强制输出极限值打死方向盘拉回！
  */
float Sensor_Get_Track_Error(void)
{
    float sum_weight = 0.0f;
    uint8_t active_count = 0;
    static float last_error = 0.0f;
    
    // 1. 数组 + for循环，优雅读取8路传感器
    for (uint8_t i = 0; i < TRACK_COUNT; i++)
    {
        // 假设检测到黑线返回 GPIO_PIN_SET (1)，请根据你的硬件极性修改
        if (HAL_GPIO_ReadPin(Track_Ports[i], Track_Pins[i]) == GPIO_PIN_SET)
        {
            sum_weight += Track_Weights[i];
            active_count++;
        }
    }
    
    // 2. 强力脱轨保护机制 (吸收了 Version 1 的优点)
    if (active_count == 0)
    {
        // 冲出赛道时，根据最后一次的偏离方向，强制输出一个极大的修正值 (+8.0 或 -8.0)
        if (last_error > 0) return 8.0f;  // 从右边冲出去了，狠狠往左打方向
        if (last_error < 0) return -8.0f; // 从左边冲出去了，狠狠往右打方向
        return 0.0f; // 极小概率发生
    }
    
    // 3. 正常情况下的加权平均计算
    last_error = sum_weight / (float)active_count;
    
    return last_error;
}
