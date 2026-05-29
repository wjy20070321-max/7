#include "bsp_sensor.h"

/* 静态数组存放引脚，便于循环读取 */
static GPIO_TypeDef *Track_Ports[TRACK_COUNT] = {
    TRACK1_PORT, TRACK2_PORT, TRACK3_PORT, TRACK4_PORT,
    TRACK5_PORT, TRACK6_PORT, TRACK7_PORT, TRACK8_PORT
};

static uint16_t Track_Pins[TRACK_COUNT] = {
    TRACK1_PIN, TRACK2_PIN, TRACK3_PIN, TRACK4_PIN,
    TRACK5_PIN, TRACK6_PIN, TRACK7_PIN, TRACK8_PIN
};

/* 8路灰度权重：4、5号靠近中心，向两边递增 */
static const float Track_Weights[TRACK_COUNT] = {
    -7.0f, -5.0f, -3.0f, -1.0f, 1.0f, 3.0f, 5.0f, 7.0f
};

void Sensor_Init(void)
{
    /* GPIO 初始化已由 CubeMX 完成 */
}

uint8_t Sensor_Is_On_Line(uint8_t index)
{
    if (index >= TRACK_COUNT) {
        return 0;
    }

    return (HAL_GPIO_ReadPin(Track_Ports[index], Track_Pins[index]) == TRACK_LINE_ACTIVE_LEVEL) ? 1U : 0U;
}

uint8_t Sensor_Count_Line_Active(void)
{
    uint8_t active_count = 0;

    for (uint8_t i = 0; i < TRACK_COUNT; i++) {
        if (Sensor_Is_On_Line(i)) {
            active_count++;
        }
    }

    return active_count;
}

uint8_t Sensor_Count_Center_Line_Active(void)
{
    uint8_t active_count = 0;

    /* 中间四路：TRACK3 ~ TRACK6，对应数组下标 2 ~ 5 */
    for (uint8_t i = 2; i <= 5; i++) {
        if (Sensor_Is_On_Line(i)) {
            active_count++;
        }
    }

    return active_count;
}

/**
 * @brief  加权算法计算寻迹黑线偏差值。
 * @retval 偏差范围约 -7.0 到 +7.0；脱轨时根据上次偏差给出拉回方向。
 */
float Sensor_Get_Track_Error(void)
{
    float sum_weight = 0.0f;
    uint8_t active_count = 0;
    static float last_error = 0.0f;

    for (uint8_t i = 0; i < TRACK_COUNT; i++) {
        if (Sensor_Is_On_Line(i)) {
            sum_weight += Track_Weights[i];
            active_count++;
        }
    }

    if (active_count == 0U) {
        if (last_error > 0.0f) {
            return 8.0f;
        }
        if (last_error < 0.0f) {
            return -8.0f;
        }
        return 0.0f;
    }

    last_error = sum_weight / (float)active_count;
    return last_error;
}
