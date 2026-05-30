#include "bsp_sensor.h"

static GPIO_TypeDef *Track_Ports[TRACK_COUNT] = {
    TRACK1_PORT, TRACK2_PORT, TRACK3_PORT, TRACK4_PORT,
    TRACK5_PORT, TRACK6_PORT, TRACK7_PORT, TRACK8_PORT
};

static uint16_t Track_Pins[TRACK_COUNT] = {
    TRACK1_PIN, TRACK2_PIN, TRACK3_PIN, TRACK4_PIN,
    TRACK5_PIN, TRACK6_PIN, TRACK7_PIN, TRACK8_PIN
};

/*
 * 传感器从左到右的权重。
 * 线在左侧时误差为负，线在右侧时误差为正。
 */
static const float Track_Weights[TRACK_COUNT] = {
    -7.0f, -5.0f, -3.0f, -1.0f, 1.0f, 3.0f, 5.0f, 7.0f
};

static float last_track_error_internal = 0.0f;

void Sensor_Init(void)
{
    /* GPIO 初始化由 CubeMX 完成，这里只清寻迹状态 */
    Sensor_Reset_Track_State();
}

void Sensor_Reset_Track_State(void)
{
    last_track_error_internal = 0.0f;
}

uint8_t Sensor_Is_On_Line(uint8_t index)
{
    if (index >= TRACK_COUNT) {
        return 0U;
    }

    return (HAL_GPIO_ReadPin(Track_Ports[index], Track_Pins[index]) == TRACK_BLACK_LEVEL) ? 1U : 0U;
}

uint8_t Sensor_Count_Black(void)
{
    uint8_t count = 0U;

    for (uint8_t i = 0U; i < TRACK_COUNT; i++) {
        count += Sensor_Is_On_Line(i);
    }

    return count;
}

uint8_t Sensor_Count_Center_Black(void)
{
    uint8_t count = 0U;

    /* 中间四路：TRACK3 ~ TRACK6，对应数组下标 2 ~ 5 */
    for (uint8_t i = 2U; i <= 5U; i++) {
        count += Sensor_Is_On_Line(i);
    }

    return count;
}

uint8_t Sensor_Get_Raw_Bits(void)
{
    uint8_t bits = 0U;

    for (uint8_t i = 0U; i < TRACK_COUNT; i++) {
        if (Sensor_Is_On_Line(i)) {
            bits |= (uint8_t)(1U << i);
        }
    }

    return bits;
}

uint8_t Sensor_Count_Line_Active(void)
{
    return Sensor_Count_Black();
}

uint8_t Sensor_Count_Center_Line_Active(void)
{
    return Sensor_Count_Center_Black();
}

float Sensor_Get_Track_Error(void)
{
    float sum_weight = 0.0f;
    uint8_t active_count = 0U;

    for (uint8_t i = 0U; i < TRACK_COUNT; i++) {
        if (Sensor_Is_On_Line(i)) {
            sum_weight += Track_Weights[i];
            active_count++;
        }
    }

    /* 脱线时沿用上一次方向拉回，避免直接变成0导致继续直冲 */
    if (active_count == 0U) {
        if (last_track_error_internal > 0.0f) {
            return 8.0f;
        }
        if (last_track_error_internal < 0.0f) {
            return -8.0f;
        }
        return 0.0f;
    }

    last_track_error_internal = sum_weight / (float)active_count;
    return last_track_error_internal;
}
