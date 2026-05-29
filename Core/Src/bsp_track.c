#include "bsp_track.h"

#ifndef TRACK_CH1_Pin
#define TRACK_CH1_Pin GPIO_PIN_12
#define TRACK_CH1_GPIO_Port GPIOF

#define TRACK_CH2_Pin GPIO_PIN_7
#define TRACK_CH2_GPIO_Port GPIOE

#define TRACK_CH3_Pin GPIO_PIN_4
#define TRACK_CH3_GPIO_Port GPIOE

#define TRACK_CH4_Pin GPIO_PIN_2
#define TRACK_CH4_GPIO_Port GPIOF

#define TRACK_CH5_Pin GPIO_PIN_4
#define TRACK_CH5_GPIO_Port GPIOG

#define TRACK_CH6_Pin GPIO_PIN_3
#define TRACK_CH6_GPIO_Port GPIOG

#define TRACK_CH7_Pin GPIO_PIN_0
#define TRACK_CH7_GPIO_Port GPIOG

#define TRACK_CH8_Pin GPIO_PIN_14
#define TRACK_CH8_GPIO_Port GPIOF
#endif

// 初始化 
void BSP_Track_Init(void) {

}

// 读取 8 路灰度传感器的状态
void BSP_Track_Read(Track_Data_t* track_data) {
    // 读取引脚电平
    track_data->ch1 = HAL_GPIO_ReadPin(TRACK_CH1_GPIO_Port, TRACK_CH1_Pin);
    track_data->ch2 = HAL_GPIO_ReadPin(TRACK_CH2_GPIO_Port, TRACK_CH2_Pin);
    track_data->ch3 = HAL_GPIO_ReadPin(TRACK_CH3_GPIO_Port, TRACK_CH3_Pin);
    track_data->ch4 = HAL_GPIO_ReadPin(TRACK_CH4_GPIO_Port, TRACK_CH4_Pin);
    track_data->ch5 = HAL_GPIO_ReadPin(TRACK_CH5_GPIO_Port, TRACK_CH5_Pin);
    track_data->ch6 = HAL_GPIO_ReadPin(TRACK_CH6_GPIO_Port, TRACK_CH6_Pin);
    track_data->ch7 = HAL_GPIO_ReadPin(TRACK_CH7_GPIO_Port, TRACK_CH7_Pin);
    track_data->ch8 = HAL_GPIO_ReadPin(TRACK_CH8_GPIO_Port, TRACK_CH8_Pin);
}
