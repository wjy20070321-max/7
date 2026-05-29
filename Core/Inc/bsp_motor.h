#ifndef __BSP_MOTOR_H
#define __BSP_MOTOR_H

#include "main.h"

/* TIM1 PWM 重装载值 ARR 设为 8399，对应的最大占空比计数值 */
#define MOTOR_MAX_PWM   8399

/* 左轮引脚定义 (Motor 1) */
#define MOTOR_L_IN1_PORT  GPIOG
#define MOTOR_L_IN1_PIN   GPIO_PIN_7
#define MOTOR_L_IN2_PORT  GPIOG
#define MOTOR_L_IN2_PIN   GPIO_PIN_8

/* 右轮引脚定义 (Motor 2) */
#define MOTOR_R_IN1_PORT  GPIOG
#define MOTOR_R_IN1_PIN   GPIO_PIN_5
#define MOTOR_R_IN2_PORT  GPIOG
#define MOTOR_R_IN2_PIN   GPIO_PIN_6

/* 函数声明 */
void Motor_Init(void);
void Motor_Set_Speed(int16_t left_speed, int16_t right_speed);
void Motor_Stop(void);

#endif /* __BSP_MOTOR_H */
