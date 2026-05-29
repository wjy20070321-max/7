#include "bsp_motor.h"
#include "tim.h"
#include <stdlib.h>

/**
  * @brief  电机驱动初始化，启动 TIM1 的 PWM 输出通道
  * @param  无
  * @retval 无
  */
void Motor_Init(void)
{
    // 启动 TIM1 通道 1 (左轮 PWM - PE9)
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    // 启动 TIM1 通道 2 (右轮 PWM - PE11)
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    
    // 初始状态让电机停止
    Motor_Stop();
}

/**
  * @brief  设置左右轮电机的速度与方向
  * @param  left_speed:  左轮速度指令 (-8399 到 8399)
  * @param  right_speed: 右轮速度指令 (-8399 到 8399)
  * @retval 无
  */
void Motor_Set_Speed(int16_t left_speed, int16_t right_speed)
{
    /* 1. 限幅保护：防止软件 PID 算出的值超过定时器 ARR 最大值 */
    if (left_speed > MOTOR_MAX_PWM)   left_speed = MOTOR_MAX_PWM;
    if (left_speed < -MOTOR_MAX_PWM)  left_speed = -MOTOR_MAX_PWM;
    if (right_speed > MOTOR_MAX_PWM)  right_speed = MOTOR_MAX_PWM;
    if (right_speed < -MOTOR_MAX_PWM) right_speed = -MOTOR_MAX_PWM;

    /* 2. 左轮方向与速度控制 (PG7, PG8) */
    if (left_speed > 0)
    {
        HAL_GPIO_WritePin(MOTOR_L_IN1_PORT, MOTOR_L_IN1_PIN, GPIO_PIN_SET);
        HAL_GPIO_WritePin(MOTOR_L_IN2_PORT, MOTOR_L_IN2_PIN, GPIO_PIN_RESET);
    }
    else if (left_speed < 0)
    {
        HAL_GPIO_WritePin(MOTOR_L_IN1_PORT, MOTOR_L_IN1_PIN, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(MOTOR_L_IN2_PORT, MOTOR_L_IN2_PIN, GPIO_PIN_SET);
    }
    else
    {
        HAL_GPIO_WritePin(MOTOR_L_IN1_PORT, MOTOR_L_IN1_PIN, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(MOTOR_L_IN2_PORT, MOTOR_L_IN2_PIN, GPIO_PIN_RESET);
    }
    // 写入左轮 PWM 寄存器 (TIM1_CH1)
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, abs(left_speed));

    /* 3. 右轮方向与速度控制 (PG5, PG6) */
    /* 注意：由于左右电机镜像对称安装，若发现小车原地打转，可将下方右轮的 SET/RESET 逻辑对调 */
    if (right_speed > 0)
    {
        HAL_GPIO_WritePin(MOTOR_R_IN1_PORT, MOTOR_R_IN1_PIN, GPIO_PIN_SET);
        HAL_GPIO_WritePin(MOTOR_R_IN2_PORT, MOTOR_R_IN2_PIN, GPIO_PIN_RESET);
    }
    else if (right_speed < 0)
    {
        HAL_GPIO_WritePin(MOTOR_R_IN1_PORT, MOTOR_R_IN1_PIN, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(MOTOR_R_IN2_PORT, MOTOR_R_IN2_PIN, GPIO_PIN_SET);
    }
    else
    {
        HAL_GPIO_WritePin(MOTOR_R_IN1_PORT, MOTOR_R_IN1_PIN, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(MOTOR_R_IN2_PORT, MOTOR_R_IN2_PIN, GPIO_PIN_RESET);
    }
    // 写入右轮 PWM 寄存器 (TIM1_CH2)
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, abs(right_speed));
}

/**
  * @brief  紧急停止电机（刹车状态）
  * @param  无
  * @retval 无
  */
void Motor_Stop(void)
{
    HAL_GPIO_WritePin(MOTOR_L_IN1_PORT, MOTOR_L_IN1_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MOTOR_L_IN2_PORT, MOTOR_L_IN2_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MOTOR_R_IN1_PORT, MOTOR_R_IN1_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MOTOR_R_IN2_PORT, MOTOR_R_IN2_PIN, GPIO_PIN_RESET);
    
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
}
