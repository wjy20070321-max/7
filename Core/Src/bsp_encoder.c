#include "bsp_encoder.h"
#include "tim.h"

/**
  * @brief  正交编码器接口初始化，开启硬件计数
  * @param  无
  * @retval 无
  */
void Encoder_Init(void)
{
    // 开启 TIM3 编码器接口 (左轮 - PB4, PB5)
    HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
    // 开启 TIM4 编码器接口 (右轮 - PD12, PD13)
    HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);
    
    // 计数器初始清零
    __HAL_TIM_SET_COUNTER(&htim3, 0);
    __HAL_TIM_SET_COUNTER(&htim4, 0);
}

/**
  * @brief  获取左轮在当前采样周期内的脉冲增量（代表左轮速度）
  * @param  无
  * @retval 带有符号的 16 位速度值 (正数代表正转，负数代表反转)
  */
int16_t Encoder_Get_Left(void)
{
    int16_t encoder_val;
    
    // 强制转换为有符号 16 位短整型，STM32 硬件会自动处理正负反转对应的补码
    encoder_val = (int16_t)__HAL_TIM_GET_COUNTER(&htim3);
    
    // 【关键核心】读取后立刻将内部计数器清零，为下一个 5ms 周期做准备
    __HAL_TIM_SET_COUNTER(&htim3, 0);
    
    return encoder_val;
}

/**
  * @brief  获取右轮在当前采样周期内的脉冲增量（代表右轮速度）
  * @param  无
  * @retval 带有符号的 16 位速度值 (正数代表正转，负数代表反转)
  */
int16_t Encoder_Get_Right(void)
{
    int16_t encoder_val;
    
    // 读取 TIM4 计数器
    encoder_val = (int16_t)__HAL_TIM_GET_COUNTER(&htim4);
    
    // 读取后立刻将内部计数器清零
    __HAL_TIM_SET_COUNTER(&htim4, 0);
    
    /* 【小车调线小贴士】
       由于左右电机面对面安装，当小车整体向前走时，一个编码器会计正数，另一个会计负数。
       如果为了让后续 PID 算法统一“向前为正”，可以根据实际测试情况，在下方给右轮加个负号：
       return -encoder_val; 
    */
    return encoder_val; 
}
