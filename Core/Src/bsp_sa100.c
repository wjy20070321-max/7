#include "bsp_sa100.h"
#include "tim.h"

/* 内部捕获变量 */
volatile uint32_t Period_Value = 0; // 存放总周期计数值 (TIM5_CH1)
volatile uint32_t Pulse_Value = 0;  // 存放高电平计数值 (TIM5_CH2)
volatile float Duty_Cycle = 0.0f;   // 实时占空比 (0.0f ~ 1.0f)
volatile uint32_t debug_capture_count = 0; // 找个地方定义一下全局变量

/* 全局摆杆角度（初始化为垂直向上目标值，防止开机瞬间误触发保护） */
volatile float Pendulum_Angle = 180.0f; 

/**
  * @brief  初始化 SA100 角度传感器，启动 TIM5 的双通道硬件输入捕获中断
  */
void SA100_Init(void)
{
    // 启动 TIM5 通道 1 (捕获周期 - 上升沿)
    HAL_TIM_IC_Start_IT(&htim5, TIM_CHANNEL_1);
    // 启动 TIM5 通道 2 (捕获高电平 - 下降沿)
    HAL_TIM_IC_Start_IT(&htim5, TIM_CHANNEL_2);
}

/**
  * @brief  定时器输入捕获中断回调函数（STM32 硬件自动触发）
  */
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM5)
    {
		debug_capture_count++; // 每次进中断，它就加 1
		
        // 当通道 1 捕获到上升沿，说明一个完整的 PWM 周期结束
        if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
        {
            // 读取通道 1 的计数值（总周期）
            Period_Value = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
            // 读取通道 2 的计数值（高电平持续时间）
            Pulse_Value = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);
            
            if (Period_Value != 0)
            {
                // 计算捕获的占空比
                Duty_Cycle = (float)Pulse_Value / (float)Period_Value;
                
                /* 角度线性换算：
                   标准的 SA100 在 0%~100% 占空比下通常对应 0~360 度。
                   注：如果买到的传感器是 0-5V 模拟量转出来的，或者是 0-270 度的，
                   只需把下面的 360.0f 改为对应的最大量程即可。
                */
                Pendulum_Angle = Duty_Cycle * 360.0f;
            }
        }
    }
}
