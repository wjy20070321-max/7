/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ã·¨Ä£ï¿½ï¿½Í·ï¿½Ä¼ï¿½È«ï¿½ï¿½×¢ï¿½ï¿½ */
#include "bsp_motor.h"
#include "bsp_encoder.h"
#include "bsp_sensor.h"
#include "bsp_sa100.h"
#include "bsp_hmi.h"
#include "app_control.h"
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* ï¿½ï¿½ï¿½ï¿½ï¿½â²¿×´Ì¬ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Ç°Ì¨ï¿½ë´®ï¿½ï¿½ï¿½ï¿½ï¿½Ä·ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ì²½ï¿½ï¿½ï¿½ï¿½ */
extern volatile float Pendulum_Angle;
extern uint8_t sys_start_flag;
extern float Left_Motor_Out;
extern float Right_Motor_Out;
extern volatile Target_Task_e Current_Task;
extern UART_HandleTypeDef huart2; // ï¿½ï¿½ï¿½ï¿½ VOFA ï¿½Ä´ï¿½ï¿½ï¿½ 2 ï¿½ï¿½ï¿½
extern float Actual_Speed_L;
extern float Target_Speed_L;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM8_Init();
  MX_TIM10_Init();
  MX_TIM11_Init();
  MX_TIM12_Init();
  MX_USART1_UART_Init();
  MX_TIM5_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  /* --- È«ÏµÍ³ï¿½×²ï¿½Ó²ï¿½ï¿½ï¿½ï¿½ï¿½ã·¨Ä£ï¿½ï¿½ï¿½Ê¼ï¿½ï¿½ --- */
  Motor_Init();         // 1. ï¿½ï¿½Ê¼ï¿½ï¿½ TB6612 ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ TIM1 PWM 
  Encoder_Init();       // 2. ï¿½ï¿½ï¿½ï¿½ TIM3/TIM4 ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Ó²ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½
  SA100_Init();         // 3. ï¿½ï¿½ï¿½ï¿½ TIM5 ï¿½Ç¶È´ï¿½ï¿½ï¿½ï¿½ï¿½Ë«Í¨ï¿½ï¿½ï¿½ï¿½ï¿½ë²¶ï¿½ï¿½ï¿½Ð¶ï¿½
  HMI_Init();           // 4. ï¿½ï¿½ï¿½ï¿½ USART1 ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Ð¶ï¿½
  Control_Task_Init();  // 5. ï¿½ï¿½Ê¼ï¿½ï¿½ï¿½à»· PID ï¿½á¹¹ï¿½å¡¢ï¿½Ë²ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Ä£ï¿½ï¿½
  
  /* 6. ï¿½ï¿½ï¿½ï¿½×¨ï¿½Å¸ï¿½ï¿½ï¿½ 5ms ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ÚµÄ¶ï¿½Ê±ï¿½ï¿½ï¿½Ð¶ï¿½ (ï¿½Ë´ï¿½ï¿½ï¿½ TIM10 Îªï¿½ï¿½) */
  /* ï¿½ï¿½È·ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ CubeMX ï¿½Ð½ï¿½ï¿½Ã¶ï¿½Ê±ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Îªï¿½ï¿½ 5ms ï¿½ï¿½ï¿½ï¿½Ò»ï¿½ï¿½ï¿½Ð¶ï¿½ */
  HAL_TIM_Base_Start_IT(&htim10); 
  
  /* ï¿½ï¿½ï¿½ï¿½Ç°Ì¨ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Ê±ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Ú¿ï¿½ï¿½Æ¸ï¿½ï¿½ï¿½Ä»ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Ýµï¿½Æµï¿½ï¿½ */
  uint32_t HMI_Refresh_Timer = HAL_GetTick();
  uint32_t VOFA_Refresh_Timer = HAL_GetTick();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* 1. ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Ç°Ì¨ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ */
    HMI_Process_Command();
    
    /* 2. ï¿½ï¿½Ê±ï¿½ò´®¿ï¿½ï¿½ï¿½Ë¢ï¿½ï¿½ï¿½ï¿½ï¿½Ý£ï¿½Ã¿ 50ms Ò»ï¿½ï¿½ */
    if (HAL_GetTick() - HMI_Refresh_Timer >= 50)
    {
        HMI_Refresh_Timer = HAL_GetTick();
        HMI_Send_Data(Pendulum_Angle, (float)((int16_t)__HAL_TIM_GET_COUNTER(&htim3)));
    }
    
    /* 3. VOFA+ ï¿½ï¿½Î»ï¿½ï¿½ï¿½ï¿½ï¿½Ù²ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Ã¿ 10ms Ë¢ï¿½ï¿½Ò»ï¿½Î£ï¿½ï¿½ï¿½ï¿½Ú¿ï¿½ PID ï¿½Ë¶ï¿½ï¿½ï¿½ï¿½ï¿½ */
    if (HAL_GetTick() - VOFA_Refresh_Timer >= 10)
    {
        VOFA_Refresh_Timer = HAL_GetTick();
        
        // ï¿½ï¿½ï¿½ï¿½ JustFloat Ð­ï¿½ï¿½ï¿½ï¿½ï¿½Ý°ï¿½ (4ï¿½ï¿½Í¨ï¿½ï¿½)
        float vofa_data[4];
        vofa_data[0] = Target_Speed_L; // CH0 ï¿½ï¿½Îªï¿½ï¿½Ä¿ï¿½ï¿½×ªï¿½ï¿½Ö±ï¿½ï¿½ (RPM)
        vofa_data[1] = Actual_Speed_L; // CH1 ï¿½ï¿½Îªï¿½ï¿½Êµï¿½Ê²ï¿½ï¿½ï¿½×ªï¿½ï¿½ (RPM)
        vofa_data[2] = Left_Motor_Out; // CH2 ï¿½ï¿½ï¿½Ö£ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ PWM (ï¿½ï¿½ï¿½Ú¶ï¿½ï¿½ï¿½)
        vofa_data[3] = Pendulum_Angle; // CH3 ï¿½ï¿½Îªï¿½ï¿½ÊµÊ±ï¿½Ç¶ï¿½ (ï¿½ï¿½ï¿½ï¿½ï¿½Ôºï¿½)

        // Ö¡Î²ï¿½ï¿½ï¿½ï¿½ (JustFloat ï¿½ï¿½×¼Ö¡Î²ï¿½ï¿½0x00 0x00 0x80 0x7F)
        uint8_t tail[4] = {0x00, 0x00, 0x80, 0x7F};
        
        // ï¿½ï¿½ï¿½Í¸ï¿½ï¿½ï¿½ï¿½Ô¶Ëµï¿½ VOFA+
        HAL_UART_Transmit(&huart2, (uint8_t*)vofa_data, sizeof(vofa_data), 10);
        HAL_UART_Transmit(&huart2, tail, 4, 10);
    }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        // Èç¹û´¥·¢ÁË ORE Òç³ö´íÎó
        if (__HAL_UART_GET_FLAG(huart, UART_FLAG_ORE) != RESET)
        {
            __HAL_UART_CLEAR_OREFLAG(huart); // Çå³ý±êÖ¾Î»

            // ? ÖØµã£ºÕâÀïÐèÒªÖØÐÂ¿ªÆôÄãµÄÖÐ¶Ï½ÓÊÕ
            // ±ÈÈçÄãµÄ HMI_Init ÀïÃæÊÇÓÃÏÂÃæÕâ¾ä¿ªÆôµÄ£¬¾Í°ÑËüÔ­Ñù³­¹ýÀ´£º
            // HAL_UART_Receive_IT(&huart1, &ÄãµÄ½ÓÊÕ±äÁ¿, 1);
        }
    }
}
/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */
  /* ï¿½ï¿½ï¿½ï¿½ï¿½Þ¸ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ TIM10 ï¿½ï¿½ï¿½Ð¶Ï´ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Ú¹ï¿½ï¿½ï¿½ 5ms ï¿½Äºï¿½ï¿½Ä±Õ»ï¿½ï¿½ã·¨ */
  /* ï¿½ï¿½ï¿½ï¿½Ð´ï¿½ï¿½ BEGIN ï¿½ï¿½ END Ö®ï¿½ä£¬ï¿½ï¿½Ö¹ï¿½ï¿½ CubeMX É¾ï¿½ï¿½ */
  if (htim->Instance == TIM10)
  {
    Control_Task_Loop_5ms(); 
  }
  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
