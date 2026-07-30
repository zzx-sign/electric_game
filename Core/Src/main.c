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
#include "OLED.h"
#include "app.h"
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
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM8_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_TIM5_Init();
  /* USER CODE BEGIN 2 */
  App_Init();
  HAL_TIM_Base_Start_IT(&htim5);
  __HAL_UART_ENABLE(&huart1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    App_KeyScan();

    static uint32_t last_disp = 0;
    uint32_t now = HAL_GetTick();
    if (now - last_disp >= 100) {
      last_disp = now;
      OLED_Clear();
      OLED_ShowString(0, 0, "M3:", OLED_6X8);
      OLED_ShowSignedNum(18, 0, (int32_t)g_m3_speed_mmps, 3, OLED_6X8);
      OLED_ShowString(48, 0, "M4:", OLED_6X8);
      OLED_ShowSignedNum(66, 0, (int32_t)g_m4_speed_mmps, 3, OLED_6X8);
      OLED_ShowString(0, 8, "ST:", OLED_6X8);
      switch (g_app_state) {
        case APP_STATE_IDLE:    OLED_ShowString(24, 8, "IDLE", OLED_6X8); break;
        case APP_STATE_RUNNING:  OLED_ShowString(24, 8, "RUN",  OLED_6X8); break;
        case APP_STATE_DONE:    OLED_ShowString(24, 8, "DONE", OLED_6X8); break;
      }
      OLED_ShowString(0, 16, "MODE:", OLED_6X8);
      switch (g_app_mode) {
        case APP_MODE_KEY2: OLED_ShowString(36, 16, "key2", OLED_6X8); break;
        case APP_MODE_KEY3: OLED_ShowString(36, 16, "key3", OLED_6X8); break;
        case APP_MODE_KEY4: OLED_ShowString(36, 16, "key4", OLED_6X8); break;
        default:            OLED_ShowString(36, 16, "----", OLED_6X8); break;
      }

      /* 显示运行时间：
       * - 正在跑：显示当前经过时间
       * - 触发过丢线停车：显示定格时间
       * - 还没跑过：显示 ---- */
      uint32_t elapsed_ms = 0;
      if (g_app_state == APP_STATE_RUNNING && g_run_started) {
        elapsed_ms = now - g_run_start_tick;
      } else if (g_run_had_finish) {
        elapsed_ms = g_run_stop_tick - g_run_start_tick;
      }
      uint32_t sec_int = elapsed_ms / 1000U;
      uint32_t sec_frac = (elapsed_ms % 1000U) / 100U;
      OLED_ShowString(0, 24, "T:", OLED_6X8);
      if (!g_run_started) {
        OLED_ShowString(12, 24, "----", OLED_6X8);
      } else {
        OLED_ShowNum(12, 24, sec_int, 3, OLED_6X8);
        OLED_ShowChar(36, 24, '.', OLED_6X8);
        OLED_ShowNum(42, 24, sec_frac, 1, OLED_6X8);
        OLED_ShowString(54, 24, "s", OLED_6X8);
      }

      /* 显示已行走距离（毫米），保留一位小数 */
      OLED_ShowString(0, 32, "D:", OLED_6X8);
      if (!g_run_started) {
        OLED_ShowString(12, 32, "----", OLED_6X8);
      } else {
        OLED_ShowFloatNum(12, 32, g_accum_distance_mm, 4, 1, OLED_6X8);
        OLED_ShowString(54, 32, "mm", OLED_6X8);
      }
      OLED_Update();
    }
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
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 192;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim == &htim5) {
    Cascade_Update();
  }
}
/* USER CODE END 4 */

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
