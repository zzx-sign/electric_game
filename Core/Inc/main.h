/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define key_2_Pin GPIO_PIN_2
#define key_2_GPIO_Port GPIOE
#define key_3_Pin GPIO_PIN_3
#define key_3_GPIO_Port GPIOE
#define key_4_Pin GPIO_PIN_4
#define key_4_GPIO_Port GPIOE
#define key_1_Pin GPIO_PIN_0
#define key_1_GPIO_Port GPIOA
#define Encoder4_1_Pin GPIO_PIN_1
#define Encoder4_1_GPIO_Port GPIOA
#define Encoder4_2_Pin GPIO_PIN_5
#define Encoder4_2_GPIO_Port GPIOA
#define Encoder1_1_Pin GPIO_PIN_6
#define Encoder1_1_GPIO_Port GPIOA
#define Encoder1_2_Pin GPIO_PIN_7
#define Encoder1_2_GPIO_Port GPIOA
#define PWM1_Pin GPIO_PIN_9
#define PWM1_GPIO_Port GPIOE
#define PWM2_Pin GPIO_PIN_11
#define PWM2_GPIO_Port GPIOE
#define PWM3_Pin GPIO_PIN_13
#define PWM3_GPIO_Port GPIOE
#define PWM4_Pin GPIO_PIN_14
#define PWM4_GPIO_Port GPIOE
#define Encoder3_2_Pin GPIO_PIN_12
#define Encoder3_2_GPIO_Port GPIOD
#define Encoder3_1_Pin GPIO_PIN_13
#define Encoder3_1_GPIO_Port GPIOD
#define Encoder2_2_Pin GPIO_PIN_6
#define Encoder2_2_GPIO_Port GPIOC
#define Encoder2_1_Pin GPIO_PIN_7
#define Encoder2_1_GPIO_Port GPIOC
#define Trail_AD0_Pin GPIO_PIN_11
#define Trail_AD0_GPIO_Port GPIOA
#define Trail_AD1_Pin GPIO_PIN_12
#define Trail_AD1_GPIO_Port GPIOA
#define Trail_AD2_Pin GPIO_PIN_15
#define Trail_AD2_GPIO_Port GPIOA
#define Trail_Out_Pin GPIO_PIN_10
#define Trail_Out_GPIO_Port GPIOC
#define AIN4_2_Pin GPIO_PIN_4
#define AIN4_2_GPIO_Port GPIOD
#define AIN4_1_Pin GPIO_PIN_5
#define AIN4_1_GPIO_Port GPIOD
#define AIN3_2_Pin GPIO_PIN_6
#define AIN3_2_GPIO_Port GPIOD
#define AIN3_1_Pin GPIO_PIN_7
#define AIN3_1_GPIO_Port GPIOD
#define AIN2_2_Pin GPIO_PIN_3
#define AIN2_2_GPIO_Port GPIOB
#define AIN2_1_Pin GPIO_PIN_5
#define AIN2_1_GPIO_Port GPIOB
#define AIN1_1_Pin GPIO_PIN_6
#define AIN1_1_GPIO_Port GPIOB
#define AIN1_2_Pin GPIO_PIN_7
#define AIN1_2_GPIO_Port GPIOB
#define OLED_SCL_Pin GPIO_PIN_8
#define OLED_SCL_GPIO_Port GPIOB
#define OLED_SDA_Pin GPIO_PIN_9
#define OLED_SDA_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
