/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "stm32h5xx_hal.h"

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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LED2_Pin GPIO_PIN_0
#define LED2_GPIO_Port GPIOC
#define LED3_Pin GPIO_PIN_1
#define LED3_GPIO_Port GPIOC
#define PWM4_Pin GPIO_PIN_2
#define PWM4_GPIO_Port GPIOC
#define PH1D_Pin GPIO_PIN_3
#define PH1D_GPIO_Port GPIOC
#define VBAT_Pin GPIO_PIN_0
#define VBAT_GPIO_Port GPIOA
#define RC2_Pin GPIO_PIN_1
#define RC2_GPIO_Port GPIOA
#define RC3_Pin GPIO_PIN_2
#define RC3_GPIO_Port GPIOA
#define RC4_Pin GPIO_PIN_3
#define RC4_GPIO_Port GPIOA
#define STOP_Pin GPIO_PIN_4
#define STOP_GPIO_Port GPIOA
#define RC1_Pin GPIO_PIN_5
#define RC1_GPIO_Port GPIOA
#define IR1_AN_Pin GPIO_PIN_6
#define IR1_AN_GPIO_Port GPIOA
#define IR2_AN_Pin GPIO_PIN_7
#define IR2_AN_GPIO_Port GPIOA
#define IR3_AN_Pin GPIO_PIN_0
#define IR3_AN_GPIO_Port GPIOB
#define IR4_AN_Pin GPIO_PIN_1
#define IR4_AN_GPIO_Port GPIOB
#define LED1_Pin GPIO_PIN_2
#define LED1_GPIO_Port GPIOB
#define PH2D_Pin GPIO_PIN_13
#define PH2D_GPIO_Port GPIOB
#define PIXEL_Pin GPIO_PIN_14
#define PIXEL_GPIO_Port GPIOB
#define PH_EN_Pin GPIO_PIN_15
#define PH_EN_GPIO_Port GPIOB
#define IR_REC_Pin GPIO_PIN_6
#define IR_REC_GPIO_Port GPIOC
#define IR2_Pin GPIO_PIN_7
#define IR2_GPIO_Port GPIOC
#define SD_SW_Pin GPIO_PIN_8
#define SD_SW_GPIO_Port GPIOA
#define IR1_Pin GPIO_PIN_4
#define IR1_GPIO_Port GPIOB
#define PWM1_Pin GPIO_PIN_6
#define PWM1_GPIO_Port GPIOB
#define PWM2_Pin GPIO_PIN_7
#define PWM2_GPIO_Port GPIOB
#define PWM3_Pin GPIO_PIN_8
#define PWM3_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
