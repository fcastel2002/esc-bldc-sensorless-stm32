/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdio.h>
#include <stdbool.h>
#include <stdlib.h>
#include <stdarg.h>
#include "basic_config.h"
#include "state_machine.h"

#include "math.h"

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
//timers
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim1;


// comms
extern const uint32_t CPU_clock;
//extern UART_HandleTypeDef huart1;
// dma
//extern DMA_HandleTypeDef hdma_usart1_tx;

extern  CRC_HandleTypeDef hcrc;

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
extern volatile uint32_t led_tick;

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LED_Pin GPIO_PIN_13
#define LED_GPIO_Port GPIOC
#define W_Pin GPIO_PIN_0
#define W_GPIO_Port GPIOA
#define U_Pin GPIO_PIN_1
#define U_GPIO_Port GPIOA
#define EN_W_Pin GPIO_PIN_5
#define EN_W_GPIO_Port GPIOA
#define EN_V_Pin GPIO_PIN_1
#define EN_V_GPIO_Port GPIOB
#define V_Pin GPIO_PIN_10
#define V_GPIO_Port GPIOB
#define EN_U_Pin GPIO_PIN_13
#define EN_U_GPIO_Port GPIOB
#define IN_U_Pin GPIO_PIN_8
#define IN_U_GPIO_Port GPIOA
#define IN_V_Pin GPIO_PIN_9
#define IN_V_GPIO_Port GPIOA
#define IN_W_Pin GPIO_PIN_10
#define IN_W_GPIO_Port GPIOA

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
