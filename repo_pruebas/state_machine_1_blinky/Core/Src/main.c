/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdio.h>
#include <stdbool.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum {
	IDLE,
	PROCESS_COMMAND,
	PROCESS_EVENT,
	FINISH

}state;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define BUFFER_SIZE 32
#define IN_U TIM_CHANNEL_1
#define IN_V TIM_CHANNEL_2
#define IN_W TIM_CHANNEL_3
#define EN_U (1U << 10)  // PB10
#define EN_V (1U << 1)   // PB1
#define EN_W (1U << 11)  // PB11

#define CCR_TIM3 800
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
uint8_t rx_data[1];
uint8_t rx_buffer[BUFFER_SIZE];
uint8_t rx_index = 0;
volatile uint32_t timer4_channel2 = 0;
volatile int blink_count = 0;
volatile state state_machine = IDLE;
volatile uint32_t timerCounter = 0;
uint32_t eventDuration = 10;
uint32_t timer;
volatile bool eventFlag = false;
volatile uint8_t stepCounter = 0;
volatile uint8_t step = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM4_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */
void processCommand(char *input);
void clearRxBuffer(void);
void blink(void);
void handleState(void);
void ledON(void);
void commutation(uint8_t step);
void alignment(void);
typedef struct {
    char *command;               // Texto del comando
    void (*execute)();
} Command;
Command commandTable[] = {
    {"ALIGN", alignment},
	{"LEDON", ledON},
    {NULL, NULL}  // Marca el final de la tabla
};


/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void commutation(uint8_t step) {
    GPIOB->ODR &= ~(EN_U | EN_V | EN_W);

    switch(step) {
        case 0:
            GPIOB->ODR |= (EN_U | EN_V);  // Activar EN_U y EN_V
            HAL_TIM_PWM_Start(&htim3, IN_U);

            GPIOB->ODR &= ~EN_W;         // Desactivar EN_W
            __HAL_TIM_SET_COMPARE(&htim3, IN_U, CCR_TIM3);
            __HAL_TIM_SET_COMPARE(&htim3, IN_V, 0);

            break;
        case 1:
            GPIOB->ODR |= (EN_U | EN_W); // Activar EN_U y EN_W
            HAL_TIM_PWM_Start(&htim3, IN_U);

            GPIOB->ODR &= ~EN_V;         // Desactivar EN_V
            __HAL_TIM_SET_COMPARE(&htim3, IN_U, CCR_TIM3);
            __HAL_TIM_SET_COMPARE(&htim3, IN_W, 0);
            break;
        case 2:
            GPIOB->ODR |= (EN_V | EN_W); // Activar EN_V y EN_W
            HAL_TIM_PWM_Start(&htim3, IN_V);

            GPIOB->ODR &= ~EN_U;         // Desactivar EN_U
            __HAL_TIM_SET_COMPARE(&htim3, IN_V, CCR_TIM3);
            __HAL_TIM_SET_COMPARE(&htim3, IN_W, 0);
            break;
        case 3:
            GPIOB->ODR |= (EN_U | EN_V); // Activar EN_U y EN_V
            HAL_TIM_PWM_Start(&htim3, IN_V);

            GPIOB->ODR &= ~EN_W;         // Desactivar EN_W
            __HAL_TIM_SET_COMPARE(&htim3, IN_V, CCR_TIM3);
            __HAL_TIM_SET_COMPARE(&htim3, IN_U, 0);
            break;
        case 4:
            GPIOB->ODR |= (EN_U | EN_W); // Activar EN_U y EN_W
            HAL_TIM_PWM_Start(&htim3, IN_W);

            GPIOB->ODR &= ~EN_V;         // Desactivar EN_V
            __HAL_TIM_SET_COMPARE(&htim3, IN_W, CCR_TIM3);
            __HAL_TIM_SET_COMPARE(&htim3, IN_U, 0);
            break;
        case 5:
            GPIOB->ODR |= (EN_V | EN_W); // Activar EN_V y EN_W
            HAL_TIM_PWM_Start(&htim3, IN_W);

            GPIOB->ODR &= ~(EN_U | EN_V | EN_W);         // Desactivar EN_U
            __HAL_TIM_SET_COMPARE(&htim3, IN_W, CCR_TIM3);
            __HAL_TIM_SET_COMPARE(&htim3, IN_V, 0);
            break;
        default:
            break;
    }
}

void alignment(void){
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 499);
	stepCounter = 0;
	step = 0;
	commutation(1);
	HAL_TIM_OC_Start_IT(&htim4, TIM_CHANNEL_1);
}
void blink(void){
	eventDuration = 10;
	HAL_TIM_Base_Start_IT(&htim4);
}

void ledON(){

	eventFlag = true;

	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 19999);

	HAL_TIM_OC_Start_IT(&htim4, TIM_CHANNEL_1);


}
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if (htim->Instance == TIM4) {
		eventFlag = true;
		if (eventDuration > 0) {
			eventDuration--;
			HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
		} else {
			HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);

			HAL_TIM_Base_Stop_IT(&htim4);
			eventFlag = false;
	}
}
}

void processCommand(char *input) {
    for (int i = 0; commandTable[i].command != NULL; i++) {
        if (strcmp(input, commandTable[i].command) == 0) {  // Compara el comando recibido
            state_machine = PROCESS_EVENT;
            eventFlag = true;
        	commandTable[i].execute();  // Ejecuta la función asociada
            clearRxBuffer();  // Limpia el buffer acumulador

            return;
        }
    }
    // Si no se encuentra el comando
    HAL_UART_Transmit(&huart2, (uint8_t *)"Unknown Command\r\n", 17, 3);
    clearRxBuffer();
    state_machine = IDLE;
    return;// Limpia el buffer acumulador

}


void clearRxBuffer(void) {
    memset(rx_buffer, 0, BUFFER_SIZE);  // Establece todos los elementos a 0
    rx_index = 0;                          // Reinicia el índice del buffer
}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART2) {  // Verifica el UART correcto
        if (rx_data[0] == '\r') {  // Detecta el carácter de fin de línea
            rx_buffer[rx_index] = '\0';  // Finaliza la cadena con terminador nulo
            rx_index = 0;  // Reinicia el índice para el próximo mensaje

            // Procesa el mensaje completo
            HAL_UART_Transmit(&huart2, (uint8_t *)"\r\nReceived: ", 12, 3);
            HAL_UART_Transmit(&huart2, rx_buffer, strlen((char *)rx_buffer), 3);
            HAL_UART_Transmit(&huart2, (uint8_t *)"\r\n", 2, HAL_MAX_DELAY);
            state_machine = PROCESS_COMMAND;
        } else if (rx_data[0] == '\b' || rx_data[0] == '\177') {  // Detecta Backspace
            if (rx_index > 0) {  // Si hay algo en el buffer
                rx_index--;  // Retrocede el índice
                // Simula el borrado en el terminal
                HAL_UART_Transmit(&huart2, (uint8_t *)"\b \b", 3, 3);
            }
        } else {  // Cualquier otro carácter
            rx_buffer[rx_index++] = rx_data[0];  // Almacena el carácter en el buffer
            HAL_UART_Transmit(&huart2, rx_data, 1, HAL_MAX_DELAY);  // Eco del carácter recibido

            // Evita desbordamientos
            if (rx_index >= BUFFER_SIZE) {
                HAL_UART_Transmit(&huart2, (uint8_t *)"\r\nBuffer Overflow\r\n", 19, 3);
                rx_index = 0;  // Reinicia el índice en caso de desbordamiento
            }
        }

        // Reinicia la recepción para el siguiente byte
        HAL_UART_Receive_IT(&huart2, rx_data, 1);
    }
}

void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim) {
	if (htim->Instance == TIM4 && htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1) {
	    //TIM3->CCER &= ~(TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC3E | TIM_CCER_CC4E);
		__HAL_TIM_SET_COUNTER(&htim4, 0);
		if(stepCounter > 12){
			HAL_UART_Transmit(&huart2, (uint8_t*) "Delay concluido\r\n", 17, 4);

		HAL_TIM_PWM_Stop(&htim3, IN_U);
		HAL_TIM_PWM_Stop(&htim3, IN_V);
		HAL_TIM_PWM_Stop(&htim3, IN_W);
		eventFlag = false;
		HAL_TIM_OC_Stop_IT(&htim4, TIM_CHANNEL_1);
		return;
		}
		stepCounter++;
		commutation((step++) % 6);
	}
}

void handleState(void){
	switch(state_machine){
	case IDLE:
		if(!__HAL_UART_GET_IT_SOURCE(&huart2, UART_IT_RXNE)){
			__HAL_UART_ENABLE_IT(&huart2, UART_IT_RXNE);
		}
		break;

	case PROCESS_COMMAND:
	__HAL_UART_DISABLE_IT(&huart2, UART_IT_RXNE);
	processCommand((char*) rx_buffer);
	break;
	case PROCESS_EVENT:
		__HAL_UART_DISABLE_IT(&huart2, UART_IT_RXNE);
	if (!eventFlag) {
		HAL_UART_Transmit(&huart2, (uint8_t*) "Event Finished\r\n", 16, 3);
		state_machine = FINISH;
	}
	break;

	  case FINISH:
	            // Asegúrate de que las interrupciones de UART estén habilitadas al finalizar
	            timerCounter = 0; // Reinicia el contador
	            __HAL_UART_ENABLE_IT(&huart2, UART_IT_RXNE);
	            state_machine = IDLE; // Vuelve al estado IDLE
	            break;


}
}
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
  MX_TIM4_Init();
  MX_USART2_UART_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  //HAL_TIM_Base_Start_IT(&htim4);
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

  HAL_UART_Receive_IT(&huart2, rx_data, 1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  handleState();
		timer = __HAL_TIM_GET_COUNTER(&htim4);

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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 1799;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */
  TIM_OnePulse_InitTypeDef sConfig;
  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 7199;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 19999;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_OC_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  __HAL_TIM_ENABLE_OCxPRELOAD(&htim4, TIM_CHANNEL_1);
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, EN_V_Pin|EN_U_Pin|IN_WB11_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : EN_V_Pin EN_U_Pin IN_WB11_Pin */
  GPIO_InitStruct.Pin = EN_V_Pin|EN_U_Pin|IN_WB11_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

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

#ifdef  USE_FULL_ASSERT
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
