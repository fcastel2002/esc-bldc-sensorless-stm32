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
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim3;
 UART_HandleTypeDef huart2;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_tx;

/* USER CODE BEGIN PV */
const uint8_t float_phase_lookuptable[6] = {PHASE_W,PHASE_V,PHASE_U,PHASE_W,PHASE_V,PHASE_U};
 typedef enum {
	IDLE,
	PROCESS_COMMAND,
	STARTUP,
	RUNNING,
	ALIGNING,
	READY, // aligned
	FINISH

}state;
 void commutation(uint8_t step,uint16_t pwm);
 void commutator(void);
 void processCommand(char *input);
 void clearRxBuffer(void);

 void blink(void);
 state handleState(void);
 void ledON(void);
 void alignment(void);
 void startup(void);
 void pwm_stop(void);
 void pwm_init(void);
void enableZCP(uint8_t step);
 long map(long x, long in_min, long in_max, long out_min, long out_max) {
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
 uint8_t rx_data[1];
 uint8_t rx_buffer[BUFFER_SIZE];
 uint8_t rx_index = 0;
 volatile state state_machine = IDLE;
 volatile bool eventFlag = false;

 volatile uint16_t pwmVal = 0;
 volatile uint8_t commutationStep = 0;
 volatile uint16_t commutationTime = 0;
volatile uint8_t zcp_u  = 0;
volatile uint16_t reference_comm = 0;
volatile uint16_t reference_zcp = 0;
volatile uint32_t adc_val = 0;
volatile uint16_t adc_val_2;
volatile uint16_t zcp_period=800;
volatile uint16_t zcp_period_next;
volatile uint16_t zcp_period_flt;
volatile uint16_t nextCommutTime;
volatile uint16_t halfCommutTime;
 bool aligned_flag = false;
 bool startup_flag = false;
 bool startup_ok = false;
 typedef struct {
     char *command;               // Texto del comando
     void (*execute)();
 } Command;

 Command commandTable[] = {
     {"ALIGN", alignment},
 	{"LEDON", ledON},
 	{"STARTUP", startup},
     {NULL, NULL}  // Marca el final de la tabla
 };
 typedef struct{
 	state currentState;
 	bool *eventFlag;
 	uint32_t delay;
 }TimerDependentEvent;

 TimerDependentEvent eventTable[]={
 		{ALIGNING, &aligned_flag, 1},
 		{RUNNING, &eventFlag, 1},
		{STARTUP, &startup_flag, STARTUP_TIME_X},
 		{IDLE, NULL, 0}
 };


#define COEF_ADVC_ANGLE 0.375
#define NUM_TIMER_EVENTS (sizeof(eventTable)/sizeof(eventTable[0]))
#define PWM_STOP 		HAL_TIM_PWM_Stop(&htim3, IN_U);\
						HAL_TIM_PWM_Stop(&htim3, IN_V);\
						HAL_TIM_PWM_Stop(&htim3, IN_W);
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM4_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM2_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM1_Init(void);
/* USER CODE BEGIN PFP */




/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


void pwm_stop(void) {
	HAL_TIM_PWM_Stop(&htim3, IN_U);
	HAL_TIM_PWM_Stop(&htim3, IN_V);
	HAL_TIM_PWM_Stop(&htim3, IN_W);
}

void pwm_init(void){
	__HAL_TIM_SET_COMPARE(&htim3, IN_U, 0);
	__HAL_TIM_SET_COMPARE(&htim3, IN_V, 0);
	__HAL_TIM_SET_COMPARE(&htim3, IN_W, 0);

	HAL_TIM_PWM_Start(&htim3, IN_U);
	HAL_TIM_PWM_Start(&htim3,IN_V);
	HAL_TIM_PWM_Start(&htim3, IN_W);
	return;
}

void alignment(void){
	aligned_flag = false;
	state_machine = ALIGNING;
	HAL_TIM_Base_Stop_IT(&htim4);
	HAL_TIM_OC_Stop_IT(&htim4, TIM_CHANNEL_1);
	HAL_TIM_Base_Stop(&htim4);

	__HAL_TIM_SET_COUNTER(&htim4, 0);
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, ALIGN_TIME);
	pwm_init();

	commutation(POS_UV, DC_ALIGN);
	HAL_TIM_Base_Start_IT(&htim4);
	HAL_TIM_OC_Start_IT(&htim4, TIM_CHANNEL_1);
	while(!aligned_flag);
	state_machine = READY;
	pwm_stop();
}

void startup(void){
	alignment();
	state_machine =  STARTUP;
	bool  localFlag = false;
	pwmVal = DC_STARTUP_INIT;
	commutationStep = POS_UV;
	commutationTime = TIME_STARTUP_COMMUTATION_INIT;
	uint16_t duration = 0;
	pwm_init();
	while(!localFlag){
		//__HAL_TIM_SET_COUNTER(&htim4, 0);
		//__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, time);
		HAL_Delay(commutationTime);
		commutation(commutationStep, pwmVal);
		//HAL_TIM_Base_Start_IT(&htim4);
		//HAL_TIM_OC_Start_IT(&htim4, TIM_CHANNEL_1);
		//while(!startup_flag);

		commutationStep = (commutationStep + 1) % NUM_POS;
		(pwmVal < DC_STARTUP_END) ? pwmVal += DC_STARTUP_STEP : pwmVal;
	    if (commutationTime > TIME_STARTUP_COMMUTATION_FINAL) {
	    	commutationTime -= TIME_STARTUP_STEP;
	    } else if (duration < 500){
	        duration++;

	    }
	    else {

			localFlag = true;
			__HAL_TIM_SET_COUNTER(&htim4, 0);


	    }
//		startup_flag = false;


	}
	//commutationTime*=10;
	HAL_TIM_OC_Stop_IT(&htim4, TIM_CHANNEL_1); // Detiene canal 1
//	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, commutationTime); // Configura canal 2
	HAL_TIMEx_HallSensor_Start_IT(&htim2);
	reference_zcp = 0;
	startup_ok = true;
	TIM4->ARR = 0xFFFF;
	TIM4->PSC = 4;
	//HAL_ADC_Start_IT(&hadc1);
	HAL_TIM_OC_Start_IT(&htim4, TIM_CHANNEL_2);
	__HAL_TIM_SET_COUNTER(&htim4, 0);


	// Inicia el canal 2 en modo Output Compare


}


void ledON(){

	eventFlag = true;

	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 19999);

	HAL_TIM_OC_Start_IT(&htim4, TIM_CHANNEL_1);


}
void processCommand(char *input) {
    for (int i = 0; commandTable[i].command != NULL; i++) {
        if (strcmp(input, commandTable[i].command) == 0) {  // Compara el comando recibido
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
            HAL_UART_Transmit(&huart2, rx_buffer	, strlen((char *)rx_buffer), 3);
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
state handleState(void){
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

	case STARTUP:
		__HAL_UART_DISABLE_IT(&huart2, UART_IT_RXNE);
		//startup();
		break;
	case ALIGNING:
		__HAL_UART_DISABLE_IT(&huart2, UART_IT_RXNE);
		alignment();
		break;
	case RUNNING:
		__HAL_UART_DISABLE_IT(&huart2, UART_IT_RXNE);
		break;

	case READY:
		if(!__HAL_UART_GET_IT_SOURCE(&huart2, UART_IT_RXNE)){
					__HAL_UART_ENABLE_IT(&huart2, UART_IT_RXNE);
				}
		break;
	case FINISH:
		// Asegúrate de que las interrupciones de UART estén habilitadas al finalizar
		//timerCounter = 0; // Reinicia el contador
		__HAL_UART_ENABLE_IT(&huart2, UART_IT_RXNE);
		state_machine = IDLE; // Vuelve al estado IDLE

		break;


}
	return (state_machine);
}
/*
 __disable_irq();
//HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
realCommutTime = HAL_TIM_ReadCapturedValue(&htim2, TIM_CHANNEL_1);
TIM4->CCR2 = realCommutTime;
__enable_irq();
*/


void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim){
	if(htim->Instance == TIM2){
		HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
		//reference_zcp++;
		zcp_period_next = HAL_TIM_ReadCapturedValue(&htim2, TIM_CHANNEL_1);

		zcp_period_flt = zcp_period*0.5 + zcp_period_next*0.5;
		halfCommutTime = COEF_ADVC_ANGLE * zcp_period_flt;
		zcp_period = zcp_period_next;
		nextCommutTime = zcp_period + halfCommutTime;

		__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, nextCommutTime);

	}
}

void commutation(uint8_t step, uint16_t pwm) {

    switch(step) {

        case POS_UV:

            GPIOB->ODR |= EN_U;
			GPIOB->ODR |= EN_V;  // Activar EN_U y EN_V

            GPIOA->ODR &= ~EN_W;         // Desactivar EN_W
            __HAL_TIM_SET_COMPARE(&htim3, IN_U, pwm);
            __HAL_TIM_SET_COMPARE(&htim3, IN_V, 0);
            HAL_TIM_PWM_Start(&htim3, IN_U);
            HAL_TIM_PWM_Start(&htim3, IN_V);

            break;
        case POS_UW:

            GPIOB->ODR |= EN_U ; // Activar EN_U y EN_W
            GPIOA->ODR |=  EN_W;
            GPIOB->ODR &= ~EN_V;         // Desactivar EN_V
            __HAL_TIM_SET_COMPARE(&htim3, IN_U, pwm);
            __HAL_TIM_SET_COMPARE(&htim3, IN_W, 0);
            HAL_TIM_PWM_Start(&htim3, IN_U);
            HAL_TIM_PWM_Start(&htim3, IN_W);
            break;
        case POS_VW:

            GPIOB->ODR |= EN_V; // Activar EN_V y EN_W
            GPIOA->ODR |=  EN_W;

            GPIOB->ODR &= ~EN_U;         // Desactivar EN_U
            __HAL_TIM_SET_COMPARE(&htim3, IN_V, pwm);
            __HAL_TIM_SET_COMPARE(&htim3, IN_W, 0);
            HAL_TIM_PWM_Start(&htim3, IN_V);
            HAL_TIM_PWM_Start(&htim3, IN_W);
            break;
        case POS_VU:

            GPIOB->ODR |= EN_U; // Activar EN_U y EN_V
            GPIOB->ODR |=  EN_V;
            GPIOA->ODR &= ~EN_W;         // Desactivar EN_W
            __HAL_TIM_SET_COMPARE(&htim3, IN_V, pwm);
            __HAL_TIM_SET_COMPARE(&htim3, IN_U, 0);
            HAL_TIM_PWM_Start(&htim3, IN_V);
            HAL_TIM_PWM_Start(&htim3, IN_U);

            break;
        case POS_WU:

            GPIOB->ODR |= EN_U; // Activar EN_U y EN_W
            GPIOA->ODR |=  EN_W;
            GPIOB->ODR &= ~EN_V;         // Desactivar EN_V
            __HAL_TIM_SET_COMPARE(&htim3, IN_W, pwm);
            __HAL_TIM_SET_COMPARE(&htim3, IN_U, 0);
            HAL_TIM_PWM_Start(&htim3, IN_W);
            HAL_TIM_PWM_Start(&htim3, IN_U);
            break;
        case POS_WV:

            GPIOB->ODR |= EN_V; // Activar EN_V y EN_W
            GPIOA->ODR |=  EN_W;
            GPIOB->ODR &= ~EN_U;         // Desactivar EN_U
            __HAL_TIM_SET_COMPARE(&htim3, IN_W, pwm);
            __HAL_TIM_SET_COMPARE(&htim3, IN_V, 0);
            HAL_TIM_PWM_Start(&htim3, IN_W);
            HAL_TIM_PWM_Start(&htim3, IN_V);

            break;
        default:
            break;
    }
}



void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim) {
	if (htim->Instance == TIM4 && htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1) {
	    //TIM3->CCER &= ~(TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC3E | TIM_CCER_CC4E);
		__HAL_TIM_DISABLE(&htim4);

		__HAL_TIM_SET_COUNTER(&htim4, 0);
		HAL_TIM_Base_Stop_IT(&htim4);
		HAL_TIM_Base_Stop(&htim4);

		HAL_TIM_OC_Stop_IT(&htim4, TIM_CHANNEL_1);
		for (int i = 0; i < NUM_TIMER_EVENTS; i++) {
			// Detener el temporizador y deshabilitar el contador
			if (eventTable[i].currentState == state_machine) {
				if (eventTable[i].eventFlag != NULL) {
					*(eventTable[i].eventFlag) = true;
					__HAL_TIM_ENABLE(&htim4);
					__HAL_TIM_SET_COUNTER(&htim4, 0);

					HAL_TIM_OC_Stop_IT(&htim4, TIM_CHANNEL_1);

					return;

				}
				break;

			}


		}
		__HAL_TIM_SET_COUNTER(&htim4, 0);
		HAL_TIM_Base_Start_IT(&htim4);
		HAL_TIM_OC_Start_IT(&htim4, TIM_CHANNEL_1);
		__HAL_TIM_ENABLE(&htim4);


	}
	if (htim->Instance == TIM4 && htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2) {
        //TIM4->CCR2 = commutationTime;
		//(commutationStep);
		__disable_irq();
		commutation(commutationStep, pwmVal);

		commutationStep = (commutationStep + 1) % NUM_POS;
		reference_comm++;
		__HAL_TIM_SET_COUNTER(&htim4, 0);
		__enable_irq();
	}
	if(htim->Instance == TIM2 && htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2){
		reference_zcp++;
	}
}


void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc){
	HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
	adc_val = HAL_ADC_GetValue(hadc);
	pwmVal = map(adc_val, 0, 4095, 1250, 1799);
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
  MX_DMA_Init();
  MX_TIM4_Init();
  MX_USART2_UART_Init();
  MX_TIM3_Init();
  MX_TIM2_Init();
  MX_ADC1_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
  //HAL_TIM_Base_Start_IT(&htim4);
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
  state statetete;


  HAL_UART_Receive_IT(&huart2, rx_data, 1);
  /*
  HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1);
  HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_2);
  HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_3);
*/
//	HAL_TIMEx_HallSensor_Start_IT(&htim2);

  //HAL_TIM_OC_Start(&htim1, TIM_CHANNEL_1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  statetete = handleState();
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV8;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T1_CC1;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_SlaveConfigTypeDef sSlaveConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 2;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 1799;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_TRIGGER;
  sSlaveConfig.InputTrigger = TIM_TS_ITR0;
  if (HAL_TIM_SlaveConfigSynchro(&htim1, &sSlaveConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_OC2REF;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_ENABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  sConfigOC.Pulse = 1770;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_OC_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_HallSensor_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 4;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 0xffff;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV2;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 15;
  sConfig.Commutation_Delay = 0;
  if (HAL_TIMEx_HallSensor_Init(&htim2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_OC2REF;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
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
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
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
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
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
  if (HAL_TIM_OC_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel7_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel7_IRQn);

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
  HAL_GPIO_WritePin(EN_W_GPIO_Port, EN_W_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, EN_V_Pin|EN_U_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : EN_W_Pin */
  GPIO_InitStruct.Pin = EN_W_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(EN_W_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : EN_V_Pin EN_U_Pin */
  GPIO_InitStruct.Pin = EN_V_Pin|EN_U_Pin;
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
