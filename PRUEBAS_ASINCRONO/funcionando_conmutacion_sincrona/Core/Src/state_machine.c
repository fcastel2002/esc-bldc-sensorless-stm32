/*
 * app_state.c
 *
 *  Created on: Feb 3, 2025
 *      Author: Usuario
 */


#include "state_machine.h"

volatile App_States_t app_state = IDLE;
volatile bool eventFlag = false;
bool aligned_flag = false;
bool startup_flag = false;
bool startup_ok = false;

const TimerDependentEvent eventTable[]={
		{ALIGNING, &aligned_flag, ALIGN_TIME},
		{RUNNING, &eventFlag, 1},
		{STARTUP, &startup_flag, STARTUP_TIME_X},
		{IDLE, NULL, 0}
};

App_States_t handleState(void){
	static uint8_t comm_initialized = 0;
	static uint32_t last_uart_check = 0;
	static uint8_t flash_initialized = 0;
	if(!comm_initialized){
		commInit();
		comm_initialized = 1;
	}

	if (app_state != RUNNING && app_state != CLOSEDLOOP) {
		processUartData();
	}

	if(!flash_initialized){
		FlashResultCode result = flash_config_init();
		if(result == FLASH_RESULT_OK || result == FLASH_RESULT_EMPTY){
			update_all_esc();
			update_all_motor_control();
			flash_initialized = 1;
		}else{
			app_state = HARD_ERROR;
			flash_initialized = 0;
		}
	}
	if(motor_stalled && speed_setpoint > 200){
		foc_startup();
		motor_stalled = false;
	}
	switch(app_state){
	case IDLE:
		break;

	case PROCESS_COMMAND:
		if (cmd_received_ack) {
			// Procesar comando actual
			processCurrentCommand();

			if (!running_cmd_ack && !set_cmd_received_ack) {
				app_state = IDLE;
			}
			else if (set_cmd_received_ack) {
				esc_config_done = 0;
				motor_control_config_done = 0;
				set_cmd_received_ack = 0;
				app_state = CONFIG;
			}

		}
		cmd_received_ack = 0;
		if(cmd_speed_received_ack){
			app_state = CLOSEDLOOP;
			cmd_speed_received_ack = 0;
		}
		break;

	case STARTUP:
		break;
	case ALIGNING:
		break;
	case FOC_STARTUP:
		break;
	case RUNNING:
		uint32_t current_time = HAL_GetTick();
		if (current_time - last_uart_check > 50) {
			last_uart_check = current_time;

			// Procesar datos UART
			processUartData();

			// Procesar comandos de velocidad si hay alguno disponible
			if (cmd_received_ack) {
				if (processSpeedCommand()) {
					// Si se procesó correctamente un comando de velocidad
					cmd_received_ack = 0;
				} else {
					// Ignorar otros comandos en este estado
					cmd_received_ack = 0;
				}
			}
		}
		if (consistent_zero_crossing) {
			app_state = READY;
		}
		break;
	case CONFIG:
		if(!esc_config_done && !motor_control_config_done)update_all_esc();
		if(esc_config_done  && !motor_control_config_done)update_all_motor_control();
		if(esc_config_done && motor_control_config_done) app_state = IDLE;
		break;
	case CLOSEDLOOP:

		HAL_TIM_OC_Start_IT(&htim4, TIM_CHANNEL_1);
		current_time = HAL_GetTick();
		if (current_time - last_uart_check > 50) {
			last_uart_check = current_time;

			// Procesar datos UART
			processUartData();

			// Procesar comandos de velocidad si hay alguno disponible
			if (cmd_received_ack) {
				if (processSpeedCommand()) {
					// Si se procesó correctamente un comando de velocidad
					cmd_received_ack = 0;
				} else {
					// Ignorar otros comandos en este estado
					cmd_received_ack = 0;
				}
			}
		}
		//closedLoop();
		break;
	case READY:
		TIM4->PSC = 2;
		TIM4->ARR = 0xFFFF; //
		__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1,48000); //output compare 1kHz
		app_state = CLOSEDLOOP;

		break;
	case STOPPED:
		__HAL_UART_DISABLE_IT(&huart2, UART_IT_RXNE);
		break;

	case FINISH:
		// Asegúrate de que las interrupciones de UART estén habilitadas al finalizar
		__HAL_UART_ENABLE_IT(&huart2, UART_IT_RXNE);
		app_state = IDLE; // Vuelve al estado IDLE

		break;

	case HARD_ERROR:
			Error_Handler();
			break;

	}

	return (app_state);
}

void event_delay(void) {
	__HAL_TIM_DISABLE(&htim4);

	__HAL_TIM_SET_COUNTER(&htim4, 0);
	HAL_TIM_Base_Stop_IT(&htim4);
	HAL_TIM_Base_Stop(&htim4);

	HAL_TIM_OC_Stop_IT(&htim4, TIM_CHANNEL_3);
	for (int i = 0; i < NUM_TIMER_EVENTS; i++) {
		// Detener el temporizador y deshabilitar el contador
		if (eventTable[i].currentState == app_state) {
			if (eventTable[i].eventFlag != NULL) {
				*(eventTable[i].eventFlag) = true;
				__HAL_TIM_ENABLE(&htim4);
				__HAL_TIM_SET_COUNTER(&htim4, 0);

				HAL_TIM_OC_Stop_IT(&htim4, TIM_CHANNEL_3);

				return;

			}
			break;

		}


	}
	__HAL_TIM_SET_COUNTER(&htim4, 0);
	HAL_TIM_Base_Start_IT(&htim4);
	HAL_TIM_OC_Start_IT(&htim4, TIM_CHANNEL_3);
	__HAL_TIM_ENABLE(&htim4);


}
