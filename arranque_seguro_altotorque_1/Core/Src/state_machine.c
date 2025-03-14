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
	switch(app_state){
	case IDLE:
		if(!__HAL_UART_GET_IT_SOURCE(&huart2, UART_IT_RXNE)){
			__HAL_UART_ENABLE_IT(&huart2, UART_IT_RXNE);
			HAL_UART_Receive_IT(&huart2, rx_data, 1);
		}
		break;

	case PROCESS_COMMAND:
		__HAL_UART_DISABLE_IT(&huart2, UART_IT_RXNE);

		if(cmd_received_ack ){
			processCommand((char*) rx_buffer);
			if(!running_cmd_ack && !set_cmd_received_ack)app_state = IDLE;
			if(set_cmd_received_ack){
				esc_config_done = 0;
				motor_control_config_done = 0;
				app_state = CONFIG;

			}
		}
		cmd_received_ack = 0;
		break;

	case STARTUP:
		__HAL_UART_DISABLE_IT(&huart2, UART_IT_RXNE);
		break;
	case ALIGNING:
		__HAL_UART_DISABLE_IT(&huart2, UART_IT_RXNE);
		break;
	case FOC_STARTUP:
		__HAL_UART_DISABLE_IT(&huart2, UART_IT_RXNE);
		break;
	case RUNNING:
		__HAL_UART_DISABLE_IT(&huart2, UART_IT_RXNE);
		break;
	case CONFIG:
		if(!esc_config_done && !motor_control_config_done)update_all_esc();
		if(esc_config_done  && !motor_control_config_done)update_all_motor_control();
		if(esc_config_done && motor_control_config_done) app_state = IDLE;
		break;
	case CLOSEDLOOP:
		__HAL_UART_DISABLE_IT(&huart2, UART_IT_RXNE);

		HAL_TIM_OC_Start_IT(&htim4, TIM_CHANNEL_1);
		__HAL_TIM_SET_COUNTER(&htim4, 0);
		//closedLoop();
		break;
	case READY:
		TIM4->PSC = 2;
		TIM4->ARR = 0xFFFF; //
		__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 48000); //output compare 1kHz

		__HAL_UART_DISABLE_IT(&huart2, UART_IT_RXNE);

		break;
	case STOPPED:
		__HAL_UART_DISABLE_IT(&huart2, UART_IT_RXNE);
		break;

	case FINISH:
		// Asegúrate de que las interrupciones de UART estén habilitadas al finalizar
		__HAL_UART_ENABLE_IT(&huart2, UART_IT_RXNE);
		app_state = IDLE; // Vuelve al estado IDLE

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
