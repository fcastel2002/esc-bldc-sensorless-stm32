/*
 * app_state.c
 *
 *  Created on: Feb 3, 2025
 *      Author: Usuario
 */


#include "state_machine.h"

volatile App_States_t app_state = IDLE;




App_States_t handleState(void){

	static uint8_t comm_initialized = 0;
	static uint32_t last_uart_check = 0;
	static uint8_t flash_initialized = 0;
    uint32_t current_time = HAL_GetTick();

	if(!comm_initialized){
		commInit();
		comm_initialized = 1;
	}

	if(!flash_initialized){
		FlashResultCode result = flash_config_init();
		if(result == FLASH_RESULT_OK || result == FLASH_RESULT_EMPTY){
			update_all_esc();
			updateAllMotorControl();
			flash_initialized = 1;
		}else{
			app_state = HARD_ERROR;
			flash_initialized = 0;
		}
	}


	if(app_state != IDLE && app_state != HARD_ERROR) {
        processLoggingQueue();
    }
	
	if(cmd_received_ack && (app_state == IDLE || app_state == RUNNING || app_state == CLOSEDLOOP)){
		if(processSpeedCommand()){
			cmd_speed_received_ack = 0;
		}
		else{
			 processCurrentCommand();
			 handleCommandEffects();

		}
		cmd_received_ack = 0;
	}
	switch(app_state){
	case IDLE:
		if(current_time - last_uart_check > 50) {
                processUartData();
                last_uart_check = current_time;
            }
		break;


	case STARTUP:
		break;

	case FOC_STARTUP:
		break;
	case RUNNING:
		if(motor_stalled){
		foc_startup();
		motor_stalled = false;
		}	
		if (current_time - last_uart_check > 50) {
			last_uart_check = current_time;
			// Procesar datos UART
			processUartData();
		}
		if(consistent_zero_crossing)app_state = READY;
		break;

	case CONFIG:
		if(!esc_config_done && !motor_control_config_done)update_all_esc();
		if(esc_config_done  && !motor_control_config_done)updateAllMotorControl();
		if(esc_config_done && motor_control_config_done) app_state = IDLE;
		break;
	case CLOSEDLOOP:
		HAL_TIM_OC_Start_IT(&htim4, TIM_CHANNEL_1);
		current_time = HAL_GetTick();
		if(motor_stalled){
		foc_startup();
		motor_stalled = false;
		}
		if (current_time - last_uart_check > 50) {
			last_uart_check = current_time;

			// Procesar datos UART
			processUartData();
		}
		if( 0 == consistent_zero_crossing) app_state = RUNNING;
		break;
	case READY:
		TIM4->PSC = 2;
		TIM4->ARR = 0xFFFF; //
		__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1,48000); //output compare 1kHz
		app_state = CLOSEDLOOP;

		break;
	case STOPPED:
		HAL_TIM_OC_Stop_IT(&htim4, TIM_CHANNEL_1);
		
	break;

	case FINISH:
		// Asegúrate de que las interrupciones de UART estén habilitadas al finalizar
		__HAL_UART_ENABLE_IT(&huart1, UART_IT_RXNE);
		app_state = IDLE; // Vuelve al estado IDLE

		break;

	case HARD_ERROR:
			Error_Handler();
			break;

	}

	return (app_state);
}

