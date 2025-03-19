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
	 static uint8_t dma_initialized = 0;

	    // Inicializar DMA si aún no se ha hecho
	    if (!dma_initialized) {
	        initUartDmaRx();
	        dma_initialized = 1;
	    }
	switch(app_state){
	case IDLE:
		checkDmaBuffer();
		if(cmd_received_ack){
			app_state = PROCESS_COMMAND;
		}
		break;

	case PROCESS_COMMAND:
	    // Procesar todos los comandos en la cola
	    if (cmd_received_ack) {
	        char cmd[MAX_CMD_LEN];

	        // Extraer y procesar un comando de la cola
	        if (dequeueCmd(&cmd_queue, cmd)) {
	            processCommand(cmd);

	            // Verificar resultados del procesamiento
	            if (!running_cmd_ack && !set_cmd_received_ack) {
	                // Si no hay más comandos en la cola, volver a IDLE
	                if (isQueueEmpty(&cmd_queue)) {
	                    cmd_received_ack = 0;
	                    app_state = IDLE;
	                }
	                // Si hay más comandos, continuar en PROCESS_COMMAND
	            } else if (set_cmd_received_ack) {
	                esc_config_done = 0;
	                motor_control_config_done = 0;
	                app_state = CONFIG;
	            }
	        } else {
	            // Si no hay comandos (raro, pero posible por condiciones de carrera)
	            cmd_received_ack = 0;
	            app_state = IDLE;
	        }
	    } else {
	        // Protección contra condiciones inesperadas
	        app_state = IDLE;
	    }
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
		if(!motor_stalled)checkDmaBuffer();

		if(consistent_zero_crossing){
			app_state = CLOSEDLOOP;
		}
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
	    static uint32_t last_check_time = 0;
	    uint32_t current_time = HAL_GetTick();

	    if (current_time - last_check_time > 50) {  // Verificar cada 50ms
	    	last_check_time = current_time;
	    	checkDmaBuffer();
	    	if (!isQueueEmpty(&cmd_queue)) {
	    		processSpeedCommand();
	    	}
	    }

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
