/*
 * comm.c
 *
 *  Created on: Feb 3, 2025
 *      Author: Usuario
 */


#include  "comm.h"

#include "hard_config.h"


uint8_t rx_index = 0;
uint8_t rx_data[1];
uint8_t rx_buffer[BUFFER_SIZE];

volatile uint8_t dma_tx_busy = 0;
volatile uint8_t cmd_received_ack = 0;
volatile uint8_t set_cmd_received_ack = 0;
volatile uint8_t running_cmd_ack = 0;

ConfigStatus executeCommand(CommandAction action, CommandParam param, char* value);
const char* configStatusToString(ConfigStatus status);

void clearRxBuffer(void) {
    memset(rx_buffer, 0, BUFFER_SIZE);
    rx_index = 0;
}

void processCommand(char *cmd) {
	CommandStatus status = CMD_UNKNOWN;
	if(cmd[0] != ':'){
		transmitirUART("ERROR: Invalid format\r\n");
		return;
	}

	char action_str[16];
	char param_str[16];
	char value_str[16];

	int parsed = sscanf(cmd, ":%[^:]:%[^:]:%s", action_str, param_str, value_str);

	if(parsed < 2 && (strcmp(action_str,"RUN")!=0 && strcmp(action_str,"STP{")!=0)){
		transmitirUART("ERROR: Invalid command\r\n");
		app_state = IDLE;
		return;
	}
	CommandAction action = parseAction(action_str);
	if(action == ACTION_UNKNOWN){
		transmitirUART("ERROR: Unknown action\r\n");
		app_state = IDLE;

		return;
	}
	CommandParam param = parseParameter(param_str);
	if(param == PARAM_UNKNOWN && (action != ACTION_RUN && action != ACTION_STOP)){
		transmitirUART("ERROR: Unknown parameter\r\n");
		app_state = IDLE;

		return;
	}
	ConfigStatus result = executeCommand(action, param, parsed == 3? value_str : NULL);
	if(action == ACTION_SET) {
		set_cmd_received_ack = 1;

		transmitirUART(configStatusToString(result),NULL);
	}



}
const char *configStatusToString(ConfigStatus status){
	static const char* status_strings[] = {
			[CONFIG_OK] = "CONFIG_OK\r\n",
			[CONFIG_ERROR_OVERLIMIT] = "CONFIG_ERROR_OL",
			[CONFIG_ERROR_UNDERLIMIT] = "CONFIG_ERROR_UL",
			[CONFIG_ERROR_NaN] = "CONFIG_ERROR_NaN",
	};
	if(status >= CONFIG_OK && status <= CONFIG_ERROR_NaN){
		return status_strings[status];
	}
	return "CONFIG_UNKNOWN";
}
ConfigStatus executeCommand(CommandAction action, CommandParam param,  char* value){
	ConfigStatus result = CONFIG_OK;
	switch(param){
	case PARAM_PWM_FREQ:
		switch(action){
		case ACTION_SET:
				uint16_t new_freq = atoi(value);
				result = set_pwm_freq(new_freq);
			break;
		case ACTION_GET:
				uint16_t act_freq = get_pwm_freq();
				transmitirUART("PWM FREQ: %d Hz\r\n",act_freq);
			break;
		case ACTION_RESET:
			break;

		}
		break;
		case PARAM_CURRENT_LIMIT:
			switch(action){
			case ACTION_SET:
				break;
			case ACTION_GET:
				break;
			case ACTION_RESET:
				break;

			}
			break;
		case PARAM_TEMP_LIMIT:
			switch(action){
			case ACTION_SET:
				break;
			case ACTION_GET:
				break;
			case ACTION_RESET:
				break;

			}
			break;
			case PARAM_UNKNOWN:
				switch(action){
				case ACTION_RUN:
					running_cmd_ack =1;
					foc_startup();
					break;

				case ACTION_STOP:
					break;
				}
				break;
	}
	return result;
}

CommandParam parseParameter(char *param_str){
	if(strcmp(param_str, "PWM_FREQ") == 0) return PARAM_PWM_FREQ;
	if(strcmp(param_str, "CURRENT_LIMIT") == 0) return PARAM_CURRENT_LIMIT;
	if(strcmp(param_str, "TEMP_LIMIT") == 0) return PARAM_TEMP_LIMIT;
	return PARAM_UNKNOWN;

}
CommandAction parseAction(char* action_str){
	if(strcmp( action_str, "SET") == 0) return ACTION_SET;
	if(strcmp( action_str, "GET") == 0) return ACTION_GET;
	if(strcmp( action_str, "RESET") == 0) return ACTION_RESET;
	if(strcmp (action_str, "RUN") == 0) return ACTION_RUN;
	if(strcmp(action_str, "STOP") == 0) return ACTION_STOP;
	return ACTION_UNKNOWN;
}
void transmitirUART(const char* formato, ...){
	va_list args;
	va_start(args, formato);
	static char buffer[64];
	vsnprintf(buffer, sizeof(buffer), formato, args);


	while(dma_tx_busy);
	dma_tx_busy = 1;
	HAL_UART_Transmit_IT(&huart2, (uint8_t*)buffer, strlen(buffer));
	va_end(args);


}
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart){
	if(huart->Instance == USART2){
		dma_tx_busy = 0;
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART2) {
        if (rx_data[0] == '\r' || rx_data[0] == '\n') {
            if (rx_index > 0) { // Verificar que haya datos
                rx_buffer[rx_index] = '\0';
                cmd_received_ack = 1;
                app_state = PROCESS_COMMAND;
                rx_index = 0;
            }
        } else {
            if (rx_index < BUFFER_SIZE - 1) {
                rx_buffer[rx_index++] = rx_data[0];
            } else {
                // Manejo de overflow
                while(dma_tx_busy); // Esperar transmisión previa
                dma_tx_busy = 1;
                static const char overflow_msg[] = "ERROR: 0x00\r\n";
                HAL_UART_Transmit_IT(&huart2, (uint8_t*)overflow_msg, sizeof(overflow_msg)-1);
                clearRxBuffer();
            }
        }
        HAL_UART_Receive_IT(&huart2, rx_data, 1);
    }
}
