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
volatile char rx_buffer[BUFFER_SIZE];
volatile uint16_t rx_head = 0;
volatile uint16_t rx_tail = 0;

volatile uint8_t tx_busy = 0;
volatile uint8_t cmd_received_ack = 0;
volatile uint8_t cmd_speed_received_ack = 0;
volatile uint8_t set_cmd_received_ack = 0;
volatile uint8_t running_cmd_ack = 0;
static char cmd_buffer[MAX_CMD_LEN];
static uint8_t cmd_index = 0;
ConfigStatus executeCommand(CommandAction action, CommandParam param, char* value);
const char* configStatusToString(ConfigStatus status);
void commInit(void){
	memset((void*)rx_buffer,0,BUFFER_SIZE);
	rx_head = 0;
	rx_tail = 0;
	cmd_index = 0;
	HAL_UART_Receive_IT(&huart1, (uint8_t*)&rx_buffer[rx_head],1);
}
void processCurrentCommand(void){
	processCommand(cmd_buffer);
}
void processUartData(void){
	if(rx_head == rx_tail)return;
	while(rx_tail != rx_head){
		char byte = rx_buffer[rx_tail];
		rx_tail = (rx_tail + 1) % BUFFER_SIZE;
		if(byte == '\r' || byte == '\n'){
			if(cmd_index > 0){
				cmd_buffer[cmd_index] = '\0';
				cmd_received_ack = 1;
				cmd_index = 0;
				app_state = PROCESS_COMMAND;
				break;
			}
		}else if(cmd_index < MAX_CMD_LEN - 1){
			cmd_buffer[cmd_index++] = byte;
		}else{
			cmd_index  = 0;
			transmitirUART("ERROR: Command too long\r\n");
		}
	}
}
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

	// Permitir comandos con solo action (parsed == 1) para RUN, STOP, EMERGENCY_STOP
	if(parsed < 1 || (parsed < 2 && (strcmp(action_str,"RUN")!=0 && strcmp(action_str,"STOP")!=0 && strcmp(action_str,"ESTOP")!=0))){
		transmitirUART("ERROR: Invalid command\r\n");
		if(app_state != RUNNING && app_state != CLOSEDLOOP)app_state = IDLE;
		return;
	}
	CommandAction action = parseAction(action_str);
	if(action == ACTION_UNKNOWN){
		transmitirUART("ERROR: Unknown action\r\n");
		if(app_state != RUNNING && app_state != CLOSEDLOOP)app_state = IDLE;
		return;
	}
	
	// Solo parsear parámetro si hay más de 1 parte en el comando
	CommandParam param = PARAM_UNKNOWN;
	if(parsed >= 2) {
		param = parseParameter(param_str);
	}
	
	if(param == PARAM_UNKNOWN && (action != ACTION_RUN && action != ACTION_STOP && action != ACTION_EMERGENCY)){
		transmitirUART("ERROR: Unknown parameter\r\n");
		app_state = IDLE;
		return;
	}
	if(app_state == CLOSEDLOOP && (action != ACTION_STOP || action!= ACTION_EMERGENCY)){
		transmitirUART("ERROR: Cannot change parameters while in CLOSEDLOOP\r\n");
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
	if(status <= CONFIG_ERROR_NaN){
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
		{
				uint16_t new_freq = atoi(value);
				result = set_pwm_freq(new_freq);
			break;
		}
		case ACTION_GET:
		{
				uint16_t act_freq = get_pwm_freq();
				transmitirUART("PWM FREQ: %d Hz\r\n",act_freq);
			break;
		}
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
				stop_motor(0);
				break;
			case ACTION_EMERGENCY:
				stop_motor(1);
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
	if(strcmp(action_str, "STOP") == 0) return ACTION_EMERGENCY;
	return ACTION_UNKNOWN;
}
void transmitirUART(const char* formato, ...) {
    va_list args;
    va_start(args, formato);
    static char buffer[64];
    vsnprintf(buffer, sizeof(buffer), formato, args);

    // Esperar a que termine transmisión anterior
    uint32_t timeout = HAL_GetTick() + 100;  // 100ms timeout
    while (tx_busy && HAL_GetTick() < timeout);

    tx_busy = 1;
    HAL_UART_Transmit_IT(&huart1, (uint8_t*)buffer, strlen(buffer));
    va_end(args);
}
// Procesar comando de velocidad
uint8_t processSpeedCommand(void) {
    uint8_t processed = 0;
    // Verificar si el comando actual es de velocidad
    if (strncmp(cmd_buffer, ":SPEED:", 7) == 0) {
        cmd_speed_received_ack = 1;

    	int speed_value;
        if (sscanf(cmd_buffer, ":SPEED:%d", &speed_value) == 1) {
            // Validar y aplicar nueva velocidad
            if (speed_value >= 0) {
            	if(speed_value > 50)speed_setpoint = speed_value;
            	else speed_setpoint = 0;
                processed = 1;
                transmitirUART("SPEED set to %d\r\n", speed_value);
            }
        }
    }

    return processed;
}
uint16_t getActualSpeed(void){

	return (uint16_t)diff_speed;
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart){
	if(huart->Instance == USART1){
		tx_busy = 0;
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART1) {
    	rx_head = (rx_head + 1) % BUFFER_SIZE;

    	HAL_UART_Receive_IT(&huart1, (uint8_t*)&rx_buffer[rx_head], 1);    }
}
