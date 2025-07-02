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
volatile uint8_t stop_cmd_ack = 0;
volatile uint8_t running_cmd_ack = 0;
volatile uint8_t emergency_cmd_ack = 0;
static char cmd_buffer[MAX_CMD_LEN];
static uint8_t cmd_index = 0;
ConfigStatus executeCommand(CommandAction action, CommandParam param, char* value);
volatile App_States_t cmd_origin_state = IDLE;
const char* configStatusToString(ConfigStatus status);
volatile uint16_t loggin_rate_ms = 1000;
static LoggingQueue logging_queue = { .count = 0 };
static const char* paramToString(CommandParam param);
static TxQueue tx_queue = { .head = 0, .tail = 0, .count = 0, .busy = 0 };

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
				cmd_origin_state = app_state;
				cmd_received_ack = 1;
				cmd_index = 0;
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
		transmitirUART("ERROR: Parametro desconocido\r\n");
		app_state = IDLE;
		return;
	}
	if(cmd_origin_state == CLOSEDLOOP && action != ACTION_STOP && action != ACTION_EMERGENCY && action != ACTION_LOGGING){
		transmitirUART("ERROR: No se puede ejecutar este comando en CLOSEDLOOP\r\n");
		return;
	}
	ConfigStatus result = executeCommand(action, param, parsed == 3? value_str : NULL);
	if(action == ACTION_SET) {
		set_cmd_received_ack = 1;

		transmitirUART(configStatusToString(result),NULL);
	}



}
void handleCommandEffects(void) {
    if(set_cmd_received_ack) {
        // Transición a estado CONFIG
        esc_config_done = 0;
        motor_control_config_done = 0;
        set_cmd_received_ack = 0;
        app_state = CONFIG;
    }
    else if(running_cmd_ack) {
        // Transición a estado RUNNING
        running_cmd_ack = 0;
        foc_startup();
        
    }
    else if(stop_cmd_ack) {
        // Transición a estado IDLE
        stop_cmd_ack = 0;
        stopMotor(0);
        app_state = IDLE;
    }
    else if(emergency_cmd_ack) {
        // Transición de emergencia a IDLE
        emergency_cmd_ack = 0;
        stopMotor(1);
        app_state = IDLE;
    }

}
const char *configStatusToString(ConfigStatus status){
	static const char* status_strings[] = {
			[CONFIG_OK] = "CONFIG_OK\r\n",
			[CONFIG_ERROR_OVERLIMIT] = "CONFIG_ERROR_OL\r\n", //over limit value
			[CONFIG_ERROR_UNDERLIMIT] = "CONFIG_ERROR_UL\r\n", // under limit value
			[CONFIG_ERROR_NaN] = "CONFIG_ERROR_NaN\r\n", // not a number value
	};
	if(status <= CONFIG_ERROR_NaN){
		return status_strings[status];
	}
	return "CONFIG_UNKNOWN\r\n";
}
ConfigStatus executeCommand(CommandAction action, CommandParam param,  char* value){
	ConfigStatus result = CONFIG_OK;
	if(action != ACTION_LOGGING){
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
					stopMotor(0);
					break;
				case ACTION_EMERGENCY:
					stopMotor(1);
					break;
				}
				break;
			case PARAM_POLEP:
				switch(action){
				case ACTION_SET:
					if(value != NULL) {
						uint8_t new_pole_pairs = atoi(value);
						result = set_pole_pairs(new_pole_pairs);
					} else {
						transmitirUART("ERROR: No pole pairs specified\r\n");
					}
					break;
				case ACTION_GET:
					transmitirUART("POLE PAIRS: %d\r\n", current_esc_params.pole_pairs);
					break;
				case ACTION_RESET:
					break;
				}
				break;

		}
	}
	switch(action){
		case ACTION_LOGGING:
			switch(param){
				case PARAM_START:
					if(value == NULL){
						transmitirUART("ERROR: No parameter specified for logging\r\n");
					}else{
						CommandParam log_param = parseParameter(value);
						if(log_param != PARAM_UNKNOWN){
							startLoggingParam(log_param);
						}else{
							transmitirUART("ERROR: Invalid parameter for logging\r\n");
						}
					}
					break;
				case PARAM_STOP:
					if(value == NULL){
						transmitirUART("ERROR: No parameter specified for stopping logging\r\n");	
					}else{
						CommandParam log_param = parseParameter(value);
						if(log_param != PARAM_UNKNOWN){
							stopLoggingParam(log_param);	
						}else{
							log_param = PARAM_ALL;
							stopLoggingParam(log_param);
						}
					}
					break;
    			case PARAM_RATE:
					if(value != NULL) {
						uint16_t new_rate = atoi(value);
						if(new_rate >= 100 && new_rate <= 5000) {
							loggin_rate_ms = new_rate;
							transmitirUART("Logging rate set to %d ms\r\n", new_rate);
						}
					}
                	break;
				default:
					transmitirUART("ERROR: Invalid logging command\r\n");
					break;
			}
			break;
	}

	return result;
}

void processLoggingQueue(void) {
    static uint32_t last_log_tick = 0;
    uint32_t now = HAL_GetTick();
    
    if (now - last_log_tick < loggin_rate_ms) {
        return;
    }
    last_log_tick = now;
    
    for (int i = 0; i < logging_queue.count; i++) {
        CommandParam param = logging_queue.params[i];
        switch(param) {
            case PARAM_PWM_FREQ: 
                uint16_t freq = get_pwm_freq();
                transmitirUART("[LOG] PWM_FREQ: %d Hz\r\n", freq);
                break;
            
            case PARAM_SPEED: 
                uint16_t speed = getActualSpeed();
				speed = periodToRpm(speed); // Convertir a RPM
                transmitirUART("[LOG] SPEED: %d RPM\r\n", speed);
                break;
            
            case PARAM_CURRENT_LIMIT: 
                // Obtener valor actual del límite de corriente

                break;
            case PARAM_ALL:
				// Loggear todos los parámetros
				uint16_t all_freq = get_pwm_freq();
				uint16_t all_speed = getActualSpeed();
				transmitirUART("[LOG] ALL - PWM_FREQ: %d Hz, SPEED: %d RPM\r\n", all_freq, all_speed);
				break;
            case PARAM_TEMP_LIMIT: 

                break;
            
            default:
                break;
        }
    }
}
static const char* paramToString(CommandParam param){
	switch(param){
		case PARAM_PWM_FREQ: return "PWM_FREQ";
		case PARAM_CURRENT_LIMIT: return "CURRENT_LIMIT";
		case PARAM_TEMP_LIMIT: return "TEMP_LIMIT";
		case PARAM_ALL: return "ALL";
		case PARAM_SPEED: return "SPEED";
		case PARAM_START: return "START";
		case PARAM_STOP: return "STOP";
		case PARAM_RATE: return "RATE";
		default: return "UNKNOWN";
	}
}
void startLoggingParam(CommandParam param){

	if (logging_queue.count >= MAX_LOGGED_PARAMS) {
		transmitirUART("ERROR: Logging queue full\r\n");
		return;
	}
	for(int i = 0; i < logging_queue.count; i++){
		if(logging_queue.params[i] == param){
			return;
		}
	}
	logging_queue.params[logging_queue.count++] = param;
	transmitirUART("Logging started for parameter: %s\r\n", paramToString(param));
}
void stopLoggingParam(CommandParam param) {
    for (int i = 0; i < logging_queue.count; i++) {
        if (logging_queue.params[i] == param) {
            // Mover los elementos restantes
            for (int j = i; j < logging_queue.count - 1; j++) {
                logging_queue.params[j] = logging_queue.params[j+1];
            }
            logging_queue.count--;
            transmitirUART("Logging stopped for param: %s\r\n", paramToString(param));
            return;
        }
    }
	transmitirUART("ERROR: Parameter not found in logging queue\r\n");
}
CommandParam parseParameter(char *param_str){
	if(strcmp(param_str, "PWM_FREQ") == 0) return PARAM_PWM_FREQ;
	if(strcmp(param_str, "POLEP") == 0) return PARAM_POLEP;
	if(strcmp(param_str, "CURRENT_LIMIT") == 0) return PARAM_CURRENT_LIMIT;
	if(strcmp(param_str, "TEMP_LIMIT") == 0) return PARAM_TEMP_LIMIT;
	if(strcmp(param_str, "ALL") == 0) return PARAM_ALL;
	if(strcmp(param_str, "SPEED") == 0) return PARAM_SPEED;
	if(strcmp(param_str, "START") == 0) return PARAM_START;
    if(strcmp(param_str, "STOP") == 0) return PARAM_STOP;
    if(strcmp(param_str, "RATE") == 0) return PARAM_RATE;
	
	return PARAM_UNKNOWN;

}
CommandAction parseAction(char* action_str){
	if(strcmp( action_str, "SET") == 0) return ACTION_SET;
	if(strcmp( action_str, "GET") == 0) return ACTION_GET;
	if(strcmp( action_str, "RESET") == 0) return ACTION_RESET;
	if(strcmp (action_str, "RUN") == 0) return ACTION_RUN;
	if(strcmp(action_str, "STOP") == 0) return ACTION_STOP;
	if(strcmp(action_str, "ESTOP") == 0) return ACTION_EMERGENCY;
	if(strcmp(action_str, "LOG") == 0) return ACTION_LOGGING;
	return ACTION_UNKNOWN;
}
void transmitirUART(const char* formato, ...) {
        __disable_irq();
    
    // Verificar si hay espacio en la cola
    if (tx_queue.count >= TX_QUEUE_SIZE) {
        __enable_irq();
        return; // Cola llena - descartar mensaje
    }   
	
	va_list args;
    va_start(args, formato);
    char* target = tx_queue.buffers[tx_queue.head];
    int len = vsnprintf(target, TX_BUFFER_SIZE, formato, args);
    va_end(args);


	if (len >= TX_BUFFER_SIZE) {
        strcpy(&target[TX_BUFFER_SIZE - 6], "...\r\n"); // Indicar truncamiento
    }
	    // Actualizar cola
    tx_queue.head = (tx_queue.head + 1) % TX_QUEUE_SIZE;
    tx_queue.count++;
    
    // Iniciar transmisión si no está activa
    if (!tx_queue.busy) {
        tx_queue.busy = 1;
        char* msg = tx_queue.buffers[tx_queue.tail];
        HAL_UART_Transmit_IT(&huart1, (uint8_t*)msg, strlen(msg));
    }
    
    __enable_irq();
}
// Procesar comando de velocidad
uint8_t processSpeedCommand(void) {
    uint8_t processed = 0;
    // Verificar si el comando actual es de velocidad
    if (strncmp(cmd_buffer, ":SPEED:", 7) == 0) {
        cmd_speed_received_ack = 1;

    	uint16_t speed_value;
        if (sscanf(cmd_buffer, ":SPEED:%d", &speed_value) == 1) {
            // Validar y aplicar nueva velocidad
            if (speed_value >= 0) {
            	if(speed_value > 100){
					speed_setpoint_rpm = speed_value;
					//speed_setpoint_rpm = periodToPwm(speed_setpoint);
				}else{
					speed_setpoint_rpm = 100;
					//speed_setpoint_rpm =periodToPwm(speed_setpoint);
				}
				processed = 1;
                transmitirUART("SPEED set to %d\r\n", speed_value);
            }
        }
    }

    return processed;
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart){
	if(huart->Instance == USART1){
		__disable_irq();
        
        // Eliminar mensaje completado
        tx_queue.tail = (tx_queue.tail + 1) % TX_QUEUE_SIZE;
        tx_queue.count--;
        
        // Transmitir siguiente si hay mensajes
        if (tx_queue.count > 0) {
            char* next_msg = tx_queue.buffers[tx_queue.tail];
            HAL_UART_Transmit_IT(&huart1, (uint8_t*)next_msg, strlen(next_msg));
        } else {
            tx_queue.busy = 0;
        }
        
        __enable_irq();
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART1) {
    	rx_head = (rx_head + 1) % BUFFER_SIZE;

    	HAL_UART_Receive_IT(&huart1, (uint8_t*)&rx_buffer[rx_head], 1);    }
}
