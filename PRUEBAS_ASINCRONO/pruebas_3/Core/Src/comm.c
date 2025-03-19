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
volatile uint8_t dma_rx_last_pos = 0;
volatile uint8_t rx_dma_buffer[BUFFER_SIZE];
CommandQueue cmd_queue;

volatile uint8_t dma_tx_busy = 0;
volatile uint8_t cmd_received_ack = 0;
volatile uint8_t set_cmd_received_ack = 0;
volatile uint8_t running_cmd_ack = 0;
extern volatile uint16_t speed_setpoint;
ConfigStatus executeCommand(CommandAction action, CommandParam param, char* value);
const char* configStatusToString(ConfigStatus status);

void initCmdQueue(CommandQueue* queue){
	queue->cabeza = 0;
	queue->cola = 0;
	queue->contador = 0;

}
uint8_t isQueueFull(CommandQueue* queue){
	return queue->contador >= MAX_CMD_QUEUE;
}
uint8_t isQueueEmpty(CommandQueue* queue){
	return queue->contador == 0;

}
void enqueueCmd(CommandQueue* queue, const char* cmd){
	if(!isQueueFull(queue)){
		strncpy(queue->cmds[queue->cola],cmd, MAX_CMD_LEN - 1);
		queue->cmds[queue->cola][MAX_CMD_LEN - 1] = '\0';
		queue->cola = (queue->cola + 1) % MAX_CMD_QUEUE;
		queue->contador++;
	}
}
uint8_t dequeueCmd(CommandQueue* queue, char* cmd_out){
	if(isQueueEmpty(queue)){
		return 0;
	}
	strncpy(cmd_out, queue ->cmds[queue->cabeza], MAX_CMD_LEN);
	queue->cabeza = (queue->cabeza + 1) % MAX_CMD_QUEUE;
	queue->contador++;
	return 1;
}

void initUartDmaRx(void){
	initCmdQueue(&cmd_queue);
	memset((void*)rx_dma_buffer, 0, BUFFER_SIZE);
	HAL_UART_DMAStop(&huart2);
	HAL_UART_Receive_DMA(&huart2, (uint8_t*)rx_dma_buffer, BUFFER_SIZE);
	dma_rx_last_pos = 0;
}
void checkDmaBuffer(void){
	// Guardar y deshabilitar interrupciones brevemente
    uint32_t primask = __get_PRIMASK();
    __disable_irq();

    // Capturar posición actual del DMA (operación rápida)
    uint16_t current_pos = BUFFER_SIZE - __HAL_DMA_GET_COUNTER(huart2.hdmarx);

    // Restaurar interrupciones inmediatamente
    if (!primask) __enable_irq();

    // Procesar datos si hay cambios (con interrupciones habilitadas)
    if (current_pos != dma_rx_last_pos) {
        processDmaData(dma_rx_last_pos, current_pos);
        dma_rx_last_pos = current_pos;
    }
}

void processDmaData(uint16_t start, uint16_t end){
	static uint8_t cmd_in_progress  = 0;
	static uint16_t cmd_start_pos = 0;
	static char temp_cmd [MAX_CMD_LEN];
	static uint16_t temp_cmd_len = 0;
	for(uint16_t i = start; i < end; i++){
		uint8_t current_byte = rx_dma_buffer[i];
		if (current_byte == ':' && !cmd_in_progress){
			cmd_in_progress = 1;
			cmd_start_pos = i;
			temp_cmd_len = 0;
			temp_cmd[temp_cmd_len++] = current_byte;
		}
		//dentro de un comando
		else if(cmd_in_progress){
			//añade si hay espacio
			if(temp_cmd_len < MAX_CMD_LEN - 1){
				temp_cmd[temp_cmd_len++] = current_byte;
			}
			if(current_byte == '\r' || current_byte == '\n'){
				temp_cmd[temp_cmd_len] = '\0';
				if(temp_cmd_len > 1 && temp_cmd[0] == ':'){
					enqueueCmd(&cmd_queue, temp_cmd);
					cmd_received_ack =1;

				}
				cmd_in_progress = 0;
				temp_cmd_len = 0;
			}
		}
	}
}

void processNextCmd(void){
	char cmd[MAX_CMD_LEN];
	if(dequeueCmd(&cmd_queue, cmd)){
		processCommand(cmd);
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
// Añadir a comm.c
uint8_t processSpeedCommand(void) {
    char cmd[MAX_CMD_LEN];
    uint8_t processed = 0;

    // Buscar en la cola un comando de velocidad
    for (uint8_t i = 0; i < cmd_queue.contador; i++) {
        uint8_t idx = (cmd_queue.cabeza + i) % MAX_CMD_QUEUE;

        // Si encontramos un comando de velocidad
        if (strstr(cmd_queue.cmds[idx], ":SPEED:") == cmd_queue.cmds[idx]) {
            // Copiar comando
            strncpy(cmd, cmd_queue.cmds[idx], MAX_CMD_LEN);

            // Eliminarlo de la cola (reorganizar)
            for (uint8_t j = i; j < cmd_queue.contador - 1; j++) {
                uint8_t src_idx = (cmd_queue.cabeza + j + 1) % MAX_CMD_QUEUE;
                uint8_t dst_idx = (cmd_queue.cabeza + j) % MAX_CMD_QUEUE;
                strncpy(cmd_queue.cmds[dst_idx], cmd_queue.cmds[src_idx], MAX_CMD_LEN);
            }

            cmd_queue.contador--;

            // Procesar comando de velocidad
            // Ejemplo: extraer valor y actualizar speed_setpoint
            int speed_value;
            if (sscanf(cmd, ":SPEED:%d", &speed_value) == 1) {
                // Validar y aplicar nueva velocidad
                if (speed_value >= 0) {
                    speed_setpoint = speed_value;
                    processed = 1;
                }
            }

            break;
        }
    }

    return processed;
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
void resetUartRx(void) {
    // Reiniciar el DMA si es necesario
    HAL_UART_DMAStop(&huart2);

    // Limpiar buffer y variables
    memset((void*)rx_dma_buffer, 0, BUFFER_SIZE);
    dma_rx_last_pos = 0;

    // Reiniciar la recepción
    HAL_UART_Receive_DMA(&huart2, (uint8_t*)rx_dma_buffer, BUFFER_SIZE);
}
void initCommSystem(void) {
    // Inicializar variables
    dma_tx_busy = 0;
    cmd_received_ack = 0;
    set_cmd_received_ack = 0;
    running_cmd_ack = 0;

    // Inicializar DMA para recepción
    initUartDmaRx();
}
void transmitirUART(const char* formato, ...) {
    va_list args;
    va_start(args, formato);
    static char buffer[64];
    vsnprintf(buffer, sizeof(buffer), formato, args);

    // Esperar a que la transmisión anterior termine
    uint32_t timeout = HAL_GetTick() + 100;  // 100ms timeout
    while (dma_tx_busy && HAL_GetTick() < timeout);

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
