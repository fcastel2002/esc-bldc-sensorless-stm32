/*
 * comm.h
 *
 *  Created on: Feb 3, 2025
 *      Author: Usuario
 */

#ifndef INC_COMM_H_
#define INC_COMM_H_

#include "main.h"

//defines

#define BUFFER_SIZE 128
#define MAX_CMD_QUEUE 5
#define MAX_CMD_LEN 64
//variables
extern uint8_t rx_data[1];
extern uint8_t rx_buffer[BUFFER_SIZE];
extern uint8_t rx_index;
extern volatile uint8_t dma_rx_last_pos;
extern volatile uint8_t rx_dma_buffer[BUFFER_SIZE];
extern volatile uint8_t cmd_received_ack;
extern volatile uint8_t set_cmd_received_ack;
extern volatile uint8_t running_cmd_ack;

 //types

 typedef struct {
     char *command;               // Texto del comando
     void (*execute)();
 } Command;

 typedef struct{
	 char cmds[MAX_CMD_QUEUE][MAX_CMD_LEN];
	 uint8_t cabeza;
	 uint8_t cola;
	 uint8_t contador;

 }CommandQueue;
 typedef enum {

	 CMD_OK		= 0x00,
	 CMD_UNKNOWN = 0x01,
	 CMD_CRC_ERR = 0x02,
	 CMD_OVF_ERR = 0x03,

 }CommandStatus;

 typedef enum{
 	PARAM_PWM_FREQ,
 	PARAM_CURRENT_LIMIT,
 	PARAM_TEMP_LIMIT,
	PARAM_UNKNOWN,
 }CommandParam;


 typedef enum{
 	ACTION_SET,
 	ACTION_GET,
 	ACTION_RESET,
 	ACTION_UNKNOWN,
	ACTION_RUN,
	ACTION_STOP,
 }CommandAction;

 extern CommandQueue cmd_queue;
 // functions

 void processCommand(char *cmd);
 void clearRxBuffer(void);

 extern void initCmdQueue(CommandQueue* queue);
extern  uint8_t isQueueFull(CommandQueue* queue);
 extern uint8_t isQueueEmpty(CommandQueue* queue);
 extern void enqueueCmd(CommandQueue* queue, const char* cmd);
extern  uint8_t dequeueCmd(CommandQueue* queue, char* cmd_out);
extern  void initUartDmaRx(void);
extern void checkDmaBuffer(void);
extern void resetUartRx(void);
extern void processDmaData(uint16_t start, uint16_t end);
extern void processNextCmd(void);
extern void initCommSystem(void);
extern uint8_t processSpeedCommand(void);
 CommandParam parseParameter(char* param_str);
 CommandAction parseAction(char* action_str);
 void transmitirUART(const char *formato, ...);
 // const
 extern const Command commandTable[];



#endif /* INC_COMM_H_ */
