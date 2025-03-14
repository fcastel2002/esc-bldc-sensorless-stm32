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

#define BUFFER_SIZE 32

//variables
extern uint8_t rx_data[1];
extern uint8_t rx_buffer[BUFFER_SIZE];
extern uint8_t rx_index;
extern volatile uint8_t cmd_received_ack;
extern volatile uint8_t set_cmd_received_ack;
extern volatile uint8_t running_cmd_ack;

 //types

 typedef struct {
     char *command;               // Texto del comando
     void (*execute)();
 } Command;

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
 // functions

 void processCommand(char *cmd);
 void clearRxBuffer(void);

 CommandParam parseParameter(char* param_str);
 CommandAction parseAction(char* action_str);
 void transmitirUART(const char *formato, ...);
 // const
 extern const Command commandTable[];



#endif /* INC_COMM_H_ */
