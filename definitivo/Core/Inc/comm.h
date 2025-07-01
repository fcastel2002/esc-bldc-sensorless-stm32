/*
 * comm.h
 *
 *  Created on: Feb 3, 2025
 *      Author: Usuario
 */

#ifndef INC_COMM_H_
#define INC_COMM_H_

#include "usart.h"


//defines

#define BUFFER_SIZE 128
#define MAX_CMD_LEN 64
#define MAX_LOGGED_PARAMS 5

//variables
extern uint8_t rx_data[1];
extern volatile char rx_buffer[BUFFER_SIZE];
extern volatile uint16_t rx_head;
extern volatile uint16_t rx_tail;
extern uint8_t rx_index;
extern volatile uint8_t cmd_received_ack;
extern volatile uint8_t cmd_speed_received_ack;
extern volatile uint8_t set_cmd_received_ack;
extern volatile uint8_t running_cmd_ack;
extern volatile uint8_t stop_cmd_ack;
extern volatile uint8_t emergency_cmd_ack;
extern volatile uint8_t logger_config_done;

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
	PARAM_KP,
	PARAM_KI,
	PARAM_KD,
	PARAM_MAXSPEED,
	PARAM_MINSPEED,
	PARAM_ALL,
	PARAM_SPEED,
	PARAM_UNKNOWN,
	PARAM_LOG_RATE,
	PARAM_START,
	PARAM_STOP,
	PARAM_RATE,

 }CommandParam;

typedef struct{
	CommandParam params[MAX_LOGGED_PARAMS]; // Array de parámetros a loggear
	uint8_t count;

 }LoggingQueue;



 typedef enum{
 	ACTION_SET,
 	ACTION_GET,
 	ACTION_RESET,
 	ACTION_UNKNOWN,
	ACTION_RUN,
	ACTION_STOP,
	ACTION_EMERGENCY,
	ACTION_LOGGING,
 }CommandAction;
 // functions

 void processCommand(char *cmd);
 extern void processCurrentCommand(void);
 void clearRxBuffer(void);

 void commInit(void);
 void processUartData(void);
 extern uint8_t processSpeedCommand(void);
extern void handleCommandEffects(void);
 extern uint16_t getActualSpeed(void);
 CommandParam parseParameter(char* param_str);
 CommandAction parseAction(char* action_str);
 void transmitirUART(const char *formato, ...);
 // const
 extern const Command commandTable[];
extern volatile uint16_t logging_rate_ms;
 
extern void parameterLogging(CommandParam *parameter, uint16_t rate_ms);
void startLoggingParam(CommandParam param);
void stopLoggingParam(CommandParam param);
extern void processLoggingQueue(void);

#endif /* INC_COMM_H_ */
