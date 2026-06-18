/*
 * comm.h
 *
 * Public communication facade used by the state machine.
 */

#ifndef INC_COMM_H_
#define INC_COMM_H_

#include "main.h"
#include <stdint.h>

#define BUFFER_SIZE 128
#define MAX_CMD_LEN 64
#define MAX_LOGGED_PARAMS 5

extern uint8_t rx_data[1];
extern volatile uint8_t rx_buffer[BUFFER_SIZE];
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
extern volatile uint16_t loggin_rate_ms;

typedef enum {
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
  PARAM_POLEP,
  PARAM_TEMP,
  PARAM_CURRENT,
} CommandParam;

typedef enum {
  VAR_TEMP,
  VAR_CURRENT,
  VAR_SPEED,
} LoggeableVariable;

typedef struct {
  LoggeableVariable params[MAX_LOGGED_PARAMS];
  uint8_t count;
} LoggingQueue;

typedef enum {
  ACTION_SET,
  ACTION_GET,
  ACTION_RESET,
  ACTION_UNKNOWN,
  ACTION_RUN,
  ACTION_STOP,
  ACTION_EMERGENCY,
  ACTION_LOGGING,
  ACTION_HELP,
} CommandAction;

void commInit(void);
void processUartData(void);
void processCurrentCommand(void);
void clearRxBuffer(void);
uint8_t process_speed_command(void);
void handleCommandEffects(void);
void transmitir_UART(const char* formato, ...);

void start_logging_param(LoggeableVariable variable);
void stop_logging_param(LoggeableVariable variable);
void stop_logging_all(void);
uint8_t set_logging_rate_ms(uint16_t rate_ms);
void process_logging_queue(void);

#endif /* INC_COMM_H_ */

