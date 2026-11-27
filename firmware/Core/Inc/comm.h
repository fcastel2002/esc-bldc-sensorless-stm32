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
#define MAX_LOGGED_PARAMS 4

extern volatile uint8_t rx_buffer[BUFFER_SIZE];
extern volatile uint16_t rx_head;
extern volatile uint16_t rx_tail;

extern volatile uint8_t cmd_received_ack;
extern volatile uint8_t cmd_speed_received_ack;
extern volatile uint16_t loggin_rate_ms;

typedef enum {
  VAR_SPEED,
  VAR_BUS_VOLTAGE,
  VAR_BUS_CURRENT,
  VAR_DUTY,
} LoggeableVariable;

typedef struct {
  LoggeableVariable params[MAX_LOGGED_PARAMS];
  uint8_t count;
} LoggingQueue;

void commInit(void);
void processUartData(void);
void processCurrentCommand(void);
uint8_t process_speed_command(void);
void handleCommandEffects(void);

void start_logging_param(LoggeableVariable variable);
void stop_logging_param(LoggeableVariable variable);
void stop_logging_all(void);
uint8_t set_logging_rate_ms(uint16_t rate_ms);
void process_logging_queue(void);

#endif /* INC_COMM_H_ */
