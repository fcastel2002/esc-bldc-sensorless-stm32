/*
 * state_machine.h
 *
 *  Created on: Feb 3, 2025
 *      Author: Usuario
 */

#ifndef INC_STATE_MACHINE_H_
#define INC_STATE_MACHINE_H_

#include "comm.h"
#include "flash_config.h"
#include "hard_config.h"
#include "main.h"
#include "motor_control.h"
#include "startup.h"
#define NUM_TIMER_EVENTS (sizeof(eventTable) / sizeof(eventTable[0]))

// types
typedef enum {
  IDLE,
  STARTUP,
  FOC_STARTUP,
  CONFIG,
  RUNNING,
  READY,
  CLOSEDLOOP, // aligned
  STOPPED,
  HARD_ERROR,
  FINISH,
  SINE_DRIVE,
  STEP_TEST,
  FREEWHEEL

} App_States_t;

typedef struct {
  App_States_t currentState;
  bool*        eventFlag;
  uint32_t     delay;
} TimerDependentEvent;

// functions
App_States_t handleState(void);
void         event_delay(void);
// variables
extern volatile App_States_t app_state;
/** Incremented every time the state machine brakes the motor for an automatic restart. */
extern volatile uint16_t     motor_restart_count;

#endif /* INC_STATE_MACHINE_H_ */
