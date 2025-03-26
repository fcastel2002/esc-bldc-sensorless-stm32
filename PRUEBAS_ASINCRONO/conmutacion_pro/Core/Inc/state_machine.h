/*
 * state_machine.h
 *
 *  Created on: Feb 3, 2025
 *      Author: Usuario
 */

#ifndef INC_STATE_MACHINE_H_
#define INC_STATE_MACHINE_H_
#include "main.h"
#include "comm.h"
#include "hard_config.h"
#include "motor_control.h"
#include "startup.h"
#include "flash_config.h"
#define NUM_TIMER_EVENTS (sizeof(eventTable)/sizeof(eventTable[0]))


//types
 typedef enum {
	IDLE,
	PROCESS_COMMAND,
	STARTUP,
	FOC_STARTUP,
	CONFIG,
	RUNNING,
	ALIGNING,
	READY,
	CLOSEDLOOP,// aligned
	STOPPED,
	HARD_ERROR,
	FINISH

}App_States_t;

typedef enum{

	SENSORLESS_EVENT_NONE = 0,
	SENSORLESS_EVENT_ZC_U,
	SENSORLESS_EVENT_ZC_V,
	SENSORLESS_EVENT_ZC_W,
	SENSORLESS_EVENT_ERROR

}SENSORLESS_EVENT;

 typedef struct{
 	App_States_t currentState;
 	bool *eventFlag;
 	uint32_t delay;
 }TimerDependentEvent;

 // functions
 App_States_t handleState(void);
 void event_delay(void);
 //variables
extern volatile App_States_t app_state;
extern volatile bool eventFlag;
extern bool aligned_flag ;
extern bool startup_flag;
extern bool startup_ok ;
 //consts
 extern const TimerDependentEvent eventTable[];

#endif /* INC_STATE_MACHINE_H_ */
