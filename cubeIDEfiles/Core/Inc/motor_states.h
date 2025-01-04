/*
 * motor_states.h
 *
 *  Created on: Dec 27, 2024
 *      Author: francisco
 */

#ifndef INC_MOTOR_STATES_H_
#define INC_MOTOR_STATES_H_

static void M_StateInit(void);
static void M_StateRun(void);
static void M_StateStop(void);
static void M_StateFault(void);


// funciones de transicion



static const MOTOR_STATE_FUNCTIONS motorStates = { M_StateInit, M_StateRun,
		M_StateStop, M_StateFault };
static const MOTOR_TRANSITION_FUNCTIONS motorTransitions = { M_FaultInit,
		M_InitFault, M_InitStop, M_StopRun, M_StopFault, M_RunStop, M_RunFault };
// subestados del estado RUN

typedef enum{
	M_READY = 0,
	M_ALIGN = 1,
	M_STARTUP = 2,
	M_SPIN = 3,
	M_FREEWHEEL = 4
}M_RUN_SUBSTATE;


#endif /* INC_MOTOR_STATES_H_ */
