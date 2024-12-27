/*
 * states.h
 *
 *  Created on: Dec 26, 2024
 *      Author: francisco
 */

#ifndef INC_STATES_H_
#define INC_STATES_H_

//definicion de las flags///////////
typedef unsigned short APP_CTRL_FLAGS;
#define FLAG_APP_RESET 0x00
#define FLAG_APP_INIT 0x01
#define FLAG_APP_INIT_DONE 0x02
#define FLAG_APP_FAULT 0x04
#define FLAG_APP_FAULT_CLEAR 0x08
#define FLAG_APP_STOP 0x10
#define FLAG_APP_STOP_ACK 0x20
#define FLAG_APP_RUN_ACK 0x40
#define FLAG_APP_RUN 0x80
//////////////////////////////////


// estructura de control de la aplicacion
typedef struct
{
	void (*init)(void);
	void (*run)(void);
	void (*stop)(void);
	void (*fault)(void);
}APP_STATE_FUNCTIONS;

typedef struct
{
 void (*FaultInit)(void);

 void (*InitFault)(void);
 void (*InitStop)(void);

 void (*StopRun)(void);
 void (*StopFault)(void);

 void (*RunStop)(void);
 void (*RunFault)(void);

}APP_TRANSITION_FUNCTIONS;


typedef enum{
	INIT = 0,
	RUN = 1,
	STOP = 2,
	FAULT = 3
}APP_STATE;

typedef struct
{
	APP_STATE_FUNCTIONS const* pState;  // puntero a la function de estado
	APP_TRANSITION_FUNCTIONS const* pTransition;
	APP_CTRL_FLAGS  ctrlFlag;
	APP_STATE currentState;
}APP_CTRL_STRUCT;
void mainStateInit(APP_CTRL_STRUCT *pAppCtrl);
void mainStateRun(APP_CTRL_STRUCT *pAppCtrl);
void mainStateStop(APP_CTRL_STRUCT *pAppCtrl);
void mainStateFault(APP_CTRL_STRUCT *pAppCtrl);

extern void (*STATE_TABLE[])(APP_CTRL_STRUCT *);

static inline void APP_StateMachine(APP_CTRL_STRUCT *pAppCtrl) {
	STATE_TABLE[pAppCtrl -> currentState] (pAppCtrl);
}


#endif /* INC_STATES_H_ */
