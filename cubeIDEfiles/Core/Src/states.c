/*
 * states.c
 *
 *  Created on: Dec 27, 2024
 *      Author: francisco
 */


#include "states.h"


void mainStateInit(APP_CTRL_STRUCT *pAppCtrl){
	pAppCtrl->currentState = INIT;
	pAppCtrl->ctrlFlag = FLAG_APP_INIT;
}

void mainStateRun(APP_CTRL_STRUCT *pAppCtrl) {
	pAppCtrl->currentState = RUN;
	pAppCtrl->ctrlFlag = FLAG_APP_RUN;
}

void mainStateStop(APP_CTRL_STRUCT *pAppCtrl) {
	pAppCtrl->currentState = STOP;
	pAppCtrl->ctrlFlag = FLAG_APP_STOP;
}

void mainStateFault(APP_CTRL_STRUCT *pAppCtrl) {
	pAppCtrl->currentState = FAULT;
	pAppCtrl->ctrlFlag = FLAG_APP_FAULT;
}

void (*STATE_TABLE[])(APP_CTRL_STRUCT *) = {
	mainStateInit,
	mainStateRun,
	mainStateStop,
	mainStateFault
};
