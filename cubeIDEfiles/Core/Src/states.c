/*
 * states.c
 *
 *  Created on: Dec 27, 2024
 *      Author: francisco
 */


#include "states.h"


void mainStateInit(APP_CTRL_STRUCT *pAppCtrl){
	pAppCtrl->state = INIT;
	pAppCtrl->flags = FLAG_APP_INIT;
}

void mainStateRun(APP_CTRL_STRUCT *pAppCtrl) {
	pAppCtrl->state = RUN;
	pAppCtrl->flags = FLAG_APP_RUN;
}

