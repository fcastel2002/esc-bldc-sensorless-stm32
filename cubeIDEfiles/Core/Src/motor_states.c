/*
 * motor_states.c
 *
 *  Created on: Jan 3, 2025
 *      Author: Usuario
 */
#include "motor_states.h"
#include "states.h"

static void M_FaultInit(void);
static void M_InitFault(void);
static void M_InitStop(void);
static void M_StopRun(void);
static void M_StopFault(void);
static void M_RunStop(void);
static void M_RunFault(void);



