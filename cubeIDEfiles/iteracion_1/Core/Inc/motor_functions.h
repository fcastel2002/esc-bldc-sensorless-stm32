/*
 * motor_functions.h
 *
 *  Created on: Jan 3, 2025
 *      Author: Usuario
 */

#ifndef INC_MOTOR_FUNCTIONS_H_
#define INC_MOTOR_FUNCTIONS_H_

#define MOTOR_PHASE_A 0
#define MOTOR_PHASE_B 1
#define MOTOR_PHASE_C 2

#define PWM_ON 1
#define PWM_OFF 0
#define FLOATING 2

static void alignMotor(void);
static void setPhase(uint8_t phase, uint8_t state);
static void setPWM(uint8_t dutyCycle);

#endif /* INC_MOTOR_FUNCTIONS_H_ */
