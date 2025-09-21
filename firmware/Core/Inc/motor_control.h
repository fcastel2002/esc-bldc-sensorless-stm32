/*
 * basic_func.h
 *
 *  Created on: Jan 10, 2025
 *      Author: francisco
 */

#ifndef INC_MOTOR_CONTROL_H_
#define INC_MOTOR_CONTROL_H_
#include "hard_config.h"
#include "speed_sensor.h"
#include "bldc_driver.h"
#include "utilities.h"
#include <stdint.h>
#include <stdbool.h>




// RUNTIME VARIABLES
extern volatile bool     motor_stalled;

extern volatile uint16_t max_pwm;
extern volatile uint16_t speed_setpoint;
extern volatile uint16_t speed_setpoint_rpm;
extern volatile uint16_t speed_command;

extern volatile uint8_t  motor_control_config_done;
extern volatile uint8_t  consistent_zero_crossing;

extern volatile uint8_t  direction;

// RUNTIME FUNCTIONS
extern void updateAllMotorControl(void);
void        pi_control(void);
extern void detect_motor(void);
// void commutate(int8_t step);
extern void     zero_crossing_handler(uint8_t fase);
extern void     check_motor_status(void);
extern void     stop_motor(uint8_t mode);


// MACROS PARA MEDICION DE VELOCIDAD



#endif /* INC_MOTOR_CONTROL_H_ */
