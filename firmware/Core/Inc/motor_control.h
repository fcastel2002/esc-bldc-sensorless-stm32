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
#include "rpm_pi_controller.h"
#include <stdint.h>
#include <stdbool.h>

#define MOTOR_CONTROL_DT_US 2000U
#define MOTOR_CONTROL_TIMER_PSC 2U
#define MOTOR_CONTROL_TIMER_ARR 47999U
#define MOTOR_CONTROL_TIMER_COMPARE 47999U

// RUNTIME VARIABLES
extern volatile bool     motor_stalled;

extern volatile uint16_t max_pwm;
extern volatile uint16_t speed_setpoint;
/** Commanded speed target in RPM; the PI follows the slew-limited reference. */
extern volatile uint16_t speed_setpoint_rpm;

extern volatile uint8_t  motor_control_config_done;
extern volatile uint8_t  consistent_zero_crossing;

extern volatile uint8_t  direction;

// RUNTIME FUNCTIONS
extern void updateAllMotorControl(void);
extern void motor_control_reset_runtime(void);
extern void motor_control_prepare_closed_loop(void);
void        pi_control(void);
extern void detect_motor(void);
extern void     zero_crossing_handler(uint8_t phase, uint16_t capture_timestamp);
extern void     motor_control_arm_zc_blanking(void);
extern void     check_motor_status(void);
extern void     stop_motor(uint8_t mode);
/* Start the TIM4 CC1 control-period interrupt without enabling the TIM4_CH1
 * output (PB6 is shared with the INA219 SCL line). Idempotent. */
void            motor_control_timer_start(void);

/** Slew-limited speed reference currently applied to the PI, RPM. */
uint16_t motor_control_get_reference_rpm(void);
/** True when the slew-limited reference equals the commanded target. */
bool     motor_control_reference_at_target(void);
/** Temporarily replace the configured slew rate (RPM/s); 0 restores the config value. */
void     motor_control_set_ramp_override(uint16_t rpm_per_s);
/** Commanded PWM duty in units of 1/10000 of the active TIM1 period. */
uint16_t motor_control_get_duty_e4(void);

// MACROS PARA MEDICION DE VELOCIDAD



#endif /* INC_MOTOR_CONTROL_H_ */
