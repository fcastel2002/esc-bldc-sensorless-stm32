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

typedef enum {
  CONTROL_RUNTIME_NORMAL = 0,
  CONTROL_RUNTIME_MONITOR_ONLY = 1,
  CONTROL_RUNTIME_HIL_SIM = 2,
} ControlRuntimeMode;

typedef struct {
  uint32_t accepted_run_id;
  uint32_t accepted_source_seq;
  uint32_t accepted_generation;
  uint32_t applied_run_id;
  uint32_t applied_source_seq;
  uint32_t output_generation;
  uint32_t pwm_update_tick;
} HilValidationProvenance;




// RUNTIME VARIABLES
extern volatile bool     motor_stalled;

extern volatile uint16_t max_pwm;
extern volatile uint16_t speed_setpoint;
extern volatile uint16_t speed_setpoint_rpm;
extern volatile uint16_t speed_command;

extern volatile uint8_t  motor_control_config_done;
extern volatile uint8_t  consistent_zero_crossing;

extern volatile uint8_t  direction;
extern volatile ControlRuntimeMode control_runtime_mode;

// RUNTIME FUNCTIONS
extern void updateAllMotorControl(void);
void        pi_control(void);
extern void detect_motor(void);
// void commutate(int8_t step);
extern void     zero_crossing_handler(uint8_t fase);
extern void     check_motor_status(void);
extern void     stop_motor(uint8_t mode);
extern uint16_t convert_speed_ticks(uint16_t value, bool to_ticks);
extern uint16_t period_to_pwm(uint16_t period);
extern uint8_t  control_mode_set(uint8_t mode);
extern uint8_t  hil_start(void);
extern void     hil_stop(void);
extern void     hil_set_inputs(uint16_t speed_rpm,
                               int16_t load_torque,
                               uint8_t flags,
                               uint32_t run_id,
                               uint32_t source_seq);
extern uint8_t  hil_is_active(void);
extern uint8_t  hil_has_timeout(void);
extern uint16_t hil_get_speed_rpm(void);
extern uint16_t hil_get_pwm_command(void);
extern uint8_t  hil_get_flags(void);
extern void     hil_get_validation_provenance(HilValidationProvenance* provenance);

// MACROS PARA MEDICION DE VELOCIDAD



#endif /* INC_MOTOR_CONTROL_H_ */
