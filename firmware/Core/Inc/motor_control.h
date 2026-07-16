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

#define MOTOR_CONTROL_ALGORITHM_VERSION 2U
#define MOTOR_CONTROL_DT_US 2000U
#define MOTOR_CONTROL_DT_SECONDS RPM_PI_DT_SECONDS
#define MOTOR_CONTROL_MIN_PWM_DIVISOR 20U
#define MOTOR_CONTROL_SCALE 1
#define MOTOR_CONTROL_TIMER_PSC 2U
#define MOTOR_CONTROL_TIMER_ARR 47999U
#define MOTOR_CONTROL_TIMER_COMPARE 47999U
#define HIL_MAX_STEP_COUNT 1000U

typedef enum {
  CONTROL_RUNTIME_NORMAL = 0,
  CONTROL_RUNTIME_MONITOR_ONLY = 1,
  CONTROL_RUNTIME_HIL_SIM = 2,
} ControlRuntimeMode;

typedef enum {
  HIL_EXECUTION_PERIODIC = 0,
  HIL_EXECUTION_STEPPED = 1,
} HilExecutionMode;

typedef enum {
  HIL_STEP_INVALID = 0,
  HIL_STEP_APPLIED = 1,
  HIL_STEP_REPLAYED = 2,
} HilStepResult;

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
extern volatile uint8_t  hil_session_state;
extern volatile uint16_t hil_speed_rpm;
extern volatile uint8_t  hil_flags;

// RUNTIME FUNCTIONS
extern void updateAllMotorControl(void);
extern void motor_control_reset_runtime(void);
extern void motor_control_prepare_closed_loop(void);
void        pi_control(void);
extern void detect_motor(void);
// void commutate(int8_t step);
extern void     zero_crossing_handler(uint8_t fase);
extern void     check_motor_status(void);
extern void     stop_motor(uint8_t mode);
extern uint8_t  control_mode_set(uint8_t mode);
extern void     hil_start(uint16_t input_timeout_ms, HilExecutionMode execution_mode);
extern void     hil_stop(void);
extern void     hil_set_inputs(uint16_t speed_rpm,
                               int16_t load_torque,
                               uint8_t flags,
                               uint32_t run_id,
                               uint32_t source_seq);
extern uint8_t  hil_has_timeout(void);
extern void     hil_get_validation_provenance(HilValidationProvenance* provenance);
extern HilStepResult hil_step(uint16_t speed_rpm,
                              uint8_t flags,
                              uint32_t run_id,
                              uint32_t source_seq,
                              uint16_t requested_steps);

// MACROS PARA MEDICION DE VELOCIDAD



#endif /* INC_MOTOR_CONTROL_H_ */
