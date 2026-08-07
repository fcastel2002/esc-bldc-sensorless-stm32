/*
 * motor_control.c
 *
 *  Created on: Feb 3, 2025
 *      Author: Usuario
 */

#include "motor_control.h"
#include "rpm_pi_controller.h"
#include <stdlib.h>
#include <math.h>
#include "stm32f1xx_hal.h"

#pragma GCC optimize("Oz")

/*
 *@brief: Macros para la medicion y mapeo de velocidad
 * unidades: PERIODO.
 * TIMER CLOCK: 72MHz
 * PSC: 400 (399 + 1)
 * ARR: 0xFFFF
 *
 * VELOCIDAD MINIMA RPM: 380
 * VELOCIDAD MAXIMA RPM: 12000
 *
 */


////////////

// limites PWM

// STALL HANDLING
#define TIMEOUT_MOTOR_STALL_MS 200
#define STALL_CHECK_TIME_MS 25
#define HIL_DEFAULT_INPUT_TIMEOUT_MS 50U
#define HIL_MIN_INPUT_TIMEOUT_MS 10U
#define HIL_MAX_INPUT_TIMEOUT_MS 5000U
////////


///
#define MAX_RPM 5600
#define MIN_RPM 400



// CONFIGURACION DE MOTOR
// Variables
volatile uint8_t  motor_control_config_done = 0;
volatile uint16_t max_pwm                   = 0;
volatile bool     motor_stalled             = false;
volatile ControlRuntimeMode control_runtime_mode = CONTROL_RUNTIME_NORMAL;

static volatile uint32_t last_zc_timestamp = 0;
static volatile uint16_t max_limit_pwm;
// Funciones

// VARIABLES PARA CONTROL PI DE VELOCIDAD
volatile uint16_t        speed_setpoint           = 0;
volatile uint16_t        speed_setpoint_rpm       = 0; // RPM
static volatile uint16_t max_limit_pwm            = 0;
static RpmPiController rpm_controller = {0};
//====================================================

// VARIABLES PARA MANEJO DE VELOCIDAD
volatile int32_t diff_speed = 0;
volatile uint8_t direction  = 0;
//====================================================


volatile uint8_t consistent_zero_crossing = 0; // flag
//====================================================

volatile uint16_t hil_speed_rpm = 0;
volatile uint8_t  hil_flags = 0;
static volatile uint32_t hil_last_input_tick = 0;
static volatile uint16_t hil_input_timeout_ms = HIL_DEFAULT_INPUT_TIMEOUT_MS;
static volatile uint32_t hil_input_run_id = 0;
static volatile uint32_t hil_input_source_seq = 0;
static volatile HilValidationProvenance hil_validation_provenance = {0};
static volatile uint8_t hil_input_received = 0;
volatile uint8_t hil_session_state = 0;
static volatile uint16_t hil_last_step_count = 0;

__attribute__((optimize("Oz"))) static uint32_t hil_lock(void)
{
  uint32_t primask = __get_PRIMASK();
  __disable_irq();
  return primask;
}

__attribute__((optimize("Oz"))) static void hil_unlock(uint32_t primask)
{
  if (primask == 0U) {
    __enable_irq();
  }
}

static void hil_stop_timer(void)
{
  HAL_TIM_OC_Stop_IT(&htim4, TIM_CHANNEL_1);
  HAL_TIM_Base_Stop_IT(&htim4);
  __HAL_TIM_SET_COUNTER(&htim4, 0U);
  __HAL_TIM_CLEAR_FLAG(&htim4, TIM_FLAG_UPDATE | TIM_FLAG_CC1);
}

static inline bool hil_runtime_mode_selected(void)
{
  return control_runtime_mode == CONTROL_RUNTIME_HIL_SIM;
}

static uint16_t measured_speed_rpm(void)
{
  if (hil_session_state != 0U) {
    return hil_speed_rpm;
  }
  return speed_sensor_get_speed_rpm();
}

void updateAllMotorControl()
{
  max_limit_pwm            = TIM1->ARR;
  max_pwm                  = max_limit_pwm;
  rpm_pi_configure(&rpm_controller, current_esc_params.speed_kp, current_esc_params.speed_ki);
  if (speed_setpoint_rpm == 0U) {
    speed_setpoint_rpm = 1000;
  }
  motor_control_config_done = 1;
}

void motor_control_reset_runtime(void)
{
  uint32_t primask = hil_lock();
  hil_stop_timer();

  motor_stalled = false;
  consistent_zero_crossing = 0;
  last_zc_timestamp = 0;
  rpm_pi_reset(&rpm_controller);
  speed_sensor_reset();
  hil_unlock(primask);
}

void motor_control_prepare_closed_loop(void)
{
  rpm_pi_configure(&rpm_controller, current_esc_params.speed_kp, current_esc_params.speed_ki);
  rpm_pi_prepare_bumpless(&rpm_controller,
                          speed_setpoint_rpm,
                          speed_sensor_get_speed_rpm(),
                          bldc_get_pwm(),
                          max_limit_pwm);
}


static inline bool state_allows_commutation(void){
  return !hil_runtime_mode_selected() && control_runtime_mode != CONTROL_RUNTIME_MONITOR_ONLY &&
         (app_state == RUNNING || app_state == CLOSEDLOOP);
}
static inline bool state_allows_speed_measurement(void){
  return !hil_runtime_mode_selected() &&
         (app_state == RUNNING || app_state == CLOSEDLOOP || app_state == FOC_STARTUP);
}
static inline bool state_allows_speed_update(void){
  return (app_state == CLOSEDLOOP);
}

static void commutate(bool update_timestamp){
  if(!state_allows_commutation()) return;
  if(update_timestamp) last_zc_timestamp = HAL_GetTick();
  bldc_commutate();
}
void zero_crossing_handler(uint8_t fase)
{
  static uint8_t speed_calc_counter = 0;
  if (hil_runtime_mode_selected()) {
    return;
  }

  switch (fase) {
  case 1: // Fase W
  
    commutate(true);

    // Procesar medición de velocidad para fase W
    if (state_allows_speed_measurement()) {
      speed_sensor_handle_W_measurement();
    }
    if(state_allows_speed_update()) speed_sensor_handle_consensus();
    break;

  case 2: // Fase V
    commutate(false);

    // Procesar medición de velocidad para fase V
    if (state_allows_speed_measurement()) {
      uint16_t current_timestamp = HAL_TIM_ReadCapturedValue(&htim2, TIM_CHANNEL_2);
      speed_sensor_process_phase_measurement(1, current_timestamp); // Fase V = índice 1
    }
    break;

  case 3: // Fase U
    commutate(false);

    // Procesar medición de velocidad para fase U
    if (state_allows_speed_measurement()) {
      uint16_t current_timestamp = HAL_TIM_ReadCapturedValue(&htim2, TIM_CHANNEL_3);
      speed_sensor_process_phase_measurement(2, current_timestamp); // Fase U = índice 2
    }
    break;
  }
}

void detect_motor()
{
  TIM1->PSC        = 7;
  uint16_t arr_pwm = 0;
  arr_pwm          = 1000;
  TIM1->ARR        = arr_pwm;
  bldc_set_pwm(arr_pwm * 0.3f);
  uint8_t step     = POS_UV;
  for (int k = 0; k < 3; k++) {
    PWM_STOP();
    HAL_Delay(20);
    PWM_INIT();
    for (int i = 0; i < 140; i++) {
      commutate(step);
      step = safe_mod(step + 1, NUM_POS);

      HAL_Delay(1);
    }
  }
  PWM_STOP();

  bldc_set_pwm(0);
}

void check_motor_status()
{
  static uint32_t last_check_time = 0;
  static uint8_t  stall_counter   = 0;
  static uint8_t  running_counter = 0;

  uint32_t current_time = HAL_GetTick();
  if (current_time - last_check_time >= STALL_CHECK_TIME_MS) {
    last_check_time = current_time;

    if (hil_session_state != 0U) {
      if (hil_has_timeout()) {
        motor_stalled = true;
        hil_stop();
      } else {
        motor_stalled = false;
      }
      return;
    }

    if (app_state == RUNNING || app_state == CLOSEDLOOP) {
      if (last_zc_timestamp > 0 && ((current_time - last_zc_timestamp) > TIMEOUT_MOTOR_STALL_MS)) {
        stall_counter++;
        running_counter = 0;
        if (stall_counter >= 3) {
          motor_stalled = true;
        }

      } else {
        running_counter++;
        stall_counter = 0;
        if (running_counter >= 2 && motor_stalled) {
          motor_stalled   = false;
          running_counter = 0;
        }
      }
    } else {
      running_counter = 0;
      stall_counter   = 0;
    }
  }
}


__attribute__((optimize("Oz"))) static void apply_pi_tick(void)
{
  uint16_t speed_output = rpm_pi_step(&rpm_controller,
                                      speed_setpoint_rpm,
                                      measured_speed_rpm(),
                                      max_limit_pwm);
  bldc_set_pwm(speed_output);

  if (hil_session_state != 0U) {
    hil_validation_provenance.applied_run_id = hil_input_run_id;
    hil_validation_provenance.applied_source_seq = hil_input_source_seq;
    hil_validation_provenance.output_generation++;
    hil_validation_provenance.pwm_update_tick = HAL_GetTick();
  }
}

__attribute__((optimize("Oz"))) void pi_control()
{
  if ((hil_session_state == 0U && !speed_sensor_is_ready()) ||
      (hil_session_state != 0U && !hil_input_received)) {
    return;
  }
  apply_pi_tick();
}


void stop_motor(uint8_t mode)
{
  //filtered_speed = 0;
  if (hil_session_state != 0U) {
    hil_stop();
    return;
  }
  if (hil_runtime_mode_selected() || control_runtime_mode == CONTROL_RUNTIME_MONITOR_ONLY) {
    bldc_disable_power_stage();
    bldc_set_pwm(0);
    motor_control_reset_runtime();
    app_state = IDLE;
    return;
  }

  switch (mode) {
  case 0:
    // gradual stop
    bldc_disable_power_stage();
    bldc_set_pwm(0);
    motor_control_reset_runtime();
    app_state = IDLE;
    break;
  case 1:
    // emergency stop
    app_state = STOPPED;
    bldc_disable_power_stage();
    bldc_set_pwm(0);
    motor_control_reset_runtime();
    app_state = IDLE;
    break;
  }
}

uint8_t control_mode_set(uint8_t mode)
{
  if (mode > CONTROL_RUNTIME_HIL_SIM) {
    return 0;
  }

  if (hil_session_state != 0U && mode != CONTROL_RUNTIME_HIL_SIM) {
    hil_stop();
  }

  if (mode != CONTROL_RUNTIME_NORMAL) {
    bldc_disable_power_stage();
    bldc_set_pwm(0);
    app_state = IDLE;
  }

  control_runtime_mode = (ControlRuntimeMode)mode;
  consistent_zero_crossing = mode == CONTROL_RUNTIME_HIL_SIM ? 1U : 0U;
  return 1;
}

__attribute__((optimize("Oz"))) void
hil_start(uint16_t input_timeout_ms, HilExecutionMode execution_mode)
{
  hil_session_state = (uint8_t)HIL_EXECUTION_STEPPED + 1U;
  PWM_STOP();
  bldc_set_pwm(0);
  hil_stop_timer();
  rpm_pi_reset(&rpm_controller);
  hil_speed_rpm = 0;
  hil_flags = 0;
  hil_input_run_id = 0;
  hil_input_source_seq = 0;
  hil_input_received = 0;
  hil_validation_provenance = (HilValidationProvenance){0};
  hil_input_timeout_ms = input_timeout_ms == 0U ? HIL_DEFAULT_INPUT_TIMEOUT_MS : input_timeout_ms;
  hil_last_input_tick = HAL_GetTick();
  hil_session_state = (uint8_t)execution_mode + 1U;
  motor_stalled = false;
  consistent_zero_crossing = 1;
  TIM4->PSC = MOTOR_CONTROL_TIMER_PSC;
  TIM4->ARR = MOTOR_CONTROL_TIMER_ARR;
  __HAL_TIM_SET_COUNTER(&htim4, 0U);
  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, MOTOR_CONTROL_TIMER_COMPARE);
  TIM4->EGR = TIM_EGR_UG;
  __HAL_TIM_CLEAR_FLAG(&htim4, TIM_FLAG_UPDATE | TIM_FLAG_CC1);
  app_state = CLOSEDLOOP;
  if (execution_mode == HIL_EXECUTION_PERIODIC) {
    HAL_TIM_OC_Start_IT(&htim4, TIM_CHANNEL_1);
  }
}

__attribute__((optimize("Oz"))) void hil_stop(void)
{
  hil_session_state = (uint8_t)HIL_EXECUTION_STEPPED + 1U;
  PWM_STOP();
  bldc_set_pwm(0);
  hil_input_received = 0;
  hil_stop_timer();
  hil_session_state = 0;
  control_runtime_mode = CONTROL_RUNTIME_NORMAL;
  consistent_zero_crossing = 0;
  app_state = IDLE;
}

__attribute__((optimize("Oz"))) void hil_set_inputs(uint16_t speed_rpm,
                                                      int16_t load_torque,
                                                      uint8_t flags,
                                                      uint32_t run_id,
                                                      uint32_t source_seq)
{
  uint32_t primask = hil_lock();
  hil_speed_rpm = speed_rpm;
  (void)load_torque;
  hil_flags = flags;
  hil_input_run_id = run_id;
  hil_input_source_seq = source_seq;
  hil_input_received = 1;
  hil_validation_provenance.accepted_run_id = run_id;
  hil_validation_provenance.accepted_source_seq = source_seq;
  hil_validation_provenance.accepted_generation =
      hil_validation_provenance.output_generation;
  hil_last_input_tick = HAL_GetTick();
  consistent_zero_crossing = 1;
  hil_unlock(primask);
}

uint8_t hil_has_timeout(void)
{
  if (hil_session_state == 0U) {
    return 0U;
  }
  return (uint32_t)(HAL_GetTick() - hil_last_input_tick) > hil_input_timeout_ms ? 1U : 0U;
}

__attribute__((optimize("Oz"))) void
hil_get_validation_provenance(HilValidationProvenance* provenance)
{
  if (provenance == NULL) {
    return;
  }

  uint32_t primask = hil_lock();
  *provenance = hil_validation_provenance;
  hil_unlock(primask);
}

__attribute__((optimize("Oz"))) HilStepResult
hil_step(uint16_t speed_rpm,
         uint8_t flags,
         uint32_t run_id,
         uint32_t source_seq,
         uint16_t requested_steps)
{
  if (hil_input_run_id != 0U && run_id == hil_input_run_id &&
      source_seq == hil_input_source_seq && speed_rpm == hil_speed_rpm &&
      flags == hil_flags && requested_steps == hil_last_step_count) {
    hil_last_input_tick = HAL_GetTick();
    return HIL_STEP_REPLAYED;
  }

  if ((hil_input_run_id != 0U && run_id != hil_input_run_id) ||
      (hil_input_source_seq != 0U && source_seq <= hil_input_source_seq) ||
      hil_validation_provenance.output_generation > UINT32_MAX - requested_steps) {
    return HIL_STEP_INVALID;
  }

  hil_last_step_count = requested_steps;
  hil_speed_rpm = speed_rpm;
  hil_flags = flags;
  hil_input_run_id = run_id;
  hil_input_source_seq = source_seq;
  hil_input_received = 1;
  hil_validation_provenance.accepted_run_id = run_id;
  hil_validation_provenance.accepted_source_seq = source_seq;
  hil_validation_provenance.accepted_generation =
      hil_validation_provenance.output_generation;
  hil_last_input_tick = HAL_GetTick();
  consistent_zero_crossing = 1;

  for (uint16_t step = 0; step < requested_steps; step++) {
    apply_pi_tick();
  }

  return HIL_STEP_APPLIED;
}

