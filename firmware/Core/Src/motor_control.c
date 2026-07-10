/*
 * motor_control.c
 *
 *  Created on: Feb 3, 2025
 *      Author: Usuario
 */

#include "motor_control.h"
#include <stdlib.h>
#include <math.h>
#include "stm32f1xx_hal.h"

#define KP 0.75f
#define KI 1.35f // real Ki = KI * 2/SCALE
#define SCALE 1
#define dt 0.002f

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
#define HIL_INPUT_TIMEOUT_MS 50
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
static volatile uint16_t min_limit_pwm;
static volatile float    pwm_speed_range_relation;
// Funciones

// VARIABLES PARA CONTROL PI DE VELOCIDAD
volatile uint16_t        speed_setpoint           = 0;
volatile uint16_t        speed_setpoint_rpm       = 0; // RPM
static volatile int32_t  speed_prev_error         = 0;
static volatile uint16_t max_limit_pwm            = 0;
static volatile uint16_t min_limit_pwm            = 0;
static volatile float    pwm_speed_range_relation = 0.0f;
static volatile uint16_t speed_measure              = 0;
static volatile int32_t  speed_error;
static volatile int32_t  speed_output   = 0;
static volatile float    speed_integral = 0;
static volatile float    speed_proportional;
static volatile float    max_speed_integral = 0;
static volatile float    min_speed_integral = 0;
//====================================================

// VARIABLES PARA MANEJO DE VELOCIDAD
volatile int32_t diff_speed = 0;
volatile uint8_t direction  = 0;
//====================================================


volatile uint8_t consistent_zero_crossing = 0; // flag
//====================================================

static volatile uint16_t hil_speed_rpm = 0;
static volatile int16_t  hil_load_torque = 0;
static volatile uint8_t  hil_flags = 0;
static volatile uint32_t hil_last_input_tick = 0;
static volatile uint32_t hil_input_run_id = 0;
static volatile uint32_t hil_input_source_seq = 0;
static volatile HilValidationProvenance hil_validation_provenance = {0};

__attribute__((optimize("Os"))) static uint32_t hil_lock(void)
{
  uint32_t primask = __get_PRIMASK();
  __disable_irq();
  return primask;
}

__attribute__((optimize("Os"))) static void hil_unlock(uint32_t primask)
{
  if (primask == 0U) {
    __enable_irq();
  }
}

__attribute__((optimize("Os"))) static void hil_reset_dynamic_state(void)
{
  speed_prev_error = 0;
  speed_measure = 0;
  speed_error = 0;
  speed_output = 0;
  speed_integral = 0.0f;
  speed_proportional = 0.0f;
  max_speed_integral = 0.0f;
  min_speed_integral = 0.0f;
  diff_speed = 0;
}

static inline bool hil_mode_active(void)
{
  return control_runtime_mode == CONTROL_RUNTIME_HIL_SIM;
}

static uint16_t measured_speed_period(void)
{
  if (hil_mode_active()) {
    return rpm_to_period(hil_speed_rpm);
  }
  return get_actual_speed();
}

// MAPEO PERIODO a PWM
static inline uint16_t map_speed(uint16_t raw_speed)
{
  if (raw_speed == 0) {
    raw_speed = SPEED_MIN;
  }

  if (raw_speed > SPEED_MIN) {
    raw_speed = SPEED_MIN;
  } else if (raw_speed < SPEED_MAX) {
    raw_speed = SPEED_MAX;
  }
  uint16_t mapped_speed = (SPEED_MIN - raw_speed) * pwm_speed_range_relation + min_limit_pwm;
  return mapped_speed;
}
// FUNCIONES PARA MEDICION DE VELOCIDAD

void updateAllMotorControl()
{
  max_limit_pwm            = TIM1->ARR;
  max_pwm                  = max_limit_pwm;
  min_limit_pwm            = (uint16_t)(max_limit_pwm * 0.05f);
  pwm_speed_range_relation = (float)(max_limit_pwm - min_limit_pwm) / (float)speed_sensor_get_speed_range();
  if (speed_setpoint_rpm == 0U) {
    speed_setpoint_rpm = 1000;
  }
  motor_control_config_done = 1;
}


static inline bool state_allows_commutation(void){
  return !hil_mode_active() && control_runtime_mode != CONTROL_RUNTIME_MONITOR_ONLY &&
         (app_state == RUNNING || app_state == CLOSEDLOOP);
}
static inline bool state_allows_speed_measurement(void){
  return !hil_mode_active() &&
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
  if (hil_mode_active()) {
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

    if (hil_mode_active()) {
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


uint16_t period_to_pwm(uint16_t period)
{
  return map_speed(period); // Usa tu función existente
}


/**
 * @brief Controlador PI (Proporcional-Integral) para regulación de velocidad del motor
 *
 * Esta función implementa un controlador PI discreto para mantener la velocidad del motor
 * en el valor de referencia (speed_setpoint). El controlador calcula la salida PWM
 * necesaria basándose en el error entre la velocidad deseada y la velocidad medida.
 *
 * @details Algoritmo del controlador PI:
 * 1. Mapea la velocidad filtrada a unidades PWM usando map_speed()
 * 2. Calcula el error: error = setpoint - velocidad_medida
 * 3. Término proporcional: P = KP * error / SCALE
 * 4. Término integral: I += KI * (error + error_previo) * dt
 * 5. Anti-windup del integrador: limita la integral según los límites PWM
 * 6. Salida final: PWM = P + I (con saturación en límites PWM)
 *
 * @note Parámetros del controlador:
 * - KP: Ganancia proporcional (0.75f)
 * - KI: Ganancia integral (1.35f)
 * - dt: Período de muestreo (0.002s = 2ms)
 * - SCALE: Factor de escalado (1)
 *
 * @note Variables globales utilizadas:
 * - speed_setpoint: Velocidad deseada en unidades PWM
 * - filtered_speed: Velocidad medida y filtrada en unidades de período
 * - g_pwm_val: Salida PWM calculada para el motor
 * - speed_integral: Estado del integrador PI
 * - max_limit_pwm, min_limit_pwm: Límites de saturación PWM
 *
 * @details Anti-windup implementado:
 * - Calcula límites dinámicos para el integrador basados en la salida proporcional
 * - max_speed_integral = max_limit_pwm - speed_proportional
 * - min_speed_integral = min_limit_pwm - speed_proportional
 * - Previene saturación del integrador cuando la salida alcanza los límites PWM
 * - Mejora el tiempo de respuesta transitorio del controlador
 *
 * @note Solo opera cuando app_state == CLOSEDLOOP
 * @warning No utiliza protección de interrupciones, debe ser llamada desde contexto seguro
 */
__attribute__((optimize("Os"))) void pi_control()
{

  speed_measure = period_to_pwm(measured_speed_period());
  if (app_state == CLOSEDLOOP) {
    uint16_t target_period = rpm_to_period(speed_setpoint_rpm);
    uint16_t target_pwm    = period_to_pwm(target_period);
    speed_error        = target_pwm - speed_measure;
    if (diff_speed == 0) {
      diff_speed = speed_measure;
    }
    speed_proportional = (current_esc_params.speed_kp * speed_error) / SCALE;
    speed_proportional -= (current_esc_params.speed_kd * ((int32_t)speed_measure - diff_speed)) / dt;
    diff_speed = speed_measure;
    speed_integral += current_esc_params.speed_ki * (speed_error + speed_prev_error) * dt;
    speed_prev_error = speed_error;
    if (max_limit_pwm > speed_proportional) {
      max_speed_integral = (max_limit_pwm - speed_proportional);
    } else {
      max_speed_integral = 0;
    }
    if (min_limit_pwm < speed_proportional) {
      min_speed_integral = (min_limit_pwm - speed_proportional);
    } else {
      min_speed_integral = 0;
    }
    if (speed_integral > max_speed_integral) {
      speed_integral = max_speed_integral;
    } else if (speed_integral < min_speed_integral) {
      speed_integral = min_speed_integral;
    }
    speed_output = speed_proportional + speed_integral;
    if (speed_output < min_limit_pwm)
      speed_output = min_limit_pwm;
    if (speed_output > max_limit_pwm)
      speed_output = max_limit_pwm;
    bldc_set_pwm((uint16_t)speed_output);
    if (hil_mode_active()) {
      hil_validation_provenance.applied_run_id = hil_input_run_id;
      hil_validation_provenance.applied_source_seq = hil_input_source_seq;
      hil_validation_provenance.output_generation++;
      hil_validation_provenance.pwm_update_tick = HAL_GetTick();
    }
  }
}


void stop_motor(uint8_t mode)
{
  //filtered_speed = 0;
  if (hil_mode_active() || control_runtime_mode == CONTROL_RUNTIME_MONITOR_ONLY) {
    PWM_STOP();
    bldc_set_pwm(0);
    app_state = IDLE;
    return;
  }

  switch (mode) {
  case 0:
    // gradual stop
    PWM_STOP();
    app_state = IDLE;
    break;
  case 1:
    // emergency stop
    app_state = STOPPED;

    direction                = !direction;
    bldc_set_pwm(max_pwm);
    uint32_t stop_start_time = HAL_GetTick();
    commutate(POS_UV);
    while (HAL_GetTick() - stop_start_time < 5000)
      ;
    PWM_STOP();
    direction = !direction;
    bldc_set_pwm(0);
    app_state = IDLE;
    break;
  }
}

uint8_t control_mode_set(uint8_t mode)
{
  if (mode > CONTROL_RUNTIME_HIL_SIM) {
    return 0;
  }

  if (mode != CONTROL_RUNTIME_NORMAL) {
    PWM_STOP();
    bldc_set_pwm(0);
    app_state = IDLE;
  }

  control_runtime_mode = (ControlRuntimeMode)mode;
  consistent_zero_crossing = mode == CONTROL_RUNTIME_HIL_SIM ? 1U : 0U;
  return 1;
}

__attribute__((optimize("Os"))) uint8_t hil_start(void)
{
  if (!hil_mode_active()) {
    return 0;
  }

  uint32_t primask = hil_lock();
  PWM_STOP();
  bldc_set_pwm(0);
  hil_reset_dynamic_state();
  hil_speed_rpm = 0;
  hil_load_torque = 0;
  hil_flags = 0;
  hil_input_run_id = 0;
  hil_input_source_seq = 0;
  hil_validation_provenance = (HilValidationProvenance){0};
  hil_last_input_tick = HAL_GetTick();
  motor_stalled = false;
  consistent_zero_crossing = 1;
  TIM4->PSC = 2;
  TIM4->ARR = 0xFFFF;
  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 48000);
  app_state = CLOSEDLOOP;
  HAL_TIM_OC_Start_IT(&htim4, TIM_CHANNEL_1);
  hil_unlock(primask);
  return 1;
}

__attribute__((optimize("Os"))) void hil_stop(void)
{
  uint32_t primask = hil_lock();
  PWM_STOP();
  bldc_set_pwm(0);
  HAL_TIM_OC_Stop_IT(&htim4, TIM_CHANNEL_1);
  app_state = IDLE;
  hil_unlock(primask);
}

__attribute__((optimize("Os"))) void hil_set_inputs(uint16_t speed_rpm,
                                                      int16_t load_torque,
                                                      uint8_t flags,
                                                      uint32_t run_id,
                                                      uint32_t source_seq)
{
  uint32_t primask = hil_lock();
  hil_speed_rpm = speed_rpm;
  hil_load_torque = load_torque;
  hil_flags = flags;
  hil_input_run_id = run_id;
  hil_input_source_seq = source_seq;
  hil_validation_provenance.accepted_run_id = run_id;
  hil_validation_provenance.accepted_source_seq = source_seq;
  hil_validation_provenance.accepted_generation =
      hil_validation_provenance.output_generation;
  hil_last_input_tick = HAL_GetTick();
  consistent_zero_crossing = 1;
  hil_unlock(primask);
}

uint8_t hil_is_active(void)
{
  return hil_mode_active() ? 1U : 0U;
}

uint8_t hil_has_timeout(void)
{
  if (!hil_mode_active()) {
    return 0U;
  }
  return (uint32_t)(HAL_GetTick() - hil_last_input_tick) > HIL_INPUT_TIMEOUT_MS ? 1U : 0U;
}

uint16_t hil_get_speed_rpm(void)
{
  return hil_speed_rpm;
}

uint16_t hil_get_pwm_command(void)
{
  return bldc_get_pwm();
}

uint8_t hil_get_flags(void)
{
  return hil_flags;
}

__attribute__((optimize("Os"))) void
hil_get_validation_provenance(HilValidationProvenance* provenance)
{
  if (provenance == NULL) {
    return;
  }

  uint32_t primask = hil_lock();
  *provenance = hil_validation_provenance;
  hil_unlock(primask);
}

uint16_t convert_speed_ticks(uint16_t value, bool to_ticks)
{
  if (to_ticks) {
    // RPM -> setpoint (PWM units): reuse speed_sensor helpers
    if (value == 0)
      return 0;
    uint16_t period = rpm_to_period(value);
    return period_to_pwm(period);
  } else {
    // setpoint (PWM units) -> RPM
    if (value == 0)
      return 0;

    uint32_t pwm_range = (uint32_t)(max_limit_pwm - min_limit_pwm);
    uint32_t speed_range = (uint32_t)speed_sensor_get_speed_range();
    if (pwm_range == 0u)
      return 0;

    // Revertir el mapeo lineal que usamos para mapear periodo->pwm
    uint32_t timer_ticks;
    int32_t pwm_offset = (int32_t)value - (int32_t)min_limit_pwm;
    if (pwm_offset <= 0) {
      timer_ticks = SPEED_MIN;
    } else if ((uint32_t)pwm_offset >= pwm_range) {
      timer_ticks = SPEED_MAX;
    } else {
      timer_ticks = SPEED_MIN - (((uint32_t)pwm_offset * speed_range) / pwm_range);
    }

    // Clamp al rango válido
    if (timer_ticks < SPEED_MAX)
      timer_ticks = SPEED_MAX;
    if (timer_ticks > SPEED_MIN)
      timer_ticks = SPEED_MIN;

    return period_to_rpm((uint16_t)timer_ticks);
  }
}






