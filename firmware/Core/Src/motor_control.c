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
#define dt 0.002

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
////////


///
#define MAX_RPM 5600
#define MIN_RPM 400



// CONFIGURACION DE MOTOR
// Variables
volatile uint8_t  motor_control_config_done = 0;
volatile uint16_t max_pwm                   = 0;
volatile bool     motor_stalled             = false;

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

static float kp = 0;
static float ki = 0;
static float kd = 0;

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
  max_limit_pwm             = TIM1->ARR;
  max_pwm                   = max_limit_pwm;
  min_limit_pwm             = max_limit_pwm * 0.05f;
  pwm_speed_range_relation  = (float)(max_limit_pwm - min_limit_pwm) / (float)speed_sensor_get_speed_range();
  speed_setpoint_rpm        = 1000;
  kp                        = get_KP();
  ki                        = get_KI();
  kd                        = get_KD();
  motor_control_config_done = 1;
}


static inline bool state_allows_commutation(void){
  return (app_state == RUNNING || app_state == CLOSEDLOOP);
}
static inline bool state_allows_speed_measurement(void){
  return (app_state == RUNNING || app_state == CLOSEDLOOP || app_state == FOC_STARTUP);
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
void pi_control()
{

  speed_measure = period_to_pwm(get_actual_speed());
  if (app_state == CLOSEDLOOP) {
    uint16_t target_period = rpm_to_period(speed_setpoint_rpm);
    uint16_t target_pwm    = period_to_pwm(target_period);
    speed_error        = target_pwm - speed_measure;
    speed_proportional = (kp * speed_error) / SCALE;
    speed_integral += ki * (speed_error + speed_prev_error) * dt;
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
  }
}


void stop_motor(uint8_t mode)
{
  //filtered_speed = 0;
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

    // Ensure mapping params available
    if (pwm_speed_range_relation == 0.0f) {
      // protect division by zero: compute from current limits
      pwm_speed_range_relation = (float)(max_limit_pwm - min_limit_pwm) / (float)speed_sensor_get_speed_range();
    }

    // Revertir el mapeo lineal que usamos para mapear periodo->pwm
    double timer_ticks = SPEED_MIN - (((double)value - (double)min_limit_pwm) / pwm_speed_range_relation);

    // Clamp al rango válido
    if (timer_ticks < SPEED_MAX)
      timer_ticks = SPEED_MAX;
    if (timer_ticks > SPEED_MIN)
      timer_ticks = SPEED_MIN;

    // Convertir ticks -> periodo eléctrico y luego a RPM mecánicos
    double zc_period = timer_ticks / (HAL_RCC_GetHCLKFreq() / (TIM2->PSC + 1));
    double period    = zc_period * 6.0; // período eléctrico total (s)
    double frequency = 1.0 / period;    // Hz eléctricos
    double rpm_electrical = frequency * 60.0;
    double rpm_mechanical = rpm_electrical / get_motor_pole_pairs();

    // Limitar a rango razonable
    if (rpm_mechanical < get_min_speed_rpm())
      return get_min_speed_rpm();
    if (rpm_mechanical > get_max_speed_rpm())
      return get_max_speed_rpm();
    return (uint16_t)rpm_mechanical;
  }
}






