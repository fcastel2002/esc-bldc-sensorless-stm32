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

// STALL HANDLING
#define TIMEOUT_MOTOR_STALL_MS 200
#define STALL_CHECK_TIME_MS 25

// CONFIGURACION DE MOTOR
volatile uint8_t  motor_control_config_done = 0;
volatile uint16_t max_pwm                   = 0;
volatile bool     motor_stalled             = false;

static volatile uint32_t last_zc_timestamp = 0;
static volatile uint16_t max_limit_pwm     = 0;
static volatile uint16_t zc_blanking_start_count = 0U;
static volatile uint16_t zc_blanking_ticks = 0U;
static volatile uint8_t zc_blanking_armed = 0U;
static volatile uint32_t zc_blanking_start_tick_ms = 0U;

// VARIABLES PARA CONTROL PI DE VELOCIDAD
volatile uint16_t        speed_setpoint           = 0;
volatile uint16_t        speed_setpoint_rpm       = 0; // RPM objetivo comandado
static RpmPiController rpm_controller = {0};

/*
 * Limitador de pendiente de la consigna (slew limiter).
 * speed_reference_q16 es la referencia efectiva que ve el PI (RPM en Q16.16).
 * Se actualiza en el ISR del PI (cada MOTOR_CONTROL_DT_US) y avanza hacia
 * speed_setpoint_rpm a lo sumo speed_ramp_step_q16 por tick. Paso 0 = sin limite.
 */
static volatile int32_t  speed_reference_q16      = 0;
static volatile uint32_t speed_ramp_step_q16      = 0U;
static volatile uint16_t speed_ramp_override_rpm_s = 0U;
//====================================================

// VARIABLES PARA MANEJO DE VELOCIDAD
volatile int32_t diff_speed = 0;
volatile uint8_t direction  = 0;
//====================================================

volatile uint8_t consistent_zero_crossing = 0; // flag
//====================================================

static uint32_t irq_lock(void)
{
  uint32_t primask = __get_PRIMASK();
  __disable_irq();
  return primask;
}

static void irq_unlock(uint32_t primask)
{
  if (primask == 0U) {
    __enable_irq();
  }
}

static void control_timer_stop(void)
{
  HAL_TIM_OC_Stop_IT(&htim4, TIM_CHANNEL_1);
  HAL_TIM_Base_Stop_IT(&htim4);
  __HAL_TIM_SET_COUNTER(&htim4, 0U);
  __HAL_TIM_CLEAR_FLAG(&htim4, TIM_FLAG_UPDATE | TIM_FLAG_CC1);
}

static uint32_t ramp_rate_to_step_q16(uint16_t rpm_per_s)
{
  if (rpm_per_s == 0U) return 0U;
  uint64_t step = ((uint64_t)rpm_per_s << 16) * MOTOR_CONTROL_DT_US / 1000000U;
  return step == 0U ? 1U : (uint32_t)step;
}

static void apply_ramp_rate(void)
{
  uint16_t rate = speed_ramp_override_rpm_s != 0U ? speed_ramp_override_rpm_s
                                                  : get_speed_ramp();
  speed_ramp_step_q16 = ramp_rate_to_step_q16(rate);
}

void motor_control_set_ramp_override(uint16_t rpm_per_s)
{
  speed_ramp_override_rpm_s = rpm_per_s;
  apply_ramp_rate();
}

uint16_t motor_control_get_reference_rpm(void)
{
  int32_t reference = speed_reference_q16;
  if (reference <= 0) return 0U;
  return (uint16_t)((reference + 0x8000) >> 16);
}

bool motor_control_reference_at_target(void)
{
  return speed_reference_q16 == ((int32_t)speed_setpoint_rpm << 16);
}

static uint16_t advance_speed_reference(void)
{
  int32_t target = (int32_t)speed_setpoint_rpm << 16;
  int32_t reference = speed_reference_q16;
  uint32_t step = speed_ramp_step_q16;

  if (step == 0U) {
    reference = target;
  } else {
    int32_t delta = target - reference;
    if (delta > (int32_t)step) {
      reference += (int32_t)step;
    } else if (delta < -(int32_t)step) {
      reference -= (int32_t)step;
    } else {
      reference = target;
    }
  }
  speed_reference_q16 = reference;
  return (uint16_t)((reference + 0x8000) >> 16);
}

void updateAllMotorControl()
{
  max_limit_pwm            = TIM1->ARR;
  max_pwm                  = max_limit_pwm;
  rpm_pi_configure(&rpm_controller, current_esc_params.speed_kp, current_esc_params.speed_ki);
  apply_ramp_rate();
  if (speed_setpoint_rpm == 0U) {
    speed_setpoint_rpm = 1000;
  }
  motor_control_config_done = 1;
}

void motor_control_reset_runtime(void)
{
  uint32_t primask = irq_lock();
  control_timer_stop();

  motor_stalled = false;
  consistent_zero_crossing = 0;
  last_zc_timestamp = 0;
  zc_blanking_start_count = 0U;
  zc_blanking_armed = 0U;
  zc_blanking_start_tick_ms = 0U;
  uint32_t tim2_counter_hz = HAL_RCC_GetHCLKFreq() / (TIM2->PSC + 1U);
  uint32_t blanking_ticks =
      ((uint32_t)get_bemf_blanking_us() * tim2_counter_hz + 999999U) / 1000000U;
  zc_blanking_ticks = blanking_ticks > UINT16_MAX ? UINT16_MAX : (uint16_t)blanking_ticks;
  speed_reference_q16 = 0;
  rpm_pi_reset(&rpm_controller);
  speed_sensor_reset();
  irq_unlock(primask);
}

void motor_control_prepare_closed_loop(void)
{
  uint16_t measured_rpm = speed_sensor_get_speed_rpm();
  rpm_pi_configure(&rpm_controller, current_esc_params.speed_kp, current_esc_params.speed_ki);
  apply_ramp_rate();
  /* With a slew limit the reference starts at the measured speed (bumpless);
   * without one it jumps to the target, as before the limiter existed. */
  speed_reference_q16 = speed_ramp_step_q16 != 0U ? ((int32_t)measured_rpm << 16)
                                                  : ((int32_t)speed_setpoint_rpm << 16);
  rpm_pi_prepare_bumpless(&rpm_controller,
                          motor_control_get_reference_rpm(),
                          measured_rpm,
                          bldc_get_pwm(),
                          max_limit_pwm);
}

uint16_t motor_control_get_duty_e4(void)
{
  uint32_t arr = TIM1->ARR;
  if (arr == 0U) return 0U;
  uint32_t duty = ((uint32_t)bldc_get_pwm() * 10000U + arr / 2U) / arr;
  return duty > 10000U ? 10000U : (uint16_t)duty;
}

static inline bool state_allows_commutation(void){
  return app_state == RUNNING || app_state == CLOSEDLOOP;
}
static inline bool state_allows_speed_measurement(void){
  return app_state == RUNNING || app_state == CLOSEDLOOP || app_state == FOC_STARTUP;
}
static inline bool state_allows_speed_update(void){
  return (app_state == CLOSEDLOOP);
}

static bool zc_capture_is_blanked(uint16_t capture_timestamp)
{
  if (zc_blanking_armed == 0U || zc_blanking_ticks == 0U) return false;
  uint16_t current_count = (uint16_t)__HAL_TIM_GET_COUNTER(&htim2);
  uint16_t current_elapsed = (uint16_t)(current_count - zc_blanking_start_count);
  uint32_t elapsed_ms = HAL_GetTick() - zc_blanking_start_tick_ms;
  if (current_elapsed < zc_blanking_ticks && elapsed_ms <= 1U) return true;

  zc_blanking_armed = 0U;
  if (elapsed_ms > 1U) return false;
  return (uint16_t)(capture_timestamp - zc_blanking_start_count) < zc_blanking_ticks;
}

void motor_control_arm_zc_blanking(void)
{
  zc_blanking_start_count = (uint16_t)__HAL_TIM_GET_COUNTER(&htim2);
  zc_blanking_start_tick_ms = HAL_GetTick();
  zc_blanking_armed = zc_blanking_ticks != 0U ? 1U : 0U;
}

static void commutate(bool update_timestamp){
  if(!state_allows_commutation()) return;
  if(update_timestamp) last_zc_timestamp = HAL_GetTick();
  bldc_commutate();
  motor_control_arm_zc_blanking();
}
void zero_crossing_handler(uint8_t phase, uint16_t capture_timestamp)
{
  if (state_allows_commutation() && zc_capture_is_blanked(capture_timestamp)) return;

  switch (phase) {
  case 1: // Fase W

    commutate(true);

    // Procesar medición de velocidad para fase W
    if (state_allows_speed_measurement()) {
      speed_sensor_handle_W_measurement(capture_timestamp);
    }
    if(state_allows_speed_update()) speed_sensor_handle_consensus();
    break;

  case 2: // Fase U
    commutate(false);

    // Procesar medición de velocidad para fase U
    if (state_allows_speed_measurement()) {
      speed_sensor_process_phase_measurement(2, capture_timestamp); // Fase U = índice 2
    }
    break;

  case 3: // Fase V
    commutate(false);

    // Procesar medición de velocidad para fase V
    if (state_allows_speed_measurement()) {
      speed_sensor_process_phase_measurement(1, capture_timestamp); // Fase V = índice 1
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

__attribute__((optimize("Oz"))) void pi_control()
{
  if (!speed_sensor_is_ready()) {
    return;
  }
  uint16_t reference_rpm = advance_speed_reference();
  uint16_t speed_output = rpm_pi_step(&rpm_controller,
                                      reference_rpm,
                                      speed_sensor_get_speed_rpm(),
                                      max_limit_pwm);
  bldc_set_pwm(speed_output);
}


void stop_motor(uint8_t mode)
{
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

void motor_control_timer_start(void)
{
  /* HAL_TIM_OC_Start_IT() would also set CC1E, driving TIM4_CH1 onto PB6 (INA219 SCL). */
  __HAL_TIM_ENABLE_IT(&htim4, TIM_IT_CC1);
  __HAL_TIM_ENABLE(&htim4);
}
