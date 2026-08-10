/*
 * startup.c
 *
 *  Created on: Jan 10, 2025
 *      Author: francisco
 */

#include "startup.h"

#include "bldc_driver.h"
#include "hard_config.h"

#define Q15_MAX 32760U
#define SINE_TIMER_PSC 35U
#define SINE_PHASE_STEP 3U

volatile uint16_t zero_crossings = 0;

bool ready_for_update_pwm = true;
bool finished_foc_startup = false;

uint16_t sin_table_U[SIN_TABLE_SIZE] = {0};
uint16_t sin_table_V[SIN_TABLE_SIZE] = {0};
uint16_t sin_table_W[SIN_TABLE_SIZE] = {0};

static uint16_t phase_counter = 0;
static uint32_t startup_start_tick = 0;
static uint32_t sine_drive_last_command_tick = 0;
static SineDriveSettings manual_settings = {0};

static void generate_sine_tables(uint8_t selected_direction)
{
  for (uint16_t i = 0; i < SIN_TABLE_SIZE; i++) {
    float angle = 2.0f * (float)M_PI * i / SIN_TABLE_SIZE;
    uint16_t u = (uint16_t)((sinf(angle) * 0.5f + 0.5f) * Q15_MAX);
    uint16_t v = (uint16_t)((sinf(angle - 2.0f * (float)M_PI / 3.0f) * 0.5f + 0.5f) * Q15_MAX);
    uint16_t w = (uint16_t)((sinf(angle - 4.0f * (float)M_PI / 3.0f) * 0.5f + 0.5f) * Q15_MAX);

    sin_table_U[i] = u;
    sin_table_V[i] = selected_direction == 0U ? v : w;
    sin_table_W[i] = selected_direction == 0U ? w : v;
  }
}

static uint32_t sine_timer_clock_hz(void)
{
  uint32_t timer_clock_hz = HAL_RCC_GetPCLK1Freq();
  if ((RCC->CFGR & RCC_CFGR_PPRE1) != RCC_CFGR_PPRE1_DIV1) {
    timer_clock_hz *= 2U;
  }
  return timer_clock_hz;
}

static uint16_t timer_arr_from_frequency(uint32_t frequency_millihz)
{
  uint64_t counter_hz = sine_timer_clock_hz() / (SINE_TIMER_PSC + 1U);
  uint64_t denominator = (uint64_t)frequency_millihz * SIN_TABLE_SIZE;
  uint64_t ticks = (counter_hz * SINE_PHASE_STEP * 1000U + denominator / 2U) /
                   denominator;
  if (ticks < 2U) ticks = 2U;
  if (ticks > 65536U) ticks = 65536U;
  return (uint16_t)(ticks - 1U);
}

static uint32_t frequency_from_timer_arr(uint16_t timer_arr)
{
  uint64_t counter_hz = sine_timer_clock_hz() / (SINE_TIMER_PSC + 1U);
  uint64_t denominator = (uint64_t)SIN_TABLE_SIZE * ((uint32_t)timer_arr + 1U);
  return (uint32_t)((counter_hz * SINE_PHASE_STEP * 1000U + denominator / 2U) /
                    denominator);
}

static uint32_t interpolate_u32(uint32_t initial, uint32_t final, uint32_t progress_permille)
{
  int64_t delta = (int64_t)final - (int64_t)initial;
  return (uint32_t)((int64_t)initial + delta * progress_permille / 1000);
}

static uint16_t interpolate_u16(uint16_t initial, uint16_t final, uint32_t progress_permille)
{
  int32_t delta = (int32_t)final - (int32_t)initial;
  return (uint16_t)((int32_t)initial + delta * (int32_t)progress_permille / 1000);
}

static uint32_t configure_sine_frequency(uint32_t frequency_millihz)
{
  uint16_t timer_arr = timer_arr_from_frequency(frequency_millihz);
  TIM4->ARR = timer_arr;
  return frequency_from_timer_arr(timer_arr);
}

static void configure_sine_timer(uint32_t frequency_millihz)
{
  HAL_TIM_OC_Stop_IT(&htim4, TIM_CHANNEL_1);
  HAL_TIM_Base_Stop_IT(&htim4);
  TIM4->PSC = SINE_TIMER_PSC;
  TIM4->ARR = timer_arr_from_frequency(frequency_millihz);
  TIM4->CNT = 0U;
  TIM4->EGR = TIM_EGR_UG;
  __HAL_TIM_CLEAR_FLAG(&htim4, TIM_FLAG_UPDATE | TIM_FLAG_CC1);
  __HAL_TIM_ENABLE_IT(&htim4, TIM_IT_UPDATE);
}

static void enable_sine_output(void)
{
  PWM_STOP();
  PWM_INIT();
  GPIOB->ODR |= EN_U | EN_V;
  GPIOA->ODR |= EN_W;
  floating_U = false;
  floating_V = false;
  floating_W = false;
  HAL_TIM_Base_Start_IT(&htim4);
}

static void write_sine_pwm(uint16_t amplitude_permille)
{
  phase_counter += SINE_PHASE_STEP;
  if (phase_counter >= SIN_TABLE_SIZE) phase_counter -= SIN_TABLE_SIZE;

  uint32_t pwm_arr = TIM1->ARR;
  uint64_t divisor = (uint64_t)Q15_MAX * 1000U;
  uint16_t du = (uint16_t)(((uint64_t)sin_table_U[phase_counter] *
                            amplitude_permille * pwm_arr) /
                           divisor);
  uint16_t dv = (uint16_t)(((uint64_t)sin_table_V[phase_counter] *
                            amplitude_permille * pwm_arr) /
                           divisor);
  uint16_t dw = (uint16_t)(((uint64_t)sin_table_W[phase_counter] *
                            amplitude_permille * pwm_arr) /
                           divisor);

  if (du > pwm_arr) du = (uint16_t)pwm_arr;
  if (dv > pwm_arr) dv = (uint16_t)pwm_arr;
  if (dw > pwm_arr) dw = (uint16_t)pwm_arr;
  __HAL_TIM_SET_COMPARE(&htim1, IN_U, du);
  __HAL_TIM_SET_COMPARE(&htim1, IN_V, dv);
  __HAL_TIM_SET_COMPARE(&htim1, IN_W, dw);
}

void foc_startup(void)
{
  motor_control_reset_runtime();
  phase_counter = 0U;
  startup_start_tick = HAL_GetTick();
  generate_sine_tables(direction);
  configure_sine_timer(get_startup_initial_frequency());
  app_state = FOC_STARTUP;
  enable_sine_output();
}

bool sine_drive_start_or_update(uint32_t frequency_millihz,
                                uint16_t amplitude_permille,
                                SineDriveSettings* applied)
{
  if (frequency_millihz < ESC_MIN_SINE_FREQUENCY_MILLIHZ ||
      frequency_millihz > ESC_MAX_SINE_FREQUENCY_MILLIHZ ||
      amplitude_permille > ESC_MAX_SINE_AMPLITUDE_PERMILLE ||
      control_runtime_mode != CONTROL_RUNTIME_NORMAL ||
      hil_session_state != 0U ||
      (app_state != IDLE && app_state != SINE_DRIVE)) {
    return false;
  }

  uint32_t command_tick = HAL_GetTick();
  if (app_state == IDLE) {
    motor_control_reset_runtime();
    phase_counter = 0U;
    generate_sine_tables(direction);
    manual_settings.frequency_millihz = frequency_millihz;
    manual_settings.amplitude_permille = amplitude_permille;
    configure_sine_timer(frequency_millihz);
    app_state = SINE_DRIVE;
    enable_sine_output();
  } else {
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    manual_settings.frequency_millihz = frequency_millihz;
    manual_settings.amplitude_permille = amplitude_permille;
    configure_sine_frequency(frequency_millihz);
    if (primask == 0U) __enable_irq();
  }

  sine_drive_last_command_tick = command_tick;
  manual_settings.frequency_millihz =
      frequency_from_timer_arr((uint16_t)TIM4->ARR);
  if (applied != NULL) *applied = manual_settings;
  return true;
}

void sine_drive_stop(void)
{
  HAL_TIM_Base_Stop_IT(&htim4);
  PWM_STOP();
  bldc_set_pwm(0U);
  bldc_disable_power_stage();
  manual_settings = (SineDriveSettings){0};
  phase_counter = 0U;
}

void sine_drive_check_watchdog(void)
{
  if (app_state == SINE_DRIVE &&
      (uint32_t)(HAL_GetTick() - sine_drive_last_command_tick) > SINE_DRIVE_WATCHDOG_MS) {
    sine_drive_stop();
    motor_control_reset_runtime();
    app_state = IDLE;
  }
}

bool sine_drive_is_active(void)
{
  return app_state == SINE_DRIVE;
}

void update_pwm_startup_foc(void)
{
  if (app_state == SINE_DRIVE) {
    write_sine_pwm(manual_settings.amplitude_permille);
    return;
  }

  if (app_state != FOC_STARTUP) return;

  uint32_t duration_ms = get_startup_duration();
  uint32_t elapsed_ms = HAL_GetTick() - startup_start_tick;
  uint32_t progress_permille = elapsed_ms >= duration_ms
      ? 1000U
      : elapsed_ms * 1000U / duration_ms;
  uint32_t frequency_millihz = interpolate_u32(
      get_startup_initial_frequency(),
      get_startup_final_frequency(),
      progress_permille);
  uint16_t amplitude_permille = interpolate_u16(
      get_startup_initial_amplitude(),
      get_startup_final_amplitude(),
      progress_permille);

  configure_sine_frequency(frequency_millihz);
  write_sine_pwm(amplitude_permille);
  if (progress_permille < 1000U) return;

  HAL_TIM_Base_Stop_IT(&htim4);
  uint32_t primask = __get_PRIMASK();
  __disable_irq();
  bldc_set_pwm((uint16_t)(max_pwm * 0.45f)); // 45% duty cycle inicial
  bldc_disable_power_stage();
  PWM_STOP();
  PWM_INIT();
  __HAL_TIM_CLEAR_FLAG(&htim2,
                       TIM_FLAG_CC1 | TIM_FLAG_CC2 | TIM_FLAG_CC3 |
                           TIM_FLAG_CC1OF | TIM_FLAG_CC2OF | TIM_FLAG_CC3OF);
  motor_control_arm_zc_blanking();
  floating_U = true;
  floating_V = true;
  floating_W = true;
  app_state = RUNNING;
  phase_counter = 0U;
  if (primask == 0U) __enable_irq();
}
