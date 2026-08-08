#include "current_sense.h"

#include "adc.h"
#include "bldc_driver.h"
#include "motor_control.h"
#include "state_machine.h"
#include "tim.h"

#define CURRENT_SHUNT_MILLIOHM 470U
#define CURRENT_NOMINAL_VDDA_MV 3300U
#define CURRENT_EXPECTED_MAX_MA 2000U
#define CURRENT_ADC_FULL_SCALE 4095U
#define CURRENT_ADC_SATURATION 4090U
#define CURRENT_EDGE_MARGIN_COUNTS 64U

typedef struct {
  volatile uint16_t raw_adc;
  volatile int32_t current_ma;
  volatile uint16_t valid_samples;
  volatile uint8_t flags;
} CurrentSenseRuntime;

static volatile CurrentSenseRuntime phase_u = {0};
static volatile CurrentSenseRuntime phase_v = {0};
static volatile uint16_t sample_compare = 1U;
static volatile int8_t sample_commutation_step = POS_INIT;
static volatile uint32_t sample_tick_ms = 0U;
static bool calibrated = false;

static int32_t raw_to_ma(uint16_t raw_adc)
{
  uint64_t numerator = (uint64_t)raw_adc * CURRENT_NOMINAL_VDDA_MV * 1000U;
  uint32_t denominator = CURRENT_ADC_FULL_SCALE * CURRENT_SHUNT_MILLIOHM;
  return (int32_t)((numerator + denominator / 2U) / denominator);
}

static bool phase_u_is_low(int8_t step)
{
  return step == POS_VU || step == POS_WU;
}

static bool phase_v_is_low(int8_t step)
{
  return step == POS_UV || step == POS_WV;
}

static void update_channel(volatile CurrentSenseRuntime* channel,
                           uint16_t raw_adc,
                           bool conduction_valid,
                           bool window_valid)
{
  int32_t current_ma = raw_to_ma(raw_adc);
  uint8_t flags = calibrated ? CURRENT_SENSE_FLAG_CALIBRATED : 0U;

  if (conduction_valid && window_valid) {
    flags |= CURRENT_SENSE_FLAG_VALID;
    if (channel->valid_samples != UINT16_MAX) channel->valid_samples++;
  }
  if (raw_adc >= CURRENT_ADC_SATURATION) flags |= CURRENT_SENSE_FLAG_SATURATED;
  if (current_ma > (int32_t)CURRENT_EXPECTED_MAX_MA) {
    flags |= CURRENT_SENSE_FLAG_OVERCURRENT;
  }

  channel->raw_adc = raw_adc;
  channel->current_ma = current_ma;
  channel->flags = flags;
}

void current_sense_init(void)
{
  if (HAL_ADCEx_Calibration_Start(&hadc2) != HAL_OK ||
      HAL_ADCEx_Calibration_Start(&hadc1) != HAL_OK ||
      HAL_ADCEx_InjectedStart(&hadc2) != HAL_OK ||
      HAL_ADCEx_InjectedStart_IT(&hadc1) != HAL_OK) {
    Error_Handler();
  }
  calibrated = true;

  current_sense_update_pwm_window(2U * CURRENT_EDGE_MARGIN_COUNTS);
  if (HAL_TIM_OC_Start(&htim1, TIM_CHANNEL_4) != HAL_OK) {
    Error_Handler();
  }
}

void current_sense_update_pwm_window(uint16_t active_compare)
{
  uint16_t arr = (uint16_t)TIM1->ARR;
  uint16_t compare = active_compare / 2U;

  if (compare == 0U) compare = 1U;
  if (compare >= arr) compare = arr > 1U ? (uint16_t)(arr - 1U) : 1U;
  sample_compare = compare;
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, compare);
}

void current_sense_handle_conversion(void)
{
  uint16_t raw_u = (uint16_t)HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_1);
  uint16_t raw_v = (uint16_t)HAL_ADCEx_InjectedGetValue(&hadc2, ADC_INJECTED_RANK_1);
  int8_t step = bldc_get_commutation_step();
  bool running = app_state == RUNNING || app_state == CLOSEDLOOP;
  uint16_t arr = (uint16_t)TIM1->ARR;
  bool window_valid = sample_compare >= CURRENT_EDGE_MARGIN_COUNTS &&
                      arr > CURRENT_EDGE_MARGIN_COUNTS &&
                      sample_compare <= (uint16_t)(arr - CURRENT_EDGE_MARGIN_COUNTS) &&
                      bldc_get_pwm() >= (uint16_t)(2U * CURRENT_EDGE_MARGIN_COUNTS);

  update_channel(&phase_u, raw_u, running && phase_u_is_low(step), window_valid);
  update_channel(&phase_v, raw_v, running && phase_v_is_low(step), window_valid);
  sample_commutation_step = step;
  sample_tick_ms = HAL_GetTick();
}

void current_sense_get_snapshot(CurrentSenseSnapshot* snapshot)
{
  if (snapshot == NULL) return;

  uint32_t primask = __get_PRIMASK();
  __disable_irq();
  snapshot->phase_u.raw_adc = phase_u.raw_adc;
  snapshot->phase_u.current_ma = phase_u.current_ma;
  snapshot->phase_u.valid_samples = phase_u.valid_samples;
  snapshot->phase_u.flags = phase_u.flags;
  snapshot->phase_u.commutation_step = sample_commutation_step;
  snapshot->phase_u.tick_ms = sample_tick_ms;
  snapshot->phase_v.raw_adc = phase_v.raw_adc;
  snapshot->phase_v.current_ma = phase_v.current_ma;
  snapshot->phase_v.valid_samples = phase_v.valid_samples;
  snapshot->phase_v.flags = phase_v.flags;
  snapshot->phase_v.commutation_step = sample_commutation_step;
  snapshot->phase_v.tick_ms = sample_tick_ms;
  if (primask == 0U) __enable_irq();
}
