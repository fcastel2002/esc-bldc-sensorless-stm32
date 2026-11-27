#include "zc_diag.h"

#include "bldc_driver.h"
#include "hard_config.h"
#include "motor_control.h"
#include "state_machine.h"
#include "tim.h"

#include <string.h>

#pragma GCC optimize("Os")

#define ZC_DIAG_GLITCH_MIN_TICKS 10U
#define ZC_DIAG_MOTION_TIMEOUT_MS 600U
#define ZC_DIAG_PROBE_SAMPLES 64U
#define ZC_DIAG_PROBE_PHASE_TIMEOUT_MS 25U
#define ZC_DIAG_PROBE_WAIT_LOOPS 4000U

typedef struct {
  uint8_t  armed_rising[3];
  uint8_t  has_ch_ts[3];
  uint16_t last_ch_ts[3];
  uint16_t edges[3];
  uint16_t forward_steps;
  uint16_t reverse_steps;
  uint16_t invalid_steps;
  uint16_t glitch_count;
  uint8_t  seen_states_mask;
  uint8_t  has_last_event;
  uint8_t  last_seq_idx;
  uint16_t last_event_ts;
  uint32_t last_motion_tick_ms;
  uint16_t step_ticks[6];
  uint8_t  step_ticks_idx;
  uint8_t  step_ticks_count;
  uint8_t  recent_events[6];
  uint8_t  recent_count;
} ZcFreewheelRuntime;

static volatile ZcFreewheelRuntime diag = {0};

static const uint32_t hw_tim_channel[3] = {
    TIM_CHANNEL_1,
    TIM_CHANNEL_2,
    TIM_CHANNEL_3,
};

static uint8_t read_live_pins(void)
{
  uint8_t pins = 0U;
  if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == GPIO_PIN_SET) pins |= (1U << 0);
  if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1) == GPIO_PIN_SET) pins |= (1U << 1);
  if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_10) == GPIO_PIN_SET) pins |= (1U << 2);
  return pins;
}

static void sat_inc_u16(volatile uint16_t* value)
{
  if (*value != UINT16_MAX) {
    (*value)++;
  }
}

static uint8_t find_forward_seq_index(uint8_t hw_ch, uint8_t is_rising)
{
  uint8_t u = ESC_DEFAULT_ZC_CHANNEL_U;
  uint8_t v = ESC_DEFAULT_ZC_CHANNEL_V;
  uint8_t w = ESC_DEFAULT_ZC_CHANNEL_W;
  get_zc_channel_map(&u, &v, &w);

  if (hw_ch == u) return is_rising ? 0U : 3U;
  if (hw_ch == w) return is_rising ? 4U : 1U;
  if (hw_ch == v) return is_rising ? 2U : 5U;
  return 0U;
}

static void push_recent_event(uint8_t code)
{
  if (diag.recent_count < 6U) {
    diag.recent_events[diag.recent_count++] = code;
    return;
  }
  for (uint8_t i = 0U; i < 5U; i++) {
    diag.recent_events[i] = diag.recent_events[i + 1U];
  }
  diag.recent_events[5] = code;
}

bool zc_diag_freewheel_start(void)
{
  if (app_state != IDLE && app_state != FREEWHEEL) {
    return false;
  }

  HAL_TIM_OC_Stop_IT(&htim4, TIM_CHANNEL_1);
  HAL_TIM_Base_Stop_IT(&htim4);
  bldc_disable_power_stage();
  motor_control_reset_runtime();

  uint32_t primask = __get_PRIMASK();
  __disable_irq();
  memset((void*)&diag, 0, sizeof(diag));

  uint8_t pins = read_live_pins();
  for (uint8_t ch = 0U; ch < 3U; ch++) {
    uint8_t rising = ((pins & (1U << ch)) == 0U) ? 1U : 0U;
    diag.armed_rising[ch] = rising;
    __HAL_TIM_SET_CAPTUREPOLARITY(
        &htim2,
        hw_tim_channel[ch],
        rising ? TIM_INPUTCHANNELPOLARITY_RISING : TIM_INPUTCHANNELPOLARITY_FALLING);
  }
  __HAL_TIM_CLEAR_FLAG(&htim2, TIM_FLAG_CC1 | TIM_FLAG_CC2 | TIM_FLAG_CC3);
  app_state = FREEWHEEL;
  if (primask == 0U) __enable_irq();

  return true;
}

void zc_diag_freewheel_stop(void)
{
  uint32_t primask = __get_PRIMASK();
  __disable_irq();
  for (uint8_t ch = 0U; ch < 3U; ch++) {
    __HAL_TIM_SET_CAPTUREPOLARITY(&htim2, hw_tim_channel[ch], TIM_INPUTCHANNELPOLARITY_FALLING);
  }
  __HAL_TIM_CLEAR_FLAG(&htim2, TIM_FLAG_CC1 | TIM_FLAG_CC2 | TIM_FLAG_CC3);
  if (app_state == FREEWHEEL) {
    app_state = IDLE;
  }
  if (primask == 0U) __enable_irq();
  bldc_disable_power_stage();
}

bool zc_diag_is_freewheel_active(void)
{
  return app_state == FREEWHEEL;
}

void zc_diag_handle_tim2_capture(TIM_HandleTypeDef* htim)
{
  uint8_t ch = 0xFFU;
  if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1) {
    ch = 0U;
  } else if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2) {
    ch = 1U;
  } else if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_3) {
    ch = 2U;
  } else {
    return;
  }

  uint16_t ts = (uint16_t)HAL_TIM_ReadCapturedValue(htim, hw_tim_channel[ch]);
  uint8_t was_rising = diag.armed_rising[ch];
  uint8_t next_rising = was_rising ? 0U : 1U;
  diag.armed_rising[ch] = next_rising;
  __HAL_TIM_SET_CAPTUREPOLARITY(
      htim,
      hw_tim_channel[ch],
      next_rising ? TIM_INPUTCHANNELPOLARITY_RISING : TIM_INPUTCHANNELPOLARITY_FALLING);

  if (diag.has_ch_ts[ch]) {
    uint16_t dt_ch = (uint16_t)(ts - diag.last_ch_ts[ch]);
    if (dt_ch < ZC_DIAG_GLITCH_MIN_TICKS) {
      sat_inc_u16(&diag.glitch_count);
      return;
    }
  }
  diag.has_ch_ts[ch] = 1U;
  diag.last_ch_ts[ch] = ts;

  sat_inc_u16(&diag.edges[ch]);
  uint8_t pins = read_live_pins();
  diag.seen_states_mask |= (uint8_t)(1U << pins);

  uint8_t event_code = (uint8_t)(0x80U | (was_rising ? 0x04U : 0x00U) | (ch & 0x03U));
  push_recent_event(event_code);

  uint8_t seq_idx = find_forward_seq_index(ch, was_rising);
  if (diag.has_last_event) {
    uint8_t delta = (uint8_t)((seq_idx + 6U - diag.last_seq_idx) % 6U);
    uint16_t dt = (uint16_t)(ts - diag.last_event_ts);
    if (delta == 1U) {
      sat_inc_u16(&diag.forward_steps);
    } else if (delta == 5U) {
      sat_inc_u16(&diag.reverse_steps);
    } else {
      sat_inc_u16(&diag.invalid_steps);
    }

    if ((delta == 1U || delta == 5U) && dt >= ZC_DIAG_GLITCH_MIN_TICKS) {
      diag.step_ticks[diag.step_ticks_idx] = dt;
      diag.step_ticks_idx = (uint8_t)((diag.step_ticks_idx + 1U) % 6U);
      if (diag.step_ticks_count < 6U) {
        diag.step_ticks_count++;
      }
      diag.last_motion_tick_ms = HAL_GetTick();
    }
  }

  diag.last_seq_idx = seq_idx;
  diag.last_event_ts = ts;
  diag.has_last_event = 1U;
}

void zc_diag_get_report(ZcDiagReport* report)
{
  if (report == NULL) return;

  ZcFreewheelRuntime snap;
  uint8_t active = 0U;
  uint32_t primask = __get_PRIMASK();
  __disable_irq();
  memcpy(&snap, (const void*)&diag, sizeof(snap));
  active = (app_state == FREEWHEEL) ? 1U : 0U;
  if (primask == 0U) __enable_irq();

  memset(report, 0, sizeof(*report));
  report->active = active;
  report->live_pins = read_live_pins();
  report->seen_states_mask = snap.seen_states_mask;
  report->edges[0] = snap.edges[0];
  report->edges[1] = snap.edges[1];
  report->edges[2] = snap.edges[2];
  report->forward_steps = snap.forward_steps;
  report->reverse_steps = snap.reverse_steps;
  report->invalid_steps = snap.invalid_steps;
  report->glitch_count = snap.glitch_count;
  get_zc_channel_map(&report->active_map[0], &report->active_map[1], &report->active_map[2]);
  memcpy(report->recent_events, snap.recent_events, sizeof(report->recent_events));

  uint32_t now_ms = HAL_GetTick();
  if (snap.step_ticks_count > 0U &&
      (uint32_t)(now_ms - snap.last_motion_tick_ms) <= ZC_DIAG_MOTION_TIMEOUT_MS) {
    uint32_t sum = 0U;
    uint16_t min_t = UINT16_MAX;
    uint16_t max_t = 0U;
    for (uint8_t i = 0U; i < snap.step_ticks_count; i++) {
      uint16_t v = snap.step_ticks[i];
      sum += v;
      if (v < min_t) min_t = v;
      if (v > max_t) max_t = v;
    }
    uint16_t avg = (uint16_t)(sum / snap.step_ticks_count);
    report->avg_step_ticks = avg;
    if (avg > 0U) {
      uint16_t dev_hi = max_t > avg ? (uint16_t)(max_t - avg) : 0U;
      uint16_t dev_lo = avg > min_t ? (uint16_t)(avg - min_t) : 0U;
      uint16_t max_dev = dev_hi > dev_lo ? dev_hi : dev_lo;
      uint32_t asym = ((uint32_t)max_dev * 100U) / avg;
      report->step_asymmetry_pct = asym > 255U ? 255U : (uint8_t)asym;

      uint32_t timer_hz = HAL_RCC_GetHCLKFreq() / (TIM2->PSC + 1U);
      uint8_t pole_pairs = get_pole_pairs();
      if (pole_pairs == 0U) pole_pairs = 1U;
      uint32_t rpm = (timer_hz * 10U) / ((uint32_t)avg * pole_pairs);
      report->estimated_rpm = rpm > UINT16_MAX ? UINT16_MAX : (uint16_t)rpm;
    }
  }

  uint32_t total_edges = (uint32_t)snap.edges[0] + snap.edges[1] + snap.edges[2];
  uint16_t max_ch = snap.edges[0];
  if (snap.edges[1] > max_ch) max_ch = snap.edges[1];
  if (snap.edges[2] > max_ch) max_ch = snap.edges[2];

  uint8_t flags = 0U;
  if (total_edges >= 6U) {
    flags |= ZC_DIAG_HEALTH_HAS_MOTION;
  }
  if (max_ch >= 6U) {
    if (snap.edges[0] == 0U) flags |= ZC_DIAG_HEALTH_CH0_DEAD;
    if (snap.edges[1] == 0U) flags |= ZC_DIAG_HEALTH_CH1_DEAD;
    if (snap.edges[2] == 0U) flags |= ZC_DIAG_HEALTH_CH2_DEAD;
  }

  uint32_t total_steps =
      (uint32_t)snap.forward_steps + snap.reverse_steps + snap.invalid_steps;
  uint8_t any_dead =
      (flags & (ZC_DIAG_HEALTH_CH0_DEAD | ZC_DIAG_HEALTH_CH1_DEAD | ZC_DIAG_HEALTH_CH2_DEAD)) != 0U;

  if (total_steps >= 6U) {
    if ((uint32_t)snap.forward_steps > snap.reverse_steps &&
        (uint32_t)snap.forward_steps * 100U >= total_steps * 70U) {
      flags |= ZC_DIAG_HEALTH_DIR_FORWARD;
    } else if ((uint32_t)snap.reverse_steps > snap.forward_steps &&
               (uint32_t)snap.reverse_steps * 100U >= total_steps * 70U) {
      flags |= ZC_DIAG_HEALTH_DIR_REVERSE;
    }

    if (!any_dead &&
        ((uint32_t)snap.forward_steps + snap.reverse_steps) >= 6U &&
        (uint32_t)snap.invalid_steps * 100U <= total_steps * 15U) {
      flags |= ZC_DIAG_HEALTH_SEQ_VALID;
    }

    if ((uint32_t)snap.invalid_steps * 100U > total_steps * 15U) {
      flags |= ZC_DIAG_HEALTH_NOISE_OR_INVALID;
    }
  }

  if ((snap.glitch_count > 10U && (uint32_t)snap.glitch_count * 2U > total_edges) ||
      ((snap.seen_states_mask & 0x81U) != 0U && total_edges >= 12U)) {
    flags |= ZC_DIAG_HEALTH_NOISE_OR_INVALID;
  }

  report->health_flags = flags;
}

static void probe_single_phase(uint8_t phase_idx, uint16_t pwm_val, uint8_t high_pct_out[3])
{
  __HAL_TIM_SET_COMPARE(&htim1, IN_U, phase_idx == 0U ? pwm_val : 0U);
  __HAL_TIM_SET_COMPARE(&htim1, IN_V, phase_idx == 1U ? pwm_val : 0U);
  __HAL_TIM_SET_COMPARE(&htim1, IN_W, phase_idx == 2U ? pwm_val : 0U);

  HAL_Delay(5U);

  uint16_t counts[3] = {0U, 0U, 0U};
  uint16_t samples = 0U;
  uint16_t half_pwm = (uint16_t)(pwm_val / 2U);
  if (half_pwm == 0U) half_pwm = 1U;
  uint32_t start_ms = HAL_GetTick();

  while (samples < ZC_DIAG_PROBE_SAMPLES &&
         (uint32_t)(HAL_GetTick() - start_ms) < ZC_DIAG_PROBE_PHASE_TIMEOUT_MS) {
    uint32_t guard = 0U;
    while (guard < ZC_DIAG_PROBE_WAIT_LOOPS &&
           (((TIM1->CR1 & TIM_CR1_DIR) != 0U) || TIM1->CNT >= half_pwm)) {
      guard++;
    }

    uint8_t pins = read_live_pins();
    if ((pins & (1U << 0)) != 0U) counts[0]++;
    if ((pins & (1U << 1)) != 0U) counts[1]++;
    if ((pins & (1U << 2)) != 0U) counts[2]++;
    samples++;

    guard = 0U;
    while (guard < ZC_DIAG_PROBE_WAIT_LOOPS && TIM1->CNT < pwm_val) {
      guard++;
    }
  }

  PWM_STOP();
  HAL_Delay(2U);

  if (samples == 0U) {
    high_pct_out[0] = 0U;
    high_pct_out[1] = 0U;
    high_pct_out[2] = 0U;
    return;
  }

  for (uint8_t ch = 0U; ch < 3U; ch++) {
    uint32_t pct = ((uint32_t)counts[ch] * 100U + (samples / 2U)) / samples;
    high_pct_out[ch] = pct > 100U ? 100U : (uint8_t)pct;
  }
}

static bool extract_unique_map(const uint8_t pct[9], bool inverted, uint8_t out_map[3])
{
  uint8_t used = 0U;
  for (uint8_t p = 0U; p < 3U; p++) {
    uint8_t best_ch = 0U;
    uint8_t best_val = pct[p * 3U];
    uint8_t second_val = inverted ? 100U : 0U;

    for (uint8_t ch = 1U; ch < 3U; ch++) {
      uint8_t v = pct[p * 3U + ch];
      if ((!inverted && v > best_val) || (inverted && v < best_val)) {
        second_val = best_val;
        best_val = v;
        best_ch = ch;
      } else if ((!inverted && v > second_val) || (inverted && v < second_val)) {
        second_val = v;
      }
    }

    if (!inverted) {
      if (best_val < 65U || best_val < (uint8_t)(second_val + 30U)) return false;
    } else {
      if (best_val > 35U || second_val < (uint8_t)(best_val + 30U)) return false;
    }

    if ((used & (1U << best_ch)) != 0U) return false;
    used |= (uint8_t)(1U << best_ch);
    out_map[p] = best_ch;
  }
  return zc_channel_map_is_valid(out_map[0], out_map[1], out_map[2]);
}

bool zc_diag_auto_detect(uint16_t duty_permille, ZcAutoDetectResult* result)
{
  if (result == NULL) return false;
  if (app_state != IDLE) {
    return false;
  }

  if (duty_permille == 0U) {
    duty_permille = ESC_DEFAULT_ZC_AUTO_DETECT_DUTY_PERMILLE;
  }
  if (duty_permille < ESC_MIN_ZC_AUTO_DETECT_DUTY_PERMILLE ||
      duty_permille > ESC_MAX_ZC_AUTO_DETECT_DUTY_PERMILLE) {
    return false;
  }

  memset(result, 0, sizeof(*result));
  result->detected_map[0] = 0xFFU;
  result->detected_map[1] = 0xFFU;
  result->detected_map[2] = 0xFFU;
  get_zc_channel_map(&result->active_map[0], &result->active_map[1], &result->active_map[2]);

  uint16_t arr = (uint16_t)TIM1->ARR;
  uint16_t pwm_val = (uint16_t)(((uint32_t)arr * duty_permille) / 1000U);
  if (pwm_val < 20U) pwm_val = 20U;
  if (arr > 1U && pwm_val >= arr) pwm_val = (uint16_t)(arr - 1U);

  bldc_disable_power_stage();
  PWM_STOP();
  PWM_INIT();
  GPIOB->ODR |= EN_U | EN_V;
  GPIOA->ODR |= EN_W;

  for (uint8_t p = 0U; p < 3U; p++) {
    probe_single_phase(p, pwm_val, &result->high_pct[p * 3U]);
  }

  bldc_disable_power_stage();

  uint8_t detected[3] = {0xFFU, 0xFFU, 0xFFU};
  if (extract_unique_map(result->high_pct, false, detected)) {
    result->result_code = (uint8_t)ZC_AUTO_DETECT_OK;
    memcpy(result->detected_map, detected, sizeof(detected));
    return true;
  }
  if (extract_unique_map(result->high_pct, true, detected)) {
    result->result_code = (uint8_t)ZC_AUTO_DETECT_OK_INVERTED;
    memcpy(result->detected_map, detected, sizeof(detected));
    return true;
  }

  uint8_t min_all = 100U;
  uint8_t max_all = 0U;
  for (uint8_t i = 0U; i < 9U; i++) {
    if (result->high_pct[i] < min_all) min_all = result->high_pct[i];
    if (result->high_pct[i] > max_all) max_all = result->high_pct[i];
  }
  result->result_code =
      (max_all - min_all) < 20U ? (uint8_t)ZC_AUTO_DETECT_NO_SIGNAL : (uint8_t)ZC_AUTO_DETECT_AMBIGUOUS;
  return true;
}
