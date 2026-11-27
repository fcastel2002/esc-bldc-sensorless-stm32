/*
 * load_sweep.c
 *
 * Staircase / ramp load-curve sweep in closed loop. Runs from the main loop
 * (load_sweep_process); the only ISR-shared state it touches is the speed
 * target (uint16, atomic) and the slew-limit override owned by motor_control.
 */

#include "load_sweep.h"

#include "hard_config.h"
#include "motor_control.h"
#include "power_monitor.h"
#include "speed_sensor.h"
#include "startup.h"
#include "state_machine.h"

#include <string.h>

#pragma GCC optimize("Os")

#define DUTY_SATURATED_E4 9950U
#define IDLE_EMA_SHIFT 3U
#define INA_SAMPLE_MS POWER_MONITOR_POLL_MS

typedef struct {
  uint32_t n;
  uint32_t sum_reference;
  uint32_t sum_duty;
  uint16_t duty0;
  int32_t  sum_dduty;
  uint64_t sum_dduty2;
  uint16_t rpm0;
  int32_t  sum_drpm;
  uint64_t sum_drpm2;

  uint32_t np;
  uint32_t sum_mv;
  int32_t  i0_x10;
  int64_t  sum_i_x10;
  int64_t  sum_di;
  uint64_t sum_di2;

  uint8_t flags;
} WindowAccumulator;

static LoadSweepPoint points[LOAD_SWEEP_CAPACITY];
static uint16_t point_count = 0U;

static LoadSweepConfig sweep_config;
static uint8_t  sweep_state = LOAD_SWEEP_STATE_IDLE;
static uint8_t  abort_reason = LOAD_SWEEP_ABORT_NONE;
static uint32_t start_tick = 0U;
static uint32_t elapsed_ms = 0U;
static uint32_t phase_tick = 0U;
static uint16_t step_index = 0U;
static uint16_t restart_snapshot = 0U;
static bool     ramp_running = false;

static WindowAccumulator window;
static uint32_t window_start_tick = 0U;
static uint32_t last_sample_tick = 0U;
static uint32_t last_power_tick = 0U;

/* Idle current (power stage off) tracking and the value latched for a sweep. */
static int32_t idle_ema_ua = 0;
static bool    idle_ema_valid = false;
static int32_t sweep_iq_ua = 0;
static bool    sweep_iq_valid = false;
static int64_t iq_sum_ua = 0;
static uint32_t iq_count = 0U;

static uint32_t isqrt_u32(uint32_t value)
{
  uint32_t result = 0U;
  uint32_t bit = 1UL << 30;
  while (bit > value) bit >>= 2;
  while (bit != 0U) {
    if (value >= result + bit) {
      value -= result + bit;
      result = (result >> 1) + bit;
    } else {
      result >>= 1;
    }
    bit >>= 2;
  }
  return result;
}

static uint32_t std_from_sums(int64_t sum, uint64_t sum_sq, uint32_t n)
{
  if (n < 2U) return 0U;
  int64_t centered = (int64_t)sum_sq - (sum * sum) / (int64_t)n;
  if (centered <= 0) return 0U;
  uint64_t variance = (uint64_t)centered / n;
  return isqrt_u32(variance > UINT32_MAX ? UINT32_MAX : (uint32_t)variance);
}

static int16_t clamp_i16(int64_t value)
{
  if (value > INT16_MAX) return INT16_MAX;
  if (value < INT16_MIN) return INT16_MIN;
  return (int16_t)value;
}

static bool fresh_power_sample(PowerMonitorSnapshot* power)
{
  power_monitor_get_snapshot(power);
  if ((power->flags & POWER_FLAG_VALID) == 0U || power->sample_tick == last_power_tick) {
    return false;
  }
  last_power_tick = power->sample_tick;
  return true;
}

static void track_idle_current(void)
{
  PowerMonitorSnapshot power;
  if (app_state != IDLE || !fresh_power_sample(&power)) return;
  if (!idle_ema_valid) {
    idle_ema_ua = power.current_ua;
    idle_ema_valid = true;
  } else {
    idle_ema_ua += (power.current_ua - idle_ema_ua) / (1 << IDLE_EMA_SHIFT);
  }
}

static uint16_t step_count(void)
{
  uint16_t span = sweep_config.end_rpm > sweep_config.start_rpm
      ? (uint16_t)(sweep_config.end_rpm - sweep_config.start_rpm)
      : (uint16_t)(sweep_config.start_rpm - sweep_config.end_rpm);
  return (uint16_t)((span + sweep_config.step_or_rate - 1U) / sweep_config.step_or_rate + 1U);
}

static uint16_t staircase_target(uint16_t index)
{
  uint32_t offset = (uint32_t)index * sweep_config.step_or_rate;
  if (sweep_config.end_rpm >= sweep_config.start_rpm) {
    uint32_t target = sweep_config.start_rpm + offset;
    return target > sweep_config.end_rpm ? sweep_config.end_rpm : (uint16_t)target;
  }
  return offset >= (uint32_t)(sweep_config.start_rpm - sweep_config.end_rpm)
      ? sweep_config.end_rpm
      : (uint16_t)(sweep_config.start_rpm - offset);
}

static void begin_motion(uint16_t target_rpm)
{
  speed_setpoint_rpm = target_rpm;
  sweep_state = LOAD_SWEEP_STATE_MOVING;
}

static void begin_window(uint32_t now)
{
  memset(&window, 0, sizeof(window));
  window_start_tick = now;
  last_sample_tick = now - LOAD_SWEEP_SAMPLE_MS;
  PowerMonitorSnapshot power;
  (void)fresh_power_sample(&power); /* Only count INA219 samples taken inside the window. */
}

static void accumulate_speed(uint32_t now)
{
  if ((uint32_t)(now - last_sample_tick) < LOAD_SWEEP_SAMPLE_MS) return;
  last_sample_tick = now;

  uint16_t rpm = speed_sensor_get_speed_rpm();
  uint16_t duty = motor_control_get_duty_e4();
  if (window.n == 0U) {
    window.rpm0 = rpm;
    window.duty0 = duty;
  }
  int32_t drpm = (int32_t)rpm - window.rpm0;
  int32_t dduty = (int32_t)duty - window.duty0;
  window.sum_dduty += dduty;
  window.sum_dduty2 += (uint64_t)((int64_t)dduty * dduty);
  window.n++;
  window.sum_reference += motor_control_get_reference_rpm();
  window.sum_duty += duty;
  window.sum_drpm += drpm;
  window.sum_drpm2 += (uint64_t)((int64_t)drpm * drpm);
  if (duty >= DUTY_SATURATED_E4) window.flags |= LOAD_POINT_FLAG_DUTY_SATURATED;
}

static void accumulate_power(void)
{
  PowerMonitorSnapshot power;
  if (!fresh_power_sample(&power)) return;
  int32_t i_x10 = power.current_ua / 100;
  if (window.np == 0U) window.i0_x10 = i_x10;
  int64_t di = (int64_t)i_x10 - window.i0_x10;
  window.np++;
  window.sum_mv += power.bus_mv;
  window.sum_i_x10 += i_x10;
  window.sum_di += di;
  window.sum_di2 += (uint64_t)(di * di);
}

static void finalize_point(void)
{
  LoadSweepPoint point;
  memset(&point, 0, sizeof(point));
  uint32_t n = window.n == 0U ? 1U : window.n;
  uint8_t flags = window.flags;

  point.reference_rpm = (uint16_t)(window.sum_reference / n);
  point.rpm_mean = (uint16_t)((int32_t)window.rpm0 + window.sum_drpm / (int32_t)n);
  uint32_t rpm_std = std_from_sums(window.sum_drpm, window.sum_drpm2, window.n);
  point.rpm_std = rpm_std > 255U ? 255U : (uint8_t)rpm_std;
  point.duty_e4 = (uint16_t)(window.sum_duty / n);
  uint32_t duty_std = std_from_sums(window.sum_dduty, window.sum_dduty2, window.n);
  point.duty_std_e4 = duty_std > UINT16_MAX ? UINT16_MAX : (uint16_t)duty_std;

  if (window.np == 0U) {
    flags |= LOAD_POINT_FLAG_NO_POWER;
  } else {
    uint32_t expected = sweep_config.window_ms / INA_SAMPLE_MS;
    if (window.np * 2U < expected) flags |= LOAD_POINT_FLAG_PARTIAL_POWER;
    point.bus_mv = (uint16_t)(window.sum_mv / window.np);
    int64_t mean_i_x10 = window.sum_i_x10 / (int64_t)window.np;
    point.bus_ma_x10 = clamp_i16(mean_i_x10);
    uint32_t i_std = std_from_sums(window.sum_di, window.sum_di2, window.np);
    point.bus_std_ma_x10 = i_std > UINT16_MAX ? UINT16_MAX : (uint16_t)i_std;
  }
  point.flags = flags;

  if (point_count < LOAD_SWEEP_CAPACITY) {
    points[point_count++] = point;
  }
}

static void finish(LoadSweepAbortReason reason)
{
  uint32_t now = HAL_GetTick();
  bool was_ramp = ramp_running;
  ramp_running = false;
  motor_control_set_ramp_override(0U);
  if (was_ramp) {
    /* Freeze at the current reference so clearing the override cannot jump to end_rpm. */
    uint16_t reference = motor_control_get_reference_rpm();
    if (reference != 0U) speed_setpoint_rpm = reference;
  }

  abort_reason = reason;
  sweep_state = reason == LOAD_SWEEP_ABORT_NONE ? LOAD_SWEEP_STATE_DONE
                                                : LOAD_SWEEP_STATE_ABORTED;
  elapsed_ms = now - start_tick;

  bool stop = reason == LOAD_SWEEP_ABORT_MOTOR_LOST ||
              reason == LOAD_SWEEP_ABORT_STARTUP_TIMEOUT ||
              (reason == LOAD_SWEEP_ABORT_NONE &&
               (sweep_config.options & LOAD_SWEEP_OPT_STOP_AT_END) != 0U);
  if (stop && app_state != IDLE) {
    stop_motor(0);
  }
}

bool load_sweep_is_active(void)
{
  return sweep_state >= LOAD_SWEEP_STATE_MEASURING_IQ && sweep_state <= LOAD_SWEEP_STATE_MEASURING;
}

LoadSweepStartResult load_sweep_start(const LoadSweepConfig* config)
{
  if (config == NULL) return LOAD_SWEEP_START_UNKNOWN_PARAM;
  if (config->mode > LOAD_SWEEP_MODE_RAMP) return LOAD_SWEEP_START_UNKNOWN_PARAM;
  if (config->start_rpm < get_min_speed() || config->end_rpm < get_min_speed()) {
    return LOAD_SWEEP_START_UNDERLIMIT;
  }
  if (config->start_rpm > get_max_speed() || config->end_rpm > get_max_speed()) {
    return LOAD_SWEEP_START_OVERLIMIT;
  }
  if (config->window_ms < LOAD_SWEEP_MIN_WINDOW_MS) return LOAD_SWEEP_START_UNDERLIMIT;
  if (config->window_ms > LOAD_SWEEP_MAX_WINDOW_MS ||
      config->settle_ms > LOAD_SWEEP_MAX_SETTLE_MS) {
    return LOAD_SWEEP_START_OVERLIMIT;
  }

  uint32_t span = config->end_rpm > config->start_rpm
      ? (uint32_t)(config->end_rpm - config->start_rpm)
      : (uint32_t)(config->start_rpm - config->end_rpm);
  if (config->mode == LOAD_SWEEP_MODE_STAIRCASE) {
    if (config->step_or_rate < LOAD_SWEEP_MIN_STEP_RPM) return LOAD_SWEEP_START_UNDERLIMIT;
    if ((span + config->step_or_rate - 1U) / config->step_or_rate + 1U > LOAD_SWEEP_CAPACITY) {
      return LOAD_SWEEP_START_OVERLIMIT;
    }
  } else {
    if (span == 0U || config->step_or_rate < LOAD_SWEEP_MIN_RAMP_RPM_S) {
      return LOAD_SWEEP_START_UNDERLIMIT;
    }
    if (config->step_or_rate > LOAD_SWEEP_MAX_RAMP_RPM_S) return LOAD_SWEEP_START_OVERLIMIT;
    uint32_t duration_ms = span * 1000U / config->step_or_rate;
    if (duration_ms / config->window_ms + 1U > LOAD_SWEEP_CAPACITY) {
      return LOAD_SWEEP_START_OVERLIMIT;
    }
  }

  if (load_sweep_is_active() || (app_state != IDLE && app_state != CLOSEDLOOP)) {
    return LOAD_SWEEP_START_INVALID_STATE;
  }

  sweep_config = *config;
  sweep_config.options &= LOAD_SWEEP_OPT_STOP_AT_END;
  point_count = 0U;
  step_index = 0U;
  ramp_running = false;
  abort_reason = LOAD_SWEEP_ABORT_NONE;
  start_tick = HAL_GetTick();
  elapsed_ms = 0U;
  phase_tick = start_tick;

  if (app_state == IDLE) {
    iq_sum_ua = 0;
    iq_count = 0U;
    sweep_state = LOAD_SWEEP_STATE_MEASURING_IQ;
  } else {
    sweep_iq_ua = idle_ema_ua;
    sweep_iq_valid = idle_ema_valid;
    restart_snapshot = motor_restart_count;
    begin_motion(sweep_config.start_rpm);
  }
  return LOAD_SWEEP_START_OK;
}

void load_sweep_abort(LoadSweepAbortReason reason)
{
  if (load_sweep_is_active()) {
    finish(reason);
  }
}

void load_sweep_process(void)
{
  if (!load_sweep_is_active()) {
    track_idle_current();
    return;
  }

  uint32_t now = HAL_GetTick();
  if (sweep_state >= LOAD_SWEEP_STATE_MOVING &&
      (app_state != CLOSEDLOOP || motor_restart_count != restart_snapshot)) {
    finish(LOAD_SWEEP_ABORT_MOTOR_LOST);
    return;
  }

  switch (sweep_state) {
  case LOAD_SWEEP_STATE_MEASURING_IQ: {
    PowerMonitorSnapshot power;
    if (fresh_power_sample(&power)) {
      iq_sum_ua += power.current_ua;
      iq_count++;
    }
    if ((uint32_t)(now - phase_tick) < LOAD_SWEEP_IQ_MS) break;
    if (iq_count != 0U) {
      sweep_iq_ua = (int32_t)(iq_sum_ua / (int64_t)iq_count);
      sweep_iq_valid = true;
      idle_ema_ua = sweep_iq_ua;
      idle_ema_valid = true;
    } else {
      sweep_iq_ua = idle_ema_ua;
      sweep_iq_valid = idle_ema_valid;
    }
    speed_setpoint_rpm = sweep_config.start_rpm;
    foc_startup();
    phase_tick = now;
    sweep_state = LOAD_SWEEP_STATE_STARTING;
    break;
  }

  case LOAD_SWEEP_STATE_STARTING:
    if (app_state == CLOSEDLOOP) {
      restart_snapshot = motor_restart_count;
      begin_motion(sweep_config.start_rpm);
    } else if ((uint32_t)(now - phase_tick) > LOAD_SWEEP_STARTUP_TIMEOUT_MS) {
      finish(LOAD_SWEEP_ABORT_STARTUP_TIMEOUT);
    }
    break;

  case LOAD_SWEEP_STATE_MOVING:
    if (motor_control_reference_at_target()) {
      phase_tick = now;
      sweep_state = LOAD_SWEEP_STATE_SETTLING;
    }
    break;

  case LOAD_SWEEP_STATE_SETTLING:
    if ((uint32_t)(now - phase_tick) < sweep_config.settle_ms) break;
    if (sweep_config.mode == LOAD_SWEEP_MODE_RAMP) {
      motor_control_set_ramp_override(sweep_config.step_or_rate);
      speed_setpoint_rpm = sweep_config.end_rpm;
      ramp_running = true;
    }
    begin_window(now);
    sweep_state = LOAD_SWEEP_STATE_MEASURING;
    break;

  case LOAD_SWEEP_STATE_MEASURING:
    accumulate_speed(now);
    accumulate_power();
    if ((uint32_t)(now - window_start_tick) < sweep_config.window_ms) break;

    finalize_point();
    if (sweep_config.mode == LOAD_SWEEP_MODE_STAIRCASE) {
      step_index++;
      if (step_index >= step_count()) {
        finish(LOAD_SWEEP_ABORT_NONE);
      } else if (point_count >= LOAD_SWEEP_CAPACITY) {
        finish(LOAD_SWEEP_ABORT_TABLE_FULL);
      } else {
        begin_motion(staircase_target(step_index));
      }
    } else if (motor_control_reference_at_target()) {
      finish(LOAD_SWEEP_ABORT_NONE);
    } else if (point_count >= LOAD_SWEEP_CAPACITY) {
      finish(LOAD_SWEEP_ABORT_TABLE_FULL);
    } else {
      begin_window(now);
    }
    break;

  default:
    break;
  }
}

void load_sweep_get_status(LoadSweepStatus* status)
{
  if (status == NULL) return;
  status->state = sweep_state;
  status->abort_reason = abort_reason;
  status->config = sweep_config;
  status->point_count = point_count;
  status->target_rpm = speed_setpoint_rpm;
  if (sweep_state == LOAD_SWEEP_STATE_IDLE) {
    status->idle_current_ua = idle_ema_ua;
    status->idle_current_valid = idle_ema_valid;
  } else {
    status->idle_current_ua = sweep_iq_ua;
    status->idle_current_valid = sweep_iq_valid;
  }
  status->elapsed_ms = load_sweep_is_active() ? HAL_GetTick() - start_tick : elapsed_ms;
}

bool load_sweep_get_point(uint16_t index, LoadSweepPoint* point)
{
  if (point == NULL || index >= point_count) return false;
  *point = points[index];
  return true;
}
