#include "comm_protocol.h"

#include "comm.h"
#include "comm_transport.h"
#include "flash_config.h"
#include "hard_config.h"
#include "load_sweep.h"
#include "motor_control.h"
#include "power_monitor.h"
#include "speed_sensor.h"
#include "startup.h"
#include "state_machine.h"
#include "zc_diag.h"

#include <string.h>

#pragma GCC optimize("Oz")

#define DEFAULT_PWM_FREQ_HZ 18000U
#define DEFAULT_POLE_PAIRS 2U
#define DEFAULT_KP_CENTI 28
#define DEFAULT_KI_CENTI 100
#define DEFAULT_KD_CENTI 0
#define DEFAULT_MAX_SPEED_RPM 5400U
#define DEFAULT_MIN_SPEED_RPM 200U

static uint16_t read_u16_le(const uint8_t* data)
{
  return (uint16_t)data[0] | ((uint16_t)data[1] << 8);
}

static int16_t read_i16_le(const uint8_t* data)
{
  return (int16_t)read_u16_le(data);
}

static uint32_t read_u32_le(const uint8_t* data)
{
  return (uint32_t)data[0] | ((uint32_t)data[1] << 8) |
         ((uint32_t)data[2] << 16) | ((uint32_t)data[3] << 24);
}

static void write_u16_le(uint8_t* data, uint16_t value)
{
  data[0] = (uint8_t)(value & 0xFFU);
  data[1] = (uint8_t)(value >> 8);
}

static void write_i16_le(uint8_t* data, int16_t value)
{
  write_u16_le(data, (uint16_t)value);
}

static void write_u32_le(uint8_t* data, uint32_t value)
{
  data[0] = (uint8_t)(value & 0xFFU);
  data[1] = (uint8_t)(value >> 8);
  data[2] = (uint8_t)(value >> 16);
  data[3] = (uint8_t)(value >> 24);
}

static int16_t float_to_centi(float value)
{
  return (int16_t)(value * 100.0f);
}

static float centi_to_float(int16_t value)
{
  return (float)value / 100.0f;
}

uint16_t comm_protocol_crc16(const uint8_t* data, uint16_t len)
{
  uint16_t crc = 0xFFFFU;

  for (uint16_t i = 0; i < len; i++) {
    crc ^= (uint16_t)data[i] << 8;
    for (uint8_t bit = 0; bit < 8; bit++) {
      if ((crc & 0x8000U) != 0U) {
        crc = (uint16_t)((crc << 1) ^ 0x1021U);
      } else {
        crc <<= 1;
      }
    }
  }

  return crc;
}

static void finish_frame(uint8_t frame[COMM_FRAME_SIZE], uint16_t payload_len)
{
  write_u16_le(&frame[8], payload_len);
  uint16_t crc = comm_protocol_crc16(frame, COMM_CRC_OFFSET);
  write_u16_le(&frame[COMM_CRC_OFFSET], crc);
}

static void build_base(uint8_t frame[COMM_FRAME_SIZE],
                       uint8_t type,
                       uint8_t seq,
                       uint8_t opcode,
                       uint8_t param,
                       uint8_t status)
{
  memset(frame, 0, COMM_FRAME_SIZE);
  frame[0] = COMM_MAGIC_0;
  frame[1] = COMM_MAGIC_1;
  frame[2] = COMM_VERSION;
  frame[3] = type;
  frame[4] = seq;
  frame[5] = opcode;
  frame[6] = param;
  frame[7] = status;
}

static void build_response(const uint8_t request[COMM_FRAME_SIZE],
                           uint8_t status,
                           const uint8_t* payload,
                           uint16_t payload_len,
                           uint8_t response[COMM_FRAME_SIZE])
{
  if (payload_len > COMM_PAYLOAD_MAX) {
    payload_len = COMM_PAYLOAD_MAX;
    status = COMM_STATUS_BAD_LENGTH;
  }

  build_base(response,
             COMM_TYPE_RESPONSE,
             request[4],
             request[5],
             request[6],
             status);

  if (payload != NULL && payload_len > 0U) {
    memcpy(&response[COMM_PAYLOAD_OFFSET], payload, payload_len);
  }

  finish_frame(response, payload_len);
}

static uint8_t status_from_config(ConfigStatus status)
{
  switch (status) {
  case CONFIG_OK:
    return COMM_STATUS_OK;
  case CONFIG_ERROR_UNDERLIMIT:
    return COMM_STATUS_UNDERLIMIT;
  case CONFIG_ERROR_OVERLIMIT:
    return COMM_STATUS_OVERLIMIT;
  case CONFIG_ERROR_NaN:
    return COMM_STATUS_BAD_LENGTH;
  default:
    return COMM_STATUS_NOT_IMPLEMENTED;
  }
}

static uint8_t status_from_flash(FlashResultCode status)
{
  switch (status) {
  case FLASH_RESULT_OK:
    return COMM_STATUS_OK;
  case FLASH_RESULT_WRITE_ERROR:
  case FLASH_RESULT_ERASE_ERROR:
  case FLASH_RESULT_CORRUPT:
    return COMM_STATUS_FLASH_ERROR;
  case FLASH_RESULT_EMPTY:
    return COMM_STATUS_INVALID_STATE;
  default:
    return COMM_STATUS_FLASH_ERROR;
  }
}

static bool stop_state_allows_command(void)
{
  return app_state == IDLE || app_state == FOC_STARTUP || app_state == RUNNING ||
         app_state == CLOSEDLOOP || app_state == SINE_DRIVE || app_state == STEP_TEST ||
         app_state == FREEWHEEL;
}

static bool manual_output_allows_opcode(uint8_t opcode)
{
  return opcode == COMM_OPCODE_PING || opcode == COMM_OPCODE_GET_STATUS ||
         opcode == COMM_OPCODE_STOP || opcode == COMM_OPCODE_ESTOP ||
         opcode == COMM_OPCODE_SINE_DRIVE || opcode == COMM_OPCODE_STEP_TEST ||
         (opcode >= COMM_OPCODE_LOG_START && opcode <= COMM_OPCODE_LOG_RATE);
}

static bool freewheel_allows_opcode(uint8_t opcode, uint8_t param)
{
  return opcode == COMM_OPCODE_PING || opcode == COMM_OPCODE_GET_STATUS ||
         opcode == COMM_OPCODE_STOP || opcode == COMM_OPCODE_ESTOP ||
         opcode == COMM_OPCODE_GET_CONFIG || opcode == COMM_OPCODE_ZC_DIAG ||
         ((opcode == COMM_OPCODE_SET_CONFIG || opcode == COMM_OPCODE_RESET_CONFIG) &&
          param == COMM_PARAM_ZC_MAP) ||
         (opcode >= COMM_OPCODE_LOG_START && opcode <= COMM_OPCODE_LOG_RATE);
}

static void stop_manual_outputs(void)
{
  if (sine_drive_is_active()) sine_drive_stop();
  if (step_test_is_active()) step_test_stop();
  if (zc_diag_is_freewheel_active()) zc_diag_freewheel_stop();
}

static bool is_startup_param(uint8_t param)
{
  return param >= COMM_PARAM_STARTUP_INITIAL_AMPLITUDE &&
         param <= COMM_PARAM_STARTUP_DURATION;
}

static bool is_idle_only_param(uint8_t param)
{
  return is_startup_param(param) || param == COMM_PARAM_BEMF_BLANKING_US ||
         param == COMM_PARAM_ZC_MAP;
}

static bool config_write_requires_idle(uint8_t param)
{
  return is_idle_only_param(param) || param == COMM_PARAM_ALL;
}

static uint16_t serialize_zc_diag_report(const ZcDiagReport* report, uint8_t* payload_out)
{
  payload_out[0] = report->active;
  payload_out[1] = report->live_pins;
  payload_out[2] = report->seen_states_mask;
  payload_out[3] = report->health_flags;
  write_u16_le(&payload_out[4], report->edges[0]);
  write_u16_le(&payload_out[6], report->edges[1]);
  write_u16_le(&payload_out[8], report->edges[2]);
  write_u16_le(&payload_out[10], report->forward_steps);
  write_u16_le(&payload_out[12], report->reverse_steps);
  write_u16_le(&payload_out[14], report->invalid_steps);
  write_u16_le(&payload_out[16], report->glitch_count);
  write_u16_le(&payload_out[18], report->estimated_rpm);
  write_u16_le(&payload_out[20], report->avg_step_ticks);
  payload_out[22] = report->step_asymmetry_pct;
  payload_out[23] = report->active_map[0];
  payload_out[24] = report->active_map[1];
  payload_out[25] = report->active_map[2];
  memcpy(&payload_out[26], report->recent_events, 6U);
  return 32U;
}

__attribute__((optimize("Oz"))) static void apply_config_change(uint8_t param)
{
  if (is_idle_only_param(param)) return;

  if (app_state == CLOSEDLOOP) {
    if (param == COMM_PARAM_KP_RPM || param == COMM_PARAM_KI_RPM || param == COMM_PARAM_KD_RPM ||
        param == COMM_PARAM_SPEED_RAMP) {
      updateAllMotorControl();
      return;
    }

    if (param == COMM_PARAM_PWM_FREQ) {
      uint16_t pwm = bldc_get_pwm();
      int8_t step = bldc_get_commutation_step();

      __disable_irq();
      update_all_esc();
      if (pwm > TIM1->ARR) {
        pwm = TIM1->ARR;
        bldc_set_pwm(pwm);
      }
      if (step <= POS_UW) {
        __HAL_TIM_SET_COMPARE(&htim1, IN_U, pwm);
      } else if (step <= POS_VU) {
        __HAL_TIM_SET_COMPARE(&htim1, IN_V, pwm);
      } else {
        __HAL_TIM_SET_COMPARE(&htim1, IN_W, pwm);
      }
      updateAllMotorControl();
      __enable_irq();
      return;
    }
  }

  esc_config_done = 0;
  motor_control_config_done = 0;
  app_state = CONFIG;
}

static uint8_t get_config(uint8_t param, uint8_t* payload, uint16_t* payload_len)
{
  *payload_len = 0;

  switch (param) {
  case COMM_PARAM_PWM_FREQ:
    write_u16_le(payload, get_pwm_freq());
    *payload_len = 2;
    return COMM_STATUS_OK;
  case COMM_PARAM_POLE_PAIRS:
    payload[0] = get_pole_pairs();
    *payload_len = 1;
    return COMM_STATUS_OK;
  case COMM_PARAM_KP_RPM:
    write_i16_le(payload, float_to_centi(get_KP()));
    *payload_len = 2;
    return COMM_STATUS_OK;
  case COMM_PARAM_KI_RPM:
    write_i16_le(payload, float_to_centi(get_KI()));
    *payload_len = 2;
    return COMM_STATUS_OK;
  case COMM_PARAM_KD_RPM:
    write_i16_le(payload, float_to_centi(get_KD()));
    *payload_len = 2;
    return COMM_STATUS_OK;
  case COMM_PARAM_MAX_SPEED:
    write_u16_le(payload, get_max_speed());
    *payload_len = 2;
    return COMM_STATUS_OK;
  case COMM_PARAM_MIN_SPEED:
    write_u16_le(payload, get_min_speed());
    *payload_len = 2;
    return COMM_STATUS_OK;
  case COMM_PARAM_STARTUP_INITIAL_AMPLITUDE:
    write_u16_le(payload, get_startup_initial_amplitude());
    *payload_len = 2;
    return COMM_STATUS_OK;
  case COMM_PARAM_STARTUP_FINAL_AMPLITUDE:
    write_u16_le(payload, get_startup_final_amplitude());
    *payload_len = 2;
    return COMM_STATUS_OK;
  case COMM_PARAM_STARTUP_INITIAL_FREQUENCY:
    write_u32_le(payload, get_startup_initial_frequency());
    *payload_len = 4;
    return COMM_STATUS_OK;
  case COMM_PARAM_STARTUP_FINAL_FREQUENCY:
    write_u32_le(payload, get_startup_final_frequency());
    *payload_len = 4;
    return COMM_STATUS_OK;
  case COMM_PARAM_STARTUP_DURATION:
    write_u32_le(payload, get_startup_duration());
    *payload_len = 4;
    return COMM_STATUS_OK;
  case COMM_PARAM_BEMF_BLANKING_US:
    write_u16_le(payload, get_bemf_blanking_us());
    *payload_len = 2;
    return COMM_STATUS_OK;
  case COMM_PARAM_ZC_MAP:
    get_zc_channel_map(&payload[0], &payload[1], &payload[2]);
    *payload_len = 3;
    return COMM_STATUS_OK;
  case COMM_PARAM_SPEED_RAMP:
    write_u16_le(payload, get_speed_ramp());
    *payload_len = 2;
    return COMM_STATUS_OK;
  default:
    return COMM_STATUS_UNKNOWN_PARAM;
  }
}

static uint8_t set_config(uint8_t param, const uint8_t* payload, uint16_t payload_len)
{
  bool allow_in_freewheel = (param == COMM_PARAM_ZC_MAP && app_state == FREEWHEEL);
  if (config_write_requires_idle(param) && app_state != IDLE && !allow_in_freewheel) {
    return COMM_STATUS_INVALID_STATE;
  }
  ConfigStatus result = CONFIG_UNKNOWN;

  switch (param) {
  case COMM_PARAM_PWM_FREQ:
    if (payload_len != 2U) return COMM_STATUS_BAD_LENGTH;
    result = set_pwm_freq(read_u16_le(payload));
    break;
  case COMM_PARAM_POLE_PAIRS:
    if (payload_len != 1U) return COMM_STATUS_BAD_LENGTH;
    result = set_pole_pairs(payload[0]);
    break;
  case COMM_PARAM_KP_RPM:
    if (payload_len != 2U) return COMM_STATUS_BAD_LENGTH;
    result = set_KP(centi_to_float(read_i16_le(payload)));
    break;
  case COMM_PARAM_KI_RPM:
    if (payload_len != 2U) return COMM_STATUS_BAD_LENGTH;
    result = set_KI(centi_to_float(read_i16_le(payload)));
    break;
  case COMM_PARAM_KD_RPM:
    if (payload_len != 2U) return COMM_STATUS_BAD_LENGTH;
    result = set_KD(centi_to_float(read_i16_le(payload)));
    break;
  case COMM_PARAM_MAX_SPEED:
    if (payload_len != 2U) return COMM_STATUS_BAD_LENGTH;
    result = set_max_speed(read_u16_le(payload));
    break;
  case COMM_PARAM_MIN_SPEED:
    if (payload_len != 2U) return COMM_STATUS_BAD_LENGTH;
    result = set_min_speed(read_u16_le(payload));
    break;
  case COMM_PARAM_STARTUP_INITIAL_AMPLITUDE:
    if (payload_len != 2U) return COMM_STATUS_BAD_LENGTH;
    result = set_startup_initial_amplitude(read_u16_le(payload));
    break;
  case COMM_PARAM_STARTUP_FINAL_AMPLITUDE:
    if (payload_len != 2U) return COMM_STATUS_BAD_LENGTH;
    result = set_startup_final_amplitude(read_u16_le(payload));
    break;
  case COMM_PARAM_STARTUP_INITIAL_FREQUENCY:
    if (payload_len != 4U) return COMM_STATUS_BAD_LENGTH;
    result = set_startup_initial_frequency(read_u32_le(payload));
    break;
  case COMM_PARAM_STARTUP_FINAL_FREQUENCY:
    if (payload_len != 4U) return COMM_STATUS_BAD_LENGTH;
    result = set_startup_final_frequency(read_u32_le(payload));
    break;
  case COMM_PARAM_STARTUP_DURATION:
    if (payload_len != 4U) return COMM_STATUS_BAD_LENGTH;
    result = set_startup_duration(read_u32_le(payload));
    break;
  case COMM_PARAM_BEMF_BLANKING_US:
    if (payload_len != 2U) return COMM_STATUS_BAD_LENGTH;
    result = set_bemf_blanking_us(read_u16_le(payload));
    break;
  case COMM_PARAM_ZC_MAP:
    if (payload_len != 3U) return COMM_STATUS_BAD_LENGTH;
    result = set_zc_channel_map(payload[0], payload[1], payload[2]);
    break;
  case COMM_PARAM_SPEED_RAMP:
    if (payload_len != 2U) return COMM_STATUS_BAD_LENGTH;
    result = set_speed_ramp(read_u16_le(payload));
    break;
  default:
    return COMM_STATUS_UNKNOWN_PARAM;
  }

  uint8_t status = status_from_config(result);
  if (status == COMM_STATUS_OK) {
    flash_config_parameter_changed();
    apply_config_change(param);
  }
  return status;
}

static uint8_t save_config(uint8_t param)
{
  if (app_state != IDLE) return COMM_STATUS_INVALID_STATE;
  switch (param) {
  case COMM_PARAM_PWM_FREQ:
  case COMM_PARAM_POLE_PAIRS:
  case COMM_PARAM_KP_RPM:
  case COMM_PARAM_KI_RPM:
  case COMM_PARAM_KD_RPM:
  case COMM_PARAM_MAX_SPEED:
  case COMM_PARAM_MIN_SPEED:
  case COMM_PARAM_STARTUP_INITIAL_AMPLITUDE:
  case COMM_PARAM_STARTUP_FINAL_AMPLITUDE:
  case COMM_PARAM_STARTUP_INITIAL_FREQUENCY:
  case COMM_PARAM_STARTUP_FINAL_FREQUENCY:
  case COMM_PARAM_STARTUP_DURATION:
  case COMM_PARAM_BEMF_BLANKING_US:
  case COMM_PARAM_ZC_MAP:
  case COMM_PARAM_SPEED_RAMP:
  case COMM_PARAM_ALL:
    return status_from_flash(flash_config_save());
  default:
    return COMM_STATUS_UNKNOWN_PARAM;
  }
}

static uint8_t reset_config(uint8_t param)
{
  bool allow_in_freewheel = (param == COMM_PARAM_ZC_MAP && app_state == FREEWHEEL);
  if (config_write_requires_idle(param) && app_state != IDLE && !allow_in_freewheel) {
    return COMM_STATUS_INVALID_STATE;
  }
  switch (param) {
  case COMM_PARAM_ALL:
    set_default_esc_params();
    flash_config_parameter_changed();
    break;
  case COMM_PARAM_PWM_FREQ:
    current_esc_params.pwm_freq_hz = DEFAULT_PWM_FREQ_HZ;
    flash_config_parameter_changed();
    break;
  case COMM_PARAM_POLE_PAIRS:
    current_esc_params.pole_pairs = DEFAULT_POLE_PAIRS;
    flash_config_parameter_changed();
    break;
  case COMM_PARAM_KP_RPM:
    current_esc_params.speed_kp = centi_to_float(DEFAULT_KP_CENTI);
    flash_config_parameter_changed();
    break;
  case COMM_PARAM_KI_RPM:
    current_esc_params.speed_ki = centi_to_float(DEFAULT_KI_CENTI);
    flash_config_parameter_changed();
    break;
  case COMM_PARAM_KD_RPM:
    current_esc_params.speed_kd = centi_to_float(DEFAULT_KD_CENTI);
    flash_config_parameter_changed();
    break;
  case COMM_PARAM_MAX_SPEED:
    current_esc_params.speed_max_rpm = DEFAULT_MAX_SPEED_RPM;
    flash_config_parameter_changed();
    break;
  case COMM_PARAM_MIN_SPEED:
    current_esc_params.speed_min_rpm = DEFAULT_MIN_SPEED_RPM;
    flash_config_parameter_changed();
    break;
  case COMM_PARAM_STARTUP_INITIAL_AMPLITUDE:
    current_esc_params.startup_initial_amplitude_permille =
        ESC_DEFAULT_STARTUP_INITIAL_AMPLITUDE_PERMILLE;
    flash_config_parameter_changed();
    break;
  case COMM_PARAM_STARTUP_FINAL_AMPLITUDE:
    current_esc_params.startup_final_amplitude_permille =
        ESC_DEFAULT_STARTUP_FINAL_AMPLITUDE_PERMILLE;
    flash_config_parameter_changed();
    break;
  case COMM_PARAM_STARTUP_INITIAL_FREQUENCY:
    current_esc_params.startup_initial_frequency_millihz =
        ESC_DEFAULT_STARTUP_INITIAL_FREQUENCY_MILLIHZ;
    flash_config_parameter_changed();
    break;
  case COMM_PARAM_STARTUP_FINAL_FREQUENCY:
    current_esc_params.startup_final_frequency_millihz =
        ESC_DEFAULT_STARTUP_FINAL_FREQUENCY_MILLIHZ;
    flash_config_parameter_changed();
    break;
  case COMM_PARAM_STARTUP_DURATION:
    current_esc_params.startup_duration_ms = ESC_DEFAULT_STARTUP_DURATION_MS;
    flash_config_parameter_changed();
    break;
  case COMM_PARAM_BEMF_BLANKING_US:
    current_esc_params.bemf_blanking_us = ESC_DEFAULT_BEMF_BLANKING_US;
    flash_config_parameter_changed();
    break;
  case COMM_PARAM_ZC_MAP:
    current_esc_params.zc_channel_u = ESC_DEFAULT_ZC_CHANNEL_U;
    current_esc_params.zc_channel_v = ESC_DEFAULT_ZC_CHANNEL_V;
    current_esc_params.zc_channel_w = ESC_DEFAULT_ZC_CHANNEL_W;
    flash_config_parameter_changed();
    break;
  case COMM_PARAM_SPEED_RAMP:
    current_esc_params.speed_ramp_rpm_s = ESC_DEFAULT_SPEED_RAMP_RPM_S;
    flash_config_parameter_changed();
    break;
  default:
    return COMM_STATUS_UNKNOWN_PARAM;
  }

  apply_config_change(param);
  return COMM_STATUS_OK;
}

static uint8_t logging_variable_from_param(uint8_t param, LoggeableVariable* variable)
{
  switch (param) {
  case COMM_LOG_PARAM_SPEED:
    *variable = VAR_SPEED;
    return COMM_STATUS_OK;
  case COMM_LOG_PARAM_DUTY:
    *variable = VAR_DUTY;
    return COMM_STATUS_OK;
  case COMM_LOG_PARAM_BUS_VOLTAGE:
    *variable = VAR_BUS_VOLTAGE;
    return COMM_STATUS_OK;
  case COMM_LOG_PARAM_BUS_CURRENT:
    *variable = VAR_BUS_CURRENT;
    return COMM_STATUS_OK;
  default:
    return COMM_STATUS_UNKNOWN_PARAM;
  }
}

static bool load_sweep_allows_opcode(uint8_t opcode)
{
  return opcode == COMM_OPCODE_PING || opcode == COMM_OPCODE_GET_STATUS ||
         opcode == COMM_OPCODE_STOP || opcode == COMM_OPCODE_ESTOP ||
         opcode == COMM_OPCODE_GET_CONFIG || opcode == COMM_OPCODE_LOAD_SWEEP ||
         (opcode >= COMM_OPCODE_LOG_START && opcode <= COMM_OPCODE_LOG_RATE);
}

static uint8_t status_from_sweep_start(LoadSweepStartResult result)
{
  switch (result) {
  case LOAD_SWEEP_START_OK:
    return COMM_STATUS_OK;
  case LOAD_SWEEP_START_UNDERLIMIT:
    return COMM_STATUS_UNDERLIMIT;
  case LOAD_SWEEP_START_OVERLIMIT:
    return COMM_STATUS_OVERLIMIT;
  case LOAD_SWEEP_START_INVALID_STATE:
    return COMM_STATUS_INVALID_STATE;
  default:
    return COMM_STATUS_UNKNOWN_PARAM;
  }
}

static uint8_t handle_load_sweep(uint8_t param,
                                 const uint8_t* payload,
                                 uint16_t payload_len,
                                 uint8_t* payload_out,
                                 uint16_t* payload_out_len)
{
  switch (param) {
  case COMM_LOAD_SWEEP_PARAM_START: {
    if (payload_len != 12U) return COMM_STATUS_BAD_LENGTH;
    LoadSweepConfig config = {
      .mode = payload[0],
      .start_rpm = read_u16_le(&payload[1]),
      .end_rpm = read_u16_le(&payload[3]),
      .step_or_rate = read_u16_le(&payload[5]),
      .settle_ms = read_u16_le(&payload[7]),
      .window_ms = read_u16_le(&payload[9]),
      .options = payload[11],
    };
    return status_from_sweep_start(load_sweep_start(&config));
  }

  case COMM_LOAD_SWEEP_PARAM_ABORT:
    if (payload_len != 0U) return COMM_STATUS_BAD_LENGTH;
    load_sweep_abort(LOAD_SWEEP_ABORT_USER);
    return COMM_STATUS_OK;

  case COMM_LOAD_SWEEP_PARAM_GET_STATUS: {
    if (payload_len != 0U) return COMM_STATUS_BAD_LENGTH;
    LoadSweepStatus status;
    load_sweep_get_status(&status);
    payload_out[0] = status.state;
    payload_out[1] = status.abort_reason;
    payload_out[2] = status.config.mode;
    payload_out[3] = status.config.options;
    write_u16_le(&payload_out[4], status.point_count);
    write_u16_le(&payload_out[6], LOAD_SWEEP_CAPACITY);
    write_u16_le(&payload_out[8], status.config.start_rpm);
    write_u16_le(&payload_out[10], status.config.end_rpm);
    write_u16_le(&payload_out[12], status.config.step_or_rate);
    write_u16_le(&payload_out[14], status.config.settle_ms);
    write_u16_le(&payload_out[16], status.config.window_ms);
    write_u16_le(&payload_out[18], status.target_rpm);
    write_u32_le(&payload_out[20], (uint32_t)status.idle_current_ua);
    write_u32_le(&payload_out[24], status.elapsed_ms);
    payload_out[28] = status.idle_current_valid ? 1U : 0U;
    *payload_out_len = 29U;
    return COMM_STATUS_OK;
  }

  case COMM_LOAD_SWEEP_PARAM_GET_POINTS: {
    if (payload_len != 2U) return COMM_STATUS_BAD_LENGTH;
    uint16_t first = read_u16_le(payload);
    uint8_t count = 0U;
    LoadSweepPoint point;
    while (count < COMM_LOAD_SWEEP_POINTS_PER_FRAME &&
           load_sweep_get_point((uint16_t)(first + count), &point)) {
      memcpy(&payload_out[3U + count * sizeof(point)], &point, sizeof(point));
      count++;
    }
    write_u16_le(&payload_out[0], first);
    payload_out[2] = count;
    *payload_out_len = (uint16_t)(3U + count * sizeof(point));
    return COMM_STATUS_OK;
  }

  default:
    return COMM_STATUS_UNKNOWN_PARAM;
  }
}

static uint8_t
execute_request(const uint8_t request[COMM_FRAME_SIZE],
                uint16_t payload_len,
                uint8_t* payload_out,
                uint16_t* payload_out_len)
{
  const uint8_t opcode = request[5];
  const uint8_t param = request[6];
  const uint8_t* payload = &request[COMM_PAYLOAD_OFFSET];

  *payload_out_len = 0;

  if (load_sweep_is_active() && !load_sweep_allows_opcode(opcode)) {
    return COMM_STATUS_INVALID_STATE;
  }

  if ((app_state == SINE_DRIVE || app_state == STEP_TEST) &&
      !manual_output_allows_opcode(opcode)) {
    return COMM_STATUS_INVALID_STATE;
  }

  if (app_state == FREEWHEEL && !freewheel_allows_opcode(opcode, param)) {
    return COMM_STATUS_INVALID_STATE;
  }

  if (app_state == FOC_STARTUP &&
      (opcode == COMM_OPCODE_SET_CONFIG || opcode == COMM_OPCODE_RESET_CONFIG ||
       opcode == COMM_OPCODE_SAVE_CONFIG)) {
    return COMM_STATUS_INVALID_STATE;
  }

  switch (opcode) {
  case COMM_OPCODE_PING:
    memcpy(payload_out, payload, payload_len);
    *payload_out_len = payload_len;
    return COMM_STATUS_OK;

  case COMM_OPCODE_GET_STATUS: {
    if (payload_len != 0U) return COMM_STATUS_BAD_LENGTH;
    uint16_t actual_speed_rpm = period_to_rpm(get_actual_speed());
    payload_out[0] = (uint8_t)app_state;
    payload_out[1] = (uint8_t)comm_transport_get_mode();
    payload_out[2] = motor_stalled ? 1U : 0U;
    payload_out[3] = consistent_zero_crossing ? 1U : 0U;
    write_u16_le(&payload_out[4], speed_setpoint_rpm);
    write_u16_le(&payload_out[6], actual_speed_rpm);
    write_u16_le(&payload_out[8], max_pwm);
    PowerMonitorSnapshot power;
    power_monitor_get_snapshot(&power);
    payload_out[10] = power.flags;
    payload_out[11] = power.i2c_addr;
    write_u16_le(&payload_out[12], power.bus_mv);
    write_u32_le(&payload_out[14], (uint32_t)power.current_ua);
    payload_out[18] = power.last_error;
    payload_out[19] = power.error_count;
    payload_out[20] = power.line_state;
    payload_out[21] = power.fail_stage;
    write_u16_le(&payload_out[22], power.config_readback);
    write_u16_le(&payload_out[24], power.busy_recoveries);
    write_u16_le(&payload_out[26], motor_control_get_reference_rpm());
    write_u16_le(&payload_out[28], motor_control_get_duty_e4());
    payload_out[30] = load_sweep_is_active() ? 1U : 0U;
    *payload_out_len = 31;
    return COMM_STATUS_OK;
  }

  case COMM_OPCODE_RUN:
    if (payload_len != 0U) return COMM_STATUS_BAD_LENGTH;
    if (app_state != IDLE) return COMM_STATUS_INVALID_STATE;
    foc_startup();
    return COMM_STATUS_OK;

  case COMM_OPCODE_STOP:
    if (payload_len != 0U) return COMM_STATUS_BAD_LENGTH;
    if (!stop_state_allows_command()) return COMM_STATUS_INVALID_STATE;
    load_sweep_abort(LOAD_SWEEP_ABORT_STOPPED);
    stop_manual_outputs();
    stop_motor(0);
    app_state = IDLE;
    return COMM_STATUS_OK;

  case COMM_OPCODE_ESTOP:
    if (payload_len != 0U) return COMM_STATUS_BAD_LENGTH;
    load_sweep_abort(LOAD_SWEEP_ABORT_STOPPED);
    stop_manual_outputs();
    stop_motor(1);
    app_state = IDLE;
    return COMM_STATUS_OK;

  case COMM_OPCODE_SET_SPEED_RPM: {
    if (payload_len != 2U) return COMM_STATUS_BAD_LENGTH;
    uint16_t rpm = read_u16_le(payload);
    if (rpm < get_min_speed()) return COMM_STATUS_UNDERLIMIT;
    if (rpm > get_max_speed()) return COMM_STATUS_OVERLIMIT;
    speed_setpoint_rpm = rpm;
    return COMM_STATUS_OK;
  }

  case COMM_OPCODE_GET_CONFIG:
    if (payload_len != 0U) return COMM_STATUS_BAD_LENGTH;
    return get_config(param, payload_out, payload_out_len);

  case COMM_OPCODE_SET_CONFIG:
    return set_config(param, payload, payload_len);

  case COMM_OPCODE_RESET_CONFIG:
    if (payload_len != 0U) return COMM_STATUS_BAD_LENGTH;
    return reset_config(param);

  case COMM_OPCODE_SAVE_CONFIG:
    if (payload_len != 0U) return COMM_STATUS_BAD_LENGTH;
    return save_config(param);

  case COMM_OPCODE_LOG_START:
    if (payload_len != 0U) return COMM_STATUS_BAD_LENGTH;
    if (param == COMM_PARAM_ALL) {
      start_logging_param(VAR_SPEED);
      start_logging_param(VAR_BUS_VOLTAGE);
      start_logging_param(VAR_BUS_CURRENT);
      start_logging_param(VAR_DUTY);
      return COMM_STATUS_OK;
    } else {
      LoggeableVariable variable;
      uint8_t status = logging_variable_from_param(param, &variable);
      if (status != COMM_STATUS_OK) return status;
      start_logging_param(variable);
      return COMM_STATUS_OK;
    }

  case COMM_OPCODE_LOG_STOP:
    if (payload_len != 0U) return COMM_STATUS_BAD_LENGTH;
    if (param == COMM_PARAM_ALL) {
      stop_logging_all();
      return COMM_STATUS_OK;
    } else {
      LoggeableVariable variable;
      uint8_t status = logging_variable_from_param(param, &variable);
      if (status != COMM_STATUS_OK) return status;
      stop_logging_param(variable);
      return COMM_STATUS_OK;
    }

  case COMM_OPCODE_LOG_RATE:
    if (payload_len != 2U) return COMM_STATUS_BAD_LENGTH;
    return set_logging_rate_ms(read_u16_le(payload));

  case COMM_OPCODE_SINE_DRIVE: {
    if (payload_len != 6U) return COMM_STATUS_BAD_LENGTH;
    if (param != COMM_SINE_PARAM_APPLY && param != COMM_SINE_PARAM_KEEPALIVE) {
      return COMM_STATUS_UNKNOWN_PARAM;
    }
    if (param == COMM_SINE_PARAM_KEEPALIVE && app_state != SINE_DRIVE) {
      return COMM_STATUS_INVALID_STATE;
    }
    uint32_t frequency_millihz = read_u32_le(payload);
    uint16_t amplitude_permille = read_u16_le(&payload[4]);
    if (frequency_millihz < ESC_MIN_SINE_FREQUENCY_MILLIHZ) {
      return COMM_STATUS_UNDERLIMIT;
    }
    if (frequency_millihz > ESC_MAX_SINE_FREQUENCY_MILLIHZ ||
        amplitude_permille > ESC_MAX_SINE_AMPLITUDE_PERMILLE) {
      return COMM_STATUS_OVERLIMIT;
    }

    SineDriveSettings applied;
    if (!sine_drive_start_or_update(frequency_millihz, amplitude_permille, &applied)) {
      return COMM_STATUS_INVALID_STATE;
    }
    write_u32_le(payload_out, applied.frequency_millihz);
    write_u16_le(&payload_out[4], applied.amplitude_permille);
    *payload_out_len = 6U;
    return COMM_STATUS_OK;
  }

  case COMM_OPCODE_STEP_TEST: {
    if (payload_len != 3U) return COMM_STATUS_BAD_LENGTH;
    if (param != COMM_STEP_TEST_PARAM_APPLY && param != COMM_STEP_TEST_PARAM_KEEPALIVE) {
      return COMM_STATUS_UNKNOWN_PARAM;
    }
    if (param == COMM_STEP_TEST_PARAM_KEEPALIVE && app_state != STEP_TEST) {
      return COMM_STATUS_INVALID_STATE;
    }
    uint8_t step = payload[0];
    uint16_t duty_permille = read_u16_le(&payload[1]);
    if (step >= NUM_POS) return COMM_STATUS_UNKNOWN_PARAM;
    if (duty_permille > ESC_MAX_STEP_TEST_DUTY_PERMILLE) return COMM_STATUS_OVERLIMIT;
    if (!step_test_apply(step, duty_permille)) return COMM_STATUS_INVALID_STATE;

    payload_out[0] = step;
    write_u16_le(&payload_out[1], duty_permille);
    *payload_out_len = 3U;
    return COMM_STATUS_OK;
  }

  case COMM_OPCODE_ZC_DIAG: {
    switch (param) {
    case COMM_ZC_DIAG_PARAM_START: {
      if (payload_len != 0U) return COMM_STATUS_BAD_LENGTH;
      if (!zc_diag_freewheel_start()) return COMM_STATUS_INVALID_STATE;
      ZcDiagReport report;
      zc_diag_get_report(&report);
      *payload_out_len = serialize_zc_diag_report(&report, payload_out);
      return COMM_STATUS_OK;
    }
    case COMM_ZC_DIAG_PARAM_STOP:
      if (payload_len != 0U) return COMM_STATUS_BAD_LENGTH;
      if (app_state != FREEWHEEL && app_state != IDLE) return COMM_STATUS_INVALID_STATE;
      zc_diag_freewheel_stop();
      return COMM_STATUS_OK;
    case COMM_ZC_DIAG_PARAM_GET_REPORT: {
      if (payload_len != 0U) return COMM_STATUS_BAD_LENGTH;
      ZcDiagReport report;
      zc_diag_get_report(&report);
      *payload_out_len = serialize_zc_diag_report(&report, payload_out);
      return COMM_STATUS_OK;
    }
    case COMM_ZC_DIAG_PARAM_AUTO_DETECT: {
      uint16_t duty_permille = ESC_DEFAULT_ZC_AUTO_DETECT_DUTY_PERMILLE;
      if (payload_len == 2U) {
        duty_permille = read_u16_le(payload);
        if (duty_permille < ESC_MIN_ZC_AUTO_DETECT_DUTY_PERMILLE) {
          return COMM_STATUS_UNDERLIMIT;
        }
        if (duty_permille > ESC_MAX_ZC_AUTO_DETECT_DUTY_PERMILLE) {
          return COMM_STATUS_OVERLIMIT;
        }
      } else if (payload_len != 0U) {
        return COMM_STATUS_BAD_LENGTH;
      }
      if (app_state != IDLE) return COMM_STATUS_INVALID_STATE;
      ZcAutoDetectResult result;
      if (!zc_diag_auto_detect(duty_permille, &result)) {
        return COMM_STATUS_INVALID_STATE;
      }
      payload_out[0] = result.result_code;
      memcpy(&payload_out[1], result.detected_map, 3U);
      memcpy(&payload_out[4], result.active_map, 3U);
      memcpy(&payload_out[7], result.high_pct, 9U);
      *payload_out_len = 16U;
      return COMM_STATUS_OK;
    }
    default:
      return COMM_STATUS_UNKNOWN_PARAM;
    }
  }

  case COMM_OPCODE_LOAD_SWEEP:
    return handle_load_sweep(param, payload, payload_len, payload_out, payload_out_len);

  default:
    return COMM_STATUS_UNKNOWN_OPCODE;
  }
}

bool comm_protocol_handle_frame(const uint8_t request[COMM_FRAME_SIZE],
                                uint8_t response[COMM_FRAME_SIZE])
{
  uint8_t payload[COMM_PAYLOAD_MAX];
  uint16_t payload_len = 0;
  uint8_t status = COMM_STATUS_OK;

  if (request[0] != COMM_MAGIC_0 || request[1] != COMM_MAGIC_1) {
    build_response(request, COMM_STATUS_BAD_MAGIC, NULL, 0, response);
    return true;
  }

  if (request[2] != COMM_VERSION) {
    build_response(request, COMM_STATUS_BAD_VERSION, NULL, 0, response);
    return true;
  }

  uint16_t expected_crc = read_u16_le(&request[COMM_CRC_OFFSET]);
  uint16_t actual_crc = comm_protocol_crc16(request, COMM_CRC_OFFSET);
  if (expected_crc != actual_crc) {
    build_response(request, COMM_STATUS_BAD_CRC, NULL, 0, response);
    return true;
  }

  if (request[3] != COMM_TYPE_REQUEST) {
    build_response(request, COMM_STATUS_INVALID_STATE, NULL, 0, response);
    return true;
  }

  payload_len = read_u16_le(&request[8]);
  if (payload_len > COMM_PAYLOAD_MAX) {
    build_response(request, COMM_STATUS_BAD_LENGTH, NULL, 0, response);
    return true;
  }

  status = execute_request(request, payload_len, payload, &payload_len);
  build_response(request, status, payload, payload_len, response);
  return true;
}

void comm_protocol_build_event(uint8_t opcode,
                               uint8_t param,
                               const uint8_t* payload,
                               uint16_t payload_len,
                               uint8_t frame[COMM_FRAME_SIZE])
{
  static uint8_t event_seq = 0;

  if (payload_len > COMM_PAYLOAD_MAX) {
    payload_len = COMM_PAYLOAD_MAX;
  }

  build_base(frame, COMM_TYPE_EVENT, event_seq++, opcode, param, 0U);
  if (payload != NULL && payload_len > 0U) {
    memcpy(&frame[COMM_PAYLOAD_OFFSET], payload, payload_len);
  }
  finish_frame(frame, payload_len);
}
