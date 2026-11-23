#include "comm_protocol.h"

#include "comm.h"
#include "comm_transport.h"
#include "flash_config.h"
#include "hard_config.h"
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

enum {
  REQUEST_PING = 1,
  REQUEST_GET_STATUS,
  REQUEST_RUN,
  REQUEST_STOP,
  REQUEST_ESTOP,
  REQUEST_SET_SPEED_RPM,
  REQUEST_SET_CONTROL_MODE,
  REQUEST_GET_CONFIG,
  REQUEST_SET_CONFIG,
  REQUEST_RESET_CONFIG,
  REQUEST_SAVE_CONFIG,
  REQUEST_GET_VALIDATION_REFERENCE,
  REQUEST_LOG_START,
  REQUEST_LOG_STOP,
  REQUEST_LOG_RATE,
  REQUEST_HIL_START,
  REQUEST_HIL_STOP,
  REQUEST_HIL_SET_INPUTS,
  REQUEST_HIL_GET_OUTPUTS,
  REQUEST_HIL_STEP,
  REQUEST_SINE_DRIVE,
  REQUEST_STEP_TEST,
  REQUEST_ZC_DIAG,
};

static inline __attribute__((always_inline)) uint8_t request_dispatch(uint8_t opcode)
{
  static const uint8_t dispatch_base[] = {0U, 3U, 8U, 13U, 16U, 21U};
  static const uint8_t maximum_low_nibble[] = {2U, 4U, 4U, 2U, 4U, 2U};
  uint8_t group = opcode >> 4;
  uint8_t low_nibble = opcode & 0x0FU;
  if (group > 5U || low_nibble > maximum_low_nibble[group]) return 0U;
  return dispatch_base[group] + low_nibble;
}

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

__attribute__((optimize("Oz"))) static uint16_t
serialize_hil_outputs(uint8_t* payload_out)
{
  HilValidationProvenance provenance;
  uint32_t primask = __get_PRIMASK();
  __disable_irq();
  write_u32_le(&payload_out[0], HAL_GetTick());
  payload_out[4] = (uint8_t)app_state;
  payload_out[5] = (uint8_t)control_runtime_mode;
  write_u16_le(&payload_out[6], speed_setpoint_rpm);
  write_u16_le(&payload_out[8],
               hil_session_state != 0U ? hil_speed_rpm : period_to_rpm(get_actual_speed()));
  write_u16_le(&payload_out[10], bldc_get_pwm());
  payload_out[12] = (uint8_t)bldc_get_commutation_step();
  payload_out[13] = hil_flags;
  payload_out[14] = hil_has_timeout();
  hil_get_validation_provenance(&provenance);
  _Static_assert(sizeof(provenance) == 28U, "Unexpected HIL provenance layout");
  memcpy(&payload_out[15], &provenance, sizeof(provenance));
  if (primask == 0U) __enable_irq();
  return 43U;
}

__attribute__((optimize("Oz"))) static void apply_config_change(uint8_t param)
{
  if (is_idle_only_param(param)) return;

  if (app_state == CLOSEDLOOP) {
    if (param == COMM_PARAM_KP_RPM || param == COMM_PARAM_KI_RPM || param == COMM_PARAM_KD_RPM) {
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
  case COMM_PARAM_CURRENT_LIMIT:
  case COMM_PARAM_TEMP_LIMIT:
  case COMM_PARAM_KP:
  case COMM_PARAM_KI:
  case COMM_PARAM_KD:
    return COMM_STATUS_NOT_IMPLEMENTED;
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
  case COMM_PARAM_CURRENT_LIMIT:
  case COMM_PARAM_TEMP_LIMIT:
  case COMM_PARAM_KP:
  case COMM_PARAM_KI:
  case COMM_PARAM_KD:
    return COMM_STATUS_NOT_IMPLEMENTED;
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
  case COMM_PARAM_ALL:
    return status_from_flash(flash_config_save());
  case COMM_PARAM_CURRENT_LIMIT:
  case COMM_PARAM_TEMP_LIMIT:
    return COMM_STATUS_NOT_IMPLEMENTED;
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
  case COMM_PARAM_CURRENT_LIMIT:
  case COMM_PARAM_TEMP_LIMIT:
  case COMM_PARAM_KP:
  case COMM_PARAM_KI:
  case COMM_PARAM_KD:
    return COMM_STATUS_NOT_IMPLEMENTED;
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
  case COMM_LOG_PARAM_TEMP:
    *variable = VAR_TEMP;
    return COMM_STATUS_OK;
  case COMM_LOG_PARAM_CURRENT:
    *variable = VAR_CURRENT_U;
    return COMM_STATUS_OK;
  case COMM_LOG_PARAM_CURRENT_V:
    *variable = VAR_CURRENT_V;
    return COMM_STATUS_OK;
  case COMM_LOG_PARAM_BEMF_PERIOD:
    *variable = VAR_BEMF_PERIOD;
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

  if (opcode != COMM_OPCODE_ESTOP &&
      hil_session_state == (uint8_t)HIL_EXECUTION_STEPPED + 1U &&
      ((opcode >= COMM_OPCODE_SET_SPEED_RPM && opcode <= COMM_OPCODE_SET_CONTROL_MODE) ||
       (opcode >= COMM_OPCODE_SET_CONFIG && opcode <= COMM_OPCODE_SAVE_CONFIG))) {
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

  switch (request_dispatch(opcode)) {
  case REQUEST_PING:
    memcpy(payload_out, payload, payload_len);
    *payload_out_len = payload_len;
    return COMM_STATUS_OK;

  case REQUEST_GET_STATUS: {
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
    *payload_out_len = 26;
    return COMM_STATUS_OK;
  }

  case REQUEST_RUN:
    if (payload_len != 0U) return COMM_STATUS_BAD_LENGTH;
    if (app_state != IDLE) return COMM_STATUS_INVALID_STATE;
    foc_startup();
    return COMM_STATUS_OK;

  case REQUEST_STOP:
    if (payload_len != 0U) return COMM_STATUS_BAD_LENGTH;
    if (!stop_state_allows_command()) return COMM_STATUS_INVALID_STATE;
    stop_manual_outputs();
    stop_motor(0);
    app_state = IDLE;
    return COMM_STATUS_OK;

  case REQUEST_ESTOP:
    if (payload_len != 0U) return COMM_STATUS_BAD_LENGTH;
    stop_manual_outputs();
    stop_motor(1);
    app_state = IDLE;
    return COMM_STATUS_OK;

  case REQUEST_SET_SPEED_RPM: {
    if (payload_len != 2U) return COMM_STATUS_BAD_LENGTH;
    uint16_t rpm = read_u16_le(payload);
    if (rpm < get_min_speed()) return COMM_STATUS_UNDERLIMIT;
    if (rpm > get_max_speed()) return COMM_STATUS_OVERLIMIT;
    speed_setpoint_rpm = rpm;
    return COMM_STATUS_OK;
  }

  case REQUEST_SET_CONTROL_MODE:
    if (payload_len != 1U) return COMM_STATUS_BAD_LENGTH;
    return control_mode_set(payload[0]) ? COMM_STATUS_OK : COMM_STATUS_UNKNOWN_PARAM;

  case REQUEST_GET_CONFIG:
    if (payload_len != 0U) return COMM_STATUS_BAD_LENGTH;
    return get_config(param, payload_out, payload_out_len);

  case REQUEST_GET_VALIDATION_REFERENCE: {
    if (payload_len != 0U) return COMM_STATUS_BAD_LENGTH;
    uint16_t pwm_arr = (uint16_t)TIM1->ARR;
    uint32_t speed_timer_hz = HAL_RCC_GetHCLKFreq() / (TIM2->PSC + 1U);
    payload_out[0] = COMM_VALIDATION_REFERENCE_VERSION;
    payload_out[1] = MOTOR_CONTROL_ALGORITHM_VERSION;
    write_u16_le(&payload_out[2], get_pwm_freq());
    write_u16_le(&payload_out[4], pwm_arr);
    write_u32_le(&payload_out[6], speed_timer_hz);
    write_u16_le(&payload_out[10], SPEED_MIN);
    write_u16_le(&payload_out[12], SPEED_MAX);
    write_u32_le(&payload_out[14], MOTOR_CONTROL_DT_US);
    write_u16_le(&payload_out[18], pwm_arr / MOTOR_CONTROL_MIN_PWM_DIVISOR);
    payload_out[20] = COMM_VALIDATION_CAP_DETERMINISTIC;
    payload_out[21] = COMM_HIL_OPERATION_VERSION;
    write_u16_le(&payload_out[22], HIL_MAX_STEP_COUNT);
    *payload_out_len = 24U;
    return COMM_STATUS_OK;
  }

  case REQUEST_SET_CONFIG:
    return set_config(param, payload, payload_len);

  case REQUEST_RESET_CONFIG:
    if (payload_len != 0U) return COMM_STATUS_BAD_LENGTH;
    return reset_config(param);

  case REQUEST_SAVE_CONFIG:
    if (payload_len != 0U) return COMM_STATUS_BAD_LENGTH;
    return save_config(param);

  case REQUEST_LOG_START:
    if (payload_len != 0U) return COMM_STATUS_BAD_LENGTH;
    if (param == COMM_PARAM_ALL) {
      start_logging_param(VAR_SPEED);
      start_logging_param(VAR_TEMP);
      start_logging_param(VAR_CURRENT_U);
      start_logging_param(VAR_CURRENT_V);
      start_logging_param(VAR_BEMF_PERIOD);
      start_logging_param(VAR_BUS_VOLTAGE);
      start_logging_param(VAR_BUS_CURRENT);
      return COMM_STATUS_OK;
    } else {
      LoggeableVariable variable;
      uint8_t status = logging_variable_from_param(param, &variable);
      if (status != COMM_STATUS_OK) return status;
      start_logging_param(variable);
      return COMM_STATUS_OK;
    }

  case REQUEST_LOG_STOP:
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

  case REQUEST_LOG_RATE:
    if (payload_len != 2U) return COMM_STATUS_BAD_LENGTH;
    return set_logging_rate_ms(read_u16_le(payload));

  case REQUEST_HIL_START: {
    if (payload_len != 0U && payload_len != 2U && payload_len != 3U) {
      return COMM_STATUS_BAD_LENGTH;
    }
    if (hil_session_state == (uint8_t)HIL_EXECUTION_STEPPED + 1U) {
      return COMM_STATUS_INVALID_STATE;
    }
    uint16_t hil_timeout_ms = payload_len >= 2U ? read_u16_le(payload) : 0U;
    HilExecutionMode execution_mode = payload_len == 3U
        ? (HilExecutionMode)payload[2]
        : HIL_EXECUTION_PERIODIC;
    if (payload_len >= 2U && (hil_timeout_ms < 10U || hil_timeout_ms > 5000U)) {
      return COMM_STATUS_OVERLIMIT;
    }
    if (execution_mode > HIL_EXECUTION_STEPPED) return COMM_STATUS_UNKNOWN_PARAM;
    (void)control_mode_set(CONTROL_RUNTIME_HIL_SIM);
    hil_start(hil_timeout_ms, execution_mode);
    return COMM_STATUS_OK;
  }

  case REQUEST_HIL_STOP:
    if (payload_len != 0U) return COMM_STATUS_BAD_LENGTH;
    hil_stop();
    return COMM_STATUS_OK;

  case REQUEST_HIL_SET_INPUTS:
    if (payload_len != 8U && payload_len != 16U) return COMM_STATUS_BAD_LENGTH;
    if (hil_session_state == (uint8_t)HIL_EXECUTION_STEPPED + 1U) {
      return COMM_STATUS_INVALID_STATE;
    }
    if (payload[7] == 0U) {
      if (hil_session_state != 0U) hil_stop();
      return COMM_STATUS_OK;
    }
    if (hil_session_state == 0U && payload[7] != 0U) {
      (void)control_mode_set(CONTROL_RUNTIME_HIL_SIM);
      hil_start(0U, HIL_EXECUTION_PERIODIC);
    }
    hil_set_inputs(read_u16_le(payload),
                   read_i16_le(&payload[4]),
                   payload[6],
                   payload_len == 16U ? read_u32_le(&payload[8]) : 0U,
                   payload_len == 16U ? read_u32_le(&payload[12]) : 0U);
    return COMM_STATUS_OK;

  case REQUEST_HIL_GET_OUTPUTS: {
    if (payload_len != 0U) return COMM_STATUS_BAD_LENGTH;
    *payload_out_len = serialize_hil_outputs(payload_out);
    return COMM_STATUS_OK;
  }

  case REQUEST_HIL_STEP: {
    if (payload_len != 18U) return COMM_STATUS_BAD_LENGTH;
    if (hil_session_state != (uint8_t)HIL_EXECUTION_STEPPED + 1U ||
        payload[7] != 1U) {
      return COMM_STATUS_INVALID_STATE;
    }
    if (read_u16_le(&payload[2]) != 0U || read_i16_le(&payload[4]) != 0) {
      return COMM_STATUS_UNKNOWN_PARAM;
    }

    uint32_t run_id = read_u32_le(&payload[8]);
    uint32_t source_seq = read_u32_le(&payload[12]);
    uint16_t requested_steps = read_u16_le(&payload[16]);
    if (run_id == 0U || source_seq == 0U) return COMM_STATUS_INVALID_STATE;
    if (requested_steps == 0U) return COMM_STATUS_UNDERLIMIT;
    if (requested_steps > HIL_MAX_STEP_COUNT) return COMM_STATUS_OVERLIMIT;

    HilStepResult step_result = hil_step(read_u16_le(payload),
                                         payload[6],
                                         run_id,
                                         source_seq,
                                         requested_steps);
    if (step_result == HIL_STEP_INVALID) return COMM_STATUS_INVALID_STATE;

    serialize_hil_outputs(payload_out);
    write_u16_le(&payload_out[43], requested_steps);
    write_u16_le(&payload_out[45], requested_steps);
    payload_out[47] = step_result == HIL_STEP_REPLAYED ? 0x01U : 0x00U;
    *payload_out_len = 48U;
    return COMM_STATUS_OK;
  }

  case REQUEST_SINE_DRIVE: {
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

  case REQUEST_STEP_TEST: {
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

  case REQUEST_ZC_DIAG: {
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
