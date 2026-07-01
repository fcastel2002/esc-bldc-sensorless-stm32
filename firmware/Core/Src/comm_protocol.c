#include "comm_protocol.h"

#include "comm.h"
#include "comm_transport.h"
#include "flash_config.h"
#include "hard_config.h"
#include "motor_control.h"
#include "speed_sensor.h"
#include "startup.h"
#include "state_machine.h"

#include <string.h>

#define DEFAULT_PWM_FREQ_HZ 18000U
#define DEFAULT_POLE_PAIRS 2U
#define DEFAULT_KP_CENTI 75
#define DEFAULT_KI_CENTI 135
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

static bool control_state_allows_command(void)
{
  return app_state == IDLE || app_state == RUNNING || app_state == CLOSEDLOOP;
}

__attribute__((optimize("Os"))) static void apply_config_change(uint8_t param)
{
  if (app_state == CLOSEDLOOP) {
    if (param == COMM_PARAM_KP || param == COMM_PARAM_KI || param == COMM_PARAM_KD) {
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
  case COMM_PARAM_KP:
    write_i16_le(payload, float_to_centi(get_KP()));
    *payload_len = 2;
    return COMM_STATUS_OK;
  case COMM_PARAM_KI:
    write_i16_le(payload, float_to_centi(get_KI()));
    *payload_len = 2;
    return COMM_STATUS_OK;
  case COMM_PARAM_KD:
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
  case COMM_PARAM_CURRENT_LIMIT:
  case COMM_PARAM_TEMP_LIMIT:
    return COMM_STATUS_NOT_IMPLEMENTED;
  default:
    return COMM_STATUS_UNKNOWN_PARAM;
  }
}

static uint8_t set_config(uint8_t param, const uint8_t* payload, uint16_t payload_len)
{
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
  case COMM_PARAM_KP:
    if (payload_len != 2U) return COMM_STATUS_BAD_LENGTH;
    result = set_KP(centi_to_float(read_i16_le(payload)));
    break;
  case COMM_PARAM_KI:
    if (payload_len != 2U) return COMM_STATUS_BAD_LENGTH;
    result = set_KI(centi_to_float(read_i16_le(payload)));
    break;
  case COMM_PARAM_KD:
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
  case COMM_PARAM_CURRENT_LIMIT:
  case COMM_PARAM_TEMP_LIMIT:
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
  switch (param) {
  case COMM_PARAM_PWM_FREQ:
  case COMM_PARAM_POLE_PAIRS:
  case COMM_PARAM_KP:
  case COMM_PARAM_KI:
  case COMM_PARAM_KD:
  case COMM_PARAM_MAX_SPEED:
  case COMM_PARAM_MIN_SPEED:
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
  case COMM_PARAM_KP:
    current_esc_params.speed_kp = centi_to_float(DEFAULT_KP_CENTI);
    flash_config_parameter_changed();
    break;
  case COMM_PARAM_KI:
    current_esc_params.speed_ki = centi_to_float(DEFAULT_KI_CENTI);
    flash_config_parameter_changed();
    break;
  case COMM_PARAM_KD:
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
  case COMM_PARAM_CURRENT_LIMIT:
  case COMM_PARAM_TEMP_LIMIT:
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
    *variable = VAR_CURRENT;
    return COMM_STATUS_OK;
  default:
    return COMM_STATUS_UNKNOWN_PARAM;
  }
}

static uint8_t execute_request(const uint8_t request[COMM_FRAME_SIZE],
                               uint16_t payload_len,
                               uint8_t* payload_out,
                               uint16_t* payload_out_len)
{
  const uint8_t opcode = request[5];
  const uint8_t param = request[6];
  const uint8_t* payload = &request[COMM_PAYLOAD_OFFSET];

  *payload_out_len = 0;

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
    *payload_out_len = 10;
    return COMM_STATUS_OK;
  }

  case COMM_OPCODE_RUN:
    if (payload_len != 0U) return COMM_STATUS_BAD_LENGTH;
    if (!control_state_allows_command()) return COMM_STATUS_INVALID_STATE;
    foc_startup();
    return COMM_STATUS_OK;

  case COMM_OPCODE_STOP:
    if (payload_len != 0U) return COMM_STATUS_BAD_LENGTH;
    if (!control_state_allows_command()) return COMM_STATUS_INVALID_STATE;
    stop_motor(0);
    app_state = IDLE;
    return COMM_STATUS_OK;

  case COMM_OPCODE_ESTOP:
    if (payload_len != 0U) return COMM_STATUS_BAD_LENGTH;
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

  case COMM_OPCODE_SET_CONTROL_MODE:
    if (payload_len != 1U) return COMM_STATUS_BAD_LENGTH;
    return control_mode_set(payload[0]) ? COMM_STATUS_OK : COMM_STATUS_UNKNOWN_PARAM;

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
      start_logging_param(VAR_TEMP);
      start_logging_param(VAR_CURRENT);
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

  case COMM_OPCODE_HIL_START:
    if (payload_len != 0U) return COMM_STATUS_BAD_LENGTH;
    (void)control_mode_set(CONTROL_RUNTIME_HIL_SIM);
    return hil_start() ? COMM_STATUS_OK : COMM_STATUS_INVALID_STATE;

  case COMM_OPCODE_HIL_STOP:
    if (payload_len != 0U) return COMM_STATUS_BAD_LENGTH;
    hil_stop();
    return COMM_STATUS_OK;

  case COMM_OPCODE_HIL_SET_INPUTS:
    if (payload_len != 8U) return COMM_STATUS_BAD_LENGTH;
    if (!hil_is_active() && payload[7] != 0U) {
      (void)control_mode_set(CONTROL_RUNTIME_HIL_SIM);
      (void)hil_start();
    }
    if (!hil_is_active()) return COMM_STATUS_INVALID_STATE;
    if (payload[7] == 0U) {
      hil_stop();
      return COMM_STATUS_OK;
    }
    hil_set_inputs(read_u16_le(payload),
                   read_i16_le(&payload[4]),
                   payload[6]);
    return COMM_STATUS_OK;

  case COMM_OPCODE_HIL_GET_OUTPUTS:
    if (payload_len != 0U) return COMM_STATUS_BAD_LENGTH;
    write_u32_le(&payload_out[0], HAL_GetTick());
    payload_out[4] = (uint8_t)app_state;
    payload_out[5] = (uint8_t)control_runtime_mode;
    write_u16_le(&payload_out[6], speed_setpoint_rpm);
    write_u16_le(&payload_out[8], hil_is_active() ? hil_get_speed_rpm() : period_to_rpm(get_actual_speed()));
    write_u16_le(&payload_out[10], hil_get_pwm_command());
    payload_out[12] = (uint8_t)bldc_get_commutation_step();
    payload_out[13] = hil_get_flags();
    payload_out[14] = hil_has_timeout();
    *payload_out_len = 15;
    return COMM_STATUS_OK;

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
