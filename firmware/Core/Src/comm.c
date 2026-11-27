/*
 * comm.c
 *
 * Binary communication facade.
 */

#include "comm.h"

#include "comm_protocol.h"
#include "comm_transport.h"
#include "bldc_driver.h"
#include "motor_control.h"
#include "power_monitor.h"
#include "speed_sensor.h"

volatile uint8_t rx_buffer[BUFFER_SIZE];
volatile uint16_t rx_head = 0;
volatile uint16_t rx_tail = 0;

volatile uint8_t cmd_received_ack = 0;
volatile uint8_t cmd_speed_received_ack = 0;
volatile uint16_t loggin_rate_ms = 1000;

static LoggingQueue logging_queue = { .count = 0 };

void commInit(void)
{
  rx_head = 0;
  rx_tail = 0;
  memset((void*)rx_buffer, 0, sizeof(rx_buffer));
  comm_transport_init();
  power_monitor_init(comm_transport_get_mode() == COMM_TRANSPORT_USB);
}

void processUartData(void)
{
  comm_transport_process();
}

void processCurrentCommand(void)
{
  comm_transport_process();
}

uint8_t process_speed_command(void)
{
  return 0;
}

void handleCommandEffects(void)
{
}

static uint8_t logging_id_to_protocol(LoggeableVariable variable)
{
  switch (variable) {
  case VAR_SPEED:
    return COMM_LOG_PARAM_SPEED;
  case VAR_BUS_VOLTAGE:
    return COMM_LOG_PARAM_BUS_VOLTAGE;
  case VAR_BUS_CURRENT:
    return COMM_LOG_PARAM_BUS_CURRENT;
  case VAR_DUTY:
    return COMM_LOG_PARAM_DUTY;
  default:
    return 0;
  }
}

typedef struct {
  int32_t value;
  uint16_t raw;
  uint16_t valid_samples;
  uint8_t flags;
  int8_t commutation_step;
} LoggingValue;

static LoggingValue logging_value(LoggeableVariable variable, const PowerMonitorSnapshot* power)
{
  LoggingValue result = { .commutation_step = bldc_get_commutation_step() };
  switch (variable) {
  case VAR_SPEED:
    result.value = (int32_t)period_to_rpm(get_actual_speed());
    result.raw = speed_sensor_get_speed_period();
    result.flags = speed_sensor_get_bemf_quality_flags();
    return result;
  case VAR_BUS_VOLTAGE:
    result.value = power->bus_mv;
    result.raw = power->bus_mv;
    result.flags = power->flags;
    return result;
  case VAR_BUS_CURRENT:
    result.value = power->current_ua / 1000; /* mA */
    result.raw = (uint16_t)power->shunt_raw; /* 10 uV/LSB, signed */
    result.flags = power->flags;
    return result;
  case VAR_DUTY:
    result.value = motor_control_get_duty_e4(); /* 1/10000 */
    result.raw = bldc_get_pwm();                /* compare counts */
    result.flags = COMM_TELEMETRY_FLAG_VALID;
    return result;
  default:
    return result;
  }
}

void process_logging_queue(void)
{
  static uint32_t last_log_tick = 0;
  uint32_t now = HAL_GetTick();

  if ((uint32_t)(now - last_log_tick) < loggin_rate_ms) {
    return;
  }
  last_log_tick = now;

  PowerMonitorSnapshot power_snapshot;
  power_monitor_get_snapshot(&power_snapshot);

  for (uint8_t i = 0; i < logging_queue.count; i++) {
    uint8_t frame[COMM_FRAME_SIZE];
    uint8_t payload[15];
    LoggeableVariable variable = logging_queue.params[i];
    LoggingValue reading = logging_value(variable, &power_snapshot);
    uint32_t tick = HAL_GetTick();

    payload[0] = logging_id_to_protocol(variable);
    payload[1] = (uint8_t)(reading.value & 0xFF);
    payload[2] = (uint8_t)((uint32_t)reading.value >> 8);
    payload[3] = (uint8_t)((uint32_t)reading.value >> 16);
    payload[4] = (uint8_t)((uint32_t)reading.value >> 24);
    payload[5] = (uint8_t)(tick & 0xFF);
    payload[6] = (uint8_t)(tick >> 8);
    payload[7] = (uint8_t)(tick >> 16);
    payload[8] = (uint8_t)(tick >> 24);
    payload[9] = (uint8_t)(reading.raw & 0xFFU);
    payload[10] = (uint8_t)(reading.raw >> 8);
    payload[11] = reading.flags;
    payload[12] = (uint8_t)(reading.valid_samples & 0xFFU);
    payload[13] = (uint8_t)(reading.valid_samples >> 8);
    payload[14] = (uint8_t)reading.commutation_step;

    comm_protocol_build_event(COMM_OPCODE_TELEMETRY_EVENT,
                              payload[0],
                              payload,
                              sizeof(payload),
                              frame);
    (void)comm_transport_send_frame(frame);
  }
}

void start_logging_param(LoggeableVariable variable)
{
  if (logging_queue.count >= MAX_LOGGED_PARAMS) {
    return;
  }

  for (uint8_t i = 0; i < logging_queue.count; i++) {
    if (logging_queue.params[i] == variable) {
      return;
    }
  }

  logging_queue.params[logging_queue.count++] = variable;
}

void stop_logging_param(LoggeableVariable variable)
{
  for (uint8_t i = 0; i < logging_queue.count; i++) {
    if (logging_queue.params[i] == variable) {
      for (uint8_t j = i; j + 1 < logging_queue.count; j++) {
        logging_queue.params[j] = logging_queue.params[j + 1];
      }
      logging_queue.count--;
      return;
    }
  }
}

void stop_logging_all(void)
{
  logging_queue.count = 0;
}

uint8_t set_logging_rate_ms(uint16_t rate_ms)
{
  if (rate_ms < 100U) {
    return COMM_STATUS_UNDERLIMIT;
  }
  if (rate_ms > 5000U) {
    return COMM_STATUS_OVERLIMIT;
  }
  loggin_rate_ms = rate_ms;
  return COMM_STATUS_OK;
}
