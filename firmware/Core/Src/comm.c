/*
 * comm.c
 *
 * Binary communication facade.
 */

#include "comm.h"

#include "comm_protocol.h"
#include "comm_transport.h"
#include "speed_sensor.h"

uint8_t rx_index = 0;
uint8_t rx_data[1];
volatile uint8_t rx_buffer[BUFFER_SIZE];
volatile uint16_t rx_head = 0;
volatile uint16_t rx_tail = 0;

volatile uint8_t cmd_received_ack = 0;
volatile uint8_t cmd_speed_received_ack = 0;
volatile uint8_t set_cmd_received_ack = 0;
volatile uint8_t stop_cmd_ack = 0;
volatile uint8_t running_cmd_ack = 0;
volatile uint8_t emergency_cmd_ack = 0;
volatile uint16_t loggin_rate_ms = 1000;
volatile App_States_t cmd_origin_state = IDLE;

static LoggingQueue logging_queue = { .count = 0 };

void commInit(void)
{
  rx_index = 0;
  rx_head = 0;
  rx_tail = 0;
  memset((void*)rx_buffer, 0, sizeof(rx_buffer));
  comm_transport_init();
}

void processUartData(void)
{
  comm_transport_process();
}

void clearRxBuffer(void)
{
  rx_index = 0;
  rx_head = 0;
  rx_tail = 0;
  memset((void*)rx_buffer, 0, sizeof(rx_buffer));
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

void transmitir_UART(const char* formato, ...)
{
  (void)formato;
}

static uint8_t logging_id_to_protocol(LoggeableVariable variable)
{
  switch (variable) {
  case VAR_SPEED:
    return COMM_LOG_PARAM_SPEED;
  case VAR_TEMP:
    return COMM_LOG_PARAM_TEMP;
  case VAR_CURRENT:
    return COMM_LOG_PARAM_CURRENT;
  default:
    return 0;
  }
}

static int32_t logging_value(LoggeableVariable variable)
{
  switch (variable) {
  case VAR_SPEED:
    return (int32_t)period_to_rpm(get_actual_speed());
  case VAR_TEMP:
    return 3640; /* centi-degrees C, placeholder until a real sensor exists. */
  case VAR_CURRENT:
    return 1230; /* mA, placeholder until current feedback is implemented. */
  default:
    return 0;
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

  for (uint8_t i = 0; i < logging_queue.count; i++) {
    uint8_t frame[COMM_FRAME_SIZE];
    uint8_t payload[9];
    LoggeableVariable variable = logging_queue.params[i];
    int32_t value = logging_value(variable);
    uint32_t tick = HAL_GetTick();

    payload[0] = logging_id_to_protocol(variable);
    payload[1] = (uint8_t)(value & 0xFF);
    payload[2] = (uint8_t)((uint32_t)value >> 8);
    payload[3] = (uint8_t)((uint32_t)value >> 16);
    payload[4] = (uint8_t)((uint32_t)value >> 24);
    payload[5] = (uint8_t)(tick & 0xFF);
    payload[6] = (uint8_t)(tick >> 8);
    payload[7] = (uint8_t)(tick >> 16);
    payload[8] = (uint8_t)(tick >> 24);

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
