#include "comm_transport.h"

#include "comm.h"
#include "gpio.h"
#include "usart.h"
#include "usb_device.h"
#include "usbd_customhid.h"

#include <string.h>

static CommTransportMode active_mode = COMM_TRANSPORT_USB;
static uint8_t uart_frame[COMM_FRAME_SIZE];
static uint8_t uart_frame_index = 0;

static volatile uint8_t usb_report_pending = 0;
static volatile uint16_t usb_report_len = 0;
static uint8_t usb_report[COMM_FRAME_SIZE];

extern USBD_HandleTypeDef USBD_Device;

#define USB_SEND_RETRY_TIMEOUT_MS 10U

CommTransportMode comm_transport_get_mode(void)
{
  return active_mode;
}

bool comm_transport_is_usb_ready(void)
{
  return active_mode == COMM_TRANSPORT_USB &&
         USBD_Device.dev_state == USBD_STATE_CONFIGURED;
}

void comm_transport_init(void)
{
  active_mode = (HAL_GPIO_ReadPin(COMM_MODE_GPIO_Port, COMM_MODE_Pin) == GPIO_PIN_RESET)
                  ? COMM_TRANSPORT_UART
                  : COMM_TRANSPORT_USB;

  if (active_mode == COMM_TRANSPORT_USB) {
    MX_USB_DEVICE_Init();
  } else {
    MX_USART1_UART_Init();
    (void)HAL_UART_Receive_IT(&huart1, (uint8_t*)&rx_buffer[rx_head], 1);
  }
}

static void handle_frame(const uint8_t frame[COMM_FRAME_SIZE])
{
  uint8_t response[COMM_FRAME_SIZE];

  if (comm_protocol_handle_frame(frame, response)) {
    (void)comm_transport_send_frame(response);
  }
}

static void process_uart(void)
{
  while (rx_tail != rx_head) {
    uint8_t byte = rx_buffer[rx_tail];
    rx_tail = (uint16_t)((rx_tail + 1U) % BUFFER_SIZE);

    uart_frame[uart_frame_index++] = byte;
    if (uart_frame_index >= COMM_FRAME_SIZE) {
      handle_frame(uart_frame);
      uart_frame_index = 0;
    }
  }
}

static void process_usb(void)
{
  uint8_t local_report[COMM_FRAME_SIZE];
  uint16_t local_len = 0;

  __disable_irq();
  if (usb_report_pending) {
    memcpy(local_report, usb_report, COMM_FRAME_SIZE);
    local_len = usb_report_len;
    usb_report_pending = 0;
  }
  __enable_irq();

  if (local_len == COMM_FRAME_SIZE) {
    handle_frame(local_report);
  }
}

void comm_transport_process(void)
{
  if (active_mode == COMM_TRANSPORT_USB) {
    process_usb();
  } else {
    process_uart();
  }
}

bool comm_transport_send_frame(const uint8_t frame[COMM_FRAME_SIZE])
{
  if (active_mode == COMM_TRANSPORT_USB) {
    if (!comm_transport_is_usb_ready()) {
      return false;
    }

    uint32_t start_tick = HAL_GetTick();
    do {
      uint8_t result = USBD_CUSTOM_HID_SendReport(&USBD_Device, (uint8_t*)frame, COMM_FRAME_SIZE);
      if (result == USBD_OK) {
        return true;
      }
    } while ((uint32_t)(HAL_GetTick() - start_tick) < USB_SEND_RETRY_TIMEOUT_MS);

    return false;
  }

  return HAL_UART_Transmit(&huart1, (uint8_t*)frame, COMM_FRAME_SIZE, 20) == HAL_OK;
}

void comm_transport_receive_usb_report(const uint8_t* report, uint16_t len)
{
  if (report == NULL || len != COMM_FRAME_SIZE || usb_report_pending) {
    return;
  }

  memcpy(usb_report, report, COMM_FRAME_SIZE);
  usb_report_len = len;
  usb_report_pending = 1;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef* huart)
{
  if (active_mode != COMM_TRANSPORT_UART || huart->Instance != USART1) {
    return;
  }

  uint16_t next_head = (uint16_t)((rx_head + 1U) % BUFFER_SIZE);
  if (next_head == rx_tail) {
    rx_tail = (uint16_t)((rx_tail + 1U) % BUFFER_SIZE);
  }
  rx_head = next_head;

  (void)HAL_UART_Receive_IT(&huart1, (uint8_t*)&rx_buffer[rx_head], 1);
}
