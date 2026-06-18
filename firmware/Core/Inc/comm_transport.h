#ifndef INC_COMM_TRANSPORT_H_
#define INC_COMM_TRANSPORT_H_

#include "comm_protocol.h"
#include <stdbool.h>
#include <stdint.h>

typedef enum {
  COMM_TRANSPORT_UART = 0,
  COMM_TRANSPORT_USB = 1,
} CommTransportMode;

void comm_transport_init(void);
void comm_transport_process(void);
bool comm_transport_send_frame(const uint8_t frame[COMM_FRAME_SIZE]);
CommTransportMode comm_transport_get_mode(void);
bool comm_transport_is_usb_ready(void);
void comm_transport_receive_usb_report(const uint8_t* report, uint16_t len);

#endif /* INC_COMM_TRANSPORT_H_ */

