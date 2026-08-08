#ifndef INC_CURRENT_SENSE_H_
#define INC_CURRENT_SENSE_H_

#include "stm32f1xx_hal.h"
#include <stdbool.h>
#include <stdint.h>

#define CURRENT_SENSE_FLAG_VALID       (1U << 0)
#define CURRENT_SENSE_FLAG_CALIBRATED  (1U << 1)
#define CURRENT_SENSE_FLAG_SATURATED   (1U << 2)
#define CURRENT_SENSE_FLAG_OVERCURRENT (1U << 3)

typedef struct {
  uint16_t raw_adc;
  int32_t current_ma;
  uint16_t valid_samples;
  uint8_t flags;
  int8_t commutation_step;
  uint32_t tick_ms;
} CurrentSenseChannelSnapshot;

typedef struct {
  CurrentSenseChannelSnapshot phase_u;
  CurrentSenseChannelSnapshot phase_v;
} CurrentSenseSnapshot;

void current_sense_init(void);
void current_sense_handle_conversion(void);
void current_sense_update_pwm_window(uint16_t active_compare);
void current_sense_get_snapshot(CurrentSenseSnapshot* snapshot);

#endif /* INC_CURRENT_SENSE_H_ */
