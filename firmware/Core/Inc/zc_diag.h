#ifndef INC_ZC_DIAG_H_
#define INC_ZC_DIAG_H_

#include "main.h"

#include <stdbool.h>
#include <stdint.h>

#define ZC_DIAG_HEALTH_HAS_MOTION       (1U << 0)
#define ZC_DIAG_HEALTH_SEQ_VALID        (1U << 1)
#define ZC_DIAG_HEALTH_DIR_FORWARD      (1U << 2)
#define ZC_DIAG_HEALTH_DIR_REVERSE      (1U << 3)
#define ZC_DIAG_HEALTH_CH0_DEAD         (1U << 4)
#define ZC_DIAG_HEALTH_CH1_DEAD         (1U << 5)
#define ZC_DIAG_HEALTH_CH2_DEAD         (1U << 6)
#define ZC_DIAG_HEALTH_NOISE_OR_INVALID (1U << 7)

typedef enum {
  ZC_AUTO_DETECT_OK = 0,
  ZC_AUTO_DETECT_OK_INVERTED = 1,
  ZC_AUTO_DETECT_NO_SIGNAL = 2,
  ZC_AUTO_DETECT_AMBIGUOUS = 3,
} ZcAutoDetectCode;

typedef struct {
  uint8_t  active;
  uint8_t  live_pins;
  uint8_t  seen_states_mask;
  uint8_t  health_flags;
  uint16_t edges[3];
  uint16_t forward_steps;
  uint16_t reverse_steps;
  uint16_t invalid_steps;
  uint16_t glitch_count;
  uint16_t estimated_rpm;
  uint16_t avg_step_ticks;
  uint8_t  step_asymmetry_pct;
  uint8_t  active_map[3];
  uint8_t  recent_events[6];
} ZcDiagReport;

typedef struct {
  uint8_t result_code;
  uint8_t detected_map[3];
  uint8_t active_map[3];
  uint8_t high_pct[9];
} ZcAutoDetectResult;

bool zc_diag_freewheel_start(void);
void zc_diag_freewheel_stop(void);
bool zc_diag_is_freewheel_active(void);
void zc_diag_handle_tim2_capture(TIM_HandleTypeDef* htim);
void zc_diag_get_report(ZcDiagReport* report);
bool zc_diag_auto_detect(uint16_t duty_permille, ZcAutoDetectResult* result);

#endif /* INC_ZC_DIAG_H_ */
