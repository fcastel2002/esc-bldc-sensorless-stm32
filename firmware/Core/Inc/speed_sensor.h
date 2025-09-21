#ifndef INC_SPEED_SENSOR_H_
#define INC_SPEED_SENSOR_H_

#define ZCP_BUFFER_SIZE 4
#define PHASE_COUNT 3
#define SPEED_CONSENSUS_THRESHOLD 15 // % de tolerancia entre fases
#include <stdint.h>
#include <stdbool.h>

// Estructura para cada fase
typedef struct {
  uint16_t last_timestamp;
  uint16_t periods[ZCP_BUFFER_SIZE];
  uint8_t  period_idx;
  uint8_t  valid_periods;
  uint16_t avg_period;
  uint8_t  is_consistent;
} PhaseMeasurementThreePhase;


extern uint16_t period_to_pwm(uint16_t period);
extern uint16_t period_to_rpm(uint16_t period);
extern uint16_t rpm_to_period(uint16_t rpm);
extern uint16_t get_actual_speed(void);

// api externa
extern void speed_sensor_init(void);
extern void speed_sensor_zc_detected(uint8_t phase, uint16_t timestamp);
extern uint16_t speed_sensor_get_speed_rpm(void);
extern uint16_t speed_sensor_get_speed_period(void);
extern bool speed_sensor_is_ready(void);
extern void speed_sensor_handle_W_measurement(void);
extern void speed_sensor_handle_consensus(void);
extern uint16_t speed_sensor_get_speed_range(void);
#endif /* INC_SPEED_SENSOR_H_ */