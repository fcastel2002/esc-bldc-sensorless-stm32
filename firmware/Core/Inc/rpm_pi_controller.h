#ifndef INC_RPM_PI_CONTROLLER_H_
#define INC_RPM_PI_CONTROLLER_H_

#include <stdint.h>

#define RPM_PI_ALGORITHM_VERSION 2U
#define RPM_PI_CANONICAL_ARR 2000U
#define RPM_PI_CANONICAL_MIN 100U
#define RPM_PI_Q_SHIFT 16U
#define RPM_PI_DT_SECONDS 0.002f

typedef struct {
  int32_t kp_q16;
  int32_t ki_dt_half_q30;
  int32_t integral_q16;
  int32_t previous_error_rpm;
  uint16_t canonical_output;
} RpmPiController;

void rpm_pi_configure(RpmPiController* controller, float kp, float ki);
void rpm_pi_reset(RpmPiController* controller);
void rpm_pi_prepare_bumpless(RpmPiController* controller,
                             uint16_t target_rpm,
                             uint16_t measured_rpm,
                             uint16_t current_pwm,
                             uint16_t active_arr);
uint16_t rpm_pi_step(RpmPiController* controller,
                     uint16_t target_rpm,
                     uint16_t measured_rpm,
                     uint16_t active_arr);

#endif /* INC_RPM_PI_CONTROLLER_H_ */
