#include "rpm_pi_controller.h"

#include <limits.h>

static int32_t clamp_i64_to_i32(int64_t value)
{
  if (value > INT32_MAX) return INT32_MAX;
  if (value < INT32_MIN) return INT32_MIN;
  return (int32_t)value;
}

static int64_t clamp_i64(int64_t value, int64_t minimum, int64_t maximum)
{
  if (value < minimum) return minimum;
  if (value > maximum) return maximum;
  return value;
}

static int32_t gain_to_q16(float gain)
{
  return (int32_t)(gain * 65536.0f + 0.5f);
}

static int32_t integral_step_to_q30(float gain)
{
  const float dt_half = RPM_PI_DT_SECONDS * 0.5f;
  return (int32_t)(gain * dt_half * 1073741824.0f + 0.5f);
}

static int64_t proportional_q16(const RpmPiController* controller, int32_t error_rpm)
{
  return (int64_t)controller->kp_q16 * error_rpm;
}

static uint16_t scale_to_active_arr(uint16_t canonical_pwm, uint16_t active_arr)
{
  uint32_t scaled = (uint32_t)canonical_pwm * active_arr + (RPM_PI_CANONICAL_ARR / 2U);
  return (uint16_t)(scaled / RPM_PI_CANONICAL_ARR);
}

void rpm_pi_configure(RpmPiController* controller, float kp, float ki)
{
  controller->kp_q16 = gain_to_q16(kp);
  controller->ki_dt_half_q30 = integral_step_to_q30(ki);
}

void rpm_pi_reset(RpmPiController* controller)
{
  controller->integral_q16 = 0;
  controller->previous_error_rpm = 0;
  controller->canonical_output = RPM_PI_CANONICAL_MIN;
}

void rpm_pi_prepare_bumpless(RpmPiController* controller,
                             uint16_t target_rpm,
                             uint16_t measured_rpm,
                             uint16_t current_pwm,
                             uint16_t active_arr)
{
  int32_t error_rpm = (int32_t)target_rpm - measured_rpm;
  int64_t p_q16 = proportional_q16(controller, error_rpm);
  uint32_t canonical = active_arr == 0U
      ? RPM_PI_CANONICAL_MIN
      : ((uint32_t)current_pwm * RPM_PI_CANONICAL_ARR + active_arr / 2U) / active_arr;
  canonical = (uint32_t)clamp_i64(canonical, RPM_PI_CANONICAL_MIN, RPM_PI_CANONICAL_ARR);
  int64_t desired_q16 = (int64_t)canonical << RPM_PI_Q_SHIFT;
  int64_t minimum_integral = ((int64_t)RPM_PI_CANONICAL_MIN << RPM_PI_Q_SHIFT) - p_q16;
  int64_t maximum_integral = ((int64_t)RPM_PI_CANONICAL_ARR << RPM_PI_Q_SHIFT) - p_q16;
  controller->integral_q16 = clamp_i64_to_i32(
      clamp_i64(desired_q16 - p_q16, minimum_integral, maximum_integral));
  controller->previous_error_rpm = error_rpm;
  controller->canonical_output = (uint16_t)canonical;
}

uint16_t rpm_pi_step(RpmPiController* controller,
                     uint16_t target_rpm,
                     uint16_t measured_rpm,
                     uint16_t active_arr)
{
  int32_t error_rpm = (int32_t)target_rpm - measured_rpm;
  int32_t error_sum = error_rpm + controller->previous_error_rpm;
  int64_t p_q16 = proportional_q16(controller, error_rpm);
  int64_t integral_delta_q16 =
      ((int64_t)controller->ki_dt_half_q30 * error_sum) >> 14;
  int64_t integral_q16 = (int64_t)controller->integral_q16 + integral_delta_q16;
  int64_t minimum_output_q16 = (int64_t)RPM_PI_CANONICAL_MIN << RPM_PI_Q_SHIFT;
  int64_t maximum_output_q16 = (int64_t)RPM_PI_CANONICAL_ARR << RPM_PI_Q_SHIFT;
  integral_q16 = clamp_i64(integral_q16,
                           minimum_output_q16 - p_q16,
                           maximum_output_q16 - p_q16);
  controller->integral_q16 = clamp_i64_to_i32(integral_q16);
  controller->previous_error_rpm = error_rpm;

  int64_t output_q16 = clamp_i64(p_q16 + controller->integral_q16,
                                 minimum_output_q16,
                                 maximum_output_q16);
  controller->canonical_output = (uint16_t)(output_q16 >> RPM_PI_Q_SHIFT);
  return scale_to_active_arr(controller->canonical_output, active_arr);
}
