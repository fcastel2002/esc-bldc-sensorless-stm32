#include "utilities.h"

int8_t safe_mod(int8_t value, int8_t mod)
{
  return (value % mod + mod) % mod;
}

uint16_t convert_speed_ticks(uint16_t value, bool to_ticks)
{
  if (to_ticks) {
    // Conversión RPM -> setpoint (valor para speed_setpoint)
    if (value == 0)
      return 0;

    // Calcular el período entre cruces por cero (en segundos)
    double rpm_mechanical = (double)value;
    double rpm_electrical = rpm_mechanical * motor_pole_pairs;
    double frequency      = rpm_electrical / 60.0; // Hz eléctricos
    double period         = 1.0 / frequency;       // Período eléctrico total (s)
    double zc_period      = period / 6.0;          // Período entre cruces por cero (s)

    // Convertir a ticks del timer de captura (TIM2)
    double timer_ticks = zc_period * (HAL_RCC_GetHCLKFreq() / (TIM2->PSC + 1));

    // Mapear al rango de setpoint usando la misma lógica que map_speed()
    if (timer_ticks > SPEED_MIN)
      timer_ticks = SPEED_MIN;
    else if (timer_ticks < SPEED_MAX)
      timer_ticks = SPEED_MAX;

    uint16_t setpoint =
        (uint16_t)((SPEED_MIN - timer_ticks) * pwm_speed_range_relation + min_limit_pwm);

    return setpoint;
  } else {
    // Conversión setpoint -> RPM
    if (value == 0)
      return 0;

    // Revertir el mapeo de map_speed()
    double timer_ticks = SPEED_MIN - ((double)value - min_limit_pwm) / pwm_speed_range_relation;

    // Convertir ticks a tiempo real
    double zc_period = timer_ticks / (HAL_RCC_GetHCLKFreq() / (TIM2->PSC + 1));
    double period    = zc_period * 6.0; // Período eléctrico total (s)
    double frequency = 1.0 / period;    // Hz eléctricos

    // Convertir a RPM mecánicos
    double rpm_electrical = frequency * 60.0;
    double rpm_mechanical = rpm_electrical / motor_pole_pairs;

    // Limitar a rango operativo del motor
    if (rpm_mechanical < 100)
      return 100;
    if (rpm_mechanical > 7000)
      return 7000;
    return (uint16_t)rpm_mechanical;
  }
}