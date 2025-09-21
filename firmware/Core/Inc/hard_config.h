/*
 * hard_config.h
 *
 *  Created on: Feb 22, 2025
 *      Author: Usuario
 */

#ifndef INC_HARD_CONFIG_H_
#define INC_HARD_CONFIG_H_
#include "flash_config.h"
#include "main.h"
/*
 * @brief
 * Esta configuración será guardada en flash, de tal forma que el usuario pueda
 * definir los parámetros que desee para el ESC.
 *
 * En su primer versión, los parámetros serán configurados mediante UART.
 * Los parámetros son:
 *
 * 			- Frecuencia de PWM : 8kHz - 24kHz
 * 			- BREAK MODE: ON/OFF
 * 			- Límite de corriente: A
 * 			- Límite de temperatura: °C/F
 * 			- PWM Deadtime: ns
 * 			- Constante Kp control velocidad
 * 			- Constante Ki control velocidad
 * 			- Constante Kd control velocidad
 * 			- Velocidad nominal motor: RPM (Los ESC comerciales tienen límite de velocidad, todavia no
 * ha sido determinado para este)
 *
 *
 *
 */
#pragma pack(push, 4)

typedef struct {
  uint32_t signature;

  uint16_t pwm_freq_hz;
  uint8_t  brake_type;
  float    current_limit;
  uint16_t temp_limit;

  float    speed_kp;
  float    speed_ki;
  float    speed_kd;
  uint16_t speed_max_rpm;
  uint16_t speed_min_rpm;

  uint32_t crc32;
  uint8_t  pole_pairs;
} ESCparams;

#pragma pack(pop)

typedef enum {
  CONFIG_OK,
  CONFIG_ERROR_OVERLIMIT,
  CONFIG_ERROR_UNDERLIMIT,
  CONFIG_ERROR_NaN,
  CONFIG_UNSETTED,
  CONFIG_UNKNOWN,

} ConfigStatus;

extern ESCparams         current_esc_params;
extern ConfigStatus      current_config_status;

extern volatile uint8_t esc_config_done;

extern void         set_default_esc_params();
extern void         update_all_esc();
extern ConfigStatus set_pwm_freq(uint16_t new_freq);
extern ConfigStatus set_current_limit(uint16_t new_current);
extern ConfigStatus set_pole_pairs(uint8_t new_pole_pairs);
extern ConfigStatus set_KP(float new_kp);
extern ConfigStatus set_KI(float new_ki);
extern ConfigStatus set_KD(float new_kd);
extern ConfigStatus set_min_speed(uint16_t new_speed);
extern ConfigStatus set_max_speed(uint16_t new_speed);

extern uint16_t get_max_speed();
extern uint16_t get_min_speed();
extern uint16_t get_pwm_freq();
extern uint16_t get_current_limit();
extern uint8_t  get_pole_pairs();
extern float    get_KP();
extern float    get_KI();
extern float    get_KD();
uint32_t        compute_crc32(ESCparams* params);

#endif /* INC_HARD_CONFIG_H_ */
