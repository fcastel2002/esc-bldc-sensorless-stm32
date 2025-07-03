/*
 * hard_config.h
 *
 *  Created on: Feb 22, 2025
 *      Author: Usuario
 */

#ifndef INC_HARD_CONFIG_H_
#define INC_HARD_CONFIG_H_
#include "main.h"
#include "flash_config.h"
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
 * 			- Velocidad nominal motor: RPM (Los ESC comerciales tienen límite de velocidad, todavia no ha sido determinado para este)
 *
 *
 *
 */
 #pragma pack(push,4)

typedef struct{
	uint32_t signature;

	uint16_t pwm_freq_hz;
	uint8_t brake_type;
	float current_limit;
	uint16_t temp_limit;
	uint16_t deadtime_ns;

	float speed_kp;
	float speed_ki;
	float speed_kd;
	uint16_t speed_max_rpm;

	uint32_t crc32;
	uint8_t pole_pairs;
}ESCparams;

#pragma pack(pop)



typedef enum{
	CONFIG_OK,
	CONFIG_ERROR_OVERLIMIT,
	CONFIG_ERROR_UNDERLIMIT,
	CONFIG_ERROR_NaN,
	CONFIG_UNSETTED,
	CONFIG_UNKNOWN,


}ConfigStatus;



extern ESCparams current_esc_params;
extern ConfigStatus current_config_status;
static volatile uint16_t pwm_freq_arr = 0;
static volatile uint16_t current_limit_adc = 0;
static volatile uint16_t max_speed_arr = 0;

static const uint16_t ESC_max_pwm_freq = 24000; // hz
static const uint16_t ESC_min_pwm_freq = 5000;
static const float ESC_max_current = 3; // amp
static const uint16_t ESC_max_speed = 12000; // rpm
static const uint16_t ESC_min_speed = 600;

extern volatile uint8_t esc_config_done;

extern void set_default_esc_params();
extern void update_all_esc();
extern ConfigStatus set_pwm_freq(uint16_t new_freq);
extern ConfigStatus set_max_speed(uint16_t new_speed);
extern ConfigStatus set_current_limit(uint16_t new_current);
extern ConfigStatus setPolePairs(uint8_t new_pole_pairs);
extern ConfigStatus setKP(float new_kp);
extern ConfigStatus setKI(float new_ki);
extern ConfigStatus setKD(float new_kd);

extern uint16_t get_max_speed();
extern uint16_t get_pwm_freq();
extern uint16_t get_current_limit();
extern uint8_t getPolePairs();
extern float getKP();
extern float getKI();
extern float getKD();
uint32_t compute_crc32(ESCparams *params);



#endif /* INC_HARD_CONFIG_H_ */
