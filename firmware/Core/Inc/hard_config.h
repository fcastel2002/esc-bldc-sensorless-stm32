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

#define ESC_PARAMS_SIGNATURE_V1 0x50415055U
#define ESC_PARAMS_SIGNATURE_V2 0x32504345U
#define ESC_PARAMS_SIGNATURE_V3 0x33504345U
#define ESC_PARAMS_SIGNATURE_V4 0x34504345U
#define ESC_PARAMS_SIGNATURE_V5 0x35504345U
#define ESC_PARAMS_SIGNATURE_V6 0x36504345U
#define ESC_DEFAULT_KP_RPM 1.00f
#define ESC_DEFAULT_KI_RPM 1.00f
#define ESC_DEFAULT_KD_RPM 0.00f
#define ESC_DEFAULT_MAX_SPEED_RPM 5400U
#define ESC_DEFAULT_MIN_SPEED_RPM 200U
#define ESC_DEFAULT_PWM_FREQ_HZ 18000U
#define ESC_DEFAULT_TEMP_LIMIT 70U // °C
#define ESC_DEFAULT_CURRENT_LIMIT 2.0f // Amp
#define ESC_DEFAULT_POLE_PAIRS 2U
#define ESC_DEFAULT_STARTUP_INITIAL_AMPLITUDE_PERMILLE 200U
#define ESC_DEFAULT_STARTUP_FINAL_AMPLITUDE_PERMILLE 1000U
#define ESC_DEFAULT_STARTUP_INITIAL_FREQUENCY_MILLIHZ 2090U
#define ESC_DEFAULT_STARTUP_FINAL_FREQUENCY_MILLIHZ 9280U
#define ESC_DEFAULT_STARTUP_DURATION_MS 3000U
#define ESC_DEFAULT_BEMF_BLANKING_US 0U
#define ESC_MAX_BEMF_BLANKING_US 200U
#define ESC_DEFAULT_SPEED_RAMP_RPM_S 0U /* 0 = no slew limit */
#define ESC_MAX_SPEED_RAMP_RPM_S 20000U
#define ESC_DEFAULT_ZC_CHANNEL_U 2U
#define ESC_DEFAULT_ZC_CHANNEL_V 1U
#define ESC_DEFAULT_ZC_CHANNEL_W 0U
#define ESC_MIN_ZC_AUTO_DETECT_DUTY_PERMILLE 50U
#define ESC_MAX_ZC_AUTO_DETECT_DUTY_PERMILLE 250U
#define ESC_DEFAULT_ZC_AUTO_DETECT_DUTY_PERMILLE 150U

#define ESC_MIN_SINE_FREQUENCY_MILLIHZ 2000U
#define ESC_MAX_SINE_FREQUENCY_MILLIHZ 10000U
#define ESC_MAX_SINE_AMPLITUDE_PERMILLE 1000U
#define ESC_MAX_STEP_TEST_DUTY_PERMILLE 300U
#define ESC_MIN_STARTUP_DURATION_MS 500U
#define ESC_MAX_STARTUP_DURATION_MS 10000U
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

  uint16_t startup_initial_amplitude_permille;
  uint16_t startup_final_amplitude_permille;
  uint32_t startup_initial_frequency_millihz;
  uint32_t startup_final_frequency_millihz;
  uint32_t startup_duration_ms;
  uint16_t bemf_blanking_us;
  uint8_t  zc_channel_u;
  uint8_t  zc_channel_v;
  uint8_t  zc_channel_w;
  uint8_t  zc_reserved;
  /* V6 */
  uint16_t speed_ramp_rpm_s; /**< Setpoint slew limit, RPM/s; 0 = off. */
  uint16_t reserved_v6;
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
extern ConfigStatus set_pole_pairs(uint8_t new_pole_pairs);
extern ConfigStatus set_KP(float new_kp);
extern ConfigStatus set_KI(float new_ki);
extern ConfigStatus set_KD(float new_kd);
extern ConfigStatus set_min_speed(uint16_t new_speed);
extern ConfigStatus set_max_speed(uint16_t new_speed);
extern ConfigStatus set_startup_initial_amplitude(uint16_t amplitude_permille);
extern ConfigStatus set_startup_final_amplitude(uint16_t amplitude_permille);
extern ConfigStatus set_startup_initial_frequency(uint32_t frequency_millihz);
extern ConfigStatus set_startup_final_frequency(uint32_t frequency_millihz);
extern ConfigStatus set_startup_duration(uint32_t duration_ms);
extern ConfigStatus set_bemf_blanking_us(uint16_t blanking_us);
extern ConfigStatus set_zc_channel_map(uint8_t ch_u, uint8_t ch_v, uint8_t ch_w);
extern ConfigStatus set_speed_ramp(uint16_t rpm_per_s);

extern uint16_t get_max_speed();
extern uint16_t get_min_speed();
extern uint16_t get_pwm_freq();
extern uint8_t  get_pole_pairs();
extern float    get_KP();
extern float    get_KI();
extern float    get_KD();
extern uint16_t get_startup_initial_amplitude(void);
extern uint16_t get_startup_final_amplitude(void);
extern uint32_t get_startup_initial_frequency(void);
extern uint32_t get_startup_final_frequency(void);
extern uint32_t get_startup_duration(void);
extern uint16_t get_bemf_blanking_us(void);
extern uint16_t get_speed_ramp(void);
extern bool     zc_channel_map_is_valid(uint8_t ch_u, uint8_t ch_v, uint8_t ch_w);
extern void     get_zc_channel_map(uint8_t* ch_u, uint8_t* ch_v, uint8_t* ch_w);
extern uint32_t zc_phase_to_tim_channel(uint8_t phase);
extern uint32_t zc_phase_to_active_channel(uint8_t phase);
uint32_t        compute_crc32(ESCparams* params);

#endif /* INC_HARD_CONFIG_H_ */
