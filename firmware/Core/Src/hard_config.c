/*
 * hard_config.c
 *
 *  Created on: Feb 22, 2025
 *      Author: Usuario
 */

#include "hard_config.h"

static volatile uint16_t pwm_freq_arr      = 0;
static volatile uint16_t current_limit_adc = 0;
static volatile uint16_t max_speed_arr     = 0;

static const uint16_t ESC_max_pwm_freq = 24000; // hz
static const uint16_t ESC_min_pwm_freq = 5000;
static const float    ESC_max_current  = 3;     // amp
static const uint16_t ESC_max_speed    = 12000; // rpm
static const uint16_t ESC_min_speed    = 600;


ConfigStatus     current_config_status = CONFIG_OK;
volatile uint8_t esc_config_done       = 0;
ESCparams        current_esc_params    = {0};

// ==========================

void             set_default_esc_params()
{
  current_esc_params.signature     = 0x50415055;
  current_esc_params.pwm_freq_hz   = 18000;
  current_esc_params.current_limit = 10;
  current_esc_params.temp_limit    = 70;
  current_esc_params.speed_kp      = 0.75f;
  current_esc_params.speed_ki      = 1.35f;
  current_esc_params.speed_kd      = 0.0f;
  current_esc_params.speed_max_rpm = 5400;
  current_esc_params.speed_min_rpm = 200; // Default minimum speed
  current_esc_params.pole_pairs    = 2; // Default pole pairs

  current_esc_params.crc32 = compute_crc32(&current_esc_params);
}

void update_all_esc()
{
  uint32_t tim_arr = CPU_clock / (2 * current_esc_params.pwm_freq_hz);
  TIM1->PSC        = 0;
  TIM1->ARR        = tim_arr;
  esc_config_done = 1;
}

uint32_t compute_crc32(ESCparams* params)
{
  // Verificar alineación
  if ((uint32_t)params % 4 != 0) {
    // La estructura no está alineada a 4 bytes, usar buffer temporal
    uint32_t size_in_words = sizeof(ESCparams) / 4;
    uint32_t temp_buffer[size_in_words];

    // Guardar el valor actual del CRC
    uint32_t stored_crc = params->crc32;
    params->crc32       = 0;

    // Copiar a buffer alineado
    memcpy(temp_buffer, params, sizeof(ESCparams));

    // Calcular CRC
    uint32_t crc = HAL_CRC_Calculate(&hcrc, temp_buffer, size_in_words);

    // Restaurar el CRC original
    params->crc32 = stored_crc;

    return crc;
  } else {
    // La estructura está alineada, calcular directamente
    uint32_t stored_crc = params->crc32;
    params->crc32       = 0;

    uint32_t crc = HAL_CRC_Calculate(&hcrc, (uint32_t*)params, sizeof(ESCparams) / 4);

    params->crc32 = stored_crc;
    return crc;
  }
}

uint16_t get_pwm_freq()
{
  return current_esc_params.pwm_freq_hz;
}

ConfigStatus set_pwm_freq(uint16_t new_freq)
{

  if (new_freq < ESC_min_pwm_freq)
    return CONFIG_ERROR_UNDERLIMIT;
  if (new_freq > ESC_max_pwm_freq)
    return CONFIG_ERROR_OVERLIMIT;
  current_esc_params.pwm_freq_hz = new_freq;
  flash_config_parameter_changed();
  return CONFIG_OK;
}
ConfigStatus set_pole_pairs(uint8_t new_pole_pairs)
{
  if (new_pole_pairs < 1)
    return CONFIG_ERROR_UNDERLIMIT; // Arbitrary limits
  if (new_pole_pairs > 48)
    return CONFIG_ERROR_OVERLIMIT; // Arbitrary limits

  current_esc_params.pole_pairs = new_pole_pairs;
  flash_config_parameter_changed();
  return CONFIG_OK;
}

uint8_t get_pole_pairs()
{
  return current_esc_params.pole_pairs;
}

ConfigStatus set_KP(float new_kp)
{
  if (new_kp < 0.0f)
    return CONFIG_ERROR_UNDERLIMIT; // Arbitrary limits
  if (new_kp > 10.0f)
    return CONFIG_ERROR_OVERLIMIT; // Arbitrary limits

  current_esc_params.speed_kp = new_kp;
  flash_config_parameter_changed();
  return CONFIG_OK;
}
float get_KP()
{
  return current_esc_params.speed_kp;
}
ConfigStatus set_KI(float new_ki)
{
  if (new_ki < 0.0f)
    return CONFIG_ERROR_UNDERLIMIT; // Arbitrary limits
  if (new_ki > 10.0f)
    return CONFIG_ERROR_OVERLIMIT; // Arbitrary limits

  current_esc_params.speed_ki = new_ki;
  flash_config_parameter_changed();
  return CONFIG_OK;
}
float get_KI()
{
  return current_esc_params.speed_ki;
}

ConfigStatus set_KD(float new_kd)
{
  if (new_kd < 0.0f)
    return CONFIG_ERROR_UNDERLIMIT;
  if (new_kd > 10.0f)
    return CONFIG_ERROR_OVERLIMIT;

  current_esc_params.speed_kd = new_kd;
  flash_config_parameter_changed();
  return CONFIG_OK;
}
float get_KD()
{
  return current_esc_params.speed_kd;
}

uint16_t get_min_speed()
{
  return current_esc_params.speed_min_rpm;
}
uint16_t get_max_speed()
{
  return current_esc_params.speed_max_rpm;
}
ConfigStatus set_max_speed(uint16_t new_speed)
{
  if (new_speed < ESC_min_speed)
    return CONFIG_ERROR_UNDERLIMIT;
  if (new_speed > ESC_max_speed)
    return CONFIG_ERROR_OVERLIMIT;
  current_esc_params.speed_max_rpm = new_speed;
  flash_config_parameter_changed();   
  return CONFIG_OK;
}

ConfigStatus set_min_speed(uint16_t new_speed)
{
  if (new_speed < ESC_min_speed)
    return CONFIG_ERROR_UNDERLIMIT;
  if (new_speed > ESC_max_speed)
    return CONFIG_ERROR_OVERLIMIT;
  current_esc_params.speed_min_rpm = new_speed;
  flash_config_parameter_changed();
  return CONFIG_OK;
}