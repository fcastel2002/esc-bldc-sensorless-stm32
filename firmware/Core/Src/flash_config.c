/*
 * flash_config.c
 *
 *  Created on: Mar 19, 2025
 *      Author: Pachi
 */

#include "flash_config.h"

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
} ESCparamsV2;
#pragma pack(pop)

typedef struct {
  uint32_t  signature;
  uint32_t  write_counter;
  ESCparams config;
  uint32_t  crc32;
} FlashStorageBlock;

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
} ESCparamsV3;
#pragma pack(pop)

typedef struct {
  uint32_t    signature;
  uint32_t    write_counter;
  ESCparamsV3 config;
  uint32_t    crc32;
} FlashStorageBlockV3;

typedef struct {
  uint32_t    signature;
  uint32_t    write_counter;
  ESCparamsV2 config;
  uint32_t    crc32;
} FlashStorageBlockV2;

_Static_assert(sizeof(ESCparamsV2) == 40U, "Unexpected legacy ESC config size");
_Static_assert(sizeof(FlashStorageBlockV2) == 52U, "Unexpected legacy flash block size");
_Static_assert(sizeof(ESCparamsV3) == 56U, "Unexpected V3 ESC config size");
_Static_assert(sizeof(FlashStorageBlockV3) == 68U, "Unexpected V3 flash block size");
_Static_assert(sizeof(ESCparams) == 60U, "Unexpected V4 ESC config size");
_Static_assert(sizeof(FlashStorageBlock) == 72U, "Unexpected V4 flash block size");

static ESCparams       flash_cached_config;
static uint8_t         flash_initialized = 0;
static uint32_t        active_page_addr  = 0;
static uint32_t        write_counter     = 0;
static uint8_t         pending_changes   = 0;
static uint8_t         is_saving         = 0;
static uint8_t         loaded_legacy     = 0;
static FlashResultCode erase_flash_page(uint32_t page_addr);
static uint32_t        calculate_block_crc(FlashStorageBlock* block);
static uint32_t        calculate_v3_block_crc(FlashStorageBlockV3* block);
static uint32_t        calculate_legacy_block_crc(FlashStorageBlockV2* block);
static FlashResultCode write_flash_block(uint32_t addr, FlashStorageBlock* block);

static uint32_t calculate_block_crc(FlashStorageBlock* block)
{
  // 1. Crear un buffer temporal alineado
  uint32_t buffer_size = (sizeof(FlashStorageBlock) + 3) / 4; // Redondear hacia arriba
  uint32_t buffer[buffer_size];

  // 2. Copiar el bloque completo al buffer alineado
  memcpy(buffer, block, sizeof(FlashStorageBlock));

  // 3. Guardar el CRC original
  uint32_t stored_crc = ((FlashStorageBlock*)buffer)->crc32;

  // 4. Establecer el CRC a cero para el cálculo
  ((FlashStorageBlock*)buffer)->crc32 = 0;

  // 5. Calcular el CRC usando el buffer alineado
  uint32_t calculated_crc = HAL_CRC_Calculate(&hcrc, buffer, buffer_size);

  // 6. No modificamos el bloque original, solo devolvemos el CRC calculado
  return calculated_crc;
}

static uint32_t calculate_legacy_block_crc(FlashStorageBlockV2* block)
{
  uint32_t buffer[sizeof(FlashStorageBlockV2) / 4U];
  memcpy(buffer, block, sizeof(FlashStorageBlockV2));
  ((FlashStorageBlockV2*)buffer)->crc32 = 0U;
  return HAL_CRC_Calculate(&hcrc, buffer, sizeof(buffer) / sizeof(buffer[0]));
}

static uint32_t calculate_v3_block_crc(FlashStorageBlockV3* block)
{
  uint32_t buffer[sizeof(FlashStorageBlockV3) / 4U];
  memcpy(buffer, block, sizeof(FlashStorageBlockV3));
  ((FlashStorageBlockV3*)buffer)->crc32 = 0U;
  return HAL_CRC_Calculate(&hcrc, buffer, sizeof(buffer) / sizeof(buffer[0]));
}

static void migrate_legacy_config(const ESCparamsV2* legacy, ESCparams* migrated)
{
  set_default_esc_params();
  *migrated = current_esc_params;
  migrated->pwm_freq_hz = legacy->pwm_freq_hz;
  migrated->brake_type = legacy->brake_type;
  migrated->current_limit = legacy->current_limit;
  migrated->temp_limit = legacy->temp_limit;
  migrated->speed_max_rpm = legacy->speed_max_rpm;
  migrated->speed_min_rpm = legacy->speed_min_rpm;
  migrated->pole_pairs = legacy->pole_pairs;
  if (legacy->signature == ESC_PARAMS_SIGNATURE_V2) {
    migrated->speed_kp = legacy->speed_kp;
    migrated->speed_ki = legacy->speed_ki;
    migrated->speed_kd = legacy->speed_kd;
  }
  migrated->signature = ESC_PARAMS_SIGNATURE_V4;
  migrated->crc32 = compute_crc32(migrated);
}

static void migrate_v3_config(const ESCparamsV3* legacy, ESCparams* migrated)
{
  set_default_esc_params();
  *migrated = current_esc_params;
  migrated->pwm_freq_hz = legacy->pwm_freq_hz;
  migrated->brake_type = legacy->brake_type;
  migrated->current_limit = legacy->current_limit;
  migrated->temp_limit = legacy->temp_limit;
  migrated->speed_kp = legacy->speed_kp;
  migrated->speed_ki = legacy->speed_ki;
  migrated->speed_kd = legacy->speed_kd;
  migrated->speed_max_rpm = legacy->speed_max_rpm;
  migrated->speed_min_rpm = legacy->speed_min_rpm;
  migrated->pole_pairs = legacy->pole_pairs;
  migrated->startup_initial_amplitude_permille =
      legacy->startup_initial_amplitude_permille;
  migrated->startup_final_amplitude_permille =
      legacy->startup_final_amplitude_permille;
  migrated->startup_initial_frequency_millihz =
      legacy->startup_initial_frequency_millihz;
  migrated->startup_final_frequency_millihz =
      legacy->startup_final_frequency_millihz;
  migrated->startup_duration_ms = legacy->startup_duration_ms;
  migrated->signature = ESC_PARAMS_SIGNATURE_V4;
  migrated->crc32 = compute_crc32(migrated);
}

static uint8_t read_page(uint32_t page_addr, ESCparams* config, uint32_t* counter, uint8_t* legacy)
{
  const FlashStorageBlock* page = (const FlashStorageBlock*)page_addr;
  if (page->signature != FLASH_CONFIG_SIGNATURE) return 0U;

  uint32_t config_signature = page->config.signature;
  if (config_signature == ESC_PARAMS_SIGNATURE_V4) {
    FlashStorageBlock copy;
    memcpy(&copy, page, sizeof(copy));
    if (calculate_block_crc(&copy) != page->crc32) return 0U;
    memcpy(config, &page->config, sizeof(*config));
    *counter = page->write_counter;
    *legacy = 0U;
    return 1U;
  }

  if (config_signature == ESC_PARAMS_SIGNATURE_V3) {
    const FlashStorageBlockV3* legacy_page = (const FlashStorageBlockV3*)page_addr;
    FlashStorageBlockV3 copy;
    memcpy(&copy, legacy_page, sizeof(copy));
    if (calculate_v3_block_crc(&copy) != legacy_page->crc32) return 0U;
    migrate_v3_config(&legacy_page->config, config);
    *counter = legacy_page->write_counter;
    *legacy = 1U;
    return 1U;
  }

  if (config_signature == ESC_PARAMS_SIGNATURE_V1 ||
      config_signature == ESC_PARAMS_SIGNATURE_V2) {
    const FlashStorageBlockV2* legacy_page = (const FlashStorageBlockV2*)page_addr;
    FlashStorageBlockV2 copy;
    memcpy(&copy, legacy_page, sizeof(copy));
    if (calculate_legacy_block_crc(&copy) != legacy_page->crc32) return 0U;
    migrate_legacy_config(&legacy_page->config, config);
    *counter = legacy_page->write_counter;
    *legacy = 1U;
    return 1U;
  }

  return 0U;
}

static FlashResultCode find_active_page(void)
{
  ESCparams page1_config;
  ESCparams page2_config;
  uint32_t page1_counter = 0U;
  uint32_t page2_counter = 0U;
  uint8_t page1_legacy = 0U;
  uint8_t page2_legacy = 0U;
  uint8_t page1_valid = read_page(
      FLASH_CONFIG_START, &page1_config, &page1_counter, &page1_legacy);
  uint8_t page2_valid = read_page(
      FLASH_CONFIG_START + FLASH_PAGE_SIZE,
      &page2_config,
      &page2_counter,
      &page2_legacy);

  if (page1_valid && (!page2_valid || page1_counter > page2_counter)) {
    active_page_addr = FLASH_CONFIG_START;
    write_counter = page1_counter;
    loaded_legacy = page1_legacy;
    memcpy(&flash_cached_config, &page1_config, sizeof(ESCparams));
    return FLASH_RESULT_OK;
  }

  if (page2_valid) {
    active_page_addr = FLASH_CONFIG_START + FLASH_PAGE_SIZE;
    write_counter = page2_counter;
    loaded_legacy = page2_legacy;
    memcpy(&flash_cached_config, &page2_config, sizeof(ESCparams));
    return FLASH_RESULT_OK;
  }

  active_page_addr = FLASH_CONFIG_START;
  loaded_legacy = 0U;
  return FLASH_RESULT_EMPTY;
}

static FlashResultCode erase_flash_page(uint32_t page_addr)
{
  FLASH_EraseInitTypeDef erase_init;
  uint32_t               page_error = 0;

  erase_init.TypeErase   = FLASH_TYPEERASE_PAGES;
  erase_init.PageAddress = page_addr;
  erase_init.NbPages     = 1;
  HAL_FLASH_Unlock();
  if (HAL_FLASHEx_Erase(&erase_init, &page_error) != HAL_OK) {
    HAL_FLASH_Lock();
    return FLASH_RESULT_ERASE_ERROR;
  }
  HAL_FLASH_Lock();
  return FLASH_RESULT_OK;
}

static FlashResultCode write_flash_block(uint32_t addr, FlashStorageBlock* block)
{
  FlashResultCode result    = FLASH_RESULT_OK;
  uint32_t*       src_data  = (uint32_t*)block;
  uint32_t        dest_addr = addr;

  HAL_FLASH_Unlock();

  for (uint32_t i = 0; i < sizeof(FlashStorageBlock) / 4; i++) {
    if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, dest_addr, src_data[i]) != HAL_OK) {
      result = FLASH_RESULT_WRITE_ERROR;
      break;
    }
    dest_addr += 4;
  }

  HAL_FLASH_Lock();
  return result;
}

FlashResultCode flash_config_init(void)
{
  FlashResultCode result;

  if (flash_initialized) {
    return FLASH_RESULT_OK;
  }
  result = find_active_page();

  if (result == FLASH_RESULT_OK) {

    memcpy(&current_esc_params, &flash_cached_config, sizeof(ESCparams));
    if (current_esc_params.signature != ESC_PARAMS_SIGNATURE_V4) {
      set_default_esc_params();
      pending_changes = 1;
    }
    if (loaded_legacy) pending_changes = 1;
    flash_initialized = 1;
    return FLASH_RESULT_OK;
  } else if (result == FLASH_RESULT_EMPTY) {
    set_default_esc_params();
    flash_initialized = 1;
    pending_changes   = 1;
    return result;
  }
  return result;
}
FlashResultCode flash_config_save(void)
{
  FlashStorageBlock block;
  FlashResultCode   result;
  uint32_t          next_page_addr;
  if (is_saving) {
    return FLASH_RESULT_OK;
  }
  if (!pending_changes && flash_initialized) {
    return FLASH_RESULT_OK;
  }

  if (!flash_initialized) {
    result = flash_config_init();
    if (result != FLASH_RESULT_OK && result != FLASH_RESULT_EMPTY) {
      is_saving = 0;
      return result;
    }
  }
  if (active_page_addr == FLASH_CONFIG_START) {
    next_page_addr = FLASH_CONFIG_START + FLASH_PAGE_SIZE;
  } else {
    next_page_addr = FLASH_CONFIG_START;
  }
  block.signature     = FLASH_CONFIG_SIGNATURE;
  block.write_counter = write_counter + 1;
  memcpy(&block.config, &current_esc_params, sizeof(ESCparams));
  block.crc32 = 0;
  block.crc32 = calculate_block_crc(&block);
  result      = erase_flash_page(next_page_addr);
  if (result != FLASH_RESULT_OK) {
    is_saving = 0;
    return result;
  }
  result = write_flash_block(next_page_addr, &block);
  if (result != FLASH_RESULT_OK) {
    is_saving = 0;
    return result;
  }
  write_counter++;
  active_page_addr = next_page_addr;
  memcpy(&flash_cached_config, &current_esc_params, sizeof(ESCparams));
  pending_changes = 0;
  is_saving       = 0;

  return FLASH_RESULT_OK;
}

FlashResultCode flash_config_restore_defaults(void)
{
  set_default_esc_params();
  pending_changes = 1;
  return FLASH_RESULT_OK;
}

uint8_t flash_config_has_pending_changes(void)
{
  if (!flash_initialized) {
    return 1;
  }
  if (memcmp(&current_esc_params, &flash_cached_config, sizeof(ESCparams)) != 0) {
    pending_changes = 1;
  }
  return pending_changes;
}

uint8_t flash_config_get_stats(uint32_t* write_count)
{
  if (!flash_initialized) {
    return 0;
  }
  if (write_count) {
    *write_count = write_counter;
  }
  return 1;
}

void flash_config_parameter_changed(void)
{
  pending_changes = 1;
}
