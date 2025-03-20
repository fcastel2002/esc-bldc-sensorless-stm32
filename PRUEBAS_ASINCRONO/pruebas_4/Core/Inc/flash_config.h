/*
 * flash_config.h
 *
 *  Created on: Mar 19, 2025
 *      Author: Pachi
 */

#ifndef INC_FLASH_CONFIG_H_
#define INC_FLASH_CONFIG_H_

#include "main.h"
#include "hard_config.h"
#include "stm32f1xx_hal_flash.h"

#define FLASH_PAGE_SIZE 1024
#define FLASH_CONFIG_PAGES 2 //nro de paginas rotativas
#define FLASH_CONFIG_START (FLASH_BASE + 64*1024 - (FLASH_PAGE_SIZE * FLASH_CONFIG_PAGES))

typedef enum{
	FLASH_RESULT_OK,
	FLASH_RESULT_EMPTY,
	FLASH_RESULT_CORRUPT,
	FLASH_RESULT_WRITE_ERROR,
	FLASH_RESULT_ERASE_ERROR
} FlashResultCode;

FlashResultCode flash_config_init(void);

FlashResultCode flash_config_save(void);

FlashResultCode flash_config_restore_defaults(void);

uint8_t flash_config_get_stats(uint32_t *write_count);

uint8_t flash_config_has_pending_changes(void);

#endif /* INC_FLASH_CONFIG_H_ */
