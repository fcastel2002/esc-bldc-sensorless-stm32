/*
 * flash_config.c
 *
 *  Created on: Mar 19, 2025
 *      Author: Pachi
 */


#include "flash_config.h"


typedef struct {

	uint32_t signature;
	uint32_t write_counter;
	ESCparams config;
	uint32_t crc32;

} FlashStorageBlock;


static ESCparams flash_cached_config;
static uint8_t flash_initialized = 0;
static uint32_t active_page_addr = 0;


static uint32_t calculate_block_crc(FlashStorageBlock *block){
	uint32_t stored_crc = block->crc32;
}
