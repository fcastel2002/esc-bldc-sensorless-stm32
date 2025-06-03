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
static uint32_t write_counter = 0;
static uint8_t pending_changes = 0;
static uint8_t is_saving = 0;
static FlashResultCode erase_flash_page(uint32_t page_addr);
static uint32_t calculate_block_crc(FlashStorageBlock *block);
static FlashResultCode write_flash_block(uint32_t addr, FlashStorageBlock* block);

static uint32_t calculate_block_crc(FlashStorageBlock *block) {
    // 1. Crear un buffer temporal alineado
    uint32_t buffer_size = (sizeof(FlashStorageBlock) + 3) / 4;  // Redondear hacia arriba
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
static inline void write_page(FlashStorageBlock* current_page) {
    // Copiar datos con seguridad
    ESCparams temp_config;

    // Copiar la configuración a un buffer temporal alineado
    memcpy(&temp_config, &current_page->config, sizeof(ESCparams));

    // Actualizar el contador de escrituras
    write_counter = current_page->write_counter;

    // Copiar la configuración a la caché
    memcpy(&flash_cached_config, &temp_config, sizeof(ESCparams));
}
static FlashResultCode find_active_page(void) {
    // Punteros a las páginas de flash
    FlashStorageBlock *page1 = (FlashStorageBlock*)FLASH_CONFIG_START;
    FlashStorageBlock *page2 = (FlashStorageBlock*)(FLASH_CONFIG_START + FLASH_PAGE_SIZE);

    uint8_t page1_valid = 0;
    uint8_t page2_valid = 0;

    // Verificar validez de página 1 con manejo de errores
    if (page1->signature == FLASH_CONFIG_SIGNATURE) {
        // Usar try-catch simulado para manejar posibles hard faults
        uint32_t crc;
        if (page1 != NULL) {
            crc = calculate_block_crc(page1);
            if (crc == page1->crc32) {
                page1_valid = 1;
            }
        }
    }

    // Verificar validez de página 2 con manejo de errores
    if (page2->signature == FLASH_CONFIG_SIGNATURE) {
        uint32_t crc;
        if (page2 != NULL) {
            crc = calculate_block_crc(page2);
            if (crc == page2->crc32) {
                page2_valid = 1;
            }
        }
    }

    // Resto de la función sin cambios...
    if (page1_valid && page2_valid) {
        if (page1->write_counter > page2->write_counter) {
            active_page_addr = FLASH_CONFIG_START;
            write_page(page1);
        } else {
            active_page_addr = FLASH_CONFIG_START + FLASH_PAGE_SIZE;
            write_page(page2);
        }
        return FLASH_RESULT_OK;
    } else if (page1_valid) {
        active_page_addr = FLASH_CONFIG_START;
        write_page(page1);
        return FLASH_RESULT_OK;
    } else if (page2_valid) {
        active_page_addr = FLASH_CONFIG_START + FLASH_PAGE_SIZE;
        write_page(page2);
        return FLASH_RESULT_OK;
    } else {
        active_page_addr = FLASH_CONFIG_START;
        return FLASH_RESULT_EMPTY;
    }
}

static FlashResultCode erase_flash_page(uint32_t page_addr){
	FLASH_EraseInitTypeDef erase_init;
	uint32_t page_error = 0;

	erase_init.TypeErase = FLASH_TYPEERASE_PAGES;
	erase_init.PageAddress = page_addr;
	erase_init.NbPages = 1;
	HAL_FLASH_Unlock();
	if(HAL_FLASHEx_Erase(&erase_init,&page_error) != HAL_OK){
		HAL_FLASH_Lock();
		return FLASH_RESULT_ERASE_ERROR;
	}
	HAL_FLASH_Lock();
	return FLASH_RESULT_OK;
}


static FlashResultCode write_flash_block(uint32_t addr, FlashStorageBlock* block){
	FlashResultCode result = FLASH_RESULT_OK;
	uint32_t* src_data = (uint32_t*) block;
	uint32_t dest_addr = addr;

	HAL_FLASH_Unlock();

	for(uint32_t i = 0; i < sizeof(FlashStorageBlock)/4; i++){
		if(HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,dest_addr, src_data[i]) != HAL_OK){
			result = FLASH_RESULT_WRITE_ERROR;
			break;
		}
		dest_addr += 4;
	}

	HAL_FLASH_Lock();
	return result;
}

FlashResultCode flash_config_init(void){
	FlashResultCode result;

	if(flash_initialized){
		return FLASH_RESULT_OK;
	}
	result = find_active_page();

	if(result == FLASH_RESULT_OK){

		memcpy(&current_esc_params, &flash_cached_config, sizeof(ESCparams));
		flash_initialized = 1;
		pending_changes = 0;
		return FLASH_RESULT_OK;
	} else if(result == FLASH_RESULT_EMPTY){
		set_default_esc_params();
		flash_initialized =1;
		pending_changes = 1;
		return result;
	}
	return result;

}
FlashResultCode flash_config_save(void){
	FlashStorageBlock block;
	FlashResultCode result;
	uint32_t next_page_addr;
	if(is_saving){
		return FLASH_RESULT_OK;
	}
	if(!pending_changes && flash_initialized){
		return FLASH_RESULT_OK;
	}

	if(!flash_initialized){
		result = flash_config_init();
		if(result != FLASH_RESULT_OK && result != FLASH_RESULT_EMPTY){
			is_saving = 0;
			return result;
		}
	}
	if(active_page_addr == FLASH_CONFIG_START){
		next_page_addr = FLASH_CONFIG_START + FLASH_PAGE_SIZE;
	}else{
		next_page_addr = FLASH_CONFIG_START;
	}
	block.signature = FLASH_CONFIG_SIGNATURE;
	block.write_counter = write_counter + 1;
	memcpy(&block.config, &current_esc_params, sizeof(ESCparams));
	block.crc32 = 0;
	block.crc32 = calculate_block_crc(&block);
	result = erase_flash_page(next_page_addr);
	if(result != FLASH_RESULT_OK){
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
	memcpy(&flash_cached_config,&current_esc_params, sizeof(ESCparams));
	pending_changes = 0;
	is_saving = 0;

	return FLASH_RESULT_OK;
}

FlashResultCode flash_config_restore_defaults(void){
	set_default_esc_params();
	pending_changes = 1;
	return FLASH_RESULT_OK;
}

uint8_t flash_config_has_pending_changes(void){
	if(!flash_initialized){
		return 1;
	}
	if(memcmp(&current_esc_params,&flash_cached_config,sizeof(ESCparams)) != 0){
		pending_changes = 1;

	}
	return pending_changes;
}

uint8_t flash_config_get_stats(uint32_t* write_count){
	if (!flash_initialized){
		return 0;
	}
	if(write_count){
		*write_count = write_counter;
	}
	return 1;
}

void flash_config_parameter_changed(void){
	pending_changes = 1;
}


