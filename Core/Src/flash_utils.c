#include "flash_utils.h"
#include <string.h>
#include "stm32f3xx_hal_flash.h"

HAL_StatusTypeDef Flash_Write (uint32_t addr, const void *data, uint32_t size) {
    FLASH_EraseInitTypeDef eraseInit = {0};
    uint32_t pageError, wordCount = (size + 3)/4;
    uint8_t tempBuffer[FLASH_PAGE_SIZE];

    if (size > FLASH_PAGE_SIZE) return HAL_ERROR;
    if (addr < 0x08000000 || addr > 0x0800FFFF) return HAL_ERROR;

    memset(tempBuffer, 0xFF, FLASH_PAGE_SIZE); // 0xFF =erased flash state
    memcpy(tempBuffer, data, size);

    HAL_FLASH_Unlock();
    eraseInit.TypeErase = FLASH_TYPEERASE_PAGES;
    eraseInit.PageAddress = addr;
    eraseInit.NbPages = 1;

    if (HAL_FLASHEx_Erase(&eraseInit, &pageError) != HAL_OK) {
        HAL_FLASH_Lock();
        return HAL_ERROR;
    }

    for (uint32_t i=0;i<wordCount;i++) {
        uint32_t word;
        memcpy(&word, tempBuffer+(i*4), 4);
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, addr + (i*4), word) != HAL_OK) {
            HAL_FLASH_Lock();
            return HAL_ERROR;
        }
    }

    HAL_FLASH_Lock();
    return HAL_OK;

}

uint8_t Flash_Read  (uint32_t addr, void *buffer, uint32_t size) {
    if (size == 0) return 1;
    if (!buffer) return 2;

    memcpy(buffer, (void*) addr, size);
    return 0;
}
