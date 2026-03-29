#ifndef __FLASH_UTILS__
#define __FLASH_UTILS__

#include <stdint.h>
#include "stm32f3xx_hal.h"

uint8_t Flash_Read  (uint32_t addr, void *buffer, uint32_t size);
HAL_StatusTypeDef Flash_Write (uint32_t addr, const void *data, uint32_t size);

#endif
