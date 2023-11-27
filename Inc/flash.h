#ifndef __FLASH_H
#define __FLASH_H

#include <stdint.h>

uint32_t flash_write(uint32_t address, uint8_t *data, uint32_t length);
void flash_read(uint32_t address, uint8_t *buffer, uint32_t length);

#endif /* __FLASH_H */
