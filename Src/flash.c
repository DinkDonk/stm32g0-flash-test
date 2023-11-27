#include "stm32g0xx.h"
#include "FreeRTOS.h"
#include "task.h"
#include "flash.h"

/**
 * @brief  Gets the page of a given address
 * @param  address: Address of the FLASH Memory
 * @retval The page of a given address
 */
static uint32_t get_page(uint32_t address)
{
  return (address - FLASH_BASE) / FLASH_PAGE_SIZE;
}

/**
 * @brief  Waits for a FLASH operation to complete.
 * @retval 0 if successful, value of FLASH->SR if not.
 */
static inline uint32_t flash_wait_for_last_operation(void)
{
  uint32_t status = 0;

  /* Wait for any FLASH operations to complete */
  while (FLASH->SR & (FLASH_SR_BSY1 | FLASH_SR_BSY2 | FLASH_SR_CFGBSY)) continue;

  if (FLASH->SR != 0)
  {
    status = FLASH->SR;
  }

  /* Clear error flags */
  FLASH->SR |= (FLASH_SR_OPERR |
    FLASH_SR_PROGERR |
    FLASH_SR_WRPERR |
    FLASH_SR_PGAERR |
    FLASH_SR_SIZERR  |
    FLASH_SR_PGSERR |
    FLASH_SR_MISERR |
    FLASH_SR_FASTERR |
    FLASH_SR_RDERR |
    FLASH_SR_OPTVERR |
    FLASH_SR_EOP);

  return status;
}

/**
 * @brief  Unlocks the FLASH control register access
 * @retval None
 */
static inline void flash_unlock() {
  taskENTER_CRITICAL();

  /* Wait last FLASH operations to complete */
  flash_wait_for_last_operation();

  /* Return if already unlocked */
  if ((FLASH->CR & FLASH_CR_LOCK) == 0)
  {
    return;
  }

  FLASH->KEYR = FLASH_KEY1;
  FLASH->KEYR = FLASH_KEY2;
}

/**
 * @brief  Locks the FLASH control register access
 * @retval None
 */
static inline void flash_lock() {
  FLASH->CR |= FLASH_CR_LOCK;

  taskEXIT_CRITICAL();
}

/**
 * @brief  Programs a double word (64-bit) at a specified address.
 * @param  address: The address to program.
 * @param  data: The data to program.
 * @retval None
 */
static void flash_program_doubleword(uint32_t address, uint64_t data)
{
  /* Program first word */
  *(uint32_t *)address = (uint32_t)data;

  /* Barrier to ensure programming is performed in 2 steps, in right order
    (independently of compiler optimization behavior) */
  __ISB();

  /* Program second word */
  *(uint32_t *)(address + 4) = (uint32_t)(data >> 32);
}

/**
 * @brief  Erases a specified page in program memory.
 * @param  page: The page to erase.
 * @retval 0 if successful, value of FLASH->SR if not.
 */
static uint32_t flash_erase_page(uint8_t page)
{
  uint32_t status = 0;

  flash_unlock();

  /* Reset the PER Bit and page number */
  /* Set the page to erase (PNB) */
  /* Set the PER bit (erase command) */
  /* Set the STRT bit to start the erase operation */
  uint32_t reg = FLASH->CR;
  reg &= ~(FLASH_CR_PER | FLASH_CR_PNB);
  reg |= page << FLASH_CR_PNB_Pos;
  reg |= FLASH_CR_PER;
  reg |= FLASH_CR_STRT;

  FLASH->CR |= reg;

  status = flash_wait_for_last_operation();

  /* Reset the PER Bit and page number */
  FLASH->CR &= ~(FLASH_CR_PER | FLASH_CR_PNB);

  flash_lock();

  return status;
}

/**
 * @brief  Erases the FLASH page of specified address.
 * @param  address: The address to erase.
 * @retval 0 if successful, value of FLASH->SR if not.
 */
static uint32_t flash_erase_address_page(uint32_t address)
{
  return flash_erase_page(get_page(address));
}

/**
 * @brief  Writes data to flash.
 * @param  address: The address to write to.
 * @param  data: The data to write.
 * @param  length: The length of the data to write.
 * @retval 0 if successful, value of FLASH->SR if not.
 */
uint32_t flash_write(uint32_t address, uint8_t *data, uint32_t length)
{
  uint32_t status = 0;

  status = flash_erase_address_page(address);

  if (status != 0)
  {
    /* Error occurred while page erase */
    return status;
  }

  flash_unlock();

  /* Set command to double word programming */
  FLASH->CR |= FLASH_CR_PG;

  for (uint32_t i = 0; i < length; i += 8)
  {
    flash_program_doubleword(address + i, *(uint64_t *)(data + i));
    status = flash_wait_for_last_operation();

    if (status != 0)
    {
      /* Error occurred while writing data in Flash memory */
      break;
    }
  }

  /* Clear programming bit */
  FLASH->CR &= ~FLASH_CR_PG;

  flash_lock();

  return status;
}

/**
 * @brief  Reads data from flash.
 * @param  address: The address to read from.
 * @param  buffer: The buffer to read into.
 * @param  length: The length of the data to read.
 * @retval None
 */
void flash_read(uint32_t address, uint8_t *buffer, uint32_t length)
{
  for (uint32_t i = 0; i < length; i++)
  {
	buffer[i] = *(uint8_t *)(address + i);
  }
}
