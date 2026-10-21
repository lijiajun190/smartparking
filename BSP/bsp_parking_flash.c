#include "main.h"
#include "parking_store.h"
#include "bsp_stepper.h"
#include <string.h>

/* STM32F411xE: sector6/7各128KiB；链接器仅使用前256KiB。 */
static const uint32_t addresses[2] = {0x08040000U, 0x08060000U};
static uint8_t writable(unsigned bank, uint32_t offset)
{
    return bank < 2U && offset <= PARK_STORE_BANK_BYTES - 4U && (offset & 3U) == 0U &&
           stepper_get_status(MOTOR_A) != MOTOR_RUNNING &&
           stepper_get_status(MOTOR_B) != MOTOR_RUNNING;
}
void parking_flash_read(unsigned bank, uint32_t offset, void *data, uint32_t bytes)
{
    if (bank < 2U && offset <= PARK_STORE_BANK_BYTES && bytes <= PARK_STORE_BANK_BYTES - offset)
        memcpy(data, (const void *)(addresses[bank] + offset), bytes);
}
uint8_t parking_flash_program(unsigned bank, uint32_t offset, uint32_t word)
{
    HAL_StatusTypeDef status;
    if (!writable(bank, offset) || HAL_FLASH_Unlock() != HAL_OK) return 0U;
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR |
                         FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR | FLASH_FLAG_PGSERR);
    status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, addresses[bank] + offset, word);
    if (HAL_FLASH_Lock() != HAL_OK) return 0U;
    return status == HAL_OK;
}
uint8_t parking_flash_erase(unsigned bank)
{
    FLASH_EraseInitTypeDef erase = {0};
    uint32_t error;
    HAL_StatusTypeDef status;
    if (!writable(bank, 0U) || HAL_FLASH_Unlock() != HAL_OK) return 0U;
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR |
                         FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR | FLASH_FLAG_PGSERR);
    erase.TypeErase = FLASH_TYPEERASE_SECTORS;
    erase.Sector = bank == 0U ? FLASH_SECTOR_6 : FLASH_SECTOR_7;
    erase.NbSectors = 1U;
    erase.VoltageRange = FLASH_VOLTAGE_RANGE_3; /* 供电2.7~3.6V */
    status = HAL_FLASHEx_Erase(&erase, &error);
    if (HAL_FLASH_Lock() != HAL_OK) return 0U;
    return status == HAL_OK;
}
