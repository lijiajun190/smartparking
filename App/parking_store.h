#ifndef PARKING_STORE_H
#define PARKING_STORE_H
#include <stdint.h>
#include "parking_config.h"

#define PARK_STORE_BANK_BYTES (128U * 1024U)
#define PARK_STORE_RECORD_BYTES 128U
typedef struct {
    uint32_t occupied;
    uint8_t uid[4];
    uint32_t start_epoch;
} ParkingStoredSlot;
typedef struct {
    ParkingStoredSlot slots[PARKING_SLOT_COUNT];
    uint32_t pending;
    uint32_t selected_slot;
    uint8_t uid[4];
} ParkingSnapshot;
typedef enum { PARK_STORE_ERROR = -1, PARK_STORE_EMPTY = 0, PARK_STORE_OK = 1 } ParkingStoreResult;

/* 单写者：ParkingTask。load每次重新扫描，save失败后必须重新load/人工核对。 */
ParkingStoreResult parking_store_load(ParkingSnapshot *snapshot);
uint8_t parking_store_save(const ParkingSnapshot *snapshot);
/* 仅人工确认后使用；会清除全部停车日志，失败时继续锁止。 */
uint8_t parking_store_format(void);

/* 仅后端访问保留的两个Flash扇区；主机测试用同一接口注入掉电。 */
void parking_flash_read(unsigned bank, uint32_t offset, void *data, uint32_t bytes);
uint8_t parking_flash_program(unsigned bank, uint32_t offset, uint32_t word);
uint8_t parking_flash_erase(unsigned bank);
#endif
