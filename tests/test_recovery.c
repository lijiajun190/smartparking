#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "parking_store.h"
#include "health_watch.h"
#define CHECK(x) do { if (!(x)) { printf("FAIL line %d: %s\n", __LINE__, #x); exit(1); } } while (0)
static uint8_t flash_memory[2][PARK_STORE_BANK_BYTES];
static uint8_t backup[2][PARK_STORE_BANK_BYTES];
static int writes_left = -1;
static uint8_t fail_erase;
static unsigned erases, tests;
void parking_flash_read(unsigned bank, uint32_t offset, void *data, uint32_t bytes)
{
    CHECK(bank < 2U && offset + bytes <= PARK_STORE_BANK_BYTES);
    memcpy(data, &flash_memory[bank][offset], bytes);
}
uint8_t parking_flash_program(unsigned bank, uint32_t offset, uint32_t word)
{
    uint32_t old;
    CHECK(bank < 2U && offset + 4U <= PARK_STORE_BANK_BYTES && (offset & 3U) == 0U);
    if (writes_left == 0) return 0U;
    if (writes_left > 0) writes_left--;
    memcpy(&old, &flash_memory[bank][offset], 4U);
    CHECK((old & word) == word); /* 真Flash不允许0->1，防止覆盖未擦除页 */
    memcpy(&flash_memory[bank][offset], &word, 4U);
    return 1U;
}
uint8_t parking_flash_erase(unsigned bank)
{
    CHECK(bank < 2U);
    if (fail_erase) return 0U;
    erases++;
    memset(flash_memory[bank], 0xFF, PARK_STORE_BANK_BYTES);
    return 1U;
}
static void blank(void)
{
    memset(flash_memory, 0xFF, sizeof(flash_memory));
    writes_left = -1; fail_erase = 0U; erases = 0U;
}
static void pass(const char *name) { tests++; printf("PASS %s\n", name); }
static void storage_tests(void)
{
    ParkingSnapshot original = {0}, restored, final;
    unsigned i, cut;
    blank();
    CHECK(parking_store_load(&restored) == PARK_STORE_EMPTY);
    original.slots[2].occupied = 1U;
    original.slots[2].uid[0] = 42U;
    original.pending = 2U; original.selected_slot = 2U; original.uid[0] = 42U;
    CHECK(parking_store_save(&original));
    CHECK(parking_store_load(&restored) == PARK_STORE_OK);
    CHECK(memcmp(&original, &restored, sizeof(original)) == 0);
    CHECK(erases == 0U);
    pass("UID, occupancy and pending operation survive reboot without unnecessary erase");

    final = original; final.pending = 0U;
    memset(&final.slots[2], 0, sizeof(final.slots[2]));
    memcpy(backup, flash_memory, sizeof(backup));
    for (cut = 0U; cut < PARK_STORE_RECORD_BYTES / 4U; cut++) {
        memcpy(flash_memory, backup, sizeof(backup));
        CHECK(parking_store_load(&restored) == PARK_STORE_OK);
        writes_left = (int)cut;
        CHECK(!parking_store_save(&final));
        CHECK(!parking_store_save(&final)); /* 无重新扫描时禁止重试 */
        writes_left = -1;
        CHECK(parking_store_load(&restored) == PARK_STORE_OK);
        CHECK(memcmp(&original, &restored, sizeof(original)) == 0);
        CHECK(parking_store_save(&final)); /* 跳过撕裂页 */
        CHECK(parking_store_load(&restored) == PARK_STORE_OK);
        CHECK(memcmp(&final, &restored, sizeof(final)) == 0);
    }
    pass("power cut at every record word including commit keeps prior pending record; retry skips torn page");

    blank(); CHECK(parking_store_load(&restored) == PARK_STORE_EMPTY);
    for (i = 0U; i < PARK_STORE_BANK_BYTES / PARK_STORE_RECORD_BYTES; i++) {
        original.slots[2].uid[1] = (uint8_t)i;
        CHECK(parking_store_save(&original));
    }
    CHECK(erases == 0U);
    memcpy(backup, flash_memory, sizeof(backup));
    fail_erase = 1U;
    CHECK(!parking_store_save(&final));
    fail_erase = 0U;
    CHECK(parking_store_load(&restored) == PARK_STORE_OK);
    CHECK(memcmp(&original, &restored, sizeof(original)) == 0);
    for (cut = 0U; cut < PARK_STORE_RECORD_BYTES / 4U; cut++) {
        memcpy(flash_memory, backup, sizeof(backup));
        CHECK(parking_store_load(&restored) == PARK_STORE_OK);
        writes_left = (int)cut;
        CHECK(!parking_store_save(&final));
        writes_left = -1;
        CHECK(parking_store_load(&restored) == PARK_STORE_OK);
        CHECK(memcmp(&original, &restored, sizeof(original)) == 0);
        CHECK(memcmp(flash_memory[0], backup[0], PARK_STORE_BANK_BYTES) == 0);
    }
    CHECK(parking_store_save(&final));
    CHECK(parking_store_load(&restored) == PARK_STORE_OK);
    CHECK(memcmp(&final, &restored, sizeof(final)) == 0);
    pass("bank rotation preserves latest record through erase failure and every torn new-bank write");

    /* 第二扇区写满后再次轮换，覆盖更旧扇区而非当前有效记录。 */
    for (i = 0U; i <= PARK_STORE_BANK_BYTES / PARK_STORE_RECORD_BYTES; i++) {
        final.slots[0].uid[0] = (uint8_t)i;
        CHECK(parking_store_save(&final));
    }
    CHECK(parking_store_load(&restored) == PARK_STORE_OK);
    CHECK(memcmp(&final, &restored, sizeof(final)) == 0);
    pass("both banks rotate and recover the newest committed snapshot");

    blank(); CHECK(parking_store_load(&restored) == PARK_STORE_EMPTY);
    CHECK(parking_store_save(&original));
    final = original; final.pending = 0U;
    CHECK(parking_store_save(&final));
    flash_memory[0][PARK_STORE_RECORD_BYTES + 20U] ^= 1U;
    CHECK(parking_store_load(&restored) == PARK_STORE_ERROR);
    CHECK(!parking_store_save(&original));
    pass("committed CRC corruption fails closed instead of silently using older occupancy");

    blank(); flash_memory[0][0] = 0U;
    CHECK(parking_store_load(&restored) == PARK_STORE_ERROR);
    blank(); CHECK(parking_store_load(&restored) == PARK_STORE_EMPTY);
    original.pending = 3U; CHECK(!parking_store_save(&original));
    original.pending = 0U; original.selected_slot = PARKING_SLOT_COUNT;
    CHECK(!parking_store_save(&original));
    original.selected_slot = 0U;
    original.slots[1] = original.slots[2];
    CHECK(!parking_store_save(&original));
    pass("dirty storage without valid record, invalid slot/pending and duplicate UID are rejected");
}
typedef struct {
    uint32_t magic, version, sequence;
    struct { uint32_t occupied; uint8_t uid[4]; } slots[4];
    uint32_t pending, selected_slot;
    uint8_t uid[4];
    uint32_t crc, commit;
} OldRecord;
static uint32_t old_crc(const void *data, unsigned bytes)
{
    uint32_t crc = 0xFFFFFFFFU;
    const uint8_t *p = data;
    unsigned bit;
    while (bytes--) {
        crc ^= *p++;
        for (bit = 0U; bit < 8U; bit++) crc = (crc >> 1U) ^ ((crc & 1U) ? 0xEDB88320U : 0U);
    }
    return ~crc;
}
static void migration_tests(void)
{
    OldRecord old;
    ParkingSnapshot restored, next;
    unsigned cut;
    blank(); memset(&old, 0, sizeof(old)); CHECK(sizeof(old) == 64U);
    old.magic = 0x5041524BU; old.version = 1U; old.sequence = 123U;
    old.slots[2].occupied = 1U; old.slots[2].uid[0] = 42U;
    old.crc = old_crc(&old, 56U); old.commit = 0x434F4D54U;
    memcpy(flash_memory[0], &old, sizeof(old));
    CHECK(parking_store_load(&restored) == PARK_STORE_OK);
    CHECK(restored.slots[2].occupied && restored.slots[2].uid[0] == 42U && !restored.slots[2].start_epoch);
    next = restored; next.slots[2].start_epoch = 1700000000U;
    memcpy(backup, flash_memory, sizeof(backup));
    for (cut = 0U; cut < PARK_STORE_RECORD_BYTES / 4U; cut++) {
        memcpy(flash_memory, backup, sizeof(backup));
        CHECK(parking_store_load(&restored) == PARK_STORE_OK);
        writes_left = (int)cut; CHECK(!parking_store_save(&next)); writes_left = -1;
        CHECK(parking_store_load(&restored) == PARK_STORE_OK && !restored.slots[2].start_epoch);
        CHECK(parking_store_save(&next));
        CHECK(parking_store_load(&restored) == PARK_STORE_OK && restored.slots[2].start_epoch == 1700000000U);
    }
    pass("V1 64-byte records migrate to mixed V2 128-byte logs with epoch preserved and every torn migration retried");

    memcpy(flash_memory, backup, sizeof(backup));
    flash_memory[0][0] ^= 1U;
    CHECK(parking_store_load(&restored) == PARK_STORE_ERROR);
    memcpy(flash_memory, backup, sizeof(backup));
    flash_memory[0][4] ^= 1U;
    CHECK(parking_store_load(&restored) == PARK_STORE_ERROR);
    blank(); CHECK(parking_store_load(&restored) == PARK_STORE_EMPTY); CHECK(parking_store_save(&next));
    flash_memory[0][0] ^= 1U;
    CHECK(parking_store_load(&restored) == PARK_STORE_ERROR);
    pass("committed V1 and V2 header corruption cannot be silently discarded");

    blank(); memcpy(&flash_memory[0][PARK_STORE_BANK_BYTES - 64U], &old, sizeof(old));
    CHECK(parking_store_load(&restored) == PARK_STORE_OK);
    CHECK(parking_store_save(&next)); CHECK(erases == 1U);
    CHECK(parking_store_load(&restored) == PARK_STORE_OK && restored.slots[2].start_epoch == 1700000000U);
    CHECK(parking_store_format());
    CHECK(parking_store_load(&restored) == PARK_STORE_EMPTY);
    CHECK(parking_store_save(&next));
    pass("legacy tail record rotates to the other bank and explicit format rebuilds writable empty storage");
}
static void health_tests(void)
{
    unsigned dead, task;
    uint32_t now;
    health_init(0U);
    for (now = 0U; now < 10000U; now += 100U) {
        for (task = 0U; task < HEALTH_COUNT; task++) health_beat((HealthTask)task, now);
        CHECK(health_check(now));
    }
    for (dead = 0U; dead < HEALTH_COUNT; dead++) {
        health_init(0U);
        for (task = 0U; task < HEALTH_COUNT; task++)
            if (task != dead) health_beat((HealthTask)task, HEALTH_TIMEOUT_MS + 1U);
        CHECK(!health_check(HEALTH_TIMEOUT_MS + 1U));
        CHECK(health_failed_mask() == (1UL << dead));
        health_beat((HealthTask)dead, HEALTH_TIMEOUT_MS + 1U);
        CHECK(!health_check(HEALTH_TIMEOUT_MS + 1U));
    }
    pass("every supervised task must progress; overdue failure remains latched even if task resumes");
    health_init(0xFFFFFF00U);
    CHECK(health_check(0x100U));
    CHECK(!health_check(0xFFFFFF00U + HEALTH_TIMEOUT_MS + 1U));
    health_init(100U);
    for (task = 0U; task < HEALTH_COUNT; task++) health_beat((HealthTask)task, 101U);
    CHECK(health_check(100U));
    pass("tick wrap and task preemption between supervisor timestamp and heartbeat read are handled");
}
int main(void)
{
    storage_tests(); migration_tests(); health_tests();
    printf("%u recovery/watchdog test groups passed\n", tests);
    return 0;
}
