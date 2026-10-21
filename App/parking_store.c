#include "parking_store.h"
#include <stddef.h>
#include <string.h>

#define STORE_MAGIC 0x5041524BU
#define STORE_VERSION 2U
#define STORE_COMMIT 0x434F4D54U
typedef struct {
    uint32_t magic, version, sequence;
    ParkingSnapshot snapshot;
    uint8_t reserved[48]; /* 固定128字节；为后续版本保留，参与CRC */
    uint32_t crc, commit;
} StoreRecord;
/* 旧版64字节记录仍可读；缺少起始时间的占用记录由业务层标记待核费。 */
typedef struct {
    uint32_t magic, version, sequence;
    struct { uint32_t occupied; uint8_t uid[4]; } slots[PARKING_SLOT_COUNT];
    uint32_t pending, selected_slot;
    uint8_t uid[4];
    uint32_t crc, commit;
} LegacyRecord;
typedef char record_size_check[(sizeof(StoreRecord) == PARK_STORE_RECORD_BYTES) ? 1 : -1];
static uint32_t latest_sequence, next_offset;
static unsigned current_bank;
static uint8_t ready, have_record;
uint8_t parking_store_format(void)
{
    ready = 0U;
    if (!parking_flash_erase(0U) || !parking_flash_erase(1U)) return 0U;
    current_bank = 0U; latest_sequence = next_offset = 0U; have_record = 0U; ready = 1U;
    return 1U;
}

static uint32_t checksum(const void *data, uint32_t bytes)
{
    const uint8_t *p = data;
    uint32_t crc = 0xFFFFFFFFU;
    unsigned bit;
    while (bytes--) {
        crc ^= *p++;
        for (bit = 0U; bit < 8U; bit++)
            crc = (crc >> 1) ^ ((crc & 1U) ? 0xEDB88320U : 0U);
    }
    return ~crc;
}

static uint8_t snapshot_valid(const ParkingSnapshot *s)
{
    unsigned i, j;
    if (s->pending > 2U || s->selected_slot >= PARKING_SLOT_COUNT) return 0U;
    for (i = 0U; i < PARKING_SLOT_COUNT; i++) {
        if (s->slots[i].occupied > 1U) return 0U;
        for (j = 0U; j < i; j++)
            if (s->slots[i].occupied && s->slots[j].occupied &&
                memcmp(s->slots[i].uid, s->slots[j].uid, 4U) == 0) return 0U;
    }
    return 1U;
}

static uint8_t erased(const StoreRecord *r)
{
    const uint8_t *p = (const uint8_t *)r;
    unsigned i;
    for (i = 0U; i < sizeof(*r); i++) if (p[i] != 0xFFU) return 0U;
    return 1U;
}

ParkingStoreResult parking_store_load(ParkingSnapshot *snapshot)
{
    StoreRecord r;
    unsigned bank;
    uint32_t offset, used[2] = {0U, 0U};
    uint8_t dirty = 0U, corrupt = 0U;
    if (!snapshot) return PARK_STORE_ERROR;
    memset(snapshot, 0, sizeof(*snapshot));
    ready = have_record = 0U;
    latest_sequence = 0U;
    current_bank = 0U;
    for (bank = 0U; bank < 2U; bank++) {
        for (offset = 0U; offset < PARK_STORE_BANK_BYTES; offset += sizeof(LegacyRecord)) {
            LegacyRecord old;
            parking_flash_read(bank, offset, &old, sizeof(old));
            if (old.commit == STORE_COMMIT &&
                (old.magic != STORE_MAGIC || old.version != 1U)) {
                /* 旧版已提交头损坏也必须锁止，不能作为撕裂写静默忽略。 */
                dirty = corrupt = 1U; used[bank] = offset + sizeof(old); continue;
            }
            if (old.magic == STORE_MAGIC && old.version == 1U && old.commit == STORE_COMMIT) {
                unsigned i;
                dirty = 1U;
                used[bank] = offset + sizeof(old);
                memset(&r, 0, sizeof(r));
                r.sequence = old.sequence;
                for (i = 0U; i < PARKING_SLOT_COUNT; i++) {
                    r.snapshot.slots[i].occupied = old.slots[i].occupied;
                    memcpy(r.snapshot.slots[i].uid, old.slots[i].uid, 4U);
                }
                r.snapshot.pending = old.pending; r.snapshot.selected_slot = old.selected_slot;
                memcpy(r.snapshot.uid, old.uid, 4U);
                if (old.crc != checksum(&old, offsetof(LegacyRecord, crc)) ||
                    !snapshot_valid(&r.snapshot)) { corrupt = 1U; continue; }
                if (!have_record || (int32_t)(r.sequence - latest_sequence) > 0) {
                    have_record = 1U; latest_sequence = r.sequence; current_bank = bank;
                    *snapshot = r.snapshot;
                }
                continue;
            }
            if (offset + sizeof(r) > PARK_STORE_BANK_BYTES) {
                const uint8_t *bytes = (const uint8_t *)&old;
                unsigned i;
                for (i = 0U; i < sizeof(old); i++) if (bytes[i] != 0xFFU) {
                    dirty = 1U; used[bank] = PARK_STORE_BANK_BYTES; break;
                }
                continue;
            }
            parking_flash_read(bank, offset, &r, sizeof(r));
            /* 空记录以64字节检查，兼容旧版日志与撕裂写边界。 */
            if (old.magic == 0xFFFFFFFFU && old.commit == 0xFFFFFFFFU) {
                const uint8_t *bytes = (const uint8_t *)&old;
                unsigned i;
                for (i = 0U; i < sizeof(old) && bytes[i] == 0xFFU; i++) {}
                if (i == sizeof(old)) continue;
            }
            if (erased(&r)) continue;
            dirty = 1U;
            used[bank] = offset + sizeof(old);
            if (r.magic == STORE_MAGIC && r.version == STORE_VERSION) {
                used[bank] = offset + sizeof(r);
                offset += sizeof(old); /* 不把新记录的后半部分当成另一条旧记录 */
            }
            if (r.commit != STORE_COMMIT) continue;
            if (r.magic != STORE_MAGIC || r.version != STORE_VERSION ||
                r.crc != checksum(&r, offsetof(StoreRecord, crc)) ||
                !snapshot_valid(&r.snapshot)) {
                corrupt = 1U; /* 已提交记录损坏不能静默回退 */
                continue;
            }
            if (!have_record || (int32_t)(r.sequence - latest_sequence) > 0) {
                have_record = 1U;
                latest_sequence = r.sequence;
                current_bank = bank;
                *snapshot = r.snapshot;
            }
        }
    }
    next_offset = used[current_bank];
    if (corrupt || (!have_record && dirty)) return PARK_STORE_ERROR;
    ready = 1U;
    return have_record ? PARK_STORE_OK : PARK_STORE_EMPTY;
}

uint8_t parking_store_save(const ParkingSnapshot *snapshot)
{
    StoreRecord r, verify;
    uint32_t offset, word;
    unsigned bank = current_bank;
    if (!ready || !snapshot || !snapshot_valid(snapshot)) return 0U;
    ready = 0U; /* 任意写失败均禁止继续盲写 */
    offset = next_offset;
    if (offset + sizeof(r) > PARK_STORE_BANK_BYTES) {
        bank ^= 1U;
        if (!parking_flash_erase(bank)) return 0U;
        offset = 0U; /* 最新有效记录仍保留在旧扇区 */
    }
    memset(&r, 0, sizeof(r));
    r.magic = STORE_MAGIC;
    r.version = STORE_VERSION;
    r.sequence = latest_sequence + 1U;
    r.snapshot = *snapshot;
    r.crc = checksum(&r, offsetof(StoreRecord, crc));
    r.commit = 0xFFFFFFFFU;
    for (word = 0U; word < offsetof(StoreRecord, commit); word += 4U) {
        uint32_t value;
        memcpy(&value, (const uint8_t *)&r + word, 4U);
        if (!parking_flash_program(bank, offset + word, value)) return 0U;
    }
    parking_flash_read(bank, offset, &verify, sizeof(verify));
    if (memcmp(&r, &verify, sizeof(r)) != 0) return 0U;
    if (!parking_flash_program(bank, offset + offsetof(StoreRecord, commit), STORE_COMMIT))
        return 0U;
    parking_flash_read(bank, offset, &verify, sizeof(verify));
    r.commit = STORE_COMMIT;
    if (memcmp(&r, &verify, sizeof(r)) != 0) return 0U;
    current_bank = bank;
    latest_sequence = r.sequence;
    next_offset = offset + sizeof(r);
    ready = have_record = 1U;
    return 1U;
}
