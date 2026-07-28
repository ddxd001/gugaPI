#include "app/seq_store.h"

#include "app/action.h"
#include "board/board_fram.h"
#include "drivers/common/driver_status.h"
#include <string.h>

namespace app {
namespace {

/* FRAM layout v2 (starting at 0x0100, after ConfigStore at 0x0000):
 *   Header:  [magic(4)] [version(2)] [slot_count(1)] [reserved(1)]
 *   Slot 0: [valid(1)] [count(1)] [instrs(count*14)] [crc(4)]
 *   Slot 1: ...
 *   ...
 *   Slot 7: ...
 *
 * Each slot is fixed-size: 2 + 64*14 + 4 = 902 bytes.
 * Total: 8 (header) + 8 * 902 = 7224 bytes.
 *
 * The remaining bytes before the board self-test address hold one complete
 * v1 slot plus a migration journal. This allows a power-loss-safe in-place
 * migration even when a growing v2 destination overlaps its v1 source.
 */

static const uint16_t kFramBase = 0x0100U;
static const uint32_t kMagic = 0x53455131U; /* "SEQ1" */
static const uint16_t kVersionV1 = 1U;
static const uint16_t kVersion = 2U;
/* Versions 1, 3 (migrating), and 2 differ only in the low byte.  Moving
 * v1 -> migrating is therefore a single-byte FRAM write: power loss cannot
 * leave a half-written 16-bit version that no subsequent boot recognizes. */
static const uint16_t kMigrationVersion = 3U;
static const uint8_t kInstrSizeV1 = 10U;
static const uint8_t kInstrSize = 14U;
static const uint16_t kSlotPayloadMax = 64U * kInstrSize; /* 896 */
static const uint16_t kSlotSize = 2U + kSlotPayloadMax + 4U; /* 902 */
static const uint16_t kSlotPayloadMaxV1 = 64U * kInstrSizeV1;
static const uint16_t kSlotSizeV1 =
    2U + kSlotPayloadMaxV1 + 4U; /* 646 */
static const uint16_t kHeaderSize = 8U;
static const uint16_t kMigrationStageAddr =
    kFramBase + kHeaderSize + SEQ_SLOT_COUNT * kSlotSize; /* 0x1D38 */
static const uint16_t kMigrationJournalAddr =
    kMigrationStageAddr + kSlotSizeV1; /* 0x1FBE */
static const uint32_t kMigrationMagic = 0x3247494DU; /* "MIG2" */
static const uint8_t kMigrationJournalSize = 12U;

uint16_t SlotAddr(uint8_t slot)
{
    return static_cast<uint16_t>(kFramBase + kHeaderSize +
                                 static_cast<uint16_t>(slot) * kSlotSize);
}

uint16_t SlotAddrV1(uint8_t slot)
{
    return static_cast<uint16_t>(kFramBase + kHeaderSize +
                                 static_cast<uint16_t>(slot) * kSlotSizeV1);
}

void WriteU16(uint8_t *buf, uint16_t v)
{
    buf[0] = static_cast<uint8_t>(v & 0xFFU);
    buf[1] = static_cast<uint8_t>((v >> 8) & 0xFFU);
}

uint16_t ReadU16(const uint8_t *buf)
{
    return static_cast<uint16_t>(buf[0]) |
           (static_cast<uint16_t>(buf[1]) << 8);
}

void WriteU32(uint8_t *buf, uint32_t v)
{
    buf[0] = static_cast<uint8_t>(v & 0xFFU);
    buf[1] = static_cast<uint8_t>((v >> 8) & 0xFFU);
    buf[2] = static_cast<uint8_t>((v >> 16) & 0xFFU);
    buf[3] = static_cast<uint8_t>((v >> 24) & 0xFFU);
}

uint32_t ReadU32(const uint8_t *buf)
{
    return static_cast<uint32_t>(buf[0]) |
           (static_cast<uint32_t>(buf[1]) << 8) |
           (static_cast<uint32_t>(buf[2]) << 16) |
           (static_cast<uint32_t>(buf[3]) << 24);
}

void WriteI32(uint8_t *buf, int32_t v)
{
    WriteU32(buf, static_cast<uint32_t>(v));
}

int32_t ReadI32(const uint8_t *buf)
{
    return static_cast<int32_t>(ReadU32(buf));
}

uint32_t Crc32(const uint8_t *data, uint16_t len)
{
    uint32_t crc = 0xFFFFFFFFU;
    for (uint16_t i = 0; i < len; i++) {
        crc ^= static_cast<uint32_t>(data[i]) << 24;
        for (uint8_t b = 0; b < 8; b++) {
            if ((crc & 0x80000000U) != 0U) {
                crc = (crc << 1) ^ 0x04C11DB7U;
            } else {
                crc <<= 1;
            }
        }
    }
    return crc ^ 0xFFFFFFFFU;
}

/* Serialize one v2 Instr to 14 bytes. */
void SerializeInstr(const Instr *instr, uint8_t *buf)
{
    buf[0] = static_cast<uint8_t>(instr->op);
    WriteI32(&buf[1], instr->param1);
    WriteU16(&buf[5], static_cast<uint16_t>(instr->param2));
    buf[7] = static_cast<uint8_t>(instr->until);
    WriteI32(&buf[8], instr->condition_value);
    buf[12] = instr->on_success;
    buf[13] = instr->on_timeout;
}

/* Deserialize 14-byte v2 and legacy 10-byte v1 instructions. */
void DeserializeInstr(const uint8_t *buf, Instr *instr)
{
    instr->op = static_cast<ActionOp>(buf[0]);
    instr->param1 = ReadI32(&buf[1]);
    instr->param2 = static_cast<int32_t>(ReadU16(&buf[5]));
    instr->until = static_cast<ActionCond>(buf[7]);
    instr->condition_value = ReadI32(&buf[8]);
    instr->on_success = buf[12];
    instr->on_timeout = buf[13];
}

void DeserializeInstrV1(const uint8_t *buf, Instr *instr)
{
    instr->op = static_cast<ActionOp>(buf[0]);
    instr->param1 = ReadI32(&buf[1]);
    instr->param2 = static_cast<int32_t>(ReadU16(&buf[5]));
    instr->until = static_cast<ActionCond>(buf[7]);
    instr->condition_value = 0;
    instr->on_success = buf[8];
    instr->on_timeout = buf[9];
}

bool BuffersEqual(const uint8_t *a, const uint8_t *b, uint16_t length)
{
    for (uint16_t i = 0U; i < length; i++) {
        if (a[i] != b[i]) {
            return false;
        }
    }
    return true;
}

drivers::DriverStatus WriteAndVerify(uint16_t address,
                                     const uint8_t *data,
                                     uint16_t length)
{
    if (board::Board_FramWrite(address, data, length) !=
        drivers::DRIVER_OK) {
        return drivers::DRIVER_ERROR;
    }
    uint8_t verify[kSlotSize];
    if ((length > sizeof(verify)) ||
        (board::Board_FramRead(address, verify, length) !=
         drivers::DRIVER_OK) ||
        !BuffersEqual(data, verify, length)) {
        return drivers::DRIVER_ERROR;
    }
    return drivers::DRIVER_OK;
}

bool ReadMigrationJournal(uint8_t slot, uint32_t expected_crc)
{
    uint8_t journal[kMigrationJournalSize];
    if (board::Board_FramRead(kMigrationJournalAddr, journal,
                             sizeof(journal)) != drivers::DRIVER_OK) {
        return false;
    }
    return (ReadU32(&journal[0]) == kMigrationMagic) &&
           (journal[4] == slot) &&
           (ReadU32(&journal[8]) == expected_crc);
}

drivers::DriverStatus WriteMigrationJournal(uint8_t slot, uint32_t crc)
{
    uint8_t journal[kMigrationJournalSize] = {};
    WriteU32(&journal[0], kMigrationMagic);
    journal[4] = slot;
    WriteU32(&journal[8], crc);
    return WriteAndVerify(kMigrationJournalAddr, journal, sizeof(journal));
}

drivers::DriverStatus StageV1Slot(uint8_t slot, uint8_t *slot_data)
{
    if (board::Board_FramRead(SlotAddrV1(slot), slot_data, kSlotSizeV1) !=
        drivers::DRIVER_OK) {
        return drivers::DRIVER_ERROR;
    }
    const uint32_t crc = Crc32(slot_data, kSlotSizeV1);
    if (WriteAndVerify(kMigrationStageAddr, slot_data, kSlotSizeV1) !=
        drivers::DRIVER_OK) {
        return drivers::DRIVER_ERROR;
    }
    return WriteMigrationJournal(slot, crc);
}

drivers::DriverStatus LoadStagedV1Slot(uint8_t slot, uint8_t *slot_data)
{
    if (board::Board_FramRead(kMigrationStageAddr, slot_data,
                             kSlotSizeV1) != drivers::DRIVER_OK) {
        return drivers::DRIVER_ERROR;
    }
    const uint32_t crc = Crc32(slot_data, kSlotSizeV1);
    if (!ReadMigrationJournal(slot, crc)) {
        return drivers::DRIVER_ERROR_NOT_INITIALIZED;
    }
    return drivers::DRIVER_OK;
}

drivers::DriverStatus ConvertStagedSlot(uint8_t slot,
                                        const uint8_t *v1)
{
    if (v1[0] != 1U) {
        const uint8_t empty[2] = { 0U, 0U };
        return WriteAndVerify(SlotAddr(slot), empty, sizeof(empty));
    }
    const uint8_t count = v1[1];
    if ((count == 0U) || (count > 64U)) {
        return drivers::DRIVER_ERROR_INVALID_ARG;
    }
    const uint16_t old_payload_len =
        static_cast<uint16_t>(2U + count * kInstrSizeV1);
    if (ReadU32(&v1[old_payload_len]) != Crc32(v1, old_payload_len)) {
        return drivers::DRIVER_ERROR;
    }

    uint8_t v2[2U + kSlotPayloadMax + 4U] = {};
    v2[0] = 1U;
    v2[1] = count;
    for (uint8_t i = 0U; i < count; i++) {
        Instr instr;
        DeserializeInstrV1(&v1[2U + i * kInstrSizeV1], &instr);
        SerializeInstr(&instr, &v2[2U + i * kInstrSize]);
    }
    const uint16_t new_payload_len =
        static_cast<uint16_t>(2U + count * kInstrSize);
    WriteU32(&v2[new_payload_len], Crc32(v2, new_payload_len));
    return WriteAndVerify(
        SlotAddr(slot), v2,
        static_cast<uint16_t>(new_payload_len + 4U));
}

drivers::DriverStatus ResumeMigration(uint8_t next_slot)
{
    uint8_t staged[kSlotSizeV1];
    int32_t slot = static_cast<int32_t>(next_slot);
    while (slot >= 0) {
        const uint8_t current = static_cast<uint8_t>(slot);
        drivers::DriverStatus status =
            LoadStagedV1Slot(current, staged);
        if (status != drivers::DRIVER_OK) {
            status = StageV1Slot(current, staged);
            if (status != drivers::DRIVER_OK) {
                return status;
            }
            status = LoadStagedV1Slot(current, staged);
            if (status != drivers::DRIVER_OK) {
                return status;
            }
        }
        status = ConvertStagedSlot(current, staged);
        if (status != drivers::DRIVER_OK) {
            return status;
        }
        const uint8_t following =
            (slot == 0) ? 0xFFU : static_cast<uint8_t>(slot - 1);
        if (board::Board_FramWriteByte(
                static_cast<uint16_t>(kFramBase + 7U), following) !=
            drivers::DRIVER_OK) {
            return drivers::DRIVER_ERROR;
        }
        slot--;
    }

    uint8_t header[kHeaderSize];
    if (board::Board_FramRead(kFramBase, header, sizeof(header)) !=
        drivers::DRIVER_OK) {
        return drivers::DRIVER_ERROR;
    }
    WriteU16(&header[4], kVersion);
    header[6] = SEQ_SLOT_COUNT;
    header[7] = 0U;
    return WriteAndVerify(kFramBase, header, sizeof(header));
}

drivers::DriverStatus EnsureLayout(void)
{
    uint8_t header[kHeaderSize];
    if (board::Board_FramRead(kFramBase, header, kHeaderSize) !=
        drivers::DRIVER_OK) {
        return drivers::DRIVER_ERROR;
    }
    if (ReadU32(&header[0]) != kMagic) {
        WriteU32(&header[0], kMagic);
        WriteU16(&header[4], kVersion);
        header[6] = SEQ_SLOT_COUNT;
        header[7] = 0U;
        return WriteAndVerify(kFramBase, header, sizeof(header));
    }
    const uint16_t version = ReadU16(&header[4]);
    if ((version == kVersion) && (header[6] == SEQ_SLOT_COUNT)) {
        return drivers::DRIVER_OK;
    }
    if (version == kVersionV1) {
        header[7] = static_cast<uint8_t>(SEQ_SLOT_COUNT - 1U);
        if (board::Board_FramWriteByte(
                static_cast<uint16_t>(kFramBase + 7U), header[7]) !=
            drivers::DRIVER_OK) {
            return drivers::DRIVER_ERROR;
        }
        if (board::Board_FramWriteByte(
                static_cast<uint16_t>(kFramBase + 4U),
                static_cast<uint8_t>(kMigrationVersion)) !=
            drivers::DRIVER_OK) {
            return drivers::DRIVER_ERROR;
        }
        return ResumeMigration(header[7]);
    }
    if (version == kMigrationVersion) {
        if (header[7] == 0xFFU) {
            WriteU16(&header[4], kVersion);
            header[7] = 0U;
            return WriteAndVerify(kFramBase, header, sizeof(header));
        }
        if (header[7] < SEQ_SLOT_COUNT) {
            return ResumeMigration(header[7]);
        }
    }
    return drivers::DRIVER_ERROR_INVALID_ARG;
}

} /* namespace */

drivers::DriverStatus SeqStore_Init(void)
{
    return EnsureLayout();
}

drivers::DriverStatus SeqStore_GetInfo(uint8_t slot, SeqSlotInfo *out_info)
{
    if ((slot >= SEQ_SLOT_COUNT) || (out_info == 0)) {
        return drivers::DRIVER_ERROR_INVALID_ARG;
    }

    out_info->valid = false;
    out_info->count = 0U;
    const drivers::DriverStatus layout_status = EnsureLayout();
    if (layout_status != drivers::DRIVER_OK) {
        return layout_status;
    }

    uint8_t header[2];
    if (board::Board_FramRead(SlotAddr(slot), header, 2U) !=
        drivers::DRIVER_OK) {
        return drivers::DRIVER_ERROR;
    }
    if (header[0] != 1U) {
        return drivers::DRIVER_OK;
    }
    if ((header[1] == 0U) || (header[1] > 64U)) {
        return drivers::DRIVER_ERROR_INVALID_ARG;
    }

    out_info->valid = true;
    out_info->count = header[1];
    return drivers::DRIVER_OK;
}

drivers::DriverStatus SeqStore_Save(uint8_t slot)
{
    if (slot >= SEQ_SLOT_COUNT) {
        return drivers::DRIVER_ERROR_INVALID_ARG;
    }
    const drivers::DriverStatus layout_status = EnsureLayout();
    if (layout_status != drivers::DRIVER_OK) {
        return layout_status;
    }

    const ActionRunnerState *state = ActionRunner_GetState();
    ActionValidationResult validation;
    if (ActionRunner_ValidateCompetition(&validation) !=
        drivers::DRIVER_OK) {
        return drivers::DRIVER_ERROR_INVALID_ARG;
    }

    /* Build slot payload: valid(1) + count(1) + instrs + crc(4) */
    uint16_t payload_len = 2U + state->count * kInstrSize;
    uint8_t buf[2U + kSlotPayloadMax + 4U];

    buf[0] = 1U; /* valid */
    buf[1] = state->count;
    for (uint8_t i = 0; i < state->count; i++) {
        SerializeInstr(&state->instrs[i], &buf[2 + i * kInstrSize]);
    }

    uint32_t crc = Crc32(buf, payload_len);
    WriteU32(&buf[payload_len], crc);

    uint16_t total = static_cast<uint16_t>(payload_len + 4U);
    uint16_t addr = SlotAddr(slot);
    if (board::Board_FramWrite(addr, buf, total) != drivers::DRIVER_OK) {
        return drivers::DRIVER_ERROR;
    }
    return drivers::DRIVER_OK;
}

drivers::DriverStatus SeqStore_Load(uint8_t slot)
{
    if (slot >= SEQ_SLOT_COUNT) {
        return drivers::DRIVER_ERROR_INVALID_ARG;
    }
    const drivers::DriverStatus layout_status = EnsureLayout();
    if (layout_status != drivers::DRIVER_OK) {
        return layout_status;
    }

    uint8_t header[2];
    uint16_t addr = SlotAddr(slot);
    if (board::Board_FramRead(addr, header, 2) != drivers::DRIVER_OK) {
        return drivers::DRIVER_ERROR;
    }
    if (header[0] != 1U) {
        return drivers::DRIVER_ERROR_NOT_INITIALIZED;
    }

    uint8_t count = header[1];
    if ((count == 0U) || (count > 64U)) {
        return drivers::DRIVER_ERROR_INVALID_ARG;
    }

    uint16_t payload_len = 2U + count * kInstrSize;
    uint8_t buf[2U + kSlotPayloadMax + 4U];
    uint16_t total = static_cast<uint16_t>(payload_len + 4U);

    if (board::Board_FramRead(addr, buf, total) != drivers::DRIVER_OK) {
        return drivers::DRIVER_ERROR;
    }

    uint32_t stored_crc = ReadU32(&buf[payload_len]);
    uint32_t actual_crc = Crc32(buf, payload_len);
    if (stored_crc != actual_crc) {
        return drivers::DRIVER_ERROR;
    }

    /* Load into ActionRunner */
    if (ActionRunner_Clear() != drivers::DRIVER_OK) {
        return drivers::DRIVER_ERROR_BUSY;
    }
    for (uint8_t i = 0; i < count; i++) {
        Instr instr;
        DeserializeInstr(&buf[2 + i * kInstrSize], &instr);
        drivers::DriverStatus add_status = drivers::DRIVER_ERROR;
        if (ActionCondition_IsCompareOp(instr.op)) {
            ActionConditionConfig condition;
            if (ActionCondition_Decode(&instr, &condition)) {
                add_status = ActionRunner_AddCompareInstr(
                    instr.op, instr.param1, &condition,
                    instr.on_success, instr.on_timeout);
            }
        } else if (instr.op == ACT_OP_ROAD_NAV) {
            add_status = ActionRunner_AddRoadNav(
                instr.condition_value,
                instr.param1,
                instr.param2,
                instr.on_success,
                instr.on_timeout);
        } else {
            add_status = ActionRunner_AddInstr(
                instr.op, instr.param1, instr.param2, instr.until,
                instr.on_success, instr.on_timeout);
        }
        if (add_status != drivers::DRIVER_OK) {
            (void) ActionRunner_Clear();
            return drivers::DRIVER_ERROR;
        }
    }
    return drivers::DRIVER_OK;
}

drivers::DriverStatus SeqStore_Read(uint8_t slot,
                                    Instr *out_instrs,
                                    uint8_t *out_count)
{
    if ((slot >= SEQ_SLOT_COUNT) || (out_instrs == 0) ||
        (out_count == 0)) {
        return drivers::DRIVER_ERROR_INVALID_ARG;
    }

    *out_count = 0U;
    const drivers::DriverStatus layout_status = EnsureLayout();
    if (layout_status != drivers::DRIVER_OK) {
        return layout_status;
    }

    uint8_t header[2];
    uint16_t addr = SlotAddr(slot);
    if (board::Board_FramRead(addr, header, 2) != drivers::DRIVER_OK) {
        return drivers::DRIVER_ERROR;
    }
    if (header[0] != 1U) {
        return drivers::DRIVER_ERROR_NOT_INITIALIZED;
    }

    uint8_t count = header[1];
    if ((count == 0U) || (count > 64U)) {
        return drivers::DRIVER_ERROR_INVALID_ARG;
    }

    uint16_t payload_len = 2U + count * kInstrSize;
    uint8_t buf[2U + kSlotPayloadMax + 4U];
    uint16_t total = static_cast<uint16_t>(payload_len + 4U);

    if (board::Board_FramRead(addr, buf, total) != drivers::DRIVER_OK) {
        return drivers::DRIVER_ERROR;
    }

    uint32_t stored_crc = ReadU32(&buf[payload_len]);
    uint32_t actual_crc = Crc32(buf, payload_len);
    if (stored_crc != actual_crc) {
        return drivers::DRIVER_ERROR;
    }

    for (uint8_t i = 0; i < count; i++) {
        DeserializeInstr(&buf[2 + i * kInstrSize], &out_instrs[i]);
    }
    *out_count = count;
    return drivers::DRIVER_OK;
}

drivers::DriverStatus SeqStore_Delete(uint8_t slot)
{
    if (slot >= SEQ_SLOT_COUNT) {
        return drivers::DRIVER_ERROR_INVALID_ARG;
    }
    const drivers::DriverStatus layout_status = EnsureLayout();
    if (layout_status != drivers::DRIVER_OK) {
        return layout_status;
    }
    uint8_t valid = 0U;
    uint16_t addr = SlotAddr(slot);
    return board::Board_FramWrite(addr, &valid, 1) == drivers::DRIVER_OK
               ? drivers::DRIVER_OK
               : drivers::DRIVER_ERROR;
}

bool SeqStore_IsValid(uint8_t slot)
{
    SeqSlotInfo info = { false, 0U };
    return (SeqStore_GetInfo(slot, &info) == drivers::DRIVER_OK) &&
           info.valid;
}

uint8_t SeqStore_GetCount(uint8_t slot)
{
    SeqSlotInfo info = { false, 0U };
    return (SeqStore_GetInfo(slot, &info) == drivers::DRIVER_OK) &&
           info.valid ? info.count : 0U;
}

} /* namespace app */
