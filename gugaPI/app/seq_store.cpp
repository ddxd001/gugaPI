#include "app/seq_store.h"

#include "app/action.h"
#include "board/board_fram.h"
#include "drivers/common/driver_status.h"
#include <string.h>

namespace app {
namespace {

/* FRAM layout (starting at 0x0100, after ConfigStore at 0x0000):
 *   Header:  [magic(4)] [version(2)] [slot_count(1)] [reserved(1)]
 *   Slot 0: [valid(1)] [count(1)] [instrs(count*10)] [crc(4)]
 *   Slot 1: ...
 *   ...
 *   Slot 7: ...
 *
 * Each slot is fixed-size: 2 + 64*10 + 4 = 646 bytes.
 * Total: 8 (header) + 8 * 646 = 5176 bytes.
 */

static const uint16_t kFramBase = 0x0100U;
static const uint32_t kMagic = 0x53455131U; /* "SEQ1" */
static const uint16_t kVersion = 1U;
static const uint8_t kInstrSize = 10U;
static const uint16_t kSlotPayloadMax = 64U * kInstrSize; /* 640 */
static const uint16_t kSlotSize = 2U + kSlotPayloadMax + 4U; /* 646 */
static const uint16_t kHeaderSize = 8U;

uint16_t SlotAddr(uint8_t slot)
{
    return static_cast<uint16_t>(kFramBase + kHeaderSize +
                                 static_cast<uint16_t>(slot) * kSlotSize);
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

/* Serialize one Instr to 10 bytes. */
void SerializeInstr(const Instr *instr, uint8_t *buf)
{
    buf[0] = static_cast<uint8_t>(instr->op);
    WriteI32(&buf[1], instr->param1);
    WriteU16(&buf[5], static_cast<uint16_t>(instr->param2));
    buf[7] = static_cast<uint8_t>(instr->until);
    buf[8] = instr->on_success;
    buf[9] = instr->on_timeout;
}

/* Deserialize 10 bytes to one Instr. */
void DeserializeInstr(const uint8_t *buf, Instr *instr)
{
    instr->op = static_cast<ActionOp>(buf[0]);
    instr->param1 = ReadI32(&buf[1]);
    instr->param2 = static_cast<int32_t>(ReadU16(&buf[5]));
    instr->until = static_cast<ActionCond>(buf[7]);
    instr->on_success = buf[8];
    instr->on_timeout = buf[9];
}

bool EnsureHeaderWritten(void)
{
    uint8_t header[kHeaderSize];
    if (board::Board_FramRead(kFramBase, header, kHeaderSize) !=
        drivers::DRIVER_OK) {
        return false;
    }
    if (ReadU32(&header[0]) == kMagic) {
        return true;
    }
    WriteU32(&header[0], kMagic);
    WriteU16(&header[4], kVersion);
    header[6] = SEQ_SLOT_COUNT;
    header[7] = 0U;
    return board::Board_FramWrite(kFramBase, header, kHeaderSize) ==
           drivers::DRIVER_OK;
}

} /* namespace */

drivers::DriverStatus SeqStore_GetInfo(uint8_t slot, SeqSlotInfo *out_info)
{
    if ((slot >= SEQ_SLOT_COUNT) || (out_info == 0)) {
        return drivers::DRIVER_ERROR_INVALID_ARG;
    }

    out_info->valid = false;
    out_info->count = 0U;

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
    if (!EnsureHeaderWritten()) {
        return drivers::DRIVER_ERROR;
    }

    const ActionRunnerState *state = ActionRunner_GetState();
    ActionValidationResult validation;
    if (ActionRunner_Validate(&validation) != drivers::DRIVER_OK) {
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
        if (ActionRunner_AddInstr(instr.op, instr.param1, instr.param2,
                                  instr.until, instr.on_success,
                                  instr.on_timeout) != drivers::DRIVER_OK) {
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
