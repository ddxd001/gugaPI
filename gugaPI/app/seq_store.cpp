#include "app/seq_store.h"

#include <string.h>

#include "app/fram_layout.h"
#include "board/board_fram.h"

namespace app {
namespace {

static const uint32_t kHeaderMagic = 0x31515347U; /* "GSQ1" */
static const uint8_t kFormatVersion = 1U;
static const uint8_t kSlotValid = 0xA5U;
static const uint16_t kSlotDataOffset = 6U;
static const uint16_t kSlotCrcOffset =
    kSlotDataOffset + FRAM_SEQ_MAX_INSTRS * FRAM_SEQ_INSTR_SIZE;
static_assert(kSlotCrcOffset + 4U == FRAM_SEQ_SLOT_SIZE,
              "SeqStore slot offsets must match the FRAM map");

uint8_t g_slotScratch[FRAM_SEQ_SLOT_SIZE];
uint8_t g_slotVerify[FRAM_SEQ_SLOT_SIZE];
Instr g_instrScratch[FRAM_SEQ_MAX_INSTRS];

uint16_t SlotAddress(uint8_t slot)
{
    return static_cast<uint16_t>(
        FRAM_SEQ_SLOT_ADDRESS +
        static_cast<uint16_t>(slot) * FRAM_SEQ_SLOT_SIZE);
}

void WriteU16(uint8_t *data, uint16_t value)
{
    data[0] = static_cast<uint8_t>(value);
    data[1] = static_cast<uint8_t>(value >> 8U);
}

uint16_t ReadU16(const uint8_t *data)
{
    return static_cast<uint16_t>(data[0]) |
           static_cast<uint16_t>(
               static_cast<uint16_t>(data[1]) << 8U);
}

void WriteU32(uint8_t *data, uint32_t value)
{
    data[0] = static_cast<uint8_t>(value);
    data[1] = static_cast<uint8_t>(value >> 8U);
    data[2] = static_cast<uint8_t>(value >> 16U);
    data[3] = static_cast<uint8_t>(value >> 24U);
}

uint32_t ReadU32(const uint8_t *data)
{
    return static_cast<uint32_t>(data[0]) |
           (static_cast<uint32_t>(data[1]) << 8U) |
           (static_cast<uint32_t>(data[2]) << 16U) |
           (static_cast<uint32_t>(data[3]) << 24U);
}

void WriteI32(uint8_t *data, int32_t value)
{
    WriteU32(data, static_cast<uint32_t>(value));
}

int32_t ReadI32(const uint8_t *data)
{
    return static_cast<int32_t>(ReadU32(data));
}

uint32_t Crc32(const uint8_t *data, uint16_t length)
{
    uint32_t crc = 0xFFFFFFFFU;
    for (uint16_t i = 0U; i < length; i++) {
        crc ^= static_cast<uint32_t>(data[i]) << 24U;
        for (uint8_t bit = 0U; bit < 8U; bit++) {
            crc = ((crc & 0x80000000U) != 0U)
                ? (crc << 1U) ^ 0x04C11DB7U
                : crc << 1U;
        }
    }
    return crc ^ 0xFFFFFFFFU;
}

uint8_t Crc8(const uint8_t *data, uint8_t length)
{
    uint8_t crc = 0U;
    for (uint8_t i = 0U; i < length; i++) {
        crc ^= data[i];
        for (uint8_t bit = 0U; bit < 8U; bit++) {
            crc = ((crc & 0x80U) != 0U)
                ? static_cast<uint8_t>((crc << 1U) ^ 0x07U)
                : static_cast<uint8_t>(crc << 1U);
        }
    }
    return crc;
}

bool Equal(const uint8_t *left, const uint8_t *right, uint16_t length)
{
    for (uint16_t i = 0U; i < length; i++) {
        if (left[i] != right[i]) {
            return false;
        }
    }
    return true;
}

drivers::DriverStatus WriteVerify(uint16_t address,
                                  const uint8_t *data,
                                  uint16_t length)
{
    if (board::Board_FramWrite(address, data, length) !=
        drivers::DRIVER_OK) {
        return drivers::DRIVER_ERROR;
    }
    if ((length > sizeof(g_slotVerify)) ||
        (board::Board_FramRead(address, g_slotVerify, length) !=
         drivers::DRIVER_OK) ||
        !Equal(data, g_slotVerify, length)) {
        return drivers::DRIVER_ERROR;
    }
    return drivers::DRIVER_OK;
}

void SerializeInstr(const Instr &instr, uint8_t *data)
{
    data[0] = static_cast<uint8_t>(instr.op);
    WriteI32(&data[1], instr.param1);
    WriteU16(&data[5], static_cast<uint16_t>(instr.param2));
    data[7] = static_cast<uint8_t>(instr.until);
    WriteI32(&data[8], instr.condition_value);
    data[12] = instr.on_success;
    data[13] = instr.on_timeout;
}

void DeserializeInstr(const uint8_t *data, Instr *instr)
{
    instr->op = static_cast<ActionOp>(data[0]);
    instr->param1 = ReadI32(&data[1]);
    instr->param2 = static_cast<int32_t>(ReadU16(&data[5]));
    instr->until = static_cast<ActionCond>(data[7]);
    instr->condition_value = ReadI32(&data[8]);
    instr->on_success = data[12];
    instr->on_timeout = data[13];
}

bool HeaderValid(const uint8_t *header)
{
    return (ReadU32(header) == kHeaderMagic) &&
           (header[4] == kFormatVersion) &&
           (header[5] == FRAM_SEQ_SLOT_COUNT) &&
           (header[6] == FRAM_SEQ_MAX_INSTRS) &&
           (header[7] == Crc8(header, 7U));
}

drivers::DriverStatus WriteHeader(void)
{
    uint8_t header[FRAM_SEQ_HEADER_SIZE] = {};
    WriteU32(header, kHeaderMagic);
    header[4] = kFormatVersion;
    header[5] = FRAM_SEQ_SLOT_COUNT;
    header[6] = FRAM_SEQ_MAX_INSTRS;
    header[7] = Crc8(header, 7U);
    return WriteVerify(FRAM_SEQ_HEADER_ADDRESS,
                       header,
                       sizeof(header));
}

drivers::DriverStatus EnsureLayout(void)
{
    uint8_t header[FRAM_SEQ_HEADER_SIZE];
    const drivers::DriverStatus read_status =
        board::Board_FramRead(FRAM_SEQ_HEADER_ADDRESS,
                              header,
                              sizeof(header));
    if (read_status != drivers::DRIVER_OK) {
        return drivers::DRIVER_ERROR;
    }
    if (HeaderValid(header)) {
        return drivers::DRIVER_OK;
    }
    return SeqStore_Format();
}

bool SlotValid(const uint8_t *slot)
{
    if ((slot[0] != kSlotValid) ||
        (slot[1] == 0U) ||
        (slot[1] > FRAM_SEQ_MAX_INSTRS)) {
        return false;
    }
    return ReadU32(&slot[kSlotCrcOffset]) ==
           Crc32(&slot[1],
                 static_cast<uint16_t>(kSlotCrcOffset - 1U));
}

drivers::DriverStatus ReadSlot(uint8_t slot, uint8_t *data)
{
    if (board::Board_FramRead(SlotAddress(slot),
                              data,
                              FRAM_SEQ_SLOT_SIZE) !=
        drivers::DRIVER_OK) {
        return drivers::DRIVER_ERROR;
    }
    return SlotValid(data)
        ? drivers::DRIVER_OK
        : drivers::DRIVER_ERROR_NOT_INITIALIZED;
}

drivers::DriverStatus AddDecodedInstr(const Instr &instr)
{
    if (ActionCondition_IsCompareOp(instr.op)) {
        ActionConditionConfig condition;
        if (!ActionCondition_Decode(&instr, &condition)) {
            return drivers::DRIVER_ERROR_INVALID_ARG;
        }
        return ActionRunner_AddCompareInstr(
            instr.op, instr.param1, &condition,
            instr.on_success, instr.on_timeout);
    }
    if (instr.op == ACT_OP_ROAD_NAV) {
        return ActionRunner_AddRoadNav(
            instr.condition_value, instr.param1, instr.param2,
            instr.on_success, instr.on_timeout);
    }
    if (instr.op == ACT_OP_DM_POSITION) {
        return ActionRunner_AddDmPosition(
            instr.until == ACT_COND_DM_RELATIVE,
            instr.param1,
            instr.param2,
            instr.condition_value,
            instr.on_success,
            instr.on_timeout);
    }
    return ActionRunner_AddInstr(
        instr.op, instr.param1, instr.param2, instr.until,
        instr.on_success, instr.on_timeout);
}

} /* namespace */

drivers::DriverStatus SeqStore_Format(void)
{
    uint8_t invalid = 0U;
    for (uint8_t slot = 0U; slot < FRAM_SEQ_SLOT_COUNT; slot++) {
        if (WriteVerify(SlotAddress(slot), &invalid, 1U) !=
            drivers::DRIVER_OK) {
            return drivers::DRIVER_ERROR;
        }
    }
    return WriteHeader();
}

drivers::DriverStatus SeqStore_Init(void)
{
    return EnsureLayout();
}

drivers::DriverStatus SeqStore_GetInfo(uint8_t slot,
                                       SeqSlotInfo *out_info)
{
    if ((slot >= FRAM_SEQ_SLOT_COUNT) || (out_info == 0)) {
        return drivers::DRIVER_ERROR_INVALID_ARG;
    }
    out_info->valid = false;
    out_info->count = 0U;
    const drivers::DriverStatus layout = EnsureLayout();
    if (layout != drivers::DRIVER_OK) {
        return layout;
    }
    const drivers::DriverStatus status = ReadSlot(slot, g_slotScratch);
    if (status == drivers::DRIVER_ERROR_NOT_INITIALIZED) {
        return drivers::DRIVER_OK;
    }
    if (status != drivers::DRIVER_OK) {
        return status;
    }
    out_info->valid = true;
    out_info->count = g_slotScratch[1];
    return drivers::DRIVER_OK;
}

drivers::DriverStatus SeqStore_Save(uint8_t slot)
{
    if (slot >= FRAM_SEQ_SLOT_COUNT) {
        return drivers::DRIVER_ERROR_INVALID_ARG;
    }
    const drivers::DriverStatus layout = EnsureLayout();
    if (layout != drivers::DRIVER_OK) {
        return layout;
    }
    ActionValidationResult validation;
    if (ActionRunner_ValidateCompetition(&validation) !=
        drivers::DRIVER_OK) {
        return drivers::DRIVER_ERROR_INVALID_ARG;
    }
    const ActionRunnerState *state = ActionRunner_GetState();
    if ((state->count == 0U) ||
        (state->count > FRAM_SEQ_MAX_INSTRS)) {
        return drivers::DRIVER_ERROR_INVALID_ARG;
    }

    uint32_t generation = 1U;
    if (ReadSlot(slot, g_slotScratch) == drivers::DRIVER_OK) {
        generation = ReadU32(&g_slotScratch[2]) + 1U;
    }

    (void) memset(g_slotScratch, 0, sizeof(g_slotScratch));
    g_slotScratch[0] = 0U;
    g_slotScratch[1] = state->count;
    WriteU32(&g_slotScratch[2], generation);
    for (uint8_t i = 0U; i < state->count; i++) {
        SerializeInstr(state->instrs[i],
                       &g_slotScratch[
                           kSlotDataOffset +
                           static_cast<uint16_t>(i) *
                           FRAM_SEQ_INSTR_SIZE]);
    }
    WriteU32(&g_slotScratch[kSlotCrcOffset],
             Crc32(&g_slotScratch[1],
                   static_cast<uint16_t>(kSlotCrcOffset - 1U)));

    const uint16_t address = SlotAddress(slot);
    uint8_t invalid = 0U;
    if ((WriteVerify(address, &invalid, 1U) != drivers::DRIVER_OK) ||
        (WriteVerify(static_cast<uint16_t>(address + 1U),
                     &g_slotScratch[1],
                     static_cast<uint16_t>(
                         FRAM_SEQ_SLOT_SIZE - 1U)) !=
         drivers::DRIVER_OK)) {
        return drivers::DRIVER_ERROR;
    }
    const uint8_t valid = kSlotValid;
    return WriteVerify(address, &valid, 1U);
}

drivers::DriverStatus SeqStore_Read(uint8_t slot,
                                    Instr *out_instrs,
                                    uint8_t *out_count)
{
    if ((slot >= FRAM_SEQ_SLOT_COUNT) ||
        (out_instrs == 0) ||
        (out_count == 0)) {
        return drivers::DRIVER_ERROR_INVALID_ARG;
    }
    *out_count = 0U;
    const drivers::DriverStatus layout = EnsureLayout();
    if (layout != drivers::DRIVER_OK) {
        return layout;
    }
    const drivers::DriverStatus status = ReadSlot(slot, g_slotScratch);
    if (status != drivers::DRIVER_OK) {
        return status;
    }
    for (uint8_t i = 0U; i < g_slotScratch[1]; i++) {
        DeserializeInstr(
            &g_slotScratch[
                kSlotDataOffset +
                static_cast<uint16_t>(i) * FRAM_SEQ_INSTR_SIZE],
            &out_instrs[i]);
    }
    *out_count = g_slotScratch[1];
    return drivers::DRIVER_OK;
}

drivers::DriverStatus SeqStore_Load(uint8_t slot)
{
    uint8_t count = 0U;
    const drivers::DriverStatus read =
        SeqStore_Read(slot, g_instrScratch, &count);
    if (read != drivers::DRIVER_OK) {
        return read;
    }
    if (ActionRunner_Clear() != drivers::DRIVER_OK) {
        return drivers::DRIVER_ERROR_BUSY;
    }
    for (uint8_t i = 0U; i < count; i++) {
        if (AddDecodedInstr(g_instrScratch[i]) != drivers::DRIVER_OK) {
            (void) ActionRunner_Clear();
            return drivers::DRIVER_ERROR;
        }
    }
    return drivers::DRIVER_OK;
}

drivers::DriverStatus SeqStore_Delete(uint8_t slot)
{
    if (slot >= FRAM_SEQ_SLOT_COUNT) {
        return drivers::DRIVER_ERROR_INVALID_ARG;
    }
    const drivers::DriverStatus layout = EnsureLayout();
    if (layout != drivers::DRIVER_OK) {
        return layout;
    }
    const uint8_t invalid = 0U;
    return WriteVerify(SlotAddress(slot), &invalid, 1U);
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
    return ((SeqStore_GetInfo(slot, &info) == drivers::DRIVER_OK) &&
            info.valid) ? info.count : 0U;
}

} /* namespace app */
