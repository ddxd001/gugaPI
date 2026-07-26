#ifndef APP_SEQ_STORE_H_
#define APP_SEQ_STORE_H_

#include <stdint.h>

#include "app/action.h"
#include "drivers/common/driver_status.h"

namespace app {

/* Action sequence FRAM persistence. 8 slots, each up to 64 instructions.
 * Stored independently from ConfigStore at FRAM address 0x0100. */

static const uint8_t SEQ_SLOT_COUNT = 8U;

struct SeqSlotInfo {
    bool valid;
    uint8_t count;
};

/* Read the lightweight slot header without touching ActionRunner.
 * A non-valid header is reported as an empty slot. */
drivers::DriverStatus SeqStore_GetInfo(uint8_t slot, SeqSlotInfo *out_info);

/* Save the current ActionRunner instruction table to slot n (0-7).
 * Overwrites any existing data in that slot. */
drivers::DriverStatus SeqStore_Save(uint8_t slot);

/* Load slot n into the ActionRunner instruction table.
 * Replaces the current table. Does not start the sequence. */
drivers::DriverStatus SeqStore_Load(uint8_t slot);

/* Read slot n into the provided array without touching ActionRunner.
 * Returns the instructions and count for the shell dump command. */
drivers::DriverStatus SeqStore_Read(uint8_t slot,
                                    Instr *out_instrs,
                                    uint8_t *out_count);

/* Delete (invalidate) slot n. */
drivers::DriverStatus SeqStore_Delete(uint8_t slot);

/* Check if slot n contains a valid sequence. */
bool SeqStore_IsValid(uint8_t slot);

/* Get the instruction count for slot n (0 if invalid/empty). */
uint8_t SeqStore_GetCount(uint8_t slot);

} /* namespace app */

#endif /* APP_SEQ_STORE_H_ */
