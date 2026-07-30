#ifndef BOARD_BOARD_H_
#define BOARD_BOARD_H_

#include <stdint.h>

#include "drivers/common/driver_status.h"

#ifdef __cplusplus
namespace board {

enum BoardInitSeverity : uint8_t {
    BOARD_INIT_OPTIONAL = 0U,
    BOARD_INIT_DEGRADED,
    BOARD_INIT_MOTION_INHIBIT
};

struct BoardInitEntry {
    const char *name;
    drivers::DriverStatus status;
    BoardInitSeverity severity;
};

static const uint8_t BOARD_INIT_MAX_ENTRIES = 14U;

struct BoardInitReport {
    BoardInitEntry entries[BOARD_INIT_MAX_ENTRIES];
    uint8_t count;
    bool has_degraded_fault;
    bool motion_inhibited;
};

/* Runs every FEATURE_ENABLE_* board-level driver init. Returns the first
 * non-OK status (or DRIVER_OK). Board_Init cannot log itself (board is below
 * services/log), so the caller reads Board_GetFailedDriver and logs it. */
drivers::DriverStatus Board_Init(void);
/* Name of the first driver whose init failed, or nullptr if all succeeded. */
const char *Board_GetFailedDriver(void);
const BoardInitReport *Board_GetInitReport(void);
void Board_LateInit(void);

} /* namespace board */
#endif

#endif /* BOARD_BOARD_H_ */
