#ifndef BOARD_BOARD_H_
#define BOARD_BOARD_H_

#include "drivers/common/driver_status.h"

#ifdef __cplusplus
namespace board {

/* Runs every FEATURE_ENABLE_* board-level driver init. Returns the first
 * non-OK status (or DRIVER_OK). Board_Init cannot log itself (board is below
 * services/log), so the caller reads Board_GetFailedDriver and logs it. */
drivers::DriverStatus Board_Init(void);
/* Name of the first driver whose init failed, or nullptr if all succeeded. */
const char *Board_GetFailedDriver(void);
void Board_LateInit(void);

} /* namespace board */
#endif

#endif /* BOARD_BOARD_H_ */