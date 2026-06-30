#ifndef BOARD_BOARD_BUTTON_H_
#define BOARD_BOARD_BUTTON_H_

#include <stdbool.h>
#include <stdint.h>

#include "drivers/common/driver_status.h"

namespace board {

enum BoardButtonId {
    BOARD_BUTTON_1 = 0,
    BOARD_BUTTON_2,
    BOARD_BUTTON_3,
    BOARD_BUTTON_COUNT
};

drivers::DriverStatus Board_ButtonsInit(void);
drivers::DriverStatus Board_ButtonsUpdate(uint32_t now_ms);
drivers::DriverStatus Board_ButtonReadRaw(BoardButtonId id, bool *pressed);
const char *Board_ButtonGetName(BoardButtonId id);
bool Board_ButtonIsReady(BoardButtonId id);
bool Board_ButtonIsPressed(BoardButtonId id);
bool Board_ButtonWasPressed(BoardButtonId id);
bool Board_ButtonWasReleased(BoardButtonId id);

} /* namespace board */

#endif /* BOARD_BOARD_BUTTON_H_ */
