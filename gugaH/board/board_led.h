#ifndef BOARD_BOARD_LED_H_
#define BOARD_BOARD_LED_H_

#include <stdbool.h>
#include <stdint.h>

#include "drivers/common/driver_status.h"

namespace board {

enum BoardLedId : uint8_t {
    BOARD_LED_ID_1 = 0U,
    BOARD_LED_ID_2 = 1U,
    BOARD_LED_ID_3 = 2U,
    BOARD_LED_ID_COUNT = 3U
};

drivers::DriverStatus Board_LedsInit(void);
drivers::DriverStatus Board_LedSet(BoardLedId id, bool on);
drivers::DriverStatus Board_LedOn(BoardLedId id);
drivers::DriverStatus Board_LedOff(BoardLedId id);
drivers::DriverStatus Board_LedToggle(BoardLedId id);
bool Board_LedIsReady(BoardLedId id);
bool Board_LedIsOn(BoardLedId id);
uint8_t Board_LedCount(void);

drivers::DriverStatus Board_StatusLedInit(void);
drivers::DriverStatus Board_StatusLedOn(void);
drivers::DriverStatus Board_StatusLedOff(void);
drivers::DriverStatus Board_StatusLedToggle(void);
bool Board_StatusLedIsReady(void);
bool Board_StatusLedIsOn(void);

} /* namespace board */

#endif /* BOARD_BOARD_LED_H_ */
