#ifndef BOARD_BOARD_LED_H_
#define BOARD_BOARD_LED_H_

#include <stdbool.h>

#include "drivers/common/driver_status.h"

namespace board {

drivers::DriverStatus Board_StatusLedInit(void);
drivers::DriverStatus Board_StatusLedOn(void);
drivers::DriverStatus Board_StatusLedOff(void);
drivers::DriverStatus Board_StatusLedToggle(void);
bool Board_StatusLedIsReady(void);
bool Board_StatusLedIsOn(void);

} /* namespace board */

#endif /* BOARD_BOARD_LED_H_ */