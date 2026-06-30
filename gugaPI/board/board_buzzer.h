#ifndef BOARD_BOARD_BUZZER_H_
#define BOARD_BOARD_BUZZER_H_

#include <stdbool.h>

#include "drivers/common/driver_status.h"

namespace board {

drivers::DriverStatus Board_BuzzerInit(void);
drivers::DriverStatus Board_BuzzerOn(void);
drivers::DriverStatus Board_BuzzerOff(void);
drivers::DriverStatus Board_BuzzerToggle(void);
bool Board_BuzzerIsReady(void);
bool Board_BuzzerIsOn(void);

} /* namespace board */

#endif /* BOARD_BOARD_BUZZER_H_ */