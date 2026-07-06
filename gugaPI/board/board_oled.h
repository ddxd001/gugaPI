#ifndef BOARD_BOARD_OLED_H_
#define BOARD_BOARD_OLED_H_

#include <stdbool.h>
#include <stdint.h>

#include "drivers/common/driver_status.h"

namespace board {

drivers::DriverStatus Board_OledInit(void);
drivers::DriverStatus Board_OledProbe(void);
drivers::DriverStatus Board_OledClear(void);
drivers::DriverStatus Board_OledFill(uint8_t pattern);
drivers::DriverStatus Board_OledTestPattern(void);
drivers::DriverStatus Board_OledWriteText(uint8_t row,
                                          uint8_t col,
                                          const char *text);
drivers::DriverStatus Board_OledSetDisplayOn(bool on);
drivers::DriverStatus Board_OledSetInvert(bool invert);
drivers::DriverStatus Board_OledGetBusStatus(uint32_t *controller_status,
                                             bool *scl_high,
                                             bool *sda_high);
bool Board_OledIsReady(void);
uint8_t Board_OledAddress(void);
uint8_t Board_OledWidth(void);
uint8_t Board_OledHeight(void);

} /* namespace board */

#endif /* BOARD_BOARD_OLED_H_ */
