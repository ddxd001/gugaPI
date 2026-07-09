#ifndef BOARD_BOARD_GRAYSCALE_H_
#define BOARD_BOARD_GRAYSCALE_H_

#include <stdbool.h>
#include <stdint.h>

#include "drivers/common/driver_status.h"

namespace board {

drivers::DriverStatus Board_GrayscaleInit(void);
bool Board_GrayscaleIsReady(void);
drivers::DriverStatus Board_GrayscaleReadChannel(uint8_t channel, uint16_t *raw);
drivers::DriverStatus Board_GrayscaleReadAll(uint16_t out[8]);

} /* namespace board */

#endif /* BOARD_BOARD_GRAYSCALE_H_ */
