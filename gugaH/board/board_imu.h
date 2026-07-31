#ifndef GUGAH_BOARD_BOARD_IMU_H_
#define GUGAH_BOARD_BOARD_IMU_H_

#include <stdbool.h>

#include "drivers/icm45686/icm45686.h"

namespace board {

drivers::DriverStatus Board_ImuInit(void);
bool Board_ImuIsReady(void);
drivers::DriverStatus Board_ImuRead(drivers::Icm45686SensorData *data);

} /* namespace board */

#endif /* GUGAH_BOARD_BOARD_IMU_H_ */
