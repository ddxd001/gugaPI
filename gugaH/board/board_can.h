#ifndef GUGAH_BOARD_BOARD_CAN_H_
#define GUGAH_BOARD_BOARD_CAN_H_

#include <stdbool.h>
#include <stdint.h>

#include "drivers/can/can.h"

namespace board {

drivers::DriverStatus Board_CanInit(void);
bool Board_CanIsReady(void);
drivers::DriverStatus Board_CanSend(const drivers::CanFrame *frame);
bool Board_CanRead(drivers::CanFrame *frame);
drivers::DriverStatus Board_CanGetStatus(drivers::CanStatus *status);
drivers::DriverStatus Board_CanRecover(void);
void Board_CanIrqHandler(void);

} /* namespace board */

#endif /* GUGAH_BOARD_BOARD_CAN_H_ */
