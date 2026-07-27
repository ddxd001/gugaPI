#ifndef BOARD_BOARD_CAN_H_
#define BOARD_BOARD_CAN_H_

#include <stdbool.h>
#include <stdint.h>

#include "drivers/can/can.h"
#include "drivers/common/driver_status.h"

namespace board {

drivers::DriverStatus Board_CanInit(void);
bool Board_CanIsReady(void);
drivers::DriverStatus Board_CanSetMode(
    drivers::CanTransceiverMode mode);
drivers::DriverStatus Board_CanSend(const drivers::CanFrame *frame);
bool Board_CanRead(drivers::CanFrame *frame);
uint16_t Board_CanRxAvailable(void);
drivers::DriverStatus Board_CanGetStatus(drivers::CanStatus *status);
drivers::DriverStatus Board_CanClear(void);
drivers::DriverStatus Board_CanCancelTx(void);
drivers::DriverStatus Board_CanRecover(void);

} /* namespace board */

#endif /* BOARD_BOARD_CAN_H_ */
