#ifndef BOARD_BOARD_MOTOR_DRIVER_H_
#define BOARD_BOARD_MOTOR_DRIVER_H_

#include <stdint.h>

#include "drivers/common/driver_status.h"

namespace board {

drivers::DriverStatus Board_MotorDriverInit(void);
drivers::DriverStatus Board_MotorDriverWriteByte(uint8_t data);
drivers::DriverStatus Board_MotorDriverWrite(const uint8_t *data,
                                             uint16_t length);
bool Board_MotorDriverReadByte(uint8_t *data);
uint16_t Board_MotorDriverGetRxAvailable(void);
uint32_t Board_MotorDriverGetRxDroppedCount(void);
uint32_t Board_MotorDriverGetBaudrate(void);
drivers::DriverStatus Board_MotorDriverGetControllerStatus(uint32_t *status);
drivers::DriverStatus Board_MotorDriverClearRxBuffer(void);
bool Board_MotorDriverIsReady(void);
void Board_MotorDriverIrqHandler(void);

} /* namespace board */

#endif /* BOARD_BOARD_MOTOR_DRIVER_H_ */
