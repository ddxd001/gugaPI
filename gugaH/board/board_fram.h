#ifndef BOARD_BOARD_FRAM_H_
#define BOARD_BOARD_FRAM_H_

#include <stdbool.h>
#include <stdint.h>

#include "drivers/common/driver_status.h"

namespace board {

drivers::DriverStatus Board_FramInit(void);
drivers::DriverStatus Board_FramRead(uint16_t address,
                                     uint8_t *data,
                                     uint16_t length);
drivers::DriverStatus Board_FramWrite(uint16_t address,
                                      const uint8_t *data,
                                      uint16_t length);
drivers::DriverStatus Board_FramReadByte(uint16_t address, uint8_t *data);
drivers::DriverStatus Board_FramWriteByte(uint16_t address, uint8_t data);
drivers::DriverStatus Board_FramSelfTest(void);
drivers::DriverStatus Board_FramRecoverBus(void);
drivers::DriverStatus Board_FramGetBusStatus(uint32_t *controller_status,
                                             bool *scl_high,
                                             bool *sda_high);
bool Board_FramIsReady(void);
uint16_t Board_FramCapacityBytes(void);

} /* namespace board */

#endif /* BOARD_BOARD_FRAM_H_ */