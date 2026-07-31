#ifndef BOARD_BOARD_I2C_BUS_H_
#define BOARD_BOARD_I2C_BUS_H_

#include <stdint.h>

#include "drivers/i2c_diag/i2c_diag.h"

namespace board {

uint32_t Board_I2cBusCount(void);
const drivers::I2cDiagBusConfig *Board_I2cBusGet(uint32_t index);
const drivers::I2cDiagBusConfig *Board_I2cBusFind(const char *name);

} /* namespace board */

#endif /* BOARD_BOARD_I2C_BUS_H_ */