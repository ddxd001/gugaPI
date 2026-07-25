#ifndef DRIVERS_I2C_DIAG_I2C_DIAG_H_
#define DRIVERS_I2C_DIAG_I2C_DIAG_H_

#include <stdbool.h>
#include <stdint.h>

#include "drivers/i2c_controller/i2c_controller.h"

namespace drivers {

static const uint8_t I2C_DIAG_MIN_7BIT_ADDRESS =
    I2C_CONTROLLER_MIN_7BIT_ADDRESS;
static const uint8_t I2C_DIAG_MAX_7BIT_ADDRESS =
    I2C_CONTROLLER_MAX_7BIT_ADDRESS;
static const uint16_t I2C_DIAG_MAX_READ_BYTES = 32U;
static const uint16_t I2C_DIAG_MAX_WRITE_BYTES = 7U;
static const uint16_t I2C_DIAG_MAX_BLOCK_WRITE_BYTES = 32U;

typedef I2cControllerConfig I2cDiagBusConfig;
typedef I2cControllerBusStatus I2cDiagBusStatus;

DriverStatus I2cDiag_GetBusStatus(const I2cDiagBusConfig *config,
                                  I2cDiagBusStatus *status);
DriverStatus I2cDiag_RecoverBus(const I2cDiagBusConfig *config);
DriverStatus I2cDiag_ProbeAddress(const I2cDiagBusConfig *config,
                                  uint8_t address);
DriverStatus I2cDiag_ReadReg8(const I2cDiagBusConfig *config,
                              uint8_t address,
                              uint8_t reg,
                              uint8_t *data,
                              uint16_t length);
DriverStatus I2cDiag_WriteReg8(const I2cDiagBusConfig *config,
                               uint8_t address,
                               uint8_t reg,
                               const uint8_t *data,
                               uint16_t length);
DriverStatus I2cDiag_WriteReg8Block(const I2cDiagBusConfig *config,
                                    uint8_t address,
                                    uint8_t reg,
                                    const uint8_t *data,
                                    uint16_t length);

} /* namespace drivers */

#endif /* DRIVERS_I2C_DIAG_I2C_DIAG_H_ */
