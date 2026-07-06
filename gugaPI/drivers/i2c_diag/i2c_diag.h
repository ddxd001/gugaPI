#ifndef DRIVERS_I2C_DIAG_I2C_DIAG_H_
#define DRIVERS_I2C_DIAG_I2C_DIAG_H_

#include <stdbool.h>
#include <stdint.h>

#include "drivers/common/driver_status.h"
#include "ti_msp_dl_config.h"

namespace drivers {

static const uint8_t I2C_DIAG_MIN_7BIT_ADDRESS = 0x08U;
static const uint8_t I2C_DIAG_MAX_7BIT_ADDRESS = 0x77U;
static const uint16_t I2C_DIAG_MAX_READ_BYTES = 32U;
static const uint16_t I2C_DIAG_MAX_WRITE_BYTES = 7U;
static const uint16_t I2C_DIAG_MAX_BLOCK_WRITE_BYTES = 32U;

struct I2cDiagBusConfig {
    const char *name;
    I2C_Regs *i2c;
    uint32_t timeout_iterations;
    GPIO_Regs *scl_port;
    uint32_t scl_pin;
    uint32_t scl_iomux;
    uint32_t scl_iomux_func;
    GPIO_Regs *sda_port;
    uint32_t sda_pin;
    uint32_t sda_iomux;
    uint32_t sda_iomux_func;
};

struct I2cDiagBusStatus {
    uint32_t controller_status;
    bool scl_high;
    bool sda_high;
};

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
