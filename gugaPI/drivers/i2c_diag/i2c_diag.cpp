#include "drivers/i2c_diag/i2c_diag.h"

namespace drivers {

DriverStatus I2cDiag_GetBusStatus(const I2cDiagBusConfig *config,
                                  I2cDiagBusStatus *status)
{
    return I2cController_GetBusStatus(config, status);
}

DriverStatus I2cDiag_RecoverBus(const I2cDiagBusConfig *config)
{
    return I2cController_RecoverBus(config);
}

DriverStatus I2cDiag_ProbeAddress(const I2cDiagBusConfig *config,
                                  uint8_t address)
{
    return I2cController_Probe(config, address);
}

DriverStatus I2cDiag_ReadReg8(const I2cDiagBusConfig *config,
                              uint8_t address,
                              uint8_t reg,
                              uint8_t *data,
                              uint16_t length)
{
    if ((data == 0) || (length == 0U) ||
        (length > I2C_DIAG_MAX_READ_BYTES)) {
        return DRIVER_ERROR_INVALID_ARG;
    }

    return I2cController_WriteRead(config,
                                   address,
                                   &reg,
                                   1U,
                                   data,
                                   length);
}

DriverStatus I2cDiag_WriteReg8(const I2cDiagBusConfig *config,
                               uint8_t address,
                               uint8_t reg,
                               const uint8_t *data,
                               uint16_t length)
{
    if ((data == 0) || (length == 0U) ||
        (length > I2C_DIAG_MAX_WRITE_BYTES)) {
        return DRIVER_ERROR_INVALID_ARG;
    }

    return I2cController_Write(config, address, &reg, 1U, data, length);
}

DriverStatus I2cDiag_WriteReg8Block(const I2cDiagBusConfig *config,
                                    uint8_t address,
                                    uint8_t reg,
                                    const uint8_t *data,
                                    uint16_t length)
{
    if ((data == 0) || (length == 0U) ||
        (length > I2C_DIAG_MAX_BLOCK_WRITE_BYTES)) {
        return DRIVER_ERROR_INVALID_ARG;
    }

    return I2cController_Write(config, address, &reg, 1U, data, length);
}

} /* namespace drivers */
