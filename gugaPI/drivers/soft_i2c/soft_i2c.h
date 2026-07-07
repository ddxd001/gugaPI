#ifndef DRIVERS_SOFT_I2C_SOFT_I2C_H_
#define DRIVERS_SOFT_I2C_SOFT_I2C_H_

#include <stdbool.h>
#include <stdint.h>

#include "drivers/common/driver_status.h"
#include "ti_msp_dl_config.h"

namespace drivers {

struct SoftI2cConfig {
    GPIO_Regs *scl_port;
    uint32_t scl_pin;
    uint32_t scl_iomux;
    GPIO_Regs *sda_port;
    uint32_t sda_pin;
    uint32_t sda_iomux;
    uint32_t half_period_cycles;
    uint32_t timeout_cycles;
};

struct SoftI2cContext {
    const SoftI2cConfig *config;
    bool initialized;
};

struct SoftI2cBusStatus {
    bool scl_high;
    bool sda_high;
};

DriverStatus SoftI2c_Init(SoftI2cContext *ctx, const SoftI2cConfig *config);
DriverStatus SoftI2c_GetBusStatus(const SoftI2cContext *ctx,
                                  SoftI2cBusStatus *status);
DriverStatus SoftI2c_RecoverBus(SoftI2cContext *ctx);
DriverStatus SoftI2c_ProbeAddress(SoftI2cContext *ctx, uint8_t address);
DriverStatus SoftI2c_ReadReg8(SoftI2cContext *ctx,
                              uint8_t address,
                              uint8_t reg,
                              uint8_t *data,
                              uint16_t length);
DriverStatus SoftI2c_WriteReg8(SoftI2cContext *ctx,
                               uint8_t address,
                               uint8_t reg,
                               const uint8_t *data,
                               uint16_t length);
bool SoftI2c_IsReady(const SoftI2cContext *ctx);

} /* namespace drivers */

#endif /* DRIVERS_SOFT_I2C_SOFT_I2C_H_ */
