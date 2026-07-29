#ifndef DRIVERS_I2C_CONTROLLER_I2C_CONTROLLER_H_
#define DRIVERS_I2C_CONTROLLER_I2C_CONTROLLER_H_

#include <stdbool.h>
#include <stdint.h>

#include "drivers/common/driver_status.h"
#include "ti_msp_dl_config.h"

namespace drivers {

static const uint8_t I2C_CONTROLLER_MIN_7BIT_ADDRESS = 0x08U;
static const uint8_t I2C_CONTROLLER_MAX_7BIT_ADDRESS = 0x77U;
static const uint16_t I2C_CONTROLLER_FIFO_BYTES = 8U;
static const uint16_t I2C_CONTROLLER_MAX_TRANSFER_BYTES = 0x0FFFU;

/*
 * target_address is always an unshifted 7-bit address. The controller owns
 * START, repeated START, final NACK and STOP generation for every operation.
 */
struct I2cControllerConfig {
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

struct I2cControllerBusStatus {
    uint32_t controller_status;
    bool scl_high;
    bool sda_high;
};

/*
 * One DMA channel is owned by one I2C controller while an asynchronous write
 * is active.  The channel must be configured by SysConfig as byte-to-fixed,
 * source increment enabled, destination increment disabled, and triggered by
 * the controller TX FIFO event. The caller must keep the data buffer unchanged
 * until I2cController_AsyncWritePoll returns a result other than BUSY.
 */
struct I2cControllerDmaTxConfig {
    DMA_Regs *dma;
    uint8_t channel_id;
    uint32_t timeout_ms;
};

bool I2cController_IsConfigValid(const I2cControllerConfig *config);
bool I2cController_IsAddressValid(uint8_t target_address);
DriverStatus I2cController_GetBusStatus(
    const I2cControllerConfig *config,
    I2cControllerBusStatus *status);
DriverStatus I2cController_RecoverBus(const I2cControllerConfig *config);
DriverStatus I2cController_Probe(const I2cControllerConfig *config,
                                 uint8_t target_address);
DriverStatus I2cController_Write(const I2cControllerConfig *config,
                                 uint8_t target_address,
                                 const uint8_t *prefix,
                                 uint16_t prefix_length,
                                 const uint8_t *data,
                                 uint16_t data_length);
DriverStatus I2cController_Read(const I2cControllerConfig *config,
                                uint8_t target_address,
                                uint8_t *data,
                                uint16_t length);
DriverStatus I2cController_WriteRead(const I2cControllerConfig *config,
                                     uint8_t target_address,
                                     const uint8_t *write_data,
                                     uint16_t write_length,
                                     uint8_t *read_data,
                                     uint16_t read_length);
DriverStatus I2cController_AsyncWriteStart(
    const I2cControllerConfig *config,
    const I2cControllerDmaTxConfig *dma_config,
    uint8_t target_address,
    const uint8_t *data,
    uint16_t length);
DriverStatus I2cController_AsyncWritePoll(
    const I2cControllerConfig *config,
    const I2cControllerDmaTxConfig *dma_config);
void I2cController_AsyncWriteHandleInterrupt(
    const I2cControllerConfig *config);
void I2cController_AsyncWriteHandleDmaFault(
    const I2cControllerConfig *config,
    const I2cControllerDmaTxConfig *dma_config);
bool I2cController_IsBusBusy(const I2cControllerConfig *config);

} /* namespace drivers */

#endif /* DRIVERS_I2C_CONTROLLER_I2C_CONTROLLER_H_ */

