#ifndef DRIVERS_FM24CL64B_FM24CL64B_H_
#define DRIVERS_FM24CL64B_FM24CL64B_H_

#include <stdbool.h>
#include <stdint.h>

#include "drivers/common/driver_status.h"
#include "ti_msp_dl_config.h"

namespace drivers {

static const uint16_t FM24CL64B_SIZE_BYTES = 8192U;
static const uint8_t FM24CL64B_DEFAULT_I2C_ADDRESS = 0x50U;
static const uint8_t FM24CL64B_SELF_TEST_LENGTH = 8U;

struct Fm24cl64bConfig {
    I2C_Regs *i2c;
    uint8_t i2c_address;
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

struct Fm24cl64bContext {
    const Fm24cl64bConfig *config;
    bool initialized;
};

struct Fm24cl64bBusStatus {
    uint32_t controller_status;
    bool scl_high;
    bool sda_high;
};

DriverStatus Fm24cl64b_Init(Fm24cl64bContext *ctx,
                            const Fm24cl64bConfig *config);
DriverStatus Fm24cl64b_Read(Fm24cl64bContext *ctx,
                            uint16_t address,
                            uint8_t *data,
                            uint16_t length);
DriverStatus Fm24cl64b_Write(Fm24cl64bContext *ctx,
                             uint16_t address,
                             const uint8_t *data,
                             uint16_t length);
DriverStatus Fm24cl64b_ReadByte(Fm24cl64bContext *ctx,
                                uint16_t address,
                                uint8_t *data);
DriverStatus Fm24cl64b_WriteByte(Fm24cl64bContext *ctx,
                                 uint16_t address,
                                 uint8_t data);
DriverStatus Fm24cl64b_SelfTest(Fm24cl64bContext *ctx,
                                uint16_t test_address);
DriverStatus Fm24cl64b_RecoverBus(Fm24cl64bContext *ctx);
DriverStatus Fm24cl64b_GetBusStatus(Fm24cl64bContext *ctx,
                                    Fm24cl64bBusStatus *status);
bool Fm24cl64b_IsReady(const Fm24cl64bContext *ctx);

} /* namespace drivers */

#endif /* DRIVERS_FM24CL64B_FM24CL64B_H_ */