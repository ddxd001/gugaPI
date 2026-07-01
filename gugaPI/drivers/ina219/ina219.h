#ifndef DRIVERS_INA219_INA219_H_
#define DRIVERS_INA219_INA219_H_

#include <stdbool.h>
#include <stdint.h>

#include "drivers/common/driver_status.h"
#include "ti_msp_dl_config.h"

namespace drivers {

// INA219 默认地址：A0/A1 接地时为 0x40。
static const uint8_t INA219_DEFAULT_I2C_ADDRESS = 0x40U;
static const uint32_t INA219_DEFAULT_SHUNT_MILLIOHMS = 5U;
static const uint32_t INA219_DEFAULT_CURRENT_LSB_UA = 200U;
static const uint16_t INA219_DEFAULT_CONFIG = 0x399FU;

struct Ina219Config {
    I2C_Regs *i2c;
    uint8_t i2c_address;
    uint32_t timeout_iterations;
    uint32_t shunt_milliohms;
    uint32_t current_lsb_ua;
    // IN+/IN- 与期望电流方向相反时，软件取反电流和分流电压。
    bool invert_current;
    GPIO_Regs *scl_port;
    uint32_t scl_pin;
    uint32_t scl_iomux;
    uint32_t scl_iomux_func;
    GPIO_Regs *sda_port;
    uint32_t sda_pin;
    uint32_t sda_iomux;
    uint32_t sda_iomux_func;
};

struct Ina219Context {
    const Ina219Config *config;
    uint8_t i2c_address;
    bool initialized;
    uint16_t calibration;
};

struct Ina219BusStatus {
    uint32_t controller_status;
    bool scl_high;
    bool sda_high;
};

struct Ina219RawRegisters {
    uint16_t config;
    int16_t shunt_voltage;
    uint16_t bus_voltage;
    uint16_t power;
    int16_t current;
    uint16_t calibration;
};

struct Ina219Measurement {
    int32_t bus_voltage_mv;
    int32_t shunt_voltage_uv;
    int32_t current_ua;
    int32_t power_mw;
    bool conversion_ready;
    bool math_overflow;
};

DriverStatus Ina219_Init(Ina219Context *ctx, const Ina219Config *config);
DriverStatus Ina219_Reset(Ina219Context *ctx);
DriverStatus Ina219_ConfigureDefault(Ina219Context *ctx);
DriverStatus Ina219_SetAddress(Ina219Context *ctx, uint8_t address);
DriverStatus Ina219_ProbeAddress(Ina219Context *ctx,
                                 uint8_t address,
                                 uint16_t *config_value);
DriverStatus Ina219_ReadRegister(Ina219Context *ctx,
                                 uint8_t reg,
                                 uint16_t *value);
DriverStatus Ina219_WriteRegister(Ina219Context *ctx,
                                  uint8_t reg,
                                  uint16_t value);
DriverStatus Ina219_ReadRawRegisters(Ina219Context *ctx,
                                      Ina219RawRegisters *raw);
DriverStatus Ina219_ReadMeasurement(Ina219Context *ctx,
                                     Ina219Measurement *measurement);
DriverStatus Ina219_RecoverBus(Ina219Context *ctx);
DriverStatus Ina219_GetBusStatus(Ina219Context *ctx,
                                 Ina219BusStatus *status);
bool Ina219_IsReady(const Ina219Context *ctx);
uint16_t Ina219_GetCalibration(const Ina219Context *ctx);
uint8_t Ina219_GetAddress(const Ina219Context *ctx);

} /* namespace drivers */

#endif /* DRIVERS_INA219_INA219_H_ */