#include "drivers/ina219/ina219.h"

namespace drivers {
namespace {

static const uint8_t kRegisterConfig = 0x00U;
static const uint8_t kRegisterShuntVoltage = 0x01U;
static const uint8_t kRegisterBusVoltage = 0x02U;
static const uint8_t kRegisterPower = 0x03U;
static const uint8_t kRegisterCurrent = 0x04U;
static const uint8_t kRegisterCalibration = 0x05U;
static const uint16_t kConfigReset = 0x8000U;
// INA219 校准公式：CAL = 0.04096 / (current_LSB * Rshunt)，这里用 uA/mΩ 避免浮点。
static const uint32_t kCalibrationNumerator = 40960000U;

bool IsConfigValid(const Ina219Config *config)
{
    return (config != 0) &&
           I2cController_IsConfigValid(config->bus) &&
           I2cController_IsAddressValid(config->i2c_address);
}

DriverStatus CheckContext(Ina219Context *ctx)
{
    if ((ctx == 0) || (!ctx->initialized) ||
        (!IsConfigValid(ctx->config))) {
        return DRIVER_ERROR_NOT_INITIALIZED;
    }

    return DRIVER_OK;
}

// 根据采样电阻和电流 LSB 计算 INA219 calibration 寄存器。
bool ComputeCalibration(const Ina219Config *config, uint16_t *calibration)
{
    if ((config == 0) || (calibration == 0) ||
        (config->shunt_milliohms == 0U) ||
        (config->current_lsb_ua == 0U)) {
        return false;
    }

    const uint32_t denominator =
        config->shunt_milliohms * config->current_lsb_ua;
    const uint32_t value = kCalibrationNumerator / denominator;

    if ((value == 0U) || (value > 0xFFFFU)) {
        return false;
    }

    *calibration = (uint16_t) value;
    return true;
}

DriverStatus WriteRegisterInternal(Ina219Context *ctx,
                                   uint8_t reg,
                                   uint16_t value)
{
    const uint8_t data[2] = {
        static_cast<uint8_t>((value >> 8U) & 0xFFU),
        static_cast<uint8_t>(value & 0xFFU)
    };
    return I2cController_Write(ctx->config->bus,
                               ctx->i2c_address,
                               &reg,
                               1U,
                               data,
                               sizeof(data));
}

// 寄存器读流程：写入 8 位寄存器地址，再 repeated-start 读取 16 位大端数据。
DriverStatus ReadRegisterInternal(Ina219Context *ctx,
                                  uint8_t reg,
                                  uint16_t *value)
{
    uint8_t rx[2] = { 0U, 0U };
    const DriverStatus status = I2cController_WriteRead(ctx->config->bus,
                                                        ctx->i2c_address,
                                                        &reg,
                                                        1U,
                                                        rx,
                                                        sizeof(rx));
    if (status != DRIVER_OK) {
        return status;
    }

    *value = ((uint16_t) rx[0] << 8U) | rx[1];
    return DRIVER_OK;
}

} /* namespace */

DriverStatus Ina219_Init(Ina219Context *ctx, const Ina219Config *config)
{
    uint16_t calibration = 0U;

    if ((ctx == 0) || (!IsConfigValid(config)) ||
        (!ComputeCalibration(config, &calibration))) {
        return DRIVER_ERROR_INVALID_ARG;
    }

    ctx->initialized = false;
    ctx->config = config;
    ctx->i2c_address = config->i2c_address;
    ctx->calibration = calibration;
    ctx->initialized = true;

    const DriverStatus status = Ina219_ConfigureDefault(ctx);
    if (status != DRIVER_OK) {
        ctx->initialized = false;
    }
    return status;
}

DriverStatus Ina219_Reset(Ina219Context *ctx)
{
    const DriverStatus status = CheckContext(ctx);

    if (status != DRIVER_OK) {
        return status;
    }

    return WriteRegisterInternal(ctx, kRegisterConfig, kConfigReset);
}

DriverStatus Ina219_ConfigureDefault(Ina219Context *ctx)
{
    DriverStatus status = CheckContext(ctx);

    if (status != DRIVER_OK) {
        return status;
    }

    status = WriteRegisterInternal(ctx, kRegisterConfig, INA219_DEFAULT_CONFIG);
    if (status != DRIVER_OK) {
        return status;
    }

    return WriteRegisterInternal(ctx, kRegisterCalibration, ctx->calibration);
}


DriverStatus Ina219_SetAddress(Ina219Context *ctx, uint8_t address)
{
    DriverStatus status = CheckContext(ctx);

    if (status != DRIVER_OK) {
        return status;
    }

    if (!I2cController_IsAddressValid(address)) {
        return DRIVER_ERROR_INVALID_ARG;
    }

    const uint8_t previous_address = ctx->i2c_address;
    ctx->i2c_address = address;

    status = Ina219_ConfigureDefault(ctx);
    if (status != DRIVER_OK) {
        ctx->i2c_address = previous_address;
    }

    return status;
}

DriverStatus Ina219_ProbeAddress(Ina219Context *ctx,
                                 uint8_t address,
                                 uint16_t *config_value)
{
    DriverStatus status = CheckContext(ctx);

    if (status != DRIVER_OK) {
        return status;
    }

    if ((!I2cController_IsAddressValid(address)) || (config_value == 0)) {
        return DRIVER_ERROR_INVALID_ARG;
    }

    const uint8_t previous_address = ctx->i2c_address;
    ctx->i2c_address = address;
    status = ReadRegisterInternal(ctx, kRegisterConfig, config_value);
    ctx->i2c_address = previous_address;

    return status;
}

DriverStatus Ina219_ReadRegister(Ina219Context *ctx,
                                 uint8_t reg,
                                 uint16_t *value)
{
    DriverStatus status = CheckContext(ctx);

    if (status != DRIVER_OK) {
        return status;
    }

    if ((value == 0) || (reg > kRegisterCalibration)) {
        return DRIVER_ERROR_INVALID_ARG;
    }

    return ReadRegisterInternal(ctx, reg, value);
}

DriverStatus Ina219_WriteRegister(Ina219Context *ctx,
                                  uint8_t reg,
                                  uint16_t value)
{
    DriverStatus status = CheckContext(ctx);

    if (status != DRIVER_OK) {
        return status;
    }

    if (reg > kRegisterCalibration) {
        return DRIVER_ERROR_INVALID_ARG;
    }

    return WriteRegisterInternal(ctx, reg, value);
}

DriverStatus Ina219_ReadRawRegisters(Ina219Context *ctx,
                                      Ina219RawRegisters *raw)
{
    uint16_t value = 0U;
    DriverStatus status = CheckContext(ctx);

    if (status != DRIVER_OK) {
        return status;
    }

    if (raw == 0) {
        return DRIVER_ERROR_INVALID_ARG;
    }

    status = ReadRegisterInternal(ctx, kRegisterConfig, &raw->config);
    if (status != DRIVER_OK) {
        return status;
    }

    status = ReadRegisterInternal(ctx, kRegisterShuntVoltage, &value);
    if (status != DRIVER_OK) {
        return status;
    }
    raw->shunt_voltage = (int16_t) value;

    status = ReadRegisterInternal(ctx, kRegisterBusVoltage, &raw->bus_voltage);
    if (status != DRIVER_OK) {
        return status;
    }

    status = ReadRegisterInternal(ctx, kRegisterPower, &raw->power);
    if (status != DRIVER_OK) {
        return status;
    }

    status = ReadRegisterInternal(ctx, kRegisterCurrent, &value);
    if (status != DRIVER_OK) {
        return status;
    }
    raw->current = (int16_t) value;

    return ReadRegisterInternal(ctx, kRegisterCalibration, &raw->calibration);
}

DriverStatus Ina219_ReadMeasurement(Ina219Context *ctx,
                                     Ina219Measurement *measurement)
{
    Ina219RawRegisters raw;

    if (measurement == 0) {
        return DRIVER_ERROR_INVALID_ARG;
    }

    const DriverStatus status = Ina219_ReadRawRegisters(ctx, &raw);

    if (status != DRIVER_OK) {
        return status;
    }

    measurement->bus_voltage_mv =
        (int32_t) (((raw.bus_voltage >> 3U) & 0x1FFFU) * 4U);
    measurement->shunt_voltage_uv = (int32_t) raw.shunt_voltage * 10;
    measurement->current_ua =
        (int32_t) raw.current * (int32_t) ctx->config->current_lsb_ua;
    if (ctx->config->invert_current) {
        measurement->shunt_voltage_uv = -measurement->shunt_voltage_uv;
        measurement->current_ua = -measurement->current_ua;
    }
    measurement->power_mw =
        (int32_t) ((uint32_t) raw.power * ctx->config->current_lsb_ua * 20U /
                   1000U);
    measurement->conversion_ready = ((raw.bus_voltage & 0x0002U) != 0U);
    measurement->math_overflow = ((raw.bus_voltage & 0x0001U) != 0U);

    return DRIVER_OK;
}

DriverStatus Ina219_RecoverBus(Ina219Context *ctx)
{
    const DriverStatus status = CheckContext(ctx);
    return (status == DRIVER_OK) ?
        I2cController_RecoverBus(ctx->config->bus) : status;
}

DriverStatus Ina219_GetBusStatus(Ina219Context *ctx,
                                 Ina219BusStatus *status)
{
    const DriverStatus context_status = CheckContext(ctx);

    if (context_status != DRIVER_OK) {
        return context_status;
    }

    return I2cController_GetBusStatus(ctx->config->bus, status);
}

bool Ina219_IsReady(const Ina219Context *ctx)
{
    return (ctx != 0) && ctx->initialized && IsConfigValid(ctx->config);
}

uint16_t Ina219_GetCalibration(const Ina219Context *ctx)
{
    if (!Ina219_IsReady(ctx)) {
        return 0U;
    }

    return ctx->calibration;
}


uint8_t Ina219_GetAddress(const Ina219Context *ctx)
{
    if (!Ina219_IsReady(ctx)) {
        return 0U;
    }

    return ctx->i2c_address;
}
} /* namespace drivers */
