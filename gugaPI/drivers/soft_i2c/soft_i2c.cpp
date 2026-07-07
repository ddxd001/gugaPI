#include "drivers/soft_i2c/soft_i2c.h"

namespace drivers {
namespace {

static const uint8_t kBusRecoveryClockPulses = 9U;

bool IsAddressValid(uint8_t address)
{
    return (address >= 0x08U) && (address <= 0x77U);
}

bool IsConfigValid(const SoftI2cConfig *config)
{
    return (config != 0) &&
           (config->scl_port != 0) && (config->scl_pin != 0U) &&
           (config->sda_port != 0) && (config->sda_pin != 0U) &&
           (config->half_period_cycles > 0U) &&
           (config->timeout_cycles > 0U);
}

bool IsContextReady(const SoftI2cContext *ctx)
{
    return (ctx != 0) && ctx->initialized && IsConfigValid(ctx->config);
}

void DelayHalfPeriod(const SoftI2cConfig *config)
{
    delay_cycles(config->half_period_cycles);
}

bool ReadLine(GPIO_Regs *port, uint32_t pin)
{
    return ((DL_GPIO_readPins(port, pin) & pin) != 0U);
}

bool ReadScl(const SoftI2cConfig *config)
{
    return ReadLine(config->scl_port, config->scl_pin);
}

bool ReadSda(const SoftI2cConfig *config)
{
    return ReadLine(config->sda_port, config->sda_pin);
}

void DriveLow(GPIO_Regs *port, uint32_t pin)
{
    DL_GPIO_clearPins(port, pin);
    DL_GPIO_enableOutput(port, pin);
}

void ReleaseLine(GPIO_Regs *port, uint32_t pin)
{
    DL_GPIO_disableOutput(port, pin);
}

void ReleaseScl(const SoftI2cConfig *config)
{
    ReleaseLine(config->scl_port, config->scl_pin);
}

void ReleaseSda(const SoftI2cConfig *config)
{
    ReleaseLine(config->sda_port, config->sda_pin);
}

void DriveSclLow(const SoftI2cConfig *config)
{
    DriveLow(config->scl_port, config->scl_pin);
}

void DriveSdaLow(const SoftI2cConfig *config)
{
    DriveLow(config->sda_port, config->sda_pin);
}

DriverStatus WaitSclHigh(const SoftI2cConfig *config)
{
    uint32_t timeout = config->timeout_cycles;

    while (!ReadScl(config)) {
        if (timeout == 0U) {
            return DRIVER_ERROR_TIMEOUT;
        }
        timeout--;
    }

    return DRIVER_OK;
}

void ConfigureInputs(const SoftI2cConfig *config)
{
    DL_GPIO_disableOutput(config->scl_port, config->scl_pin);
    DL_GPIO_disableOutput(config->sda_port, config->sda_pin);
    DL_GPIO_initDigitalInputFeatures(config->scl_iomux,
                                     DL_GPIO_INVERSION_DISABLE,
                                     DL_GPIO_RESISTOR_PULL_UP,
                                     DL_GPIO_HYSTERESIS_DISABLE,
                                     DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_initDigitalInputFeatures(config->sda_iomux,
                                     DL_GPIO_INVERSION_DISABLE,
                                     DL_GPIO_RESISTOR_PULL_UP,
                                     DL_GPIO_HYSTERESIS_DISABLE,
                                     DL_GPIO_WAKEUP_DISABLE);
}

DriverStatus StartCondition(const SoftI2cConfig *config)
{
    ReleaseSda(config);
    ReleaseScl(config);
    DriverStatus status = WaitSclHigh(config);
    if (status != DRIVER_OK) {
        return status;
    }
    DelayHalfPeriod(config);

    DriveSdaLow(config);
    DelayHalfPeriod(config);
    DriveSclLow(config);
    DelayHalfPeriod(config);

    return DRIVER_OK;
}

DriverStatus StopCondition(const SoftI2cConfig *config)
{
    DriveSdaLow(config);
    DelayHalfPeriod(config);
    ReleaseScl(config);
    DriverStatus status = WaitSclHigh(config);
    if (status != DRIVER_OK) {
        return status;
    }
    DelayHalfPeriod(config);
    ReleaseSda(config);
    DelayHalfPeriod(config);

    return DRIVER_OK;
}

DriverStatus WriteBit(const SoftI2cConfig *config, bool bit)
{
    if (bit) {
        ReleaseSda(config);
    } else {
        DriveSdaLow(config);
    }
    DelayHalfPeriod(config);

    ReleaseScl(config);
    DriverStatus status = WaitSclHigh(config);
    if (status != DRIVER_OK) {
        return status;
    }
    DelayHalfPeriod(config);

    DriveSclLow(config);
    DelayHalfPeriod(config);

    return DRIVER_OK;
}

DriverStatus ReadBit(const SoftI2cConfig *config, bool *bit)
{
    if (bit == 0) {
        return DRIVER_ERROR_INVALID_ARG;
    }

    ReleaseSda(config);
    DelayHalfPeriod(config);
    ReleaseScl(config);
    DriverStatus status = WaitSclHigh(config);
    if (status != DRIVER_OK) {
        return status;
    }
    DelayHalfPeriod(config);

    *bit = ReadSda(config);

    DriveSclLow(config);
    DelayHalfPeriod(config);

    return DRIVER_OK;
}

DriverStatus WriteByte(const SoftI2cConfig *config, uint8_t value)
{
    for (uint8_t mask = 0x80U; mask != 0U; mask >>= 1U) {
        const DriverStatus status = WriteBit(config, (value & mask) != 0U);
        if (status != DRIVER_OK) {
            return status;
        }
    }

    bool nack = true;
    DriverStatus status = ReadBit(config, &nack);
    if (status != DRIVER_OK) {
        return status;
    }

    return nack ? DRIVER_ERROR : DRIVER_OK;
}

DriverStatus ReadByte(const SoftI2cConfig *config, uint8_t *value, bool ack)
{
    if (value == 0) {
        return DRIVER_ERROR_INVALID_ARG;
    }

    uint8_t result = 0U;
    for (uint8_t i = 0U; i < 8U; i++) {
        bool bit = false;
        const DriverStatus status = ReadBit(config, &bit);
        if (status != DRIVER_OK) {
            return status;
        }
        result = static_cast<uint8_t>((result << 1U) | (bit ? 1U : 0U));
    }

    *value = result;
    return WriteBit(config, !ack);
}

DriverStatus BeginWrite(const SoftI2cConfig *config, uint8_t address)
{
    DriverStatus status = StartCondition(config);
    if (status != DRIVER_OK) {
        return status;
    }

    status = WriteByte(config, static_cast<uint8_t>(address << 1U));
    if (status != DRIVER_OK) {
        (void) StopCondition(config);
    }

    return status;
}

} /* namespace */

DriverStatus SoftI2c_Init(SoftI2cContext *ctx, const SoftI2cConfig *config)
{
    if ((ctx == 0) || (!IsConfigValid(config))) {
        return DRIVER_ERROR_INVALID_ARG;
    }

    ctx->config = config;
    ctx->initialized = false;
    ConfigureInputs(config);
    ReleaseScl(config);
    ReleaseSda(config);
    DelayHalfPeriod(config);

    ctx->initialized = true;
    return DRIVER_OK;
}

DriverStatus SoftI2c_GetBusStatus(const SoftI2cContext *ctx,
                                  SoftI2cBusStatus *status)
{
    if ((!IsContextReady(ctx)) || (status == 0)) {
        return DRIVER_ERROR_INVALID_ARG;
    }

    status->scl_high = ReadScl(ctx->config);
    status->sda_high = ReadSda(ctx->config);
    return DRIVER_OK;
}

DriverStatus SoftI2c_RecoverBus(SoftI2cContext *ctx)
{
    if (!IsContextReady(ctx)) {
        return DRIVER_ERROR_NOT_INITIALIZED;
    }

    const SoftI2cConfig *config = ctx->config;
    ConfigureInputs(config);
    ReleaseScl(config);
    ReleaseSda(config);
    DelayHalfPeriod(config);

    for (uint8_t i = 0U; i < kBusRecoveryClockPulses; i++) {
        if (ReadScl(config) && ReadSda(config)) {
            return DRIVER_OK;
        }

        DriveSclLow(config);
        DelayHalfPeriod(config);
        ReleaseScl(config);
        (void) WaitSclHigh(config);
        DelayHalfPeriod(config);
    }

    (void) StopCondition(config);
    return (ReadScl(config) && ReadSda(config)) ? DRIVER_OK :
                                                  DRIVER_ERROR_BUSY;
}

DriverStatus SoftI2c_ProbeAddress(SoftI2cContext *ctx, uint8_t address)
{
    if ((!IsContextReady(ctx)) || (!IsAddressValid(address))) {
        return DRIVER_ERROR_INVALID_ARG;
    }

    DriverStatus status = BeginWrite(ctx->config, address);
    if (status == DRIVER_OK) {
        status = StopCondition(ctx->config);
    }

    return status;
}

DriverStatus SoftI2c_ReadReg8(SoftI2cContext *ctx,
                              uint8_t address,
                              uint8_t reg,
                              uint8_t *data,
                              uint16_t length)
{
    if ((!IsContextReady(ctx)) || (!IsAddressValid(address)) ||
        (data == 0) || (length == 0U)) {
        return DRIVER_ERROR_INVALID_ARG;
    }

    const SoftI2cConfig *config = ctx->config;
    DriverStatus status = BeginWrite(config, address);
    if (status != DRIVER_OK) {
        return status;
    }

    status = WriteByte(config, reg);
    if (status != DRIVER_OK) {
        (void) StopCondition(config);
        return status;
    }

    status = StartCondition(config);
    if (status != DRIVER_OK) {
        (void) StopCondition(config);
        return status;
    }

    status = WriteByte(config, static_cast<uint8_t>((address << 1U) | 1U));
    if (status != DRIVER_OK) {
        (void) StopCondition(config);
        return status;
    }

    for (uint16_t i = 0U; i < length; i++) {
        const bool ack = (i + 1U) < length;
        status = ReadByte(config, &data[i], ack);
        if (status != DRIVER_OK) {
            (void) StopCondition(config);
            return status;
        }
    }

    return StopCondition(config);
}

DriverStatus SoftI2c_WriteReg8(SoftI2cContext *ctx,
                               uint8_t address,
                               uint8_t reg,
                               const uint8_t *data,
                               uint16_t length)
{
    if ((!IsContextReady(ctx)) || (!IsAddressValid(address)) ||
        (data == 0) || (length == 0U)) {
        return DRIVER_ERROR_INVALID_ARG;
    }

    const SoftI2cConfig *config = ctx->config;
    DriverStatus status = BeginWrite(config, address);
    if (status != DRIVER_OK) {
        return status;
    }

    status = WriteByte(config, reg);
    if (status != DRIVER_OK) {
        (void) StopCondition(config);
        return status;
    }

    for (uint16_t i = 0U; i < length; i++) {
        status = WriteByte(config, data[i]);
        if (status != DRIVER_OK) {
            (void) StopCondition(config);
            return status;
        }
    }

    return StopCondition(config);
}

bool SoftI2c_IsReady(const SoftI2cContext *ctx)
{
    return IsContextReady(ctx);
}

} /* namespace drivers */
