#include "drivers/fm24cl64b/fm24cl64b.h"

namespace drivers {
namespace {

static const uint16_t kAddressBytes = 2U;
static const uint16_t kControllerFifoBytes = 8U;
static const uint16_t kMaxWritePayloadBytes =
    kControllerFifoBytes - kAddressBytes;
static const uint16_t kMaxReadPayloadBytes = 32U;
static const uint32_t kI2cErrataDelayCycles = 100U;
static const uint32_t kBusRecoveryDelayCycles = 320U;
static const uint8_t kBusRecoveryClockPulses = 9U;

bool IsRangeValid(uint16_t address, uint16_t length)
{
    if (length == 0U) {
        return true;
    }

    if (address >= FM24CL64B_SIZE_BYTES) {
        return false;
    }

    return (length <= (FM24CL64B_SIZE_BYTES - address));
}

DriverStatus CheckContext(Fm24cl64bContext *ctx)
{
    if ((ctx == 0) || (!ctx->initialized) || (ctx->config == 0) ||
        (ctx->config->i2c == 0)) {
        return DRIVER_ERROR_NOT_INITIALIZED;
    }

    return DRIVER_OK;
}

void ResetTransfer(I2C_Regs *i2c)
{
    DL_I2C_resetControllerTransfer(i2c);
    DL_I2C_disableControllerReadOnTXEmpty(i2c);
    DL_I2C_flushControllerTXFIFO(i2c);
    DL_I2C_flushControllerRXFIFO(i2c);
}

DriverStatus CheckTransferError(I2C_Regs *i2c)
{
    if ((DL_I2C_getControllerStatus(i2c) &
         DL_I2C_CONTROLLER_STATUS_ERROR) != 0U) {
        ResetTransfer(i2c);
        return DRIVER_ERROR;
    }

    return DRIVER_OK;
}

DriverStatus WaitForIdle(I2C_Regs *i2c, uint32_t timeout)
{
    while ((DL_I2C_getControllerStatus(i2c) &
            DL_I2C_CONTROLLER_STATUS_IDLE) == 0U) {
        if (timeout == 0U) {
            ResetTransfer(i2c);
            return DRIVER_ERROR_TIMEOUT;
        }
        timeout--;
    }

    return CheckTransferError(i2c);
}

DriverStatus WaitWhileBusy(I2C_Regs *i2c, uint32_t timeout)
{
    while ((DL_I2C_getControllerStatus(i2c) &
            DL_I2C_CONTROLLER_STATUS_BUSY) != 0U) {
        if (timeout == 0U) {
            ResetTransfer(i2c);
            return DRIVER_ERROR_TIMEOUT;
        }
        timeout--;
    }

    return CheckTransferError(i2c);
}

DriverStatus WaitForRxData(I2C_Regs *i2c, uint32_t timeout)
{
    while (DL_I2C_isControllerRXFIFOEmpty(i2c)) {
        const uint32_t status = DL_I2C_getControllerStatus(i2c);

        if ((status & DL_I2C_CONTROLLER_STATUS_ERROR) != 0U) {
            ResetTransfer(i2c);
            return DRIVER_ERROR;
        }

        if (timeout == 0U) {
            ResetTransfer(i2c);
            return DRIVER_ERROR_TIMEOUT;
        }
        timeout--;
    }

    return DRIVER_OK;
}

void FillAddress(uint16_t address, uint8_t *buffer)
{
    buffer[0] = (uint8_t) ((address >> 8U) & 0xFFU);
    buffer[1] = (uint8_t) (address & 0xFFU);
}

uint16_t MinU16(uint16_t left, uint16_t right)
{
    return (left < right) ? left : right;
}

bool HasRecoveryPins(const Fm24cl64bConfig *config)
{
    return ((config != 0) && (config->scl_port != 0) &&
            (config->scl_pin != 0U) && (config->sda_port != 0) &&
            (config->sda_pin != 0U));
}

bool ReadGpioPin(GPIO_Regs *port, uint32_t pin)
{
    return ((DL_GPIO_readPins(port, pin) & pin) != 0U);
}

bool RecoveryLinesHigh(const Fm24cl64bConfig *config)
{
    return ReadGpioPin(config->scl_port, config->scl_pin) &&
           ReadGpioPin(config->sda_port, config->sda_pin);
}

void ConfigureRecoveryGpio(const Fm24cl64bConfig *config)
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

void RestoreI2cPins(const Fm24cl64bConfig *config)
{
    DL_GPIO_disableOutput(config->scl_port, config->scl_pin);
    DL_GPIO_disableOutput(config->sda_port, config->sda_pin);
    DL_GPIO_initPeripheralInputFunctionFeatures(config->sda_iomux,
                                                config->sda_iomux_func,
                                                DL_GPIO_INVERSION_DISABLE,
                                                DL_GPIO_RESISTOR_NONE,
                                                DL_GPIO_HYSTERESIS_DISABLE,
                                                DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_initPeripheralInputFunctionFeatures(config->scl_iomux,
                                                config->scl_iomux_func,
                                                DL_GPIO_INVERSION_DISABLE,
                                                DL_GPIO_RESISTOR_NONE,
                                                DL_GPIO_HYSTERESIS_DISABLE,
                                                DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_enableHiZ(config->sda_iomux);
    DL_GPIO_enableHiZ(config->scl_iomux);
}

void DriveLineLow(GPIO_Regs *port, uint32_t pin)
{
    DL_GPIO_clearPins(port, pin);
    DL_GPIO_enableOutput(port, pin);
}

void ReleaseLine(GPIO_Regs *port, uint32_t pin)
{
    DL_GPIO_disableOutput(port, pin);
}

DriverStatus WriteChunk(Fm24cl64bContext *ctx,
                        uint16_t address,
                        const uint8_t *data,
                        uint16_t length)
{
    uint8_t tx[kControllerFifoBytes];
    const uint16_t transfer_length = (uint16_t) (length + kAddressBytes);
    I2C_Regs *i2c = ctx->config->i2c;
    DriverStatus status = WaitForIdle(i2c, ctx->config->timeout_iterations);

    if (status != DRIVER_OK) {
        return status;
    }

    FillAddress(address, tx);
    for (uint16_t i = 0U; i < length; i++) {
        tx[kAddressBytes + i] = data[i];
    }

    ResetTransfer(i2c);

    if (DL_I2C_fillControllerTXFIFO(i2c, tx, transfer_length) !=
        transfer_length) {
        ResetTransfer(i2c);
        return DRIVER_ERROR_BUSY;
    }

    DL_I2C_startControllerTransfer(i2c,
                                   ctx->config->i2c_address,
                                   DL_I2C_CONTROLLER_DIRECTION_TX,
                                   transfer_length);
    delay_cycles(kI2cErrataDelayCycles);

    return WaitWhileBusy(i2c, ctx->config->timeout_iterations);
}

DriverStatus ReadChunk(Fm24cl64bContext *ctx,
                       uint16_t address,
                       uint8_t *data,
                       uint16_t length)
{
    uint8_t tx[kAddressBytes];
    I2C_Regs *i2c = ctx->config->i2c;
    DriverStatus status = WaitForIdle(i2c, ctx->config->timeout_iterations);

    if (status != DRIVER_OK) {
        return status;
    }

    FillAddress(address, tx);
    ResetTransfer(i2c);

    if (DL_I2C_fillControllerTXFIFO(i2c, tx, kAddressBytes) !=
        kAddressBytes) {
        ResetTransfer(i2c);
        return DRIVER_ERROR_BUSY;
    }

    DL_I2C_enableControllerReadOnTXEmpty(i2c);
    DL_I2C_startControllerTransferAdvanced(i2c,
                                           ctx->config->i2c_address,
                                           DL_I2C_CONTROLLER_DIRECTION_RX,
                                           length,
                                           DL_I2C_CONTROLLER_START_ENABLE,
                                           DL_I2C_CONTROLLER_STOP_ENABLE,
                                           DL_I2C_CONTROLLER_ACK_DISABLE);
    delay_cycles(kI2cErrataDelayCycles);

    for (uint16_t i = 0U; i < length; i++) {
        status = WaitForRxData(i2c, ctx->config->timeout_iterations);
        if (status != DRIVER_OK) {
            return status;
        }

        data[i] = DL_I2C_receiveControllerData(i2c);
    }

    status = WaitWhileBusy(i2c, ctx->config->timeout_iterations);
    DL_I2C_disableControllerReadOnTXEmpty(i2c);

    return status;
}

} /* namespace */

DriverStatus Fm24cl64b_Init(Fm24cl64bContext *ctx,
                            const Fm24cl64bConfig *config)
{
    if ((ctx == 0) || (config == 0) || (config->i2c == 0) ||
        (config->i2c_address > 0x7FU) ||
        (config->timeout_iterations == 0U) ||
        (!HasRecoveryPins(config))) {
        return DRIVER_ERROR_INVALID_ARG;
    }

    ctx->config = config;
    ctx->initialized = true;

    ResetTransfer(config->i2c);

    return DRIVER_OK;
}

DriverStatus Fm24cl64b_Read(Fm24cl64bContext *ctx,
                            uint16_t address,
                            uint8_t *data,
                            uint16_t length)
{
    DriverStatus status = CheckContext(ctx);
    uint16_t offset = 0U;

    if (status != DRIVER_OK) {
        return status;
    }

    if ((data == 0) || (!IsRangeValid(address, length))) {
        return DRIVER_ERROR_INVALID_ARG;
    }

    while (offset < length) {
        const uint16_t chunk = MinU16((uint16_t) (length - offset),
                                      kMaxReadPayloadBytes);

        status = ReadChunk(ctx,
                           (uint16_t) (address + offset),
                           &data[offset],
                           chunk);
        if (status != DRIVER_OK) {
            return status;
        }

        offset = (uint16_t) (offset + chunk);
    }

    return DRIVER_OK;
}

DriverStatus Fm24cl64b_Write(Fm24cl64bContext *ctx,
                             uint16_t address,
                             const uint8_t *data,
                             uint16_t length)
{
    DriverStatus status = CheckContext(ctx);
    uint16_t offset = 0U;

    if (status != DRIVER_OK) {
        return status;
    }

    if ((data == 0) || (!IsRangeValid(address, length))) {
        return DRIVER_ERROR_INVALID_ARG;
    }

    while (offset < length) {
        const uint16_t chunk = MinU16((uint16_t) (length - offset),
                                      kMaxWritePayloadBytes);

        status = WriteChunk(ctx,
                            (uint16_t) (address + offset),
                            &data[offset],
                            chunk);
        if (status != DRIVER_OK) {
            return status;
        }

        offset = (uint16_t) (offset + chunk);
    }

    return DRIVER_OK;
}

DriverStatus Fm24cl64b_ReadByte(Fm24cl64bContext *ctx,
                                uint16_t address,
                                uint8_t *data)
{
    return Fm24cl64b_Read(ctx, address, data, 1U);
}

DriverStatus Fm24cl64b_WriteByte(Fm24cl64bContext *ctx,
                                 uint16_t address,
                                 uint8_t data)
{
    return Fm24cl64b_Write(ctx, address, &data, 1U);
}

DriverStatus Fm24cl64b_SelfTest(Fm24cl64bContext *ctx,
                                uint16_t test_address)
{
    uint8_t original[FM24CL64B_SELF_TEST_LENGTH];
    uint8_t pattern[FM24CL64B_SELF_TEST_LENGTH] = {
        0xA5U, 0x5AU, 0x00U, 0xFFU, 0x12U, 0x34U, 0x56U, 0x78U
    };
    uint8_t readback[FM24CL64B_SELF_TEST_LENGTH];
    DriverStatus status = Fm24cl64b_Read(ctx,
                                         test_address,
                                         original,
                                         FM24CL64B_SELF_TEST_LENGTH);

    if (status != DRIVER_OK) {
        return status;
    }

    status = Fm24cl64b_Write(ctx,
                             test_address,
                             pattern,
                             FM24CL64B_SELF_TEST_LENGTH);
    if (status == DRIVER_OK) {
        status = Fm24cl64b_Read(ctx,
                                test_address,
                                readback,
                                FM24CL64B_SELF_TEST_LENGTH);
    }

    if (status == DRIVER_OK) {
        for (uint16_t i = 0U; i < FM24CL64B_SELF_TEST_LENGTH; i++) {
            if (readback[i] != pattern[i]) {
                status = DRIVER_ERROR;
                break;
            }
        }
    }

    const DriverStatus restore_status = Fm24cl64b_Write(
        ctx,
        test_address,
        original,
        FM24CL64B_SELF_TEST_LENGTH);

    if (status != DRIVER_OK) {
        return status;
    }

    return restore_status;
}

DriverStatus Fm24cl64b_RecoverBus(Fm24cl64bContext *ctx)
{
    DriverStatus status = CheckContext(ctx);

    if (status != DRIVER_OK) {
        return status;
    }

    const Fm24cl64bConfig *config = ctx->config;

    if (!HasRecoveryPins(config)) {
        return DRIVER_ERROR_UNSUPPORTED;
    }

    ResetTransfer(config->i2c);
    DL_I2C_disableController(config->i2c);
    ConfigureRecoveryGpio(config);

    ReleaseLine(config->sda_port, config->sda_pin);
    ReleaseLine(config->scl_port, config->scl_pin);
    delay_cycles(kBusRecoveryDelayCycles);

    for (uint8_t i = 0U; i < kBusRecoveryClockPulses; i++) {
        if (RecoveryLinesHigh(config)) {
            break;
        }

        DriveLineLow(config->scl_port, config->scl_pin);
        delay_cycles(kBusRecoveryDelayCycles);
        ReleaseLine(config->scl_port, config->scl_pin);
        delay_cycles(kBusRecoveryDelayCycles);
    }

    DriveLineLow(config->sda_port, config->sda_pin);
    delay_cycles(kBusRecoveryDelayCycles);
    ReleaseLine(config->scl_port, config->scl_pin);
    delay_cycles(kBusRecoveryDelayCycles);
    ReleaseLine(config->sda_port, config->sda_pin);
    delay_cycles(kBusRecoveryDelayCycles);

    const bool recovered = RecoveryLinesHigh(config);

    RestoreI2cPins(config);
    ResetTransfer(config->i2c);
    DL_I2C_enableController(config->i2c);
    delay_cycles(kI2cErrataDelayCycles);

    return recovered ? DRIVER_OK : DRIVER_ERROR_BUSY;
}

DriverStatus Fm24cl64b_GetBusStatus(Fm24cl64bContext *ctx,
                                    Fm24cl64bBusStatus *status)
{
    const DriverStatus context_status = CheckContext(ctx);

    if (context_status != DRIVER_OK) {
        return context_status;
    }

    if (status == 0) {
        return DRIVER_ERROR_INVALID_ARG;
    }

    status->controller_status = DL_I2C_getControllerStatus(ctx->config->i2c);
    status->scl_high =
        (DL_I2C_getSCLStatus(ctx->config->i2c) == DL_I2C_CONTROLLER_SCL_HIGH);
    status->sda_high =
        (DL_I2C_getSDAStatus(ctx->config->i2c) == DL_I2C_CONTROLLER_SDA_HIGH);

    return DRIVER_OK;
}

bool Fm24cl64b_IsReady(const Fm24cl64bContext *ctx)
{
    return ((ctx != 0) && ctx->initialized && (ctx->config != 0) &&
            (ctx->config->i2c != 0));
}

} /* namespace drivers */