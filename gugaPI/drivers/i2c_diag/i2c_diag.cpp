#include "drivers/i2c_diag/i2c_diag.h"

namespace drivers {
namespace {

static const uint32_t kI2cErrataDelayCycles = 100U;
static const uint32_t kBusRecoveryDelayCycles = 320U;
static const uint8_t kBusRecoveryClockPulses = 9U;
static const uint16_t kControllerFifoBytes = 8U;

bool IsConfigValid(const I2cDiagBusConfig *config)
{
    return (config != 0) && (config->name != 0) && (config->i2c != 0) &&
           (config->timeout_iterations > 0U) &&
           (config->scl_port != 0) && (config->scl_pin != 0U) &&
           (config->sda_port != 0) && (config->sda_pin != 0U);
}

bool IsAddressValid(uint8_t address)
{
    return (address >= I2C_DIAG_MIN_7BIT_ADDRESS) &&
           (address <= I2C_DIAG_MAX_7BIT_ADDRESS);
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

bool ReadGpioPin(GPIO_Regs *port, uint32_t pin)
{
    return ((DL_GPIO_readPins(port, pin) & pin) != 0U);
}

bool RecoveryLinesHigh(const I2cDiagBusConfig *config)
{
    return ReadGpioPin(config->scl_port, config->scl_pin) &&
           ReadGpioPin(config->sda_port, config->sda_pin);
}

void ConfigureRecoveryGpio(const I2cDiagBusConfig *config)
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

void RestoreI2cPins(const I2cDiagBusConfig *config)
{
    DL_GPIO_disableOutput(config->scl_port, config->scl_pin);
    DL_GPIO_disableOutput(config->sda_port, config->sda_pin);
    DL_GPIO_initPeripheralInputFunctionFeatures(config->sda_iomux,
                                                config->sda_iomux_func,
                                                DL_GPIO_INVERSION_DISABLE,
                                                DL_GPIO_RESISTOR_PULL_UP,
                                                DL_GPIO_HYSTERESIS_DISABLE,
                                                DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_initPeripheralInputFunctionFeatures(config->scl_iomux,
                                                config->scl_iomux_func,
                                                DL_GPIO_INVERSION_DISABLE,
                                                DL_GPIO_RESISTOR_PULL_UP,
                                                DL_GPIO_HYSTERESIS_DISABLE,
                                                DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_enableHiZ(config->sda_iomux);
    DL_GPIO_enableHiZ(config->scl_iomux);
}

void ReleaseI2cBus(const I2cDiagBusConfig *config)
{
    ResetTransfer(config->i2c);
    DL_I2C_disableController(config->i2c);
    RestoreI2cPins(config);
    ResetTransfer(config->i2c);
    DL_I2C_enableController(config->i2c);
    delay_cycles(kI2cErrataDelayCycles);
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

} /* namespace */

DriverStatus I2cDiag_GetBusStatus(const I2cDiagBusConfig *config,
                                  I2cDiagBusStatus *status)
{
    if ((!IsConfigValid(config)) || (status == 0)) {
        return DRIVER_ERROR_INVALID_ARG;
    }

    status->controller_status = DL_I2C_getControllerStatus(config->i2c);
    status->scl_high =
        (DL_I2C_getSCLStatus(config->i2c) == DL_I2C_CONTROLLER_SCL_HIGH);
    status->sda_high =
        (DL_I2C_getSDAStatus(config->i2c) == DL_I2C_CONTROLLER_SDA_HIGH);

    return DRIVER_OK;
}

DriverStatus I2cDiag_RecoverBus(const I2cDiagBusConfig *config)
{
    if (!IsConfigValid(config)) {
        return DRIVER_ERROR_INVALID_ARG;
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

    // 生成一个软件 STOP：SDA 低 -> SCL 高 -> SDA 高。
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

DriverStatus I2cDiag_ProbeAddress(const I2cDiagBusConfig *config,
                                  uint8_t address)
{
    uint8_t probe_byte = 0x00U;

    if ((!IsConfigValid(config)) || (!IsAddressValid(address))) {
        return DRIVER_ERROR_INVALID_ARG;
    }

    DriverStatus status = WaitForIdle(config->i2c, config->timeout_iterations);
    if (status != DRIVER_OK) {
        ReleaseI2cBus(config);
        return status;
    }

    ResetTransfer(config->i2c);

    if (DL_I2C_fillControllerTXFIFO(config->i2c, &probe_byte, 1U) != 1U) {
        ReleaseI2cBus(config);
        return DRIVER_ERROR_BUSY;
    }

    DL_I2C_startControllerTransfer(config->i2c,
                                   address,
                                   DL_I2C_CONTROLLER_DIRECTION_TX,
                                   1U);
    delay_cycles(kI2cErrataDelayCycles);

    uint32_t timeout = config->timeout_iterations;
    while ((DL_I2C_getControllerStatus(config->i2c) &
            DL_I2C_CONTROLLER_STATUS_BUSY) != 0U) {
        if (timeout == 0U) {
            ReleaseI2cBus(config);
            return DRIVER_ERROR_TIMEOUT;
        }
        timeout--;
    }

    const uint32_t controller_status = DL_I2C_getControllerStatus(config->i2c);
    ResetTransfer(config->i2c);

    if ((controller_status & DL_I2C_CONTROLLER_STATUS_ADDR_ACK) != 0U) {
        return DRIVER_ERROR;
    }

    if ((controller_status & DL_I2C_CONTROLLER_STATUS_ARBITRATION_LOST) != 0U) {
        ReleaseI2cBus(config);
        return DRIVER_ERROR_BUSY;
    }

    /* 数据 NACK 只说明设备不接受探测字节；地址已经 ACK，设备存在。 */
    return DRIVER_OK;
}

DriverStatus I2cDiag_ReadReg8(const I2cDiagBusConfig *config,
                              uint8_t address,
                              uint8_t reg,
                              uint8_t *data,
                              uint16_t length)
{
    if ((!IsConfigValid(config)) || (!IsAddressValid(address)) ||
        (data == 0) || (length == 0U) ||
        (length > I2C_DIAG_MAX_READ_BYTES)) {
        return DRIVER_ERROR_INVALID_ARG;
    }

    DriverStatus status = WaitForIdle(config->i2c, config->timeout_iterations);
    if (status != DRIVER_OK) {
        ReleaseI2cBus(config);
        return status;
    }

    ResetTransfer(config->i2c);

    if (DL_I2C_fillControllerTXFIFO(config->i2c, &reg, 1U) != 1U) {
        ReleaseI2cBus(config);
        return DRIVER_ERROR_BUSY;
    }

    DL_I2C_enableControllerReadOnTXEmpty(config->i2c);
    DL_I2C_startControllerTransferAdvanced(config->i2c,
                                           address,
                                           DL_I2C_CONTROLLER_DIRECTION_RX,
                                           length,
                                           DL_I2C_CONTROLLER_START_ENABLE,
                                           DL_I2C_CONTROLLER_STOP_ENABLE,
                                           DL_I2C_CONTROLLER_ACK_DISABLE);
    delay_cycles(kI2cErrataDelayCycles);

    for (uint16_t i = 0U; i < length; i++) {
        status = WaitForRxData(config->i2c, config->timeout_iterations);
        if (status != DRIVER_OK) {
            DL_I2C_disableControllerReadOnTXEmpty(config->i2c);
            ReleaseI2cBus(config);
            return status;
        }

        data[i] = DL_I2C_receiveControllerData(config->i2c);
    }

    status = WaitWhileBusy(config->i2c, config->timeout_iterations);
    DL_I2C_disableControllerReadOnTXEmpty(config->i2c);

    if (status != DRIVER_OK) {
        ReleaseI2cBus(config);
    }

    return status;
}

DriverStatus I2cDiag_WriteReg8(const I2cDiagBusConfig *config,
                               uint8_t address,
                               uint8_t reg,
                               const uint8_t *data,
                               uint16_t length)
{
    uint8_t tx[kControllerFifoBytes];

    if ((!IsConfigValid(config)) || (!IsAddressValid(address)) ||
        (data == 0) || (length == 0U) ||
        (length > I2C_DIAG_MAX_WRITE_BYTES)) {
        return DRIVER_ERROR_INVALID_ARG;
    }

    DriverStatus status = WaitForIdle(config->i2c, config->timeout_iterations);
    if (status != DRIVER_OK) {
        ReleaseI2cBus(config);
        return status;
    }

    tx[0] = reg;
    for (uint16_t i = 0U; i < length; i++) {
        tx[1U + i] = data[i];
    }

    ResetTransfer(config->i2c);

    if (DL_I2C_fillControllerTXFIFO(config->i2c, tx, (uint16_t) (length + 1U)) !=
        (uint16_t) (length + 1U)) {
        ReleaseI2cBus(config);
        return DRIVER_ERROR_BUSY;
    }

    DL_I2C_startControllerTransfer(config->i2c,
                                   address,
                                   DL_I2C_CONTROLLER_DIRECTION_TX,
                                   (uint16_t) (length + 1U));
    delay_cycles(kI2cErrataDelayCycles);

    status = WaitWhileBusy(config->i2c, config->timeout_iterations);
    if (status != DRIVER_OK) {
        ReleaseI2cBus(config);
    }

    return status;
}

} /* namespace drivers */
