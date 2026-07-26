#include "drivers/i2c_controller/i2c_controller.h"

namespace drivers {
namespace {

static const uint32_t kI2cErrataDelayCycles = 100U;
static const uint32_t kBusRecoveryDelayCycles = 320U;
static const uint8_t kBusRecoveryClockPulses = 9U;
static const uint8_t kProbeByte = 0x00U;

void ResetTransfer(I2C_Regs *i2c)
{
    DL_I2C_resetControllerTransfer(i2c);
    DL_I2C_disableControllerReadOnTXEmpty(i2c);
    DL_I2C_flushControllerTXFIFO(i2c);
    DL_I2C_flushControllerRXFIFO(i2c);
}

DriverStatus DecodeControllerStatus(uint32_t status)
{
    if ((status & DL_I2C_CONTROLLER_STATUS_ARBITRATION_LOST) != 0U) {
        return DRIVER_ERROR_BUSY;
    }
    if ((status & (DL_I2C_CONTROLLER_STATUS_ADDR_ACK |
                   DL_I2C_CONTROLLER_STATUS_DATA_ACK)) != 0U) {
        return DRIVER_ERROR_NACK;
    }
    if ((status & DL_I2C_CONTROLLER_STATUS_ERROR) != 0U) {
        return DRIVER_ERROR;
    }
    return DRIVER_OK;
}

void AbortTransfer(const I2cControllerConfig *config)
{
    if ((DL_I2C_getControllerStatus(config->i2c) &
         DL_I2C_CONTROLLER_STATUS_BUSY) != 0U) {
        DL_I2C_enableStopCondition(config->i2c);
        uint32_t timeout = config->timeout_iterations;
        while (((DL_I2C_getControllerStatus(config->i2c) &
                 DL_I2C_CONTROLLER_STATUS_BUSY) != 0U) &&
               (timeout != 0U)) {
            timeout--;
        }
    }
    ResetTransfer(config->i2c);
}

DriverStatus WaitForIdle(const I2cControllerConfig *config)
{
    uint32_t timeout = config->timeout_iterations;
    while ((DL_I2C_getControllerStatus(config->i2c) &
            DL_I2C_CONTROLLER_STATUS_IDLE) == 0U) {
        const DriverStatus status = DecodeControllerStatus(
            DL_I2C_getControllerStatus(config->i2c));
        if (status != DRIVER_OK) {
            AbortTransfer(config);
            return status;
        }
        if (timeout == 0U) {
            AbortTransfer(config);
            return DRIVER_ERROR_TIMEOUT;
        }
        timeout--;
    }

    const DriverStatus status = DecodeControllerStatus(
        DL_I2C_getControllerStatus(config->i2c));
    if (status != DRIVER_OK) {
        AbortTransfer(config);
    }
    return status;
}

DriverStatus FinishTransfer(const I2cControllerConfig *config)
{
    uint32_t timeout = config->timeout_iterations;
    while ((DL_I2C_getControllerStatus(config->i2c) &
            DL_I2C_CONTROLLER_STATUS_BUSY) != 0U) {
        const DriverStatus status = DecodeControllerStatus(
            DL_I2C_getControllerStatus(config->i2c));
        if (status != DRIVER_OK) {
            AbortTransfer(config);
            return status;
        }
        if (timeout == 0U) {
            AbortTransfer(config);
            return DRIVER_ERROR_TIMEOUT;
        }
        timeout--;
    }

    const DriverStatus status = DecodeControllerStatus(
        DL_I2C_getControllerStatus(config->i2c));
    ResetTransfer(config->i2c);
    return status;
}

bool ReadGpioPin(GPIO_Regs *port, uint32_t pin)
{
    return ((DL_GPIO_readPins(port, pin) & pin) != 0U);
}

bool RecoveryLinesHigh(const I2cControllerConfig *config)
{
    return ReadGpioPin(config->scl_port, config->scl_pin) &&
           ReadGpioPin(config->sda_port, config->sda_pin);
}

void ConfigureRecoveryGpio(const I2cControllerConfig *config)
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

void RestoreI2cPins(const I2cControllerConfig *config)
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

void DriveLineLow(GPIO_Regs *port, uint32_t pin)
{
    DL_GPIO_clearPins(port, pin);
    DL_GPIO_enableOutput(port, pin);
}

void ReleaseLine(GPIO_Regs *port, uint32_t pin)
{
    DL_GPIO_disableOutput(port, pin);
}

uint8_t WriteByteAt(const uint8_t *prefix,
                    uint16_t prefix_length,
                    const uint8_t *data,
                    uint16_t index)
{
    return (index < prefix_length) ? prefix[index] :
        data[index - prefix_length];
}

DriverStatus ReceiveStartedTransfer(const I2cControllerConfig *config,
                                    uint8_t *data,
                                    uint16_t length)
{
    uint32_t timeout = config->timeout_iterations;
    uint16_t received = 0U;

    while (received < length) {
        while (DL_I2C_isControllerRXFIFOEmpty(config->i2c)) {
            const DriverStatus status = DecodeControllerStatus(
                DL_I2C_getControllerStatus(config->i2c));
            if (status != DRIVER_OK) {
                AbortTransfer(config);
                return status;
            }
            if (timeout == 0U) {
                AbortTransfer(config);
                return DRIVER_ERROR_TIMEOUT;
            }
            timeout--;
        }
        data[received] = DL_I2C_receiveControllerData(config->i2c);
        received++;
    }

    return FinishTransfer(config);
}

} /* namespace */

bool I2cController_IsConfigValid(const I2cControllerConfig *config)
{
    return (config != 0) && (config->name != 0) && (config->i2c != 0) &&
           (config->timeout_iterations != 0U) &&
           (config->scl_port != 0) && (config->scl_pin != 0U) &&
           (config->sda_port != 0) && (config->sda_pin != 0U);
}

bool I2cController_IsAddressValid(uint8_t target_address)
{
    return (target_address >= I2C_CONTROLLER_MIN_7BIT_ADDRESS) &&
           (target_address <= I2C_CONTROLLER_MAX_7BIT_ADDRESS);
}

DriverStatus I2cController_GetBusStatus(
    const I2cControllerConfig *config,
    I2cControllerBusStatus *status)
{
    if ((!I2cController_IsConfigValid(config)) || (status == 0)) {
        return DRIVER_ERROR_INVALID_ARG;
    }

    status->controller_status = DL_I2C_getControllerStatus(config->i2c);
    status->scl_high =
        (DL_I2C_getSCLStatus(config->i2c) == DL_I2C_CONTROLLER_SCL_HIGH);
    status->sda_high =
        (DL_I2C_getSDAStatus(config->i2c) == DL_I2C_CONTROLLER_SDA_HIGH);
    return DRIVER_OK;
}

DriverStatus I2cController_RecoverBus(const I2cControllerConfig *config)
{
    if (!I2cController_IsConfigValid(config)) {
        return DRIVER_ERROR_INVALID_ARG;
    }

    AbortTransfer(config);
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

    /* Generate a software STOP: SDA low, SCL released high, SDA released. */
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

DriverStatus I2cController_Probe(const I2cControllerConfig *config,
                                 uint8_t target_address)
{
    if ((!I2cController_IsConfigValid(config)) ||
        (!I2cController_IsAddressValid(target_address))) {
        return DRIVER_ERROR_INVALID_ARG;
    }

    DriverStatus status = WaitForIdle(config);
    if (status != DRIVER_OK) {
        return status;
    }
    ResetTransfer(config->i2c);
    if (DL_I2C_fillControllerTXFIFO(config->i2c, &kProbeByte, 1U) != 1U) {
        AbortTransfer(config);
        return DRIVER_ERROR_BUSY;
    }

    DL_I2C_startControllerTransfer(config->i2c,
                                   target_address,
                                   DL_I2C_CONTROLLER_DIRECTION_TX,
                                   1U);
    delay_cycles(kI2cErrataDelayCycles);

    uint32_t timeout = config->timeout_iterations;
    while ((DL_I2C_getControllerStatus(config->i2c) &
            DL_I2C_CONTROLLER_STATUS_BUSY) != 0U) {
        if (timeout == 0U) {
            AbortTransfer(config);
            return DRIVER_ERROR_TIMEOUT;
        }
        timeout--;
    }

    const uint32_t controller_status =
        DL_I2C_getControllerStatus(config->i2c);
    ResetTransfer(config->i2c);
    if ((controller_status & DL_I2C_CONTROLLER_STATUS_ARBITRATION_LOST) != 0U) {
        return DRIVER_ERROR_BUSY;
    }
    if ((controller_status & DL_I2C_CONTROLLER_STATUS_ADDR_ACK) != 0U) {
        return DRIVER_ERROR_NACK;
    }
    /* A data NACK is acceptable: the target acknowledged its address. */
    if (((controller_status & DL_I2C_CONTROLLER_STATUS_ERROR) != 0U) &&
        ((controller_status & DL_I2C_CONTROLLER_STATUS_DATA_ACK) == 0U)) {
        return DRIVER_ERROR;
    }
    return DRIVER_OK;
}

DriverStatus I2cController_Write(const I2cControllerConfig *config,
                                 uint8_t target_address,
                                 const uint8_t *prefix,
                                 uint16_t prefix_length,
                                 const uint8_t *data,
                                 uint16_t data_length)
{
    const uint32_t total = static_cast<uint32_t>(prefix_length) + data_length;
    if ((!I2cController_IsConfigValid(config)) ||
        (!I2cController_IsAddressValid(target_address)) ||
        ((prefix == 0) && (prefix_length != 0U)) ||
        ((data == 0) && (data_length != 0U)) || (total == 0U) ||
        (total > I2C_CONTROLLER_MAX_TRANSFER_BYTES)) {
        return DRIVER_ERROR_INVALID_ARG;
    }

    DriverStatus status = WaitForIdle(config);
    if (status != DRIVER_OK) {
        return status;
    }
    ResetTransfer(config->i2c);

    uint16_t sent = 0U;
    const uint16_t transfer_length = static_cast<uint16_t>(total);
    while ((sent < transfer_length) &&
           (!DL_I2C_isControllerTXFIFOFull(config->i2c))) {
        DL_I2C_transmitControllerData(
            config->i2c,
            WriteByteAt(prefix, prefix_length, data, sent));
        sent++;
    }

    DL_I2C_startControllerTransfer(config->i2c,
                                   target_address,
                                   DL_I2C_CONTROLLER_DIRECTION_TX,
                                   transfer_length);
    delay_cycles(kI2cErrataDelayCycles);

    uint32_t timeout = config->timeout_iterations;
    while ((DL_I2C_getControllerStatus(config->i2c) &
            DL_I2C_CONTROLLER_STATUS_BUSY) != 0U) {
        while ((sent < transfer_length) &&
               (!DL_I2C_isControllerTXFIFOFull(config->i2c))) {
            DL_I2C_transmitControllerData(
                config->i2c,
                WriteByteAt(prefix, prefix_length, data, sent));
            sent++;
        }

        status = DecodeControllerStatus(
            DL_I2C_getControllerStatus(config->i2c));
        if (status != DRIVER_OK) {
            AbortTransfer(config);
            return status;
        }
        if (timeout == 0U) {
            AbortTransfer(config);
            return DRIVER_ERROR_TIMEOUT;
        }
        timeout--;
    }

    status = DecodeControllerStatus(
        DL_I2C_getControllerStatus(config->i2c));
    if ((status == DRIVER_OK) && (sent != transfer_length)) {
        status = DRIVER_ERROR;
    }
    ResetTransfer(config->i2c);
    return status;
}

DriverStatus I2cController_Read(const I2cControllerConfig *config,
                                uint8_t target_address,
                                uint8_t *data,
                                uint16_t length)
{
    if ((!I2cController_IsConfigValid(config)) ||
        (!I2cController_IsAddressValid(target_address)) || (data == 0) ||
        (length == 0U) || (length > I2C_CONTROLLER_MAX_TRANSFER_BYTES)) {
        return DRIVER_ERROR_INVALID_ARG;
    }

    DriverStatus status = WaitForIdle(config);
    if (status != DRIVER_OK) {
        return status;
    }
    ResetTransfer(config->i2c);
    DL_I2C_startControllerTransfer(config->i2c,
                                   target_address,
                                   DL_I2C_CONTROLLER_DIRECTION_RX,
                                   length);
    delay_cycles(kI2cErrataDelayCycles);
    return ReceiveStartedTransfer(config, data, length);
}

DriverStatus I2cController_WriteRead(const I2cControllerConfig *config,
                                     uint8_t target_address,
                                     const uint8_t *write_data,
                                     uint16_t write_length,
                                     uint8_t *read_data,
                                     uint16_t read_length)
{
    if ((!I2cController_IsConfigValid(config)) ||
        (!I2cController_IsAddressValid(target_address)) ||
        (write_data == 0) || (write_length == 0U) ||
        (write_length > I2C_CONTROLLER_FIFO_BYTES) || (read_data == 0) ||
        (read_length == 0U) ||
        (read_length > I2C_CONTROLLER_MAX_TRANSFER_BYTES)) {
        return DRIVER_ERROR_INVALID_ARG;
    }

    DriverStatus status = WaitForIdle(config);
    if (status != DRIVER_OK) {
        return status;
    }
    ResetTransfer(config->i2c);
    if (DL_I2C_fillControllerTXFIFO(config->i2c,
                                    write_data,
                                    write_length) != write_length) {
        AbortTransfer(config);
        return DRIVER_ERROR_BUSY;
    }

    DL_I2C_enableControllerReadOnTXEmpty(config->i2c);
    DL_I2C_startControllerTransferAdvanced(config->i2c,
                                           target_address,
                                           DL_I2C_CONTROLLER_DIRECTION_RX,
                                           read_length,
                                           DL_I2C_CONTROLLER_START_ENABLE,
                                           DL_I2C_CONTROLLER_STOP_ENABLE,
                                           DL_I2C_CONTROLLER_ACK_DISABLE);
    delay_cycles(kI2cErrataDelayCycles);
    status = ReceiveStartedTransfer(config, read_data, read_length);
    DL_I2C_disableControllerReadOnTXEmpty(config->i2c);
    return status;
}

} /* namespace drivers */

