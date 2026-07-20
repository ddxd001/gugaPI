#include "drivers/ina219/ina219.h"

namespace drivers {
namespace {

/* INA219 register map (auto-increment burst read from 0x00 covers all six):
 *   0x00 config, 0x01 shunt_voltage, 0x02 bus_voltage,
 *   0x03 power, 0x04 current, 0x05 calibration */
static const uint8_t kRegisterConfig = 0x00U;
static const uint8_t kRegisterCalibration = 0x05U;
static const uint16_t kConfigReset = 0x8000U;
// INA219 校准公式：CAL = 0.04096 / (current_LSB * Rshunt)，这里用 uA/mΩ 避免浮点。
static const uint32_t kCalibrationNumerator = 40960000U;
static const uint32_t kI2cErrataDelayCycles = 100U;
static const uint32_t kBusRecoveryDelayCycles = 320U;
static const uint8_t kBusRecoveryClockPulses = 9U;

DriverStatus CheckContext(Ina219Context *ctx)
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

bool HasRecoveryPins(const Ina219Config *config)
{
    return ((config != 0) && (config->scl_port != 0) &&
            (config->scl_pin != 0U) && (config->sda_port != 0) &&
            (config->sda_pin != 0U));
}

bool ReadGpioPin(GPIO_Regs *port, uint32_t pin)
{
    return ((DL_GPIO_readPins(port, pin) & pin) != 0U);
}

bool RecoveryLinesHigh(const Ina219Config *config)
{
    return ReadGpioPin(config->scl_port, config->scl_pin) &&
           ReadGpioPin(config->sda_port, config->sda_pin);
}

void ConfigureRecoveryGpio(const Ina219Config *config)
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

// 恢复为 I2C 复用功能，并保留内部上拉，避免无外部上拉时总线悬空。
void RestoreI2cPins(const Ina219Config *config)
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


// 超时或无 ACK 后强制释放控制器，防止 SDA/SCL 被保持为低电平。
void ReleaseI2cBus(const Ina219Config *config)
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
    uint8_t tx[3];
    I2C_Regs *i2c = ctx->config->i2c;
    DriverStatus status = WaitForIdle(i2c, ctx->config->timeout_iterations);

    if (status != DRIVER_OK) {
        ReleaseI2cBus(ctx->config);
        return status;
    }

    tx[0] = reg;
    tx[1] = (uint8_t) ((value >> 8U) & 0xFFU);
    tx[2] = (uint8_t) (value & 0xFFU);

    ResetTransfer(i2c);

    if (DL_I2C_fillControllerTXFIFO(i2c, tx, sizeof(tx)) != sizeof(tx)) {
        ReleaseI2cBus(ctx->config);
        return DRIVER_ERROR_BUSY;
    }

    DL_I2C_startControllerTransfer(i2c,
                                   ctx->i2c_address,
                                   DL_I2C_CONTROLLER_DIRECTION_TX,
                                   sizeof(tx));
    delay_cycles(kI2cErrataDelayCycles);

    status = WaitWhileBusy(i2c, ctx->config->timeout_iterations);
    if (status != DRIVER_OK) {
        ReleaseI2cBus(ctx->config);
    }

    return status;
}

// 寄存器读流程：写入 8 位寄存器地址，再 repeated-start 读取 16 位大端数据。
DriverStatus ReadRegisterInternal(Ina219Context *ctx,
                                  uint8_t reg,
                                  uint16_t *value)
{
    uint8_t tx = reg;
    uint8_t rx[2] = { 0U, 0U };
    I2C_Regs *i2c = ctx->config->i2c;
    DriverStatus status = WaitForIdle(i2c, ctx->config->timeout_iterations);

    if (status != DRIVER_OK) {
        ReleaseI2cBus(ctx->config);
        return status;
    }

    ResetTransfer(i2c);

    if (DL_I2C_fillControllerTXFIFO(i2c, &tx, 1U) != 1U) {
        ReleaseI2cBus(ctx->config);
        return DRIVER_ERROR_BUSY;
    }

    DL_I2C_enableControllerReadOnTXEmpty(i2c);
    DL_I2C_startControllerTransferAdvanced(i2c,
                                           ctx->i2c_address,
                                           DL_I2C_CONTROLLER_DIRECTION_RX,
                                           sizeof(rx),
                                           DL_I2C_CONTROLLER_START_ENABLE,
                                           DL_I2C_CONTROLLER_STOP_ENABLE,
                                           DL_I2C_CONTROLLER_ACK_DISABLE);
    delay_cycles(kI2cErrataDelayCycles);

    for (uint8_t i = 0U; i < sizeof(rx); i++) {
        status = WaitForRxData(i2c, ctx->config->timeout_iterations);
        if (status != DRIVER_OK) {
            DL_I2C_disableControllerReadOnTXEmpty(i2c);
            ReleaseI2cBus(ctx->config);
            return status;
        }

        rx[i] = DL_I2C_receiveControllerData(i2c);
    }

    status = WaitWhileBusy(i2c, ctx->config->timeout_iterations);
    DL_I2C_disableControllerReadOnTXEmpty(i2c);

    if (status != DRIVER_OK) {
        ReleaseI2cBus(ctx->config);
        return status;
    }

    *value = ((uint16_t) rx[0] << 8U) | rx[1];
    return DRIVER_OK;
}

/* Burst read of `count` bytes starting at `start_reg` in a single I2C
 * transaction (INA219 auto-increments its register pointer on multi-byte
 * reads). Used so a full register snapshot is atomic instead of 6 separate
 * transactions that can straddle a conversion boundary. */
DriverStatus ReadBurstInternal(Ina219Context *ctx,
                               uint8_t start_reg,
                               uint8_t *buf,
                               uint8_t count)
{
    uint8_t tx = start_reg;
    I2C_Regs *i2c = ctx->config->i2c;
    DriverStatus status = WaitForIdle(i2c, ctx->config->timeout_iterations);

    if (status != DRIVER_OK) {
        ReleaseI2cBus(ctx->config);
        return status;
    }

    ResetTransfer(i2c);

    if (DL_I2C_fillControllerTXFIFO(i2c, &tx, 1U) != 1U) {
        ReleaseI2cBus(ctx->config);
        return DRIVER_ERROR_BUSY;
    }

    DL_I2C_enableControllerReadOnTXEmpty(i2c);
    DL_I2C_startControllerTransferAdvanced(i2c,
                                           ctx->i2c_address,
                                           DL_I2C_CONTROLLER_DIRECTION_RX,
                                           count,
                                           DL_I2C_CONTROLLER_START_ENABLE,
                                           DL_I2C_CONTROLLER_STOP_ENABLE,
                                           DL_I2C_CONTROLLER_ACK_DISABLE);
    delay_cycles(kI2cErrataDelayCycles);

    for (uint8_t i = 0U; i < count; i++) {
        status = WaitForRxData(i2c, ctx->config->timeout_iterations);
        if (status != DRIVER_OK) {
            DL_I2C_disableControllerReadOnTXEmpty(i2c);
            ReleaseI2cBus(ctx->config);
            return status;
        }

        buf[i] = DL_I2C_receiveControllerData(i2c);
    }

    status = WaitWhileBusy(i2c, ctx->config->timeout_iterations);
    DL_I2C_disableControllerReadOnTXEmpty(i2c);

    if (status != DRIVER_OK) {
        ReleaseI2cBus(ctx->config);
        return status;
    }

    return DRIVER_OK;
}

} /* namespace */

DriverStatus Ina219_Init(Ina219Context *ctx, const Ina219Config *config)
{
    uint16_t calibration = 0U;

    if ((ctx == 0) || (config == 0) || (config->i2c == 0) ||
        (config->i2c_address > 0x7FU) ||
        (config->timeout_iterations == 0U) ||
        (!HasRecoveryPins(config)) ||
        (!ComputeCalibration(config, &calibration))) {
        return DRIVER_ERROR_INVALID_ARG;
    }

    ctx->config = config;
    ctx->i2c_address = config->i2c_address;
    ctx->calibration = calibration;
    ctx->initialized = true;

    ResetTransfer(config->i2c);

    return Ina219_ConfigureDefault(ctx);
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

    if (address > 0x7FU) {
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

    if ((address > 0x7FU) || (config_value == 0)) {
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
    DriverStatus status = CheckContext(ctx);

    if (status != DRIVER_OK) {
        return status;
    }

    if (raw == 0) {
        return DRIVER_ERROR_INVALID_ARG;
    }

    /* Single burst read of config..calibration (6 contiguous 16-bit registers,
     * 12 bytes) so the snapshot is atomic - the previous version issued 6
     * separate transactions and could mix conversion N and N+1. */
    uint8_t buf[12] = { 0U };
    status = ReadBurstInternal(ctx, kRegisterConfig, buf, sizeof(buf));
    if (status != DRIVER_OK) {
        return status;
    }

    raw->config         = (uint16_t)(((uint16_t) buf[0] << 8U) | buf[1]);
    raw->shunt_voltage  = (int16_t) (((uint16_t) buf[2] << 8U) | buf[3]);
    raw->bus_voltage    = (uint16_t)(((uint16_t) buf[4] << 8U) | buf[5]);
    raw->power          = (uint16_t)(((uint16_t) buf[6] << 8U) | buf[7]);
    raw->current        = (int16_t) (((uint16_t) buf[8] << 8U) | buf[9]);
    raw->calibration    = (uint16_t)(((uint16_t) buf[10] << 8U) | buf[11]);
    return DRIVER_OK;
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

// I2C 从设备拉低 SDA 时，用 9 个 SCL 脉冲尝试释放总线。
DriverStatus Ina219_RecoverBus(Ina219Context *ctx)
{
    DriverStatus status = CheckContext(ctx);

    if (status != DRIVER_OK) {
        return status;
    }

    const Ina219Config *config = ctx->config;

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

DriverStatus Ina219_GetBusStatus(Ina219Context *ctx,
                                 Ina219BusStatus *status)
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

bool Ina219_IsReady(const Ina219Context *ctx)
{
    return ((ctx != 0) && ctx->initialized && (ctx->config != 0) &&
            (ctx->config->i2c != 0));
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