#include "app/motor_app.h"

#include "control/motor_control.h"
#include "drivers/dip_address.h"
#include "drivers/i2c_slave.h"
#include "protocol/motor_protocol.h"
#include "ti_msp_dl_config.h"

namespace {

constexpr bool kEnableBootSpinTest = true;
constexpr uint32_t kBootSpinStepMs = 1000;
constexpr uint32_t kBootSpinBrakeMs = 1000;
constexpr uint8_t kBootSpinStepPercent = 20;

enum class BootSpinState : uint8_t {
    RampForward,
    BrakeBeforeReverse,
    RampReverse,
    BrakeBeforeForward,
};

MotorRegisterMap g_registers;
volatile uint32_t g_msTicks = 0;
uint32_t g_lastCommandMs = 0;
uint32_t g_bootSpinLastStepMs = 0;
uint8_t g_bootSpinDuty = 0;
BootSpinState g_bootSpinState = BootSpinState::RampForward;

uint32_t millis(void)
{
    return g_msTicks;
}

uint32_t watchdogTimeoutMs(void)
{
    const uint8_t timeout10ms = g_registers.read(MotorProtocol::REG_WATCHDOG_TIMEOUT);
    return static_cast<uint32_t>(timeout10ms) * 10U;
}

void forceCoastOnTimeout(void)
{
    g_registers.write(MotorProtocol::REG_M1_MODE, MotorProtocol::MOTOR_MODE_COAST);
    g_registers.write(MotorProtocol::REG_M2_MODE, MotorProtocol::MOTOR_MODE_COAST);
    g_registers.set(MotorProtocol::REG_FAULT_FLAGS,
        g_registers.read(MotorProtocol::REG_FAULT_FLAGS) | MotorProtocol::FAULT_I2C_TIMEOUT);
}

uint8_t motorStatus(uint8_t mode, uint8_t direction)
{
    uint8_t status = 0;
    if (mode == MotorProtocol::MOTOR_MODE_RUN) {
        status |= MotorProtocol::MOTOR_STATUS_ACTIVE;
    }
    if (direction == MotorProtocol::MOTOR_DIRECTION_REVERSE) {
        status |= MotorProtocol::MOTOR_STATUS_REVERSE;
    }
    return status;
}

void updateStatus(bool timedOut)
{
    const bool enabled =
        (g_registers.read(MotorProtocol::REG_CONTROL_FLAGS) & MotorProtocol::CONTROL_ENABLE) != 0U;
    const bool m1Active =
        g_registers.read(MotorProtocol::REG_M1_MODE) == MotorProtocol::MOTOR_MODE_RUN;
    const bool m2Active =
        g_registers.read(MotorProtocol::REG_M2_MODE) == MotorProtocol::MOTOR_MODE_RUN;

    uint8_t status = MotorProtocol::STATUS_READY;
    if (enabled) {
        status |= MotorProtocol::STATUS_ENABLED;
    }
    if (m1Active || m2Active) {
        status |= MotorProtocol::STATUS_ACTIVE;
    }
    if (timedOut) {
        status |= MotorProtocol::STATUS_TIMEOUT;
    }

    g_registers.set(MotorProtocol::REG_STATUS, status);
    g_registers.set(MotorProtocol::REG_M1_STATUS,
        motorStatus(g_registers.read(MotorProtocol::REG_M1_MODE),
                    g_registers.read(MotorProtocol::REG_M1_DIRECTION)));
    g_registers.set(MotorProtocol::REG_M2_STATUS,
        motorStatus(g_registers.read(MotorProtocol::REG_M2_MODE),
                    g_registers.read(MotorProtocol::REG_M2_DIRECTION)));
}

void applyBootSpinCommand(void)
{
    const uint8_t direction = (g_bootSpinState == BootSpinState::RampReverse)
        ? MotorProtocol::MOTOR_DIRECTION_REVERSE
        : MotorProtocol::MOTOR_DIRECTION_FORWARD;

    g_registers.write(MotorProtocol::REG_M1_DIRECTION, direction);
    g_registers.write(MotorProtocol::REG_M1_DUTY, g_bootSpinDuty);
    g_registers.write(MotorProtocol::REG_M1_MODE, MotorProtocol::MOTOR_MODE_RUN);

    g_registers.write(MotorProtocol::REG_M2_DIRECTION, direction);
    g_registers.write(MotorProtocol::REG_M2_DUTY, g_bootSpinDuty);
    g_registers.write(MotorProtocol::REG_M2_MODE, MotorProtocol::MOTOR_MODE_RUN);
}

void applyBootSpinBrake(void)
{
    g_registers.write(MotorProtocol::REG_M1_DUTY, 0);
    g_registers.write(MotorProtocol::REG_M1_MODE, MotorProtocol::MOTOR_MODE_BRAKE);

    g_registers.write(MotorProtocol::REG_M2_DUTY, 0);
    g_registers.write(MotorProtocol::REG_M2_MODE, MotorProtocol::MOTOR_MODE_BRAKE);
}

void startBootSpinTest(void)
{
    g_registers.write(MotorProtocol::REG_WATCHDOG_TIMEOUT, 0);
    g_bootSpinState = BootSpinState::RampForward;
    g_bootSpinDuty = 0;
    g_bootSpinLastStepMs = millis();
    applyBootSpinCommand();
}

void updateBootSpinTest(void)
{
    if ((g_bootSpinState == BootSpinState::BrakeBeforeReverse) ||
        (g_bootSpinState == BootSpinState::BrakeBeforeForward)) {
        if ((millis() - g_bootSpinLastStepMs) >= kBootSpinBrakeMs) {
            g_bootSpinLastStepMs = millis();
            g_bootSpinDuty = 0;
            g_bootSpinState = (g_bootSpinState == BootSpinState::BrakeBeforeReverse)
                ? BootSpinState::RampReverse
                : BootSpinState::RampForward;
            applyBootSpinCommand();
        }
        return;
    }

    if ((millis() - g_bootSpinLastStepMs) < kBootSpinStepMs) {
        return;
    }

    g_bootSpinLastStepMs = millis();

    if (g_bootSpinDuty < 100U) {
        g_bootSpinDuty += kBootSpinStepPercent;
        applyBootSpinCommand();
        return;
    }

    applyBootSpinBrake();
    g_bootSpinLastStepMs = millis();
    if (g_bootSpinState == BootSpinState::RampForward) {
        g_bootSpinState = BootSpinState::BrakeBeforeReverse;
    } else {
        g_bootSpinState = BootSpinState::BrakeBeforeForward;
    }
}

}  // namespace

extern "C" void SysTick_Handler(void)
{
    ++g_msTicks;
}

void MotorApp_init(void)
{
    const uint8_t i2cAddress = DipAddress_readI2cAddress();

    DL_GPIO_setDigitalInternalResistor(GPIO_I2C_TARGET_IOMUX_SCL, DL_GPIO_RESISTOR_PULL_UP);
    DL_GPIO_setDigitalInternalResistor(GPIO_I2C_TARGET_IOMUX_SDA, DL_GPIO_RESISTOR_PULL_UP);

    g_registers.init(i2cAddress);
    if (kEnableBootSpinTest) {
        startBootSpinTest();
    }

    MotorControl_init();
    I2cSlave_init(&g_registers, i2cAddress);

    SysTick_Config(CPUCLK_FREQ / 1000U);
    g_lastCommandMs = millis();
}

void MotorApp_process(void)
{
    bool timedOut = false;

    if (I2cSlave_consumeWriteActivity()) {
        g_lastCommandMs = millis();
    }

    if (kEnableBootSpinTest) {
        updateBootSpinTest();
    }

    const uint32_t timeoutMs = watchdogTimeoutMs();
    if ((timeoutMs > 0U) && ((millis() - g_lastCommandMs) > timeoutMs)) {
        forceCoastOnTimeout();
        timedOut = true;
    }

    updateStatus(timedOut);
    MotorControl_apply(g_registers);
}
