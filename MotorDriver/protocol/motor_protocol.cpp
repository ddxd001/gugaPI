#include "protocol/motor_protocol.h"

void MotorRegisterMap::init(uint8_t i2cAddress)
{
    for (uint8_t& reg : registers_) {
        reg = 0;
    }

    registers_[MotorProtocol::REG_DEVICE_ID]        = MotorProtocol::kDeviceId;
    registers_[MotorProtocol::REG_FW_VERSION]       = MotorProtocol::kFwVersion;
    registers_[MotorProtocol::REG_I2C_ADDRESS]      = i2cAddress;
    registers_[MotorProtocol::REG_STATUS]           = MotorProtocol::STATUS_READY;
    registers_[MotorProtocol::REG_CONTROL_FLAGS]    = MotorProtocol::CONTROL_ENABLE;
    registers_[MotorProtocol::REG_M1_MODE]          = MotorProtocol::MOTOR_MODE_COAST;
    registers_[MotorProtocol::REG_M2_MODE]          = MotorProtocol::MOTOR_MODE_COAST;
    registers_[MotorProtocol::REG_WATCHDOG_TIMEOUT] = 100;
}

uint8_t MotorRegisterMap::read(uint8_t reg) const
{
    if (reg >= MotorProtocol::kRegisterCount) {
        return 0x00;
    }

    return registers_[reg];
}

void MotorRegisterMap::write(uint8_t reg, uint8_t value)
{
    if (reg >= MotorProtocol::kRegisterCount) {
        return;
    }

    switch (reg) {
        case MotorProtocol::REG_DEVICE_ID:
        case MotorProtocol::REG_FW_VERSION:
        case MotorProtocol::REG_STATUS:
        case MotorProtocol::REG_I2C_ADDRESS:
        case MotorProtocol::REG_M1_STATUS:
        case MotorProtocol::REG_M2_STATUS:
            return;
        case MotorProtocol::REG_FAULT_FLAGS:
            registers_[reg] &= static_cast<uint8_t>(~value);
            return;
        case MotorProtocol::REG_M1_DUTY:
        case MotorProtocol::REG_M2_DUTY:
            registers_[reg] = (value > 100U) ? 100U : value;
            return;
        case MotorProtocol::REG_M1_MODE:
        case MotorProtocol::REG_M2_MODE:
            if (value <= MotorProtocol::MOTOR_MODE_BRAKE) {
                registers_[reg] = value;
            }
            return;
        case MotorProtocol::REG_M1_DIRECTION:
        case MotorProtocol::REG_M2_DIRECTION:
            registers_[reg] = value ? MotorProtocol::MOTOR_DIRECTION_REVERSE
                                    : MotorProtocol::MOTOR_DIRECTION_FORWARD;
            return;
        default:
            registers_[reg] = value;
            return;
    }
}

void MotorRegisterMap::set(uint8_t reg, uint8_t value)
{
    if (reg < MotorProtocol::kRegisterCount) {
        registers_[reg] = value;
    }
}

const uint8_t* MotorRegisterMap::raw() const
{
    return registers_;
}
