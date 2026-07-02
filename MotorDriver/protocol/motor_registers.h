#ifndef PROTOCOL_MOTOR_REGISTERS_H_
#define PROTOCOL_MOTOR_REGISTERS_H_

#include <stdint.h>

namespace MotorProtocol {

constexpr uint8_t kDeviceId      = 0xA5;
constexpr uint8_t kFwVersion     = 0x01;
constexpr uint8_t kI2cBaseAddr   = 0x20;
constexpr uint8_t kI2cAddrMask   = 0x07;
constexpr uint8_t kRegisterCount = 0x40;

enum Register : uint8_t {
    REG_DEVICE_ID        = 0x00,
    REG_FW_VERSION       = 0x01,
    REG_STATUS           = 0x02,
    REG_FAULT_FLAGS      = 0x03,
    REG_CONTROL_FLAGS    = 0x04,
    REG_I2C_ADDRESS      = 0x05,

    REG_M1_MODE          = 0x10,
    REG_M1_DUTY          = 0x11,
    REG_M1_DIRECTION     = 0x12,
    REG_M1_STATUS        = 0x13,

    REG_M2_MODE          = 0x20,
    REG_M2_DUTY          = 0x21,
    REG_M2_DIRECTION     = 0x22,
    REG_M2_STATUS        = 0x23,

    REG_WATCHDOG_TIMEOUT = 0x30,
};

enum MotorMode : uint8_t {
    MOTOR_MODE_COAST = 0,
    MOTOR_MODE_RUN   = 1,
    MOTOR_MODE_BRAKE = 2,
};

enum MotorDirection : uint8_t {
    MOTOR_DIRECTION_FORWARD = 0,
    MOTOR_DIRECTION_REVERSE = 1,
};

enum StatusFlags : uint8_t {
    STATUS_READY   = 0x01,
    STATUS_ENABLED = 0x02,
    STATUS_ACTIVE  = 0x04,
    STATUS_TIMEOUT = 0x08,
};

enum FaultFlags : uint8_t {
    FAULT_I2C_TIMEOUT = 0x01,
};

enum ControlFlags : uint8_t {
    CONTROL_ENABLE = 0x01,
};

enum MotorStatusFlags : uint8_t {
    MOTOR_STATUS_ACTIVE  = 0x01,
    MOTOR_STATUS_REVERSE = 0x02,
};

}  // namespace MotorProtocol

#endif  // PROTOCOL_MOTOR_REGISTERS_H_
