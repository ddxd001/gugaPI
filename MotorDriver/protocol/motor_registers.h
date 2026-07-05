#ifndef PROTOCOL_MOTOR_REGISTERS_H_
#define PROTOCOL_MOTOR_REGISTERS_H_

#include <stdint.h>

namespace protocol {

constexpr uint8_t kDeviceId = 0xA5U;
constexpr uint8_t kFirmwareVersion = 0x01U;
constexpr uint8_t kRegisterCount = 0x50U;
constexpr uint8_t kMaxPayloadLength = 32U;

constexpr uint8_t kSerialSof = 0xAAU;
constexpr uint8_t kSerialCmdRead = 0x01U;
constexpr uint8_t kSerialCmdWrite = 0x02U;
constexpr uint8_t kSerialCmdHeartbeat = 0x03U;
constexpr uint8_t kSerialResponseFlag = 0x80U;

constexpr uint8_t kSerialStatusOk = 0x00U;
constexpr uint8_t kSerialStatusError = 0x01U;

enum Register : uint8_t {
    REG_DEVICE_ID = 0x00U,
    REG_FW_VERSION = 0x01U,
    REG_STATUS = 0x02U,
    REG_FAULT_FLAGS = 0x03U,
    REG_CONTROL_FLAGS = 0x04U,
    REG_I2C_ADDRESS = 0x05U,

    REG_M1_MODE = 0x10U,
    REG_M1_DUTY = 0x11U,
    REG_M1_DIRECTION = 0x12U,
    REG_M1_STATUS = 0x13U,
    REG_M1_ENCODER_COUNT_0 = 0x14U,
    REG_M1_ENCODER_COUNT_1 = 0x15U,
    REG_M1_ENCODER_COUNT_2 = 0x16U,
    REG_M1_ENCODER_COUNT_3 = 0x17U,
    REG_M1_ENCODER_CPS_0 = 0x18U,
    REG_M1_ENCODER_CPS_1 = 0x19U,
    REG_M1_ENCODER_CPS_2 = 0x1AU,
    REG_M1_ENCODER_CPS_3 = 0x1BU,
    REG_M1_ENCODER_STATE = 0x1CU,

    REG_M2_MODE = 0x20U,
    REG_M2_DUTY = 0x21U,
    REG_M2_DIRECTION = 0x22U,
    REG_M2_STATUS = 0x23U,
    REG_M2_ENCODER_COUNT_0 = 0x24U,
    REG_M2_ENCODER_COUNT_1 = 0x25U,
    REG_M2_ENCODER_COUNT_2 = 0x26U,
    REG_M2_ENCODER_COUNT_3 = 0x27U,
    REG_M2_ENCODER_CPS_0 = 0x28U,
    REG_M2_ENCODER_CPS_1 = 0x29U,
    REG_M2_ENCODER_CPS_2 = 0x2AU,
    REG_M2_ENCODER_CPS_3 = 0x2BU,
    REG_M2_ENCODER_STATE = 0x2CU,

    REG_WATCHDOG_TIMEOUT = 0x30U,
    REG_ENCODER_CONTROL = 0x31U,

    REG_M1_TARGET_RPM_0 = 0x32U,
    REG_M1_TARGET_RPM_1 = 0x33U,
    REG_M2_TARGET_RPM_0 = 0x34U,
    REG_M2_TARGET_RPM_1 = 0x35U,
    REG_M1_MEASURED_RPM_0 = 0x36U,
    REG_M1_MEASURED_RPM_1 = 0x37U,
    REG_M2_MEASURED_RPM_0 = 0x38U,
    REG_M2_MEASURED_RPM_1 = 0x39U,
    REG_SPEED_KP_Q4_4 = 0x3AU,
    REG_SPEED_KI_Q4_4 = 0x3BU,
    REG_SPEED_KD_Q4_4 = 0x3CU,
    REG_SPEED_MAX_DUTY = 0x3DU,
    REG_SPEED_MIN_DUTY = 0x3EU,

    REG_M1_COUNTS_PER_REV_0 = 0x40U,
    REG_M1_COUNTS_PER_REV_1 = 0x41U,
    REG_M1_COUNTS_PER_REV_2 = 0x42U,
    REG_M1_COUNTS_PER_REV_3 = 0x43U,
    REG_M2_COUNTS_PER_REV_0 = 0x44U,
    REG_M2_COUNTS_PER_REV_1 = 0x45U,
    REG_M2_COUNTS_PER_REV_2 = 0x46U,
    REG_M2_COUNTS_PER_REV_3 = 0x47U,
    REG_M1_HOLD_COUNT_0 = 0x48U,
    REG_M1_HOLD_COUNT_1 = 0x49U,
    REG_M1_HOLD_COUNT_2 = 0x4AU,
    REG_M1_HOLD_COUNT_3 = 0x4BU,
    REG_M2_HOLD_COUNT_0 = 0x4CU,
    REG_M2_HOLD_COUNT_1 = 0x4DU,
    REG_M2_HOLD_COUNT_2 = 0x4EU,
    REG_M2_HOLD_COUNT_3 = 0x4FU,
};

enum MotorMode : uint8_t {
    MOTOR_MODE_COAST = 0U,
    MOTOR_MODE_RUN = 1U,
    MOTOR_MODE_BRAKE = 2U,
    MOTOR_MODE_SPEED = 3U,
};

enum MotorDirection : uint8_t {
    MOTOR_DIRECTION_FORWARD = 0U,
    MOTOR_DIRECTION_REVERSE = 1U,
};

enum StatusFlags : uint8_t {
    STATUS_READY = 0x01U,
    STATUS_ENABLED = 0x02U,
    STATUS_ACTIVE = 0x04U,
    STATUS_TIMEOUT = 0x08U,
};

enum FaultFlags : uint8_t {
    FAULT_WATCHDOG_TIMEOUT = 0x01U,
};

enum ControlFlags : uint8_t {
    CONTROL_ENABLE = 0x01U,
};

enum MotorStatusFlags : uint8_t {
    MOTOR_STATUS_ACTIVE = 0x01U,
    MOTOR_STATUS_REVERSE = 0x02U,
};

enum EncoderControlFlags : uint8_t {
    ENCODER_CONTROL_RESET_M1 = 0x01U,
    ENCODER_CONTROL_RESET_M2 = 0x02U,
};

}  // namespace protocol

#endif  // PROTOCOL_MOTOR_REGISTERS_H_
