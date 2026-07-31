#include "control/chassis.h"

#include "board/board_i2c_bus.h"
#include "board/board_pins.h"
#include "config/h_config.h"
#include "drivers/i2c_diag/i2c_diag.h"

namespace gugah {
namespace {

static const uint8_t kDeviceId = 0xA5U;
static const uint8_t kRegDeviceId = 0x00U;
static const uint8_t kRegOutputInvertFlags = 0x06U;
static const uint8_t kRegM1Mode = 0x10U;
static const uint8_t kRegM1Encoder = 0x14U;
static const uint8_t kRegM2Mode = 0x20U;
static const uint8_t kRegM2Encoder = 0x24U;
static const uint8_t kRegTargetRpmPair = 0x32U;
static const uint8_t kRegMeasuredRpmPair = 0x36U;
static const uint8_t kRegSpeedPid = 0x3AU;
static const uint8_t kRegM1CountsPerRev = 0x40U;
static const uint8_t kRegM1TargetPosition = 0x50U;
static const uint8_t kRegM2TargetPosition = 0x54U;
static const uint8_t kRegPositionPid = 0x60U;
static const uint8_t kRegPositionControl = 0x68U;
static const uint8_t kRegSpeedRamp = 0x7BU;
static const uint8_t kModeCoast = 0U;
static const uint8_t kModeSpeed = 3U;
static const uint8_t kModePosition = 4U;
static const uint8_t kPositionMinDuty = 6U;
static const uint8_t kPositionMaxDuty = 10U;
static const uint16_t kPositionExitToleranceCounts = 5U;
static const uint8_t kPositionSettle10Ms = 0U;

const drivers::I2cDiagBusConfig *g_bus = 0;
ChassisFeedback g_feedback = {};
bool g_ready = false;
uint32_t g_error_count = 0U;

uint16_t Magnitude(int16_t value)
{
    const int32_t widened = value;
    return static_cast<uint16_t>(
        (widened < 0) ? -widened : widened);
}

void Encode16(uint16_t value, uint8_t *out)
{
    out[0] = static_cast<uint8_t>(value & 0xFFU);
    out[1] = static_cast<uint8_t>((value >> 8U) & 0xFFU);
}

void Encode32(int32_t value, uint8_t *out)
{
    const uint32_t encoded = static_cast<uint32_t>(value);
    out[0] = static_cast<uint8_t>(encoded & 0xFFU);
    out[1] = static_cast<uint8_t>((encoded >> 8U) & 0xFFU);
    out[2] = static_cast<uint8_t>((encoded >> 16U) & 0xFFU);
    out[3] = static_cast<uint8_t>((encoded >> 24U) & 0xFFU);
}

int16_t Decode16(const uint8_t *data)
{
    return static_cast<int16_t>(
        static_cast<uint16_t>(data[0]) |
        (static_cast<uint16_t>(data[1]) << 8U));
}

int32_t Decode32(const uint8_t *data)
{
    return static_cast<int32_t>(
        static_cast<uint32_t>(data[0]) |
        (static_cast<uint32_t>(data[1]) << 8U) |
        (static_cast<uint32_t>(data[2]) << 16U) |
        (static_cast<uint32_t>(data[3]) << 24U));
}

drivers::DriverStatus Write(uint8_t reg,
                            const uint8_t *data,
                            uint16_t length)
{
    if (g_bus == 0) {
        return drivers::DRIVER_ERROR_NOT_INITIALIZED;
    }
    const drivers::DriverStatus status =
        drivers::I2cDiag_WriteReg8Block(
            g_bus, BOARD_MOTOR_I2C_ADDRESS, reg, data, length);
    if (status != drivers::DRIVER_OK) {
        g_error_count++;
    }
    return status;
}

drivers::DriverStatus ConfigureMode(uint8_t reg,
                                    uint8_t mode,
                                    bool reverse)
{
    const uint8_t block[3] = {
        mode, 0U, static_cast<uint8_t>(reverse ? 1U : 0U)
    };
    return Write(reg, block, sizeof(block));
}

drivers::DriverStatus StopBoth(void)
{
    const uint8_t stop[2] = { kModeCoast, 0U };
    drivers::DriverStatus status = Write(kRegM1Mode, stop, sizeof(stop));
    const drivers::DriverStatus second =
        Write(kRegM2Mode, stop, sizeof(stop));
    return (status != drivers::DRIVER_OK) ? status : second;
}

drivers::DriverStatus ApplyMotorConfig(const HConfig *config)
{
    if ((config == 0) || !HConfig_Validate(config)) {
        return drivers::DRIVER_ERROR_INVALID_ARG;
    }

    const uint8_t invert[2] = {
        config->motor_output_invert_flags,
        config->motor_encoder_invert_flags
    };
    drivers::DriverStatus status =
        Write(kRegOutputInvertFlags, invert, sizeof(invert));

    const uint8_t speed_pid[5] = {
        config->motor_speed_kp_q4_4,
        config->motor_speed_ki_q4_4,
        config->motor_speed_kd_q4_4,
        config->motor_speed_max_duty,
        config->motor_speed_min_duty
    };
    if (status == drivers::DRIVER_OK) {
        status = Write(kRegSpeedPid, speed_pid, sizeof(speed_pid));
    }

    uint8_t counts[8] = {};
    /* Physical mapping: MotorDriver M1=right, M2=left. */
    Encode32(static_cast<int32_t>(config->right_counts_per_rev), &counts[0]);
    Encode32(static_cast<int32_t>(config->left_counts_per_rev), &counts[4]);
    if (status == drivers::DRIVER_OK) {
        status = Write(kRegM1CountsPerRev, counts, sizeof(counts));
    }

    uint8_t position_pid[7] = {
        config->motor_position_kp_q4_4,
        config->motor_position_ki_q4_4,
        config->motor_position_kd_q4_4,
        0U, 0U, 0U, 0U
    };
    Encode16(config->motor_position_max_rpm, &position_pid[3]);
    Encode16(config->motor_position_tolerance_counts, &position_pid[5]);
    if (status == drivers::DRIVER_OK) {
        status = Write(kRegPositionPid, position_pid, sizeof(position_pid));
    }

    uint8_t position_control[5] = {
        kPositionMinDuty,
        kPositionMaxDuty,
        0U,
        0U,
        kPositionSettle10Ms
    };
    Encode16(kPositionExitToleranceCounts, &position_control[2]);
    if (status == drivers::DRIVER_OK) {
        status = Write(
            kRegPositionControl, position_control, sizeof(position_control));
    }

    uint8_t speed_ramp[4] = {};
    Encode16(config->motor_speed_accel_rpm_s, &speed_ramp[0]);
    Encode16(config->motor_speed_decel_rpm_s, &speed_ramp[2]);
    if (status == drivers::DRIVER_OK) {
        status = Write(kRegSpeedRamp, speed_ramp, sizeof(speed_ramp));
    }
    return status;
}

} /* namespace */

drivers::DriverStatus Chassis_Init(const HConfig *config, uint32_t now_ms)
{
    (void)now_ms;
    g_ready = false;
    g_feedback = {};
    g_bus = board::Board_I2cBusFind("motor");
    if (g_bus == 0) {
        return drivers::DRIVER_ERROR_NOT_INITIALIZED;
    }
    uint8_t id = 0U;
    drivers::DriverStatus status =
        drivers::I2cDiag_ReadReg8(
            g_bus, BOARD_MOTOR_I2C_ADDRESS, kRegDeviceId, &id, 1U);
    if ((status != drivers::DRIVER_OK) || (id != kDeviceId)) {
        g_error_count++;
        return (status != drivers::DRIVER_OK) ?
            status : drivers::DRIVER_ERROR;
    }
    status = StopBoth();
    if (status == drivers::DRIVER_OK) {
        status = ApplyMotorConfig(config);
    }
    if (status != drivers::DRIVER_OK) {
        (void)StopBoth();
    }
    g_ready = (status == drivers::DRIVER_OK);
    return status;
}

drivers::DriverStatus Chassis_SetWheelRpm(int16_t left_rpm,
                                          int16_t right_rpm,
                                          uint32_t now_ms)
{
    (void)now_ms;
    if (!g_ready) {
        return drivers::DRIVER_ERROR_NOT_INITIALIZED;
    }
    /* Physical wiring: left wheel=M2, right wheel=M1. */
    drivers::DriverStatus status =
        ConfigureMode(kRegM1Mode, kModeSpeed, right_rpm < 0);
    if (status == drivers::DRIVER_OK) {
        status = ConfigureMode(kRegM2Mode, kModeSpeed, left_rpm < 0);
    }
    uint8_t targets[4] = {};
    Encode16(Magnitude(right_rpm), &targets[0]);
    Encode16(Magnitude(left_rpm), &targets[2]);
    if (status == drivers::DRIVER_OK) {
        status = Write(kRegTargetRpmPair, targets, sizeof(targets));
    }
    return status;
}

drivers::DriverStatus Chassis_SetWheelPosition(int32_t left_count,
                                               int32_t right_count,
                                               uint32_t now_ms)
{
    (void)now_ms;
    if (!g_ready) {
        return drivers::DRIVER_ERROR_NOT_INITIALIZED;
    }
    uint8_t encoded[4] = {};
    Encode32(right_count, encoded);
    drivers::DriverStatus status =
        Write(kRegM1TargetPosition, encoded, sizeof(encoded));
    Encode32(left_count, encoded);
    if (status == drivers::DRIVER_OK) {
        status = Write(kRegM2TargetPosition, encoded, sizeof(encoded));
    }
    if (status == drivers::DRIVER_OK) {
        status = ConfigureMode(kRegM1Mode, kModePosition, false);
    }
    if (status == drivers::DRIVER_OK) {
        status = ConfigureMode(kRegM2Mode, kModePosition, false);
    }
    return status;
}

drivers::DriverStatus Chassis_Stop(uint32_t now_ms)
{
    (void)now_ms;
    return g_ready ? StopBoth() : drivers::DRIVER_ERROR_NOT_INITIALIZED;
}

drivers::DriverStatus Chassis_Update(uint32_t now_ms)
{
    if (!g_ready || (g_bus == 0)) {
        return drivers::DRIVER_ERROR_NOT_INITIALIZED;
    }
    uint8_t m1_encoder[4] = {};
    uint8_t m2_encoder[4] = {};
    uint8_t rpm[4] = {};
    drivers::DriverStatus status = drivers::I2cDiag_ReadReg8(
        g_bus, BOARD_MOTOR_I2C_ADDRESS,
        kRegM1Encoder, m1_encoder, sizeof(m1_encoder));
    if (status == drivers::DRIVER_OK) {
        status = drivers::I2cDiag_ReadReg8(
            g_bus, BOARD_MOTOR_I2C_ADDRESS,
            kRegM2Encoder, m2_encoder, sizeof(m2_encoder));
    }
    if (status == drivers::DRIVER_OK) {
        status = drivers::I2cDiag_ReadReg8(
            g_bus, BOARD_MOTOR_I2C_ADDRESS,
            kRegMeasuredRpmPair, rpm, sizeof(rpm));
    }
    if (status != drivers::DRIVER_OK) {
        g_error_count++;
        return status;
    }
    g_feedback.valid = true;
    g_feedback.right_encoder_count = Decode32(m1_encoder);
    g_feedback.left_encoder_count = Decode32(m2_encoder);
    g_feedback.right_rpm = Decode16(&rpm[0]);
    g_feedback.left_rpm = Decode16(&rpm[2]);
    g_feedback.received_ms = now_ms;
    return drivers::DRIVER_OK;
}

ChassisFeedback Chassis_GetFeedback(void)
{
    return g_feedback;
}

bool Chassis_IsReady(void)
{
    return g_ready;
}

uint32_t Chassis_GetErrorCount(void)
{
    return g_error_count;
}

} /* namespace gugah */
