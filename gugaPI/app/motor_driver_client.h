#ifndef APP_MOTOR_DRIVER_CLIENT_H_
#define APP_MOTOR_DRIVER_CLIENT_H_

#include <stdint.h>

#include "board/board_i2c_bus.h"
#include "board/board_motor_driver.h"
#include "board/board_pins.h"
#include "drivers/common/driver_status.h"
#include "drivers/i2c_diag/i2c_diag.h"

namespace app {
namespace motor_driver_client {

static const uint8_t kSof = 0xAAU;
static const uint8_t kCmdRead = 0x01U;
static const uint8_t kCmdWrite = 0x02U;
static const uint8_t kCmdHeartbeat = 0x03U;
static const uint8_t kResponseFlag = 0x80U;
static const uint8_t kStatusOk = 0x00U;
static const uint8_t kSpeedControlTelemetryMinFirmwareVersion = 0x02U;
static const uint8_t kMaxPayloadLength = 32U;
static const uint32_t kResponseWaitIterations = 5000000U;
static const char * const kI2cBusName = "motor";
static const uint8_t kI2cDefaultAddress = BOARD_MOTOR_DRIVER_I2C_ADDRESS;

static const uint8_t kRegDeviceId = 0x00U;
static const uint8_t kRegOutputInvertFlags = 0x06U;
static const uint8_t kRegEncoderInvertFlags = 0x07U;
static const uint8_t kRegM1Mode = 0x10U;
static const uint8_t kRegM1Encoder = 0x14U;
static const uint8_t kRegM2Mode = 0x20U;
static const uint8_t kRegM2Encoder = 0x24U;
static const uint8_t kRegEncoderControl = 0x31U;
static const uint8_t kRegM1TargetRpm = 0x32U;
static const uint8_t kRegM2TargetRpm = 0x34U;
static const uint8_t kRegSpeedRpmBlock = 0x32U;
static const uint8_t kRegSpeedPid = 0x3AU;
static const uint8_t kRegCountsPerRev = 0x40U;
static const uint8_t kRegM1TargetPosition = 0x50U;
static const uint8_t kRegM2TargetPosition = 0x54U;
static const uint8_t kRegPositionBlock = 0x50U;
static const uint8_t kRegPositionPid = 0x60U;
static const uint8_t kRegPositionControl = 0x68U;
static const uint8_t kRegSpeedControlTelemetry = 0x6DU;
static const uint8_t kRegSpeedRamp = 0x7BU;

static const uint8_t kDeviceInfoLength = 6U;
static const uint8_t kInvertConfigLength = 2U;
static const uint8_t kEncoderBlockLength = 9U;
static const uint8_t kSpeedRpmBlockLength = 8U;
static const uint8_t kSpeedPidLength = 5U;
static const uint8_t kCountsPerRevLength = 8U;
static const uint8_t kPositionBlockLength = 29U;
static const uint8_t kPositionPidLength = 7U;
static const uint8_t kPositionControlLength = 5U;
static const uint8_t kSpeedControlTelemetryLength = 14U;
static const uint8_t kSpeedRampLength = 4U;

static const uint8_t kModeCoast = 0U;
static const uint8_t kModeRun = 1U;
static const uint8_t kModeBrake = 2U;
static const uint8_t kModeSpeed = 3U;
static const uint8_t kModePosition = 4U;

static const uint8_t kInvertM1 = 0x01U;
static const uint8_t kInvertM2 = 0x02U;
static const uint8_t kInvertValidMask = kInvertM1 | kInvertM2;

static const uint32_t kSpeedMaxRpm = 1000U;
static const uint32_t kSpeedRampMaxRpmPerSecond = 65535U;
static const uint32_t kCountsPerRevMax = 100000000U;
static const uint32_t kPositionMaxRpm = 1000U;
static const uint32_t kPositionToleranceMax = 65535U;
static const uint32_t kPositionSettleMsMax = 2550U;
static const int32_t kPositionDegreeLimit = 360000;
static const uint8_t kI2cWriteVerifyAttempts = 5U;
static const uint32_t kI2cWriteVerifyDelayCycles = 8000U;

enum Transport : uint8_t {
    TRANSPORT_UART = 0U,
    TRANSPORT_I2C = 1U,
};

struct Client {
    Transport transport;
    uint8_t i2c_address;
};

struct Frame {
    uint8_t cmd;
    uint8_t reg;
    uint8_t length;
    uint8_t data[kMaxPayloadLength];
};

struct DeviceInfo {
    uint8_t device_id;
    uint8_t firmware_version;
    uint8_t status;
    uint8_t fault_flags;
    uint8_t control_flags;
    uint8_t i2c_address;
};

struct EncoderData {
    int32_t count;
    int32_t counts_per_second;
    uint8_t state;
};

struct RpmData {
    uint16_t target_m1;
    uint16_t target_m2;
    int16_t actual_m1;
    int16_t actual_m2;
};

struct CountsPerRev {
    uint32_t m1;
    uint32_t m2;
};

struct InvertConfig {
    uint8_t output_flags;
    uint8_t encoder_flags;
};

struct SpeedPid {
    uint8_t kp_q4_4;
    uint8_t ki_q4_4;
    uint8_t kd_q4_4;
    uint8_t max_duty;
    uint8_t min_duty;
};

struct SpeedControlTelemetry {
    uint16_t control_m1;
    uint16_t control_m2;
    int16_t error_m1;
    int16_t error_m2;
    int16_t integral_m1_q4;
    int16_t integral_m2_q4;
    uint8_t duty_m1;
    uint8_t duty_m2;
};

struct SpeedRamp {
    uint16_t accel_rpm_per_s;
    uint16_t decel_rpm_per_s;
};

struct PositionPid {
    uint8_t kp_q4_4;
    uint8_t ki_q4_4;
    uint8_t kd_q4_4;
    uint16_t max_rpm;
    uint16_t tolerance_counts;
};

struct PositionControl {
    uint8_t min_duty;
    uint8_t max_duty;
    uint16_t exit_tolerance_counts;
    uint16_t settle_ms;
};

struct PositionData {
    int32_t m1_target_count;
    int32_t m2_target_count;
    int32_t m1_error_count;
    int32_t m2_error_count;
    PositionPid pid;
    uint8_t status;
    PositionControl control;
};

struct StopResult {
    drivers::DriverStatus m1_status;
    drivers::DriverStatus m2_status;
};

struct MotionResult {
    drivers::DriverStatus read_status;
    drivers::DriverStatus config_status;
    drivers::DriverStatus target_status;
    drivers::DriverStatus mode_status;
    bool target_ack;
    bool mode_ack;
    bool range_error;
};

inline uint8_t Crc8Update(uint8_t crc, uint8_t value)
{
    crc ^= value;
    for (uint8_t bit = 0U; bit < 8U; bit++) {
        if ((crc & 0x80U) != 0U) {
            crc = static_cast<uint8_t>((crc << 1U) ^ 0x07U);
        } else {
            crc = static_cast<uint8_t>(crc << 1U);
        }
    }
    return crc;
}

inline uint16_t DecodeUint16Le(const uint8_t *data)
{
    return static_cast<uint16_t>(
        static_cast<uint16_t>(data[0]) |
        (static_cast<uint16_t>(data[1]) << 8U));
}

inline int16_t DecodeInt16Le(const uint8_t *data)
{
    return static_cast<int16_t>(DecodeUint16Le(data));
}

inline uint32_t DecodeUint32Le(const uint8_t *data)
{
    return static_cast<uint32_t>(
        static_cast<uint32_t>(data[0]) |
        (static_cast<uint32_t>(data[1]) << 8U) |
        (static_cast<uint32_t>(data[2]) << 16U) |
        (static_cast<uint32_t>(data[3]) << 24U));
}

inline int32_t DecodeInt32Le(const uint8_t *data)
{
    return static_cast<int32_t>(DecodeUint32Le(data));
}

inline void EncodeUint16Le(uint16_t value, uint8_t *data)
{
    data[0] = static_cast<uint8_t>(value & 0xFFU);
    data[1] = static_cast<uint8_t>((value >> 8U) & 0xFFU);
}

inline void EncodeUint32Le(uint32_t value, uint8_t *data)
{
    data[0] = static_cast<uint8_t>(value & 0xFFU);
    data[1] = static_cast<uint8_t>((value >> 8U) & 0xFFU);
    data[2] = static_cast<uint8_t>((value >> 16U) & 0xFFU);
    data[3] = static_cast<uint8_t>((value >> 24U) & 0xFFU);
}

inline void EncodeInt32Le(int32_t value, uint8_t *data)
{
    EncodeUint32Le(static_cast<uint32_t>(value), data);
}

inline void Init(Client *client)
{
    if (client != 0) {
        client->transport = TRANSPORT_I2C;
        client->i2c_address = kI2cDefaultAddress;
    }
}

inline bool IsReady(void)
{
    return board::Board_MotorDriverIsReady();
}

inline drivers::DriverStatus SetBus(Client *client, Transport transport)
{
    if (client == 0) {
        return drivers::DRIVER_ERROR_INVALID_ARG;
    }
    client->transport = transport;
    return drivers::DRIVER_OK;
}

inline Transport GetBus(const Client *client)
{
    return (client == 0) ? TRANSPORT_UART : client->transport;
}

inline const char *TransportText(Transport transport)
{
    return (transport == TRANSPORT_I2C) ? "i2c" : "uart";
}

inline const char *TransportText(const Client *client)
{
    return TransportText(GetBus(client));
}

inline drivers::DriverStatus SetI2cAddress(Client *client, uint8_t address)
{
    if ((client == 0) ||
        (address < drivers::I2C_DIAG_MIN_7BIT_ADDRESS) ||
        (address > drivers::I2C_DIAG_MAX_7BIT_ADDRESS)) {
        return drivers::DRIVER_ERROR_INVALID_ARG;
    }
    client->i2c_address = address;
    return drivers::DRIVER_OK;
}

inline uint8_t GetI2cAddress(const Client *client)
{
    return (client == 0) ? kI2cDefaultAddress : client->i2c_address;
}

inline const drivers::I2cDiagBusConfig *GetI2cBus(void)
{
    return board::Board_I2cBusFind(kI2cBusName);
}

inline drivers::DriverStatus RawWriteByte(uint8_t data)
{
    return board::Board_MotorDriverWriteByte(data);
}

inline drivers::DriverStatus RawWrite(const uint8_t *data, uint16_t length)
{
    return board::Board_MotorDriverWrite(data, length);
}

inline bool RawReadByte(uint8_t *data)
{
    return board::Board_MotorDriverReadByte(data);
}

inline uint16_t GetRxAvailable(void)
{
    return board::Board_MotorDriverGetRxAvailable();
}

inline uint32_t GetRxDroppedCount(void)
{
    return board::Board_MotorDriverGetRxDroppedCount();
}

inline uint32_t GetBaudrate(void)
{
    return board::Board_MotorDriverGetBaudrate();
}

inline drivers::DriverStatus GetControllerStatus(uint32_t *status)
{
    return board::Board_MotorDriverGetControllerStatus(status);
}

inline drivers::DriverStatus ClearRxBuffer(void)
{
    return board::Board_MotorDriverClearRxBuffer();
}

inline drivers::DriverStatus SendFrame(uint8_t cmd,
                                       uint8_t reg,
                                       const uint8_t *data,
                                       uint8_t length)
{
    if (length > kMaxPayloadLength) {
        return drivers::DRIVER_ERROR_INVALID_ARG;
    }

    uint8_t crc = 0U;
    const uint8_t header[4] = { kSof, cmd, reg, length };

    for (uint8_t i = 0U; i < sizeof(header); i++) {
        const drivers::DriverStatus status = RawWriteByte(header[i]);
        if (status != drivers::DRIVER_OK) {
            return status;
        }
        crc = Crc8Update(crc, header[i]);
    }

    const uint8_t payload_length = (cmd == kCmdWrite) ? length : 0U;
    for (uint8_t i = 0U; i < payload_length; i++) {
        const uint8_t value = (data == 0) ? 0U : data[i];
        const drivers::DriverStatus status = RawWriteByte(value);
        if (status != drivers::DRIVER_OK) {
            return status;
        }
        crc = Crc8Update(crc, value);
    }

    return RawWriteByte(crc);
}

inline bool ReadByteWait(uint8_t *value, uint32_t wait_iterations)
{
    while (wait_iterations > 0U) {
        if (RawReadByte(value)) {
            return true;
        }
        wait_iterations--;
    }
    return false;
}

inline drivers::DriverStatus ReceiveFrame(Frame *frame)
{
    if (frame == 0) {
        return drivers::DRIVER_ERROR_INVALID_ARG;
    }

    uint8_t value = 0U;
    uint8_t crc = 0U;

    do {
        if (!ReadByteWait(&value, kResponseWaitIterations)) {
            return drivers::DRIVER_ERROR_TIMEOUT;
        }
    } while (value != kSof);

    crc = Crc8Update(0U, value);

    if (!ReadByteWait(&frame->cmd, kResponseWaitIterations) ||
        !ReadByteWait(&frame->reg, kResponseWaitIterations) ||
        !ReadByteWait(&frame->length, kResponseWaitIterations)) {
        return drivers::DRIVER_ERROR_TIMEOUT;
    }

    crc = Crc8Update(crc, frame->cmd);
    crc = Crc8Update(crc, frame->reg);
    crc = Crc8Update(crc, frame->length);

    if (frame->length > kMaxPayloadLength) {
        return drivers::DRIVER_ERROR;
    }

    for (uint8_t i = 0U; i < frame->length; i++) {
        if (!ReadByteWait(&frame->data[i], kResponseWaitIterations)) {
            return drivers::DRIVER_ERROR_TIMEOUT;
        }
        crc = Crc8Update(crc, frame->data[i]);
    }

    uint8_t received_crc = 0U;
    if (!ReadByteWait(&received_crc, kResponseWaitIterations)) {
        return drivers::DRIVER_ERROR_TIMEOUT;
    }

    return (crc == received_crc) ? drivers::DRIVER_OK : drivers::DRIVER_ERROR;
}

inline drivers::DriverStatus I2cWriteReg(Client *client,
                                         uint8_t reg,
                                         const uint8_t *data,
                                         uint8_t length)
{
    const drivers::I2cDiagBusConfig *bus = GetI2cBus();

    if ((client == 0) || (bus == 0) || (data == 0) || (length == 0U) ||
        ((uint16_t) reg + (uint16_t) length > 256U)) {
        return drivers::DRIVER_ERROR_INVALID_ARG;
    }

    return drivers::I2cDiag_WriteReg8Block(bus,
                                           client->i2c_address,
                                           reg,
                                           data,
                                           length);
}

inline drivers::DriverStatus I2cRequest(Client *client,
                                        uint8_t cmd,
                                        uint8_t reg,
                                        const uint8_t *data,
                                        uint8_t length,
                                        Frame *response)
{
    const drivers::I2cDiagBusConfig *bus = GetI2cBus();

    if ((client == 0) || (bus == 0) || (response == 0) ||
        (length > kMaxPayloadLength)) {
        return drivers::DRIVER_ERROR_INVALID_ARG;
    }

    response->cmd = static_cast<uint8_t>(cmd | kResponseFlag);
    response->reg = reg;
    response->length = 0U;

    if (cmd == kCmdRead) {
        if (length == 0U) {
            return drivers::DRIVER_ERROR_INVALID_ARG;
        }

        const drivers::DriverStatus status = drivers::I2cDiag_ReadReg8(
            bus,
            client->i2c_address,
            reg,
            response->data,
            length);
        if (status == drivers::DRIVER_OK) {
            response->length = length;
        }
        return status;
    }

    if (cmd == kCmdWrite) {
        const drivers::DriverStatus status =
            I2cWriteReg(client, reg, data, length);
        if (status != drivers::DRIVER_OK) {
            return status;
        }
        response->length = 1U;
        response->data[0] = kStatusOk;
        return drivers::DRIVER_OK;
    }

    if (cmd == kCmdHeartbeat) {
        const drivers::DriverStatus status =
            drivers::I2cDiag_ProbeAddress(bus, client->i2c_address);
        if (status != drivers::DRIVER_OK) {
            return status;
        }
        response->length = 1U;
        response->data[0] = kStatusOk;
        return drivers::DRIVER_OK;
    }

    return drivers::DRIVER_ERROR_INVALID_ARG;
}

inline drivers::DriverStatus Request(Client *client,
                                     uint8_t cmd,
                                     uint8_t reg,
                                     const uint8_t *data,
                                     uint8_t length,
                                     Frame *response)
{
    if ((client == 0) || (response == 0)) {
        return drivers::DRIVER_ERROR_INVALID_ARG;
    }

    if (client->transport == TRANSPORT_I2C) {
        return I2cRequest(client, cmd, reg, data, length, response);
    }

    (void) ClearRxBuffer();

    drivers::DriverStatus status = SendFrame(cmd, reg, data, length);
    if (status != drivers::DRIVER_OK) {
        return status;
    }

    status = ReceiveFrame(response);
    if (status != drivers::DRIVER_OK) {
        return status;
    }

    const uint8_t expected_cmd = static_cast<uint8_t>(cmd | kResponseFlag);
    if ((response->cmd != expected_cmd) || (response->reg != reg)) {
        return drivers::DRIVER_ERROR;
    }

    return drivers::DRIVER_OK;
}

inline bool StatusOkResponse(const Frame &frame)
{
    return (frame.length == 1U) && (frame.data[0] == kStatusOk);
}

inline drivers::DriverStatus ReadRegisters(Client *client,
                                           uint8_t reg,
                                           uint8_t length,
                                           Frame *response)
{
    return Request(client, kCmdRead, reg, 0, length, response);
}

inline drivers::DriverStatus WriteRegisters(Client *client,
                                            uint8_t reg,
                                            const uint8_t *data,
                                            uint8_t length,
                                            Frame *response)
{
    return Request(client, kCmdWrite, reg, data, length, response);
}

inline bool BytesEqual(const uint8_t *left,
                       const uint8_t *right,
                       uint8_t length)
{
    for (uint8_t i = 0U; i < length; i++) {
        if (left[i] != right[i]) {
            return false;
        }
    }
    return true;
}

inline drivers::DriverStatus VerifyRegisters(Client *client,
                                             uint8_t reg,
                                             const uint8_t *expected,
                                             uint8_t length)
{
    if ((client == 0) || (expected == 0) || (length == 0U) ||
        (length > kMaxPayloadLength) ||
        ((uint16_t) reg + (uint16_t) length > 256U)) {
        return drivers::DRIVER_ERROR_INVALID_ARG;
    }

    const uint8_t attempts =
        (client->transport == TRANSPORT_I2C) ? kI2cWriteVerifyAttempts : 1U;
    drivers::DriverStatus last_status = drivers::DRIVER_ERROR;

    for (uint8_t attempt = 0U; attempt < attempts; attempt++) {
        Frame verify = {};
        last_status = ReadRegisters(client, reg, length, &verify);
        if (last_status == drivers::DRIVER_OK) {
            if ((verify.length == length) &&
                BytesEqual(verify.data, expected, length)) {
                return drivers::DRIVER_OK;
            }
            last_status = drivers::DRIVER_ERROR;
        }

        if (attempt + 1U < attempts) {
            delay_cycles(kI2cWriteVerifyDelayCycles);
        }
    }

    return last_status;
}

inline drivers::DriverStatus WriteRegistersChecked(Client *client,
                                                   uint8_t reg,
                                                   const uint8_t *data,
                                                   uint8_t length,
                                                   Frame *response)
{
    if ((client == 0) || (response == 0)) {
        return drivers::DRIVER_ERROR_INVALID_ARG;
    }

    const drivers::DriverStatus status =
        WriteRegisters(client, reg, data, length, response);
    if (status != drivers::DRIVER_OK) {
        return status;
    }
    if (!StatusOkResponse(*response)) {
        return drivers::DRIVER_ERROR;
    }

    if (client->transport == TRANSPORT_I2C) {
        return VerifyRegisters(client, reg, data, length);
    }
    return drivers::DRIVER_OK;
}

inline drivers::DriverStatus Ping(Client *client, bool *ok)
{
    Frame response = {};
    const drivers::DriverStatus status =
        Request(client, kCmdHeartbeat, 0U, 0, 0U, &response);
    if (status != drivers::DRIVER_OK) {
        return status;
    }
    if (ok != 0) {
        *ok = StatusOkResponse(response);
    }
    return drivers::DRIVER_OK;
}

inline drivers::DriverStatus ReadInfo(Client *client, DeviceInfo *info)
{
    Frame response = {};
    if (info == 0) {
        return drivers::DRIVER_ERROR_INVALID_ARG;
    }

    const drivers::DriverStatus status =
        ReadRegisters(client, kRegDeviceId, kDeviceInfoLength, &response);
    if (status != drivers::DRIVER_OK) {
        return status;
    }
    if (response.length != kDeviceInfoLength) {
        return drivers::DRIVER_ERROR;
    }

    info->device_id = response.data[0];
    info->firmware_version = response.data[1];
    info->status = response.data[2];
    info->fault_flags = response.data[3];
    info->control_flags = response.data[4];
    info->i2c_address = response.data[5];
    return drivers::DRIVER_OK;
}

inline drivers::DriverStatus ReadEncoder(Client *client,
                                         bool motor1,
                                         EncoderData *encoder)
{
    Frame response = {};
    if (encoder == 0) {
        return drivers::DRIVER_ERROR_INVALID_ARG;
    }

    const drivers::DriverStatus status =
        ReadRegisters(client,
                      motor1 ? kRegM1Encoder : kRegM2Encoder,
                      kEncoderBlockLength,
                      &response);
    if (status != drivers::DRIVER_OK) {
        return status;
    }
    if (response.length != kEncoderBlockLength) {
        return drivers::DRIVER_ERROR;
    }

    encoder->count = DecodeInt32Le(&response.data[0]);
    encoder->counts_per_second = DecodeInt32Le(&response.data[4]);
    encoder->state = response.data[8];
    return drivers::DRIVER_OK;
}

inline drivers::DriverStatus ResetEncoders(Client *client)
{
    static const uint8_t kResetBothEncoders[] = { 0x03U };
    Frame response = {};

    const drivers::DriverStatus status =
        WriteRegisters(client,
                       kRegEncoderControl,
                       kResetBothEncoders,
                       static_cast<uint8_t>(sizeof(kResetBothEncoders)),
                       &response);
    if (status != drivers::DRIVER_OK) {
        return status;
    }
    return StatusOkResponse(response) ? drivers::DRIVER_OK
                                      : drivers::DRIVER_ERROR;
}

inline drivers::DriverStatus ReadRpm(Client *client, RpmData *rpm)
{
    Frame response = {};
    if (rpm == 0) {
        return drivers::DRIVER_ERROR_INVALID_ARG;
    }

    const drivers::DriverStatus status =
        ReadRegisters(client, kRegSpeedRpmBlock, kSpeedRpmBlockLength, &response);
    if (status != drivers::DRIVER_OK) {
        return status;
    }
    if (response.length != kSpeedRpmBlockLength) {
        return drivers::DRIVER_ERROR;
    }

    rpm->target_m1 = DecodeUint16Le(&response.data[0]);
    rpm->target_m2 = DecodeUint16Le(&response.data[2]);
    rpm->actual_m1 = DecodeInt16Le(&response.data[4]);
    rpm->actual_m2 = DecodeInt16Le(&response.data[6]);
    return drivers::DRIVER_OK;
}

inline drivers::DriverStatus ReadSpeedControlTelemetry(
    Client *client,
    SpeedControlTelemetry *telemetry)
{
    Frame response = {};
    if (telemetry == 0) {
        return drivers::DRIVER_ERROR_INVALID_ARG;
    }

    DeviceInfo info = {};
    drivers::DriverStatus status = ReadInfo(client, &info);
    if (status != drivers::DRIVER_OK) {
        return status;
    }
    if (info.firmware_version < kSpeedControlTelemetryMinFirmwareVersion) {
        return drivers::DRIVER_ERROR_UNSUPPORTED;
    }

    status = ReadRegisters(
        client,
        kRegSpeedControlTelemetry,
        kSpeedControlTelemetryLength,
        &response);
    if (status != drivers::DRIVER_OK) {
        return status;
    }
    if (response.length != kSpeedControlTelemetryLength) {
        return drivers::DRIVER_ERROR;
    }

    telemetry->control_m1 = DecodeUint16Le(&response.data[0]);
    telemetry->control_m2 = DecodeUint16Le(&response.data[2]);
    telemetry->error_m1 = DecodeInt16Le(&response.data[4]);
    telemetry->error_m2 = DecodeInt16Le(&response.data[6]);
    telemetry->integral_m1_q4 = DecodeInt16Le(&response.data[8]);
    telemetry->integral_m2_q4 = DecodeInt16Le(&response.data[10]);
    telemetry->duty_m1 = response.data[12];
    telemetry->duty_m2 = response.data[13];
    return drivers::DRIVER_OK;
}

inline drivers::DriverStatus ReadSpeedRamp(Client *client, SpeedRamp *ramp)
{
    Frame response = {};
    if (ramp == 0) {
        return drivers::DRIVER_ERROR_INVALID_ARG;
    }

    DeviceInfo info = {};
    drivers::DriverStatus status = ReadInfo(client, &info);
    if (status != drivers::DRIVER_OK) {
        return status;
    }
    if (info.firmware_version < kSpeedControlTelemetryMinFirmwareVersion) {
        return drivers::DRIVER_ERROR_UNSUPPORTED;
    }

    status = ReadRegisters(client, kRegSpeedRamp, kSpeedRampLength, &response);
    if (status != drivers::DRIVER_OK) {
        return status;
    }
    if (response.length != kSpeedRampLength) {
        return drivers::DRIVER_ERROR;
    }

    ramp->accel_rpm_per_s = DecodeUint16Le(&response.data[0]);
    ramp->decel_rpm_per_s = DecodeUint16Le(&response.data[2]);
    return drivers::DRIVER_OK;
}

inline drivers::DriverStatus SetSpeedRamp(Client *client,
                                          const SpeedRamp &ramp)
{
    Frame response = {};
    DeviceInfo info = {};
    drivers::DriverStatus status = ReadInfo(client, &info);
    if (status != drivers::DRIVER_OK) {
        return status;
    }
    if (info.firmware_version < kSpeedControlTelemetryMinFirmwareVersion) {
        return drivers::DRIVER_ERROR_UNSUPPORTED;
    }

    uint8_t data[kSpeedRampLength] = { 0U, 0U, 0U, 0U };
    EncodeUint16Le(ramp.accel_rpm_per_s, &data[0]);
    EncodeUint16Le(ramp.decel_rpm_per_s, &data[2]);

    status = WriteRegistersChecked(client,
                                   kRegSpeedRamp,
                                   data,
                                   kSpeedRampLength,
                                   &response);
    return status;
}

inline drivers::DriverStatus ReadCountsPerRev(Client *client,
                                              CountsPerRev *counts)
{
    Frame response = {};
    if (counts == 0) {
        return drivers::DRIVER_ERROR_INVALID_ARG;
    }

    const drivers::DriverStatus status =
        ReadRegisters(client,
                      kRegCountsPerRev,
                      kCountsPerRevLength,
                      &response);
    if (status != drivers::DRIVER_OK) {
        return status;
    }
    if (response.length != kCountsPerRevLength) {
        return drivers::DRIVER_ERROR;
    }

    counts->m1 = DecodeUint32Le(&response.data[0]);
    counts->m2 = DecodeUint32Le(&response.data[4]);
    return drivers::DRIVER_OK;
}

inline drivers::DriverStatus SetCountsPerRev(Client *client,
                                             uint32_t m1_counts_per_rev,
                                             uint32_t m2_counts_per_rev)
{
    Frame response = {};
    uint8_t data[kCountsPerRevLength] = { 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U };

    EncodeUint32Le(m1_counts_per_rev, &data[0]);
    EncodeUint32Le(m2_counts_per_rev, &data[4]);

    const drivers::DriverStatus status =
        WriteRegistersChecked(client,
                              kRegCountsPerRev,
                              data,
                              kCountsPerRevLength,
                              &response);
    return status;
}

inline drivers::DriverStatus ReadInvertConfig(Client *client,
                                              InvertConfig *config)
{
    Frame response = {};
    if (config == 0) {
        return drivers::DRIVER_ERROR_INVALID_ARG;
    }

    const drivers::DriverStatus status =
        ReadRegisters(client,
                      kRegOutputInvertFlags,
                      kInvertConfigLength,
                      &response);
    if (status != drivers::DRIVER_OK) {
        return status;
    }
    if (response.length != kInvertConfigLength) {
        return drivers::DRIVER_ERROR;
    }

    config->output_flags = static_cast<uint8_t>(
        response.data[0] & kInvertValidMask);
    config->encoder_flags = static_cast<uint8_t>(
        response.data[1] & kInvertValidMask);
    return drivers::DRIVER_OK;
}

inline drivers::DriverStatus SetInvertConfig(Client *client,
                                             const InvertConfig &config)
{
    Frame response = {};
    uint8_t data[kInvertConfigLength] = {
        static_cast<uint8_t>(config.output_flags & kInvertValidMask),
        static_cast<uint8_t>(config.encoder_flags & kInvertValidMask),
    };

    const drivers::DriverStatus status =
        WriteRegistersChecked(client,
                              kRegOutputInvertFlags,
                              data,
                              kInvertConfigLength,
                              &response);
    return status;
}

inline drivers::DriverStatus ReadPid(Client *client, SpeedPid *pid)
{
    Frame response = {};
    if (pid == 0) {
        return drivers::DRIVER_ERROR_INVALID_ARG;
    }

    const drivers::DriverStatus status =
        ReadRegisters(client, kRegSpeedPid, kSpeedPidLength, &response);
    if (status != drivers::DRIVER_OK) {
        return status;
    }
    if (response.length != kSpeedPidLength) {
        return drivers::DRIVER_ERROR;
    }

    pid->kp_q4_4 = response.data[0];
    pid->ki_q4_4 = response.data[1];
    pid->kd_q4_4 = response.data[2];
    pid->max_duty = response.data[3];
    pid->min_duty = response.data[4];
    return drivers::DRIVER_OK;
}

inline drivers::DriverStatus SetPid(Client *client,
                                    const uint8_t *pid_bytes,
                                    uint8_t length)
{
    Frame response = {};
    const drivers::DriverStatus status =
        WriteRegistersChecked(client, kRegSpeedPid, pid_bytes, length,
                              &response);
    return status;
}

inline void DecodePositionPid(const uint8_t *data, PositionPid *pid)
{
    pid->kp_q4_4 = data[0];
    pid->ki_q4_4 = data[1];
    pid->kd_q4_4 = data[2];
    pid->max_rpm = DecodeUint16Le(&data[3]);
    pid->tolerance_counts = DecodeUint16Le(&data[5]);
}

inline void DecodePositionControl(const uint8_t *data, PositionControl *control)
{
    control->min_duty = data[0];
    control->max_duty = data[1];
    control->exit_tolerance_counts = DecodeUint16Le(&data[2]);
    control->settle_ms = static_cast<uint16_t>(
        static_cast<uint16_t>(data[4]) * 10U);
}

inline drivers::DriverStatus ReadPosition(Client *client, PositionData *position)
{
    Frame response = {};
    if (position == 0) {
        return drivers::DRIVER_ERROR_INVALID_ARG;
    }

    const drivers::DriverStatus status =
        ReadRegisters(client,
                      kRegPositionBlock,
                      kPositionBlockLength,
                      &response);
    if (status != drivers::DRIVER_OK) {
        return status;
    }
    if (response.length != kPositionBlockLength) {
        return drivers::DRIVER_ERROR;
    }

    position->m1_target_count = DecodeInt32Le(&response.data[0]);
    position->m2_target_count = DecodeInt32Le(&response.data[4]);
    position->m1_error_count = DecodeInt32Le(&response.data[8]);
    position->m2_error_count = DecodeInt32Le(&response.data[12]);
    DecodePositionPid(&response.data[16], &position->pid);
    position->status = response.data[23];
    DecodePositionControl(&response.data[24], &position->control);
    return drivers::DRIVER_OK;
}

inline drivers::DriverStatus ReadPositionPid(Client *client, PositionPid *pid)
{
    Frame response = {};
    if (pid == 0) {
        return drivers::DRIVER_ERROR_INVALID_ARG;
    }

    const drivers::DriverStatus status =
        ReadRegisters(client,
                      kRegPositionPid,
                      kPositionPidLength,
                      &response);
    if (status != drivers::DRIVER_OK) {
        return status;
    }
    if (response.length != kPositionPidLength) {
        return drivers::DRIVER_ERROR;
    }

    DecodePositionPid(response.data, pid);
    return drivers::DRIVER_OK;
}

inline drivers::DriverStatus SetPositionPid(Client *client,
                                            const uint8_t *pid_bytes,
                                            uint8_t length)
{
    Frame response = {};
    const drivers::DriverStatus status =
        WriteRegistersChecked(client, kRegPositionPid, pid_bytes, length,
                              &response);
    return status;
}

inline drivers::DriverStatus ReadPositionControl(Client *client,
                                                 PositionControl *control)
{
    Frame response = {};
    if (control == 0) {
        return drivers::DRIVER_ERROR_INVALID_ARG;
    }

    const drivers::DriverStatus status =
        ReadRegisters(client,
                      kRegPositionControl,
                      kPositionControlLength,
                      &response);
    if (status != drivers::DRIVER_OK) {
        return status;
    }
    if (response.length != kPositionControlLength) {
        return drivers::DRIVER_ERROR;
    }

    DecodePositionControl(response.data, control);
    return drivers::DRIVER_OK;
}

inline drivers::DriverStatus SetPositionControl(Client *client,
                                                const uint8_t *control_bytes,
                                                uint8_t length)
{
    Frame response = {};
    const drivers::DriverStatus status =
        WriteRegistersChecked(client,
                              kRegPositionControl,
                              control_bytes,
                              length,
                              &response);
    return status;
}

inline drivers::DriverStatus WriteMode(Client *client,
                                       bool motor1,
                                       const uint8_t *data,
                                       uint8_t length,
                                       bool *ack)
{
    Frame response = {};
    const drivers::DriverStatus status =
        WriteRegistersChecked(client,
                              motor1 ? kRegM1Mode : kRegM2Mode,
                              data,
                              length,
                              &response);
    if (ack != 0) {
        *ack = (status == drivers::DRIVER_OK);
    }
    return status;
}

inline drivers::DriverStatus WriteTargetRpm(Client *client,
                                            bool motor1,
                                            uint16_t rpm,
                                            bool *ack)
{
    Frame response = {};
    uint8_t data[2] = { 0U, 0U };
    EncodeUint16Le(rpm, data);

    const drivers::DriverStatus status =
        WriteRegistersChecked(client,
                              motor1 ? kRegM1TargetRpm : kRegM2TargetRpm,
                              data,
                              static_cast<uint8_t>(sizeof(data)),
                              &response);
    if (ack != 0) {
        *ack = (status == drivers::DRIVER_OK);
    }
    return status;
}

inline drivers::DriverStatus RefreshTargetRpmLease(Client *client,
                                                   bool motor1,
                                                   uint16_t rpm)
{
    Frame response = {};
    uint8_t data[2] = { 0U, 0U };
    EncodeUint16Le(rpm, data);

    const drivers::DriverStatus status =
        WriteRegisters(client,
                       motor1 ? kRegM1TargetRpm : kRegM2TargetRpm,
                       data,
                       static_cast<uint8_t>(sizeof(data)),
                       &response);
    if (status != drivers::DRIVER_OK) {
        return status;
    }
    return StatusOkResponse(response) ? drivers::DRIVER_OK :
                                       drivers::DRIVER_ERROR;
}

inline drivers::DriverStatus WriteTargetPosition(Client *client,
                                                 bool motor1,
                                                 int32_t target_count,
                                                 bool *ack)
{
    Frame response = {};
    uint8_t data[4] = { 0U, 0U, 0U, 0U };
    EncodeInt32Le(target_count, data);

    const drivers::DriverStatus status =
        WriteRegistersChecked(client,
                              motor1 ? kRegM1TargetPosition :
                                       kRegM2TargetPosition,
                              data,
                              static_cast<uint8_t>(sizeof(data)),
                              &response);
    if (ack != 0) {
        *ack = (status == drivers::DRIVER_OK);
    }
    return status;
}

inline void InitMotionResult(MotionResult *result)
{
    if (result != 0) {
        result->read_status = drivers::DRIVER_OK;
        result->config_status = drivers::DRIVER_OK;
        result->target_status = drivers::DRIVER_OK;
        result->mode_status = drivers::DRIVER_OK;
        result->target_ack = true;
        result->mode_ack = true;
        result->range_error = false;
    }
}

inline drivers::DriverStatus SetRun(Client *client,
                                    bool motor1,
                                    uint8_t duty,
                                    bool reverse,
                                    MotionResult *result)
{
    uint8_t data[3] = {
        kModeRun,
        duty,
        static_cast<uint8_t>(reverse ? 1U : 0U)
    };
    InitMotionResult(result);

    bool ack = false;
    const drivers::DriverStatus status =
        WriteMode(client, motor1, data, static_cast<uint8_t>(sizeof(data)),
                  &ack);
    if (result != 0) {
        result->mode_status = status;
        result->mode_ack = ack;
    }
    return status;
}

inline drivers::DriverStatus SetCoast(Client *client,
                                      bool motor1,
                                      MotionResult *result)
{
    uint8_t data[2] = { kModeCoast, 0U };
    InitMotionResult(result);

    bool ack = false;
    const drivers::DriverStatus status =
        WriteMode(client, motor1, data, static_cast<uint8_t>(sizeof(data)),
                  &ack);
    if (result != 0) {
        result->mode_status = status;
        result->mode_ack = ack;
    }
    return status;
}

inline drivers::DriverStatus SetBrake(Client *client,
                                      bool motor1,
                                      MotionResult *result)
{
    uint8_t data[2] = { kModeBrake, 0U };
    InitMotionResult(result);

    bool ack = false;
    const drivers::DriverStatus status =
        WriteMode(client, motor1, data, static_cast<uint8_t>(sizeof(data)),
                  &ack);
    if (result != 0) {
        result->mode_status = status;
        result->mode_ack = ack;
    }
    return status;
}

inline drivers::DriverStatus SetSpeed(Client *client,
                                      bool motor1,
                                      uint16_t rpm,
                                      bool reverse,
                                      MotionResult *result)
{
    uint8_t mode[3] = {
        kModeSpeed,
        0U,
        static_cast<uint8_t>(reverse ? 1U : 0U)
    };
    InitMotionResult(result);

    bool target_ack = false;
    drivers::DriverStatus status =
        WriteTargetRpm(client, motor1, rpm, &target_ack);
    if (result != 0) {
        result->target_status = status;
        result->target_ack = target_ack;
    }
    if (status != drivers::DRIVER_OK) {
        return status;
    }

    bool mode_ack = false;
    status = WriteMode(client,
                       motor1,
                       mode,
                       static_cast<uint8_t>(sizeof(mode)),
                       &mode_ack);
    if (result != 0) {
        result->mode_status = status;
        result->mode_ack = mode_ack;
    }
    return status;
}

inline bool DegreesToCounts(int32_t degrees,
                            uint32_t counts_per_rev,
                            int32_t *counts)
{
    if ((counts == 0) || (counts_per_rev == 0U)) {
        return false;
    }

    int64_t numerator =
        static_cast<int64_t>(degrees) * static_cast<int64_t>(counts_per_rev);
    if (numerator >= 0) {
        numerator += 180;
    } else {
        numerator -= 180;
    }

    const int64_t value = numerator / 360;
    if ((value > 2147483647LL) || (value < (-2147483647LL - 1LL))) {
        return false;
    }

    *counts = static_cast<int32_t>(value);
    return true;
}

inline drivers::DriverStatus SetPositionCounts(Client *client,
                                               bool motor1,
                                               int32_t target_count,
                                               MotionResult *result)
{
    uint8_t mode[3] = { kModePosition, 0U, 0U };
    InitMotionResult(result);

    bool target_ack = false;
    drivers::DriverStatus status =
        WriteTargetPosition(client, motor1, target_count, &target_ack);
    if (result != 0) {
        result->target_status = status;
        result->target_ack = target_ack;
    }
    if (status != drivers::DRIVER_OK) {
        return status;
    }

    bool mode_ack = false;
    status = WriteMode(client,
                       motor1,
                       mode,
                       static_cast<uint8_t>(sizeof(mode)),
                       &mode_ack);
    if (result != 0) {
        result->mode_status = status;
        result->mode_ack = mode_ack;
    }
    return status;
}

inline drivers::DriverStatus Hold(Client *client,
                                  bool motor1,
                                  MotionResult *result)
{
    EncoderData encoder = {};
    InitMotionResult(result);

    drivers::DriverStatus status = ReadEncoder(client, motor1, &encoder);
    if (result != 0) {
        result->read_status = status;
    }
    if (status != drivers::DRIVER_OK) {
        return status;
    }

    return SetPositionCounts(client, motor1, encoder.count, result);
}

inline drivers::DriverStatus SetPosition(Client *client,
                                         bool motor1,
                                         int32_t degrees,
                                         MotionResult *result)
{
    CountsPerRev counts = {};
    int32_t target_count = 0;
    InitMotionResult(result);

    drivers::DriverStatus status = ReadCountsPerRev(client, &counts);
    if (result != 0) {
        result->config_status = status;
    }
    if (status != drivers::DRIVER_OK) {
        return status;
    }

    if (!DegreesToCounts(degrees, motor1 ? counts.m1 : counts.m2,
                         &target_count)) {
        if (result != 0) {
            result->range_error = true;
        }
        return drivers::DRIVER_ERROR_INVALID_ARG;
    }

    return SetPositionCounts(client, motor1, target_count, result);
}

inline drivers::DriverStatus SetPositionRelative(Client *client,
                                                 bool motor1,
                                                 int32_t degrees,
                                                 MotionResult *result)
{
    CountsPerRev counts = {};
    EncoderData encoder = {};
    int32_t delta_count = 0;
    InitMotionResult(result);

    drivers::DriverStatus status = ReadCountsPerRev(client, &counts);
    if (result != 0) {
        result->config_status = status;
    }
    if (status != drivers::DRIVER_OK) {
        return status;
    }

    if (!DegreesToCounts(degrees, motor1 ? counts.m1 : counts.m2,
                         &delta_count)) {
        if (result != 0) {
            result->range_error = true;
        }
        return drivers::DRIVER_ERROR_INVALID_ARG;
    }

    status = ReadEncoder(client, motor1, &encoder);
    if (result != 0) {
        result->read_status = status;
    }
    if (status != drivers::DRIVER_OK) {
        return status;
    }

    const int64_t target =
        static_cast<int64_t>(encoder.count) + static_cast<int64_t>(delta_count);
    if ((target > 2147483647LL) || (target < (-2147483647LL - 1LL))) {
        if (result != 0) {
            result->range_error = true;
        }
        return drivers::DRIVER_ERROR_INVALID_ARG;
    }

    return SetPositionCounts(client,
                             motor1,
                             static_cast<int32_t>(target),
                             result);
}

inline drivers::DriverStatus Stop(Client *client, StopResult *result)
{
    StopResult local_result = { drivers::DRIVER_OK, drivers::DRIVER_OK };
    MotionResult motion = {};

    local_result.m1_status = SetCoast(client, true, &motion);
    local_result.m2_status = SetCoast(client, false, &motion);

    if (result != 0) {
        *result = local_result;
    }

    return (local_result.m1_status != drivers::DRIVER_OK) ?
           local_result.m1_status : local_result.m2_status;
}

} /* namespace motor_driver_client */
} /* namespace app */

#endif /* APP_MOTOR_DRIVER_CLIENT_H_ */
