#include "app/app_shell.h"

#include <stdint.h>

#include "board/board_buzzer.h"
#include "board/board_button.h"
#include "board/board_config.h"
#include "board/board_fram.h"
#include "board/board_ina219.h"
#include "board/board_imu.h"
#include "board/board_i2c_bus.h"
#include "board/board_led.h"
#include "board/board_lora.h"
#include "board/board_motor_driver.h"
#include "board/board_oled.h"
#include "board/board_pins.h"
#include "config/feature_config.h"
#include "drivers/common/driver_status.h"
#include "drivers/i2c_diag/i2c_diag.h"
#include "services/shell.h"
#include "services/time.h"
#include "ti_msp_dl_config.h"

namespace app {
namespace {

static const uint16_t kFramShellMaxReadBytes = 32U;
static const uint16_t kLoraShellMaxReadBytes = 64U;
static const uint16_t kMotorShellMaxReadBytes = 64U;
static const uint32_t kImuPinWiggleDefaultLoops = 20000U;
static const uint32_t kImuPinWiggleMaxLoops = 1000000U;
static const uint32_t kImuSpiBurstDefaultBytes = 4096U;
static const uint32_t kImuSpiBurstMaxBytes = 1000000U;
static const uint32_t kImuSpiSampleDefaultBytes = 8U;
static const uint32_t kImuSpiSampleMaxBytes = 16U;
static const uint8_t kOledTextRows = 4U;
static const uint8_t kOledTextCols = 21U;
static const uint8_t kMotorSof = 0xAAU;
static const uint8_t kMotorCmdRead = 0x01U;
static const uint8_t kMotorCmdWrite = 0x02U;
static const uint8_t kMotorCmdHeartbeat = 0x03U;
static const uint8_t kMotorResponseFlag = 0x80U;
static const uint8_t kMotorStatusOk = 0x00U;
static const uint8_t kMotorMaxPayloadLength = 32U;
static const uint32_t kMotorResponseWaitIterations = 5000000U;
static const char * const kMotorI2cBusName = "motor";
static const uint8_t kMotorI2cDefaultAddress = BOARD_MOTOR_DRIVER_I2C_ADDRESS;

static const uint8_t kMotorRegDeviceId = 0x00U;
static const uint8_t kMotorRegM1Mode = 0x10U;
static const uint8_t kMotorRegM1Encoder = 0x14U;
static const uint8_t kMotorRegM2Mode = 0x20U;
static const uint8_t kMotorRegM2Encoder = 0x24U;
static const uint8_t kMotorRegEncoderControl = 0x31U;
static const uint8_t kMotorRegM1TargetRpm = 0x32U;
static const uint8_t kMotorRegM2TargetRpm = 0x34U;
static const uint8_t kMotorRegSpeedRpmBlock = 0x32U;
static const uint8_t kMotorRegSpeedPid = 0x3AU;
static const uint8_t kMotorRegCountsPerRev = 0x40U;
static const uint8_t kMotorRegM1TargetPosition = 0x50U;
static const uint8_t kMotorRegM2TargetPosition = 0x54U;
static const uint8_t kMotorRegPositionBlock = 0x50U;
static const uint8_t kMotorRegPositionPid = 0x60U;
static const uint8_t kMotorRegPositionControl = 0x68U;
static const uint8_t kMotorEncoderBlockLength = 9U;
static const uint8_t kMotorSpeedRpmBlockLength = 8U;
static const uint8_t kMotorSpeedPidLength = 5U;
static const uint8_t kMotorCountsPerRevLength = 8U;
static const uint8_t kMotorPositionBlockLength = 29U;
static const uint8_t kMotorPositionPidLength = 7U;
static const uint8_t kMotorPositionControlLength = 5U;
static const uint8_t kMotorModeSpeed = 3U;
static const uint8_t kMotorModePosition = 4U;
static const uint32_t kMotorSpeedMaxRpm = 1000U;
static const uint32_t kMotorCountsPerRevMax = 100000000U;
static const uint32_t kMotorPositionMaxRpm = 1000U;
static const uint32_t kMotorPositionToleranceMax = 65535U;
static const uint32_t kMotorPositionSettleMsMax = 2550U;
static const int32_t kMotorPositionDegreeLimit = 360000;

struct MotorProtocolFrame {
    uint8_t cmd;
    uint8_t reg;
    uint8_t length;
    uint8_t data[kMotorMaxPayloadLength];
};

enum MotorTransport : uint8_t {
    MOTOR_TRANSPORT_UART = 0U,
    MOTOR_TRANSPORT_I2C = 1U,
};

MotorTransport g_motorTransport = MOTOR_TRANSPORT_I2C;
uint8_t g_motorI2cAddress = kMotorI2cDefaultAddress;

bool StrEqual(const char *left, const char *right)
{
    if ((left == 0) || (right == 0)) {
        return false;
    }

    while ((*left != '\0') && (*right != '\0')) {
        if (*left != *right) {
            return false;
        }
        left++;
        right++;
    }

    return (*left == '\0') && (*right == '\0');
}

uint8_t CountTextCells(const char *text, uint8_t max_cells)
{
    uint8_t count = 0U;
    while ((text != 0) && (*text != '\0') && (count < max_cells)) {
        count++;
        text++;
    }

    return count;
}

bool ParseUint32(const char *text, uint32_t maxValue, uint32_t *outValue)
{
    uint32_t value = 0U;
    uint32_t base = 10U;

    if ((text == 0) || (outValue == 0) || (*text == '\0')) {
        return false;
    }

    if ((text[0] == '0') && ((text[1] == 'x') || (text[1] == 'X'))) {
        base = 16U;
        text += 2;
        if (*text == '\0') {
            return false;
        }
    }

    while (*text != '\0') {
        uint32_t digit = 0U;

        if ((*text >= '0') && (*text <= '9')) {
            digit = (uint32_t) (*text - '0');
        } else if ((*text >= 'a') && (*text <= 'f')) {
            digit = 10U + (uint32_t) (*text - 'a');
        } else if ((*text >= 'A') && (*text <= 'F')) {
            digit = 10U + (uint32_t) (*text - 'A');
        } else {
            return false;
        }

        if (digit >= base) {
            return false;
        }

        if (value > ((maxValue - digit) / base)) {
            return false;
        }

        value = (value * base) + digit;
        text++;
    }

    *outValue = value;
    return true;
}

bool ParseInt32(const char *text,
                int32_t minValue,
                int32_t maxValue,
                int32_t *outValue)
{
    bool negative = false;
    uint32_t magnitude = 0U;
    uint32_t limit = 0U;

    if ((text == 0) || (outValue == 0) || (*text == '\0') ||
        (minValue > maxValue)) {
        return false;
    }

    if (*text == '-') {
        negative = true;
        text++;
    } else if (*text == '+') {
        text++;
    }

    if (*text == '\0') {
        return false;
    }

    if (negative) {
        limit = (uint32_t) (-static_cast<int64_t>(minValue));
    } else {
        limit = (uint32_t) maxValue;
    }

    if (!ParseUint32(text, limit, &magnitude)) {
        return false;
    }

    const int64_t signed_value =
        negative ? -static_cast<int64_t>(magnitude) :
                   static_cast<int64_t>(magnitude);
    if ((signed_value < minValue) || (signed_value > maxValue)) {
        return false;
    }

    *outValue = static_cast<int32_t>(signed_value);
    return true;
}

bool ParsePercent(const char *text, uint8_t *outValue)
{
    uint32_t value = 0U;

    if ((outValue == 0) || (!ParseUint32(text, 100U, &value))) {
        return false;
    }

    *outValue = (uint8_t) value;
    return true;
}

char HexDigit(uint8_t value)
{
    value &= 0x0FU;
    return (value < 10U) ? (char) ('0' + value) :
                           (char) ('A' + (value - 10U));
}

bool IsPrintableAscii(uint8_t value)
{
    return (value >= 0x20U) && (value <= 0x7EU);
}

void WriteHex8(uint8_t value)
{
    char text[5] = { '0', 'x', '0', '0', '\0' };

    text[2] = HexDigit((uint8_t) (value >> 4U));
    text[3] = HexDigit(value);
    services::Shell_WriteString(text);
}

void WriteHex16(uint16_t value)
{
    char text[7] = { '0', 'x', '0', '0', '0', '0', '\0' };

    text[2] = HexDigit((uint8_t) (value >> 12U));
    text[3] = HexDigit((uint8_t) (value >> 8U));
    text[4] = HexDigit((uint8_t) (value >> 4U));
    text[5] = HexDigit((uint8_t) value);
    services::Shell_WriteString(text);
}

void WriteHex32(uint32_t value)
{
    char text[11] = {
        '0', 'x', '0', '0', '0', '0', '0', '0', '0', '0', '\0'
    };

    for (uint8_t i = 0U; i < 8U; i++) {
        const uint8_t shift = (uint8_t) ((7U - i) * 4U);
        text[2U + i] = HexDigit((uint8_t) (value >> shift));
    }

    services::Shell_WriteString(text);
}

void WriteInt32(int32_t value)
{
    uint32_t magnitude = 0U;

    if (value < 0) {
        services::Shell_WriteString("-");
        magnitude = (uint32_t) (-(value + 1)) + 1U;
    } else {
        magnitude = (uint32_t) value;
    }

    services::Shell_WriteUInt32(magnitude);
}

const char *DriverStatusText(drivers::DriverStatus status)
{
    switch (status) {
        case drivers::DRIVER_OK:
            return "ok";
        case drivers::DRIVER_ERROR:
            return "error";
        case drivers::DRIVER_ERROR_INVALID_ARG:
            return "invalid-arg";
        case drivers::DRIVER_ERROR_NOT_INITIALIZED:
            return "not-initialized";
        case drivers::DRIVER_ERROR_TIMEOUT:
            return "timeout";
        case drivers::DRIVER_ERROR_BUSY:
            return "busy";
        case drivers::DRIVER_ERROR_UNSUPPORTED:
            return "unsupported";
        default:
            return "unknown";
    }
}

void WriteStatusLine(const char *prefix, drivers::DriverStatus status)
{
    services::Shell_WriteString(prefix);
    services::Shell_WriteString(DriverStatusText(status));
    services::Shell_WriteString("\r\n");
}

void PrintFramUsage(void)
{
    services::Shell_WriteLine("usage:");
    services::Shell_WriteLine("  fram status");
    services::Shell_WriteLine("  fram recover");
    services::Shell_WriteLine("  fram test");
    services::Shell_WriteLine("  fram read <addr> <len 1..32>");
    services::Shell_WriteLine("  fram write <addr> <byte>");
}

void PrintIna219Usage(void)
{
    services::Shell_WriteLine("usage:");
    services::Shell_WriteLine("  ina219 status");
    services::Shell_WriteLine("  ina219 scan");
    services::Shell_WriteLine("  ina219 addr <0x40..0x4F>");
    services::Shell_WriteLine("  ina219 recover");
    services::Shell_WriteLine("  ina219 config");
    services::Shell_WriteLine("  ina219 reset");
    services::Shell_WriteLine("  ina219 read");
    services::Shell_WriteLine("  ina219 raw");
    services::Shell_WriteLine("  ina219 reg <0..5> [value]");
}

void PrintOledUsage(void)
{
    services::Shell_WriteLine("usage:");
    services::Shell_WriteLine("  oled status");
    services::Shell_WriteLine("  oled init");
    services::Shell_WriteLine("  oled clear");
    services::Shell_WriteLine("  oled fill <0x00..0xFF>");
    services::Shell_WriteLine("  oled test");
    services::Shell_WriteLine("  oled text <row 0..3> <col 0..20> <ascii...>");
    services::Shell_WriteLine("  oled invert on|off");
    services::Shell_WriteLine("  oled on|off");
}

void PrintImuUsage(void)
{
    services::Shell_WriteLine("usage:");
    services::Shell_WriteLine("  imu status");
    services::Shell_WriteLine("  imu cs idle|icm|lis|float");
    services::Shell_WriteLine("  imu pins wiggle [loops]");
    services::Shell_WriteLine("  imu spi mode <0..3>");
    services::Shell_WriteLine("  imu spi burst [bytes] [byte]");
    services::Shell_WriteLine("  imu spi rx [count 1..16] [tx]");
    services::Shell_WriteLine("  imu lis whoami");
    services::Shell_WriteLine("  imu lis reg <addr>");
    services::Shell_WriteLine("  imu icm reg <addr>");
}

void PrintLoraUsage(void)
{
    services::Shell_WriteLine("usage:");
    services::Shell_WriteLine("  lora status");
    services::Shell_WriteLine("  lora send <text...>");
    services::Shell_WriteLine("  lora line <text...>");
    services::Shell_WriteLine("  lora hex <byte...>");
    services::Shell_WriteLine("  lora read [len 1..64]");
    services::Shell_WriteLine("  lora clear");
    services::Shell_WriteLine("  lora test");
}

void PrintMotorUsage(void)
{
    services::Shell_WriteLine("usage:");
    services::Shell_WriteLine("  motor status");
    services::Shell_WriteLine("  motor bus [uart|i2c]");
    services::Shell_WriteLine("  motor i2caddr [0x08..0x77]");
    services::Shell_WriteLine("  motor ping");
    services::Shell_WriteLine("  motor info");
    services::Shell_WriteLine("  motor enc");
    services::Shell_WriteLine("  motor enc reset");
    services::Shell_WriteLine("  motor rpm");
    services::Shell_WriteLine("  motor cfg");
    services::Shell_WriteLine("  motor cfg <m1_counts_per_rev> <m2_counts_per_rev>");
    services::Shell_WriteLine("  motor pid");
    services::Shell_WriteLine("  motor pid <kp_q4.4> <ki_q4.4> <kd_q4.4> [max_duty [min_duty]]");
    services::Shell_WriteLine("  motor pos");
    services::Shell_WriteLine("  motor pospid");
    services::Shell_WriteLine("  motor pospid <kp_q4.4> <ki_q4.4> <kd_q4.4> [max_rpm [tol_counts]]");
    services::Shell_WriteLine("  motor posctl");
    services::Shell_WriteLine("  motor posctl <min_duty> <max_duty> [exit_tol_counts [settle_ms]]");
    services::Shell_WriteLine("  motor reg <addr> <len 1..32>");
    services::Shell_WriteLine("  motor set <addr> <byte...>");
    services::Shell_WriteLine("  motor stop");
    services::Shell_WriteLine("  motor m1|m2 coast|brake");
    services::Shell_WriteLine("  motor m1|m2 run <duty 0..100> [fwd|rev]");
    services::Shell_WriteLine("  motor m1|m2 speed <rpm 0..1000> [fwd|rev]");
    services::Shell_WriteLine("  motor m1|m2 hold");
    services::Shell_WriteLine("  motor m1|m2 pos <deg>");
    services::Shell_WriteLine("  motor m1|m2 posrel <deg>");
    services::Shell_WriteLine("  motor send <text...>");
    services::Shell_WriteLine("  motor line <text...>");
    services::Shell_WriteLine("  motor hex <byte...>");
    services::Shell_WriteLine("  motor read [len 1..64]");
    services::Shell_WriteLine("  motor clear");
    services::Shell_WriteLine("  motor test");
}

uint8_t MotorCrc8Update(uint8_t crc, uint8_t value)
{
    crc ^= value;
    for (uint8_t bit = 0U; bit < 8U; bit++) {
        if ((crc & 0x80U) != 0U) {
            crc = (uint8_t) ((crc << 1U) ^ 0x07U);
        } else {
            crc = (uint8_t) (crc << 1U);
        }
    }
    return crc;
}

drivers::DriverStatus MotorSendFrame(uint8_t cmd,
                                      uint8_t reg,
                                      const uint8_t *data,
                                      uint8_t length)
{
    if (length > kMotorMaxPayloadLength) {
        return drivers::DRIVER_ERROR_INVALID_ARG;
    }

    uint8_t crc = 0U;
    const uint8_t header[4] = { kMotorSof, cmd, reg, length };

    for (uint8_t i = 0U; i < sizeof(header); i++) {
        const drivers::DriverStatus status =
            board::Board_MotorDriverWriteByte(header[i]);
        if (status != drivers::DRIVER_OK) {
            return status;
        }
        crc = MotorCrc8Update(crc, header[i]);
    }

    const uint8_t payload_length =
        (cmd == kMotorCmdWrite) ? length : 0U;

    for (uint8_t i = 0U; i < payload_length; i++) {
        const uint8_t value = (data == 0) ? 0U : data[i];
        const drivers::DriverStatus status =
            board::Board_MotorDriverWriteByte(value);
        if (status != drivers::DRIVER_OK) {
            return status;
        }
        crc = MotorCrc8Update(crc, value);
    }

    return board::Board_MotorDriverWriteByte(crc);
}

bool MotorReadByteWait(uint8_t *value, uint32_t wait_iterations)
{
    while (wait_iterations > 0U) {
        if (board::Board_MotorDriverReadByte(value)) {
            return true;
        }
        wait_iterations--;
    }
    return false;
}

drivers::DriverStatus MotorReceiveFrame(MotorProtocolFrame *frame)
{
    if (frame == 0) {
        return drivers::DRIVER_ERROR_INVALID_ARG;
    }

    uint8_t value = 0U;
    uint8_t crc = 0U;

    do {
        if (!MotorReadByteWait(&value, kMotorResponseWaitIterations)) {
            return drivers::DRIVER_ERROR_TIMEOUT;
        }
    } while (value != kMotorSof);

    crc = MotorCrc8Update(0U, value);

    if (!MotorReadByteWait(&frame->cmd, kMotorResponseWaitIterations) ||
        !MotorReadByteWait(&frame->reg, kMotorResponseWaitIterations) ||
        !MotorReadByteWait(&frame->length, kMotorResponseWaitIterations)) {
        return drivers::DRIVER_ERROR_TIMEOUT;
    }

    crc = MotorCrc8Update(crc, frame->cmd);
    crc = MotorCrc8Update(crc, frame->reg);
    crc = MotorCrc8Update(crc, frame->length);

    if (frame->length > kMotorMaxPayloadLength) {
        return drivers::DRIVER_ERROR;
    }

    for (uint8_t i = 0U; i < frame->length; i++) {
        if (!MotorReadByteWait(&frame->data[i], kMotorResponseWaitIterations)) {
            return drivers::DRIVER_ERROR_TIMEOUT;
        }
        crc = MotorCrc8Update(crc, frame->data[i]);
    }

    uint8_t received_crc = 0U;
    if (!MotorReadByteWait(&received_crc, kMotorResponseWaitIterations)) {
        return drivers::DRIVER_ERROR_TIMEOUT;
    }

    return (crc == received_crc) ? drivers::DRIVER_OK : drivers::DRIVER_ERROR;
}

const char *MotorTransportText(void)
{
    return (g_motorTransport == MOTOR_TRANSPORT_I2C) ? "i2c" : "uart";
}

const drivers::I2cDiagBusConfig *MotorI2cBus(void)
{
    return board::Board_I2cBusFind(kMotorI2cBusName);
}

drivers::DriverStatus MotorI2cWriteReg(uint8_t reg,
                                       const uint8_t *data,
                                       uint8_t length)
{
    const drivers::I2cDiagBusConfig *bus = MotorI2cBus();

    if ((bus == 0) || (data == 0) || (length == 0U) ||
        ((uint16_t) reg + (uint16_t) length > 256U)) {
        return drivers::DRIVER_ERROR_INVALID_ARG;
    }

    return drivers::I2cDiag_WriteReg8Block(bus,
                                           g_motorI2cAddress,
                                           reg,
                                           data,
                                           length);
}

drivers::DriverStatus MotorI2cRequest(uint8_t cmd,
                                      uint8_t reg,
                                      const uint8_t *data,
                                      uint8_t length,
                                      MotorProtocolFrame *response)
{
    const drivers::I2cDiagBusConfig *bus = MotorI2cBus();

    if ((bus == 0) || (response == 0) ||
        (length > kMotorMaxPayloadLength)) {
        return drivers::DRIVER_ERROR_INVALID_ARG;
    }

    response->cmd = static_cast<uint8_t>(cmd | kMotorResponseFlag);
    response->reg = reg;
    response->length = 0U;

    if (cmd == kMotorCmdRead) {
        if (length == 0U) {
            return drivers::DRIVER_ERROR_INVALID_ARG;
        }

        const drivers::DriverStatus status = drivers::I2cDiag_ReadReg8(
            bus,
            g_motorI2cAddress,
            reg,
            response->data,
            length);
        if (status == drivers::DRIVER_OK) {
            response->length = length;
        }
        return status;
    }

    if (cmd == kMotorCmdWrite) {
        const drivers::DriverStatus status =
            MotorI2cWriteReg(reg, data, length);
        if (status != drivers::DRIVER_OK) {
            return status;
        }
        response->length = 1U;
        response->data[0] = kMotorStatusOk;
        return drivers::DRIVER_OK;
    }

    if (cmd == kMotorCmdHeartbeat) {
        const drivers::DriverStatus status =
            drivers::I2cDiag_ProbeAddress(bus, g_motorI2cAddress);
        if (status != drivers::DRIVER_OK) {
            return status;
        }
        response->length = 1U;
        response->data[0] = kMotorStatusOk;
        return drivers::DRIVER_OK;
    }

    return drivers::DRIVER_ERROR_INVALID_ARG;
}

drivers::DriverStatus MotorRequest(uint8_t cmd,
                                   uint8_t reg,
                                   const uint8_t *data,
                                   uint8_t length,
                                   MotorProtocolFrame *response)
{
    if (g_motorTransport == MOTOR_TRANSPORT_I2C) {
        return MotorI2cRequest(cmd, reg, data, length, response);
    }

    (void) board::Board_MotorDriverClearRxBuffer();

    drivers::DriverStatus status = MotorSendFrame(cmd, reg, data, length);
    if (status != drivers::DRIVER_OK) {
        return status;
    }

    status = MotorReceiveFrame(response);
    if (status != drivers::DRIVER_OK) {
        return status;
    }

    const uint8_t expected_cmd = (uint8_t) (cmd | kMotorResponseFlag);
    if ((response->cmd != expected_cmd) || (response->reg != reg)) {
        return drivers::DRIVER_ERROR;
    }

    return drivers::DRIVER_OK;
}

void PrintMotorFrameData(const char *prefix, const MotorProtocolFrame &frame)
{
    services::Shell_WriteString(prefix);
    services::Shell_WriteString(" cmd=");
    WriteHex8(frame.cmd);
    services::Shell_WriteString(" reg=");
    WriteHex8(frame.reg);
    services::Shell_WriteString(" len=");
    services::Shell_WriteUInt32(frame.length);
    services::Shell_WriteString(" data:");
    for (uint8_t i = 0U; i < frame.length; i++) {
        services::Shell_WriteString(" ");
        WriteHex8(frame.data[i]);
    }
    services::Shell_WriteString("\r\n");
}

int32_t DecodeMotorInt32Le(const uint8_t *data)
{
    const uint32_t raw = ((uint32_t) data[0]) |
                         ((uint32_t) data[1] << 8U) |
                         ((uint32_t) data[2] << 16U) |
                         ((uint32_t) data[3] << 24U);
    return (int32_t) raw;
}

uint16_t DecodeMotorUint16Le(const uint8_t *data)
{
    return static_cast<uint16_t>(
        static_cast<uint16_t>(data[0]) |
        (static_cast<uint16_t>(data[1]) << 8U));
}

uint32_t DecodeMotorUint32Le(const uint8_t *data)
{
    return static_cast<uint32_t>(
        static_cast<uint32_t>(data[0]) |
        (static_cast<uint32_t>(data[1]) << 8U) |
        (static_cast<uint32_t>(data[2]) << 16U) |
        (static_cast<uint32_t>(data[3]) << 24U));
}

int16_t DecodeMotorInt16Le(const uint8_t *data)
{
    return static_cast<int16_t>(DecodeMotorUint16Le(data));
}

void EncodeMotorUint16Le(uint16_t value, uint8_t *data)
{
    data[0] = static_cast<uint8_t>(value & 0xFFU);
    data[1] = static_cast<uint8_t>((value >> 8U) & 0xFFU);
}

void EncodeMotorUint32Le(uint32_t value, uint8_t *data)
{
    data[0] = static_cast<uint8_t>(value & 0xFFU);
    data[1] = static_cast<uint8_t>((value >> 8U) & 0xFFU);
    data[2] = static_cast<uint8_t>((value >> 16U) & 0xFFU);
    data[3] = static_cast<uint8_t>((value >> 24U) & 0xFFU);
}

void EncodeMotorInt32Le(int32_t value, uint8_t *data)
{
    EncodeMotorUint32Le(static_cast<uint32_t>(value), data);
}

bool MotorStatusOkResponse(const MotorProtocolFrame &frame)
{
    return (frame.length == 1U) && (frame.data[0] == kMotorStatusOk);
}

void PrintMotorEncoderLine(const char *prefix, const MotorProtocolFrame &frame)
{
    if (frame.length != kMotorEncoderBlockLength) {
        services::Shell_WriteString(prefix);
        services::Shell_WriteLine(": bad-response");
        return;
    }

    services::Shell_WriteString(prefix);
    services::Shell_WriteString(" count=");
    WriteInt32(DecodeMotorInt32Le(&frame.data[0]));
    services::Shell_WriteString(" cps=");
    WriteInt32(DecodeMotorInt32Le(&frame.data[4]));
    services::Shell_WriteString(" state=");
    WriteHex8(frame.data[8]);
    services::Shell_WriteString("\r\n");
}

void PrintMotorRpmBlock(const MotorProtocolFrame &frame)
{
    if (frame.length != kMotorSpeedRpmBlockLength) {
        services::Shell_WriteLine("motor rpm: bad-response");
        return;
    }

    services::Shell_WriteString("motor rpm m1 target=");
    services::Shell_WriteUInt32(DecodeMotorUint16Le(&frame.data[0]));
    services::Shell_WriteString(" actual=");
    WriteInt32(DecodeMotorInt16Le(&frame.data[4]));
    services::Shell_WriteString("\r\n");

    services::Shell_WriteString("motor rpm m2 target=");
    services::Shell_WriteUInt32(DecodeMotorUint16Le(&frame.data[2]));
    services::Shell_WriteString(" actual=");
    WriteInt32(DecodeMotorInt16Le(&frame.data[6]));
    services::Shell_WriteString("\r\n");
}

void PrintMotorConfigBlock(const MotorProtocolFrame &frame)
{
    if (frame.length != kMotorCountsPerRevLength) {
        services::Shell_WriteLine("motor cfg: bad-response");
        return;
    }

    services::Shell_WriteString("motor cfg m1_counts_per_rev=");
    services::Shell_WriteUInt32(DecodeMotorUint32Le(&frame.data[0]));
    services::Shell_WriteString(" m2_counts_per_rev=");
    services::Shell_WriteUInt32(DecodeMotorUint32Le(&frame.data[4]));
    services::Shell_WriteString("\r\n");
}

void PrintMotorPidBlock(const MotorProtocolFrame &frame)
{
    if (frame.length != kMotorSpeedPidLength) {
        services::Shell_WriteLine("motor pid: bad-response");
        return;
    }

    services::Shell_WriteString("motor pid kp=");
    services::Shell_WriteUInt32(frame.data[0]);
    services::Shell_WriteString(" ki=");
    services::Shell_WriteUInt32(frame.data[1]);
    services::Shell_WriteString(" kd=");
    services::Shell_WriteUInt32(frame.data[2]);
    services::Shell_WriteString(" max=");
    services::Shell_WriteUInt32(frame.data[3]);
    services::Shell_WriteString(" min=");
    services::Shell_WriteUInt32(frame.data[4]);
    services::Shell_WriteString("\r\n");
}

void PrintMotorPositionPidFields(const uint8_t *data)
{
    services::Shell_WriteString(" kp=");
    services::Shell_WriteUInt32(data[0]);
    services::Shell_WriteString(" ki=");
    services::Shell_WriteUInt32(data[1]);
    services::Shell_WriteString(" kd=");
    services::Shell_WriteUInt32(data[2]);
    services::Shell_WriteString(" max_rpm=");
    services::Shell_WriteUInt32(DecodeMotorUint16Le(&data[3]));
    services::Shell_WriteString(" tol_counts=");
    services::Shell_WriteUInt32(DecodeMotorUint16Le(&data[5]));
}

void PrintMotorPositionControlFields(const uint8_t *data)
{
    services::Shell_WriteString(" min_duty=");
    services::Shell_WriteUInt32(data[0]);
    services::Shell_WriteString(" max_duty=");
    services::Shell_WriteUInt32(data[1]);
    services::Shell_WriteString(" exit_tol_counts=");
    services::Shell_WriteUInt32(DecodeMotorUint16Le(&data[2]));
    services::Shell_WriteString(" settle_ms=");
    services::Shell_WriteUInt32(static_cast<uint32_t>(data[4]) * 10U);
}

void PrintMotorPositionBlock(const MotorProtocolFrame &frame)
{
    if (frame.length != kMotorPositionBlockLength) {
        services::Shell_WriteLine("motor pos: bad-response");
        return;
    }

    services::Shell_WriteString("motor pos m1 target_count=");
    WriteInt32(DecodeMotorInt32Le(&frame.data[0]));
    services::Shell_WriteString(" error_count=");
    WriteInt32(DecodeMotorInt32Le(&frame.data[8]));
    services::Shell_WriteString("\r\n");

    services::Shell_WriteString("motor pos m2 target_count=");
    WriteInt32(DecodeMotorInt32Le(&frame.data[4]));
    services::Shell_WriteString(" error_count=");
    WriteInt32(DecodeMotorInt32Le(&frame.data[12]));
    services::Shell_WriteString("\r\n");

    services::Shell_WriteString("motor pospid");
    PrintMotorPositionPidFields(&frame.data[16]);
    services::Shell_WriteString(" status=");
    WriteHex8(frame.data[23]);
    services::Shell_WriteString("\r\n");

    services::Shell_WriteString("motor posctl");
    PrintMotorPositionControlFields(&frame.data[24]);
    services::Shell_WriteString("\r\n");
}

void PrintMotorPositionPidBlock(const MotorProtocolFrame &frame)
{
    if (frame.length != kMotorPositionPidLength) {
        services::Shell_WriteLine("motor pospid: bad-response");
        return;
    }

    services::Shell_WriteString("motor pospid");
    PrintMotorPositionPidFields(frame.data);
    services::Shell_WriteString("\r\n");
}

void PrintMotorPositionControlBlock(const MotorProtocolFrame &frame)
{
    if (frame.length != kMotorPositionControlLength) {
        services::Shell_WriteLine("motor posctl: bad-response");
        return;
    }

    services::Shell_WriteString("motor posctl");
    PrintMotorPositionControlFields(frame.data);
    services::Shell_WriteString("\r\n");
}

drivers::DriverStatus MotorReadEncoderCount(bool motor1, int32_t *count)
{
    MotorProtocolFrame response = {};

    if (count == 0) {
        return drivers::DRIVER_ERROR_INVALID_ARG;
    }

    const drivers::DriverStatus status = MotorRequest(
        kMotorCmdRead,
        motor1 ? kMotorRegM1Encoder : kMotorRegM2Encoder,
        0,
        kMotorEncoderBlockLength,
        &response);
    if (status != drivers::DRIVER_OK) {
        return status;
    }
    if (response.length != kMotorEncoderBlockLength) {
        return drivers::DRIVER_ERROR;
    }

    *count = DecodeMotorInt32Le(&response.data[0]);
    return drivers::DRIVER_OK;
}

drivers::DriverStatus MotorReadCountsPerRev(bool motor1, uint32_t *counts_per_rev)
{
    MotorProtocolFrame response = {};

    if (counts_per_rev == 0) {
        return drivers::DRIVER_ERROR_INVALID_ARG;
    }

    const drivers::DriverStatus status = MotorRequest(
        kMotorCmdRead,
        kMotorRegCountsPerRev,
        0,
        kMotorCountsPerRevLength,
        &response);
    if (status != drivers::DRIVER_OK) {
        return status;
    }
    if (response.length != kMotorCountsPerRevLength) {
        return drivers::DRIVER_ERROR;
    }

    *counts_per_rev =
        DecodeMotorUint32Le(motor1 ? &response.data[0] : &response.data[4]);
    return drivers::DRIVER_OK;
}

bool MotorDegreesToCounts(int32_t degrees,
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


void PrintI2cUsage(void)
{
    services::Shell_WriteLine("usage:");
    services::Shell_WriteLine("  i2c list");
    services::Shell_WriteLine("  i2c status <bus>");
    services::Shell_WriteLine("  i2c recover <bus>");
    services::Shell_WriteLine("  i2c scan <bus> [start end]");
    services::Shell_WriteLine("  i2c probe <bus> <addr>");
    services::Shell_WriteLine("  i2c read <bus> <addr> <reg8> <len 1..32>");
    services::Shell_WriteLine("  i2c write <bus> <addr> <reg8> <byte...>");
    services::Shell_WriteLine("  i2c test <bus> [start end]");
}
void VersionCommand(int argc, const char * const argv[])
{
    (void) argc;
    (void) argv;

    services::Shell_WriteString(BOARD_NAME);
    services::Shell_WriteString(" ");
    services::Shell_WriteString(BOARD_MCU_NAME);
    services::Shell_WriteString(" baud ");
    services::Shell_WriteUInt32(BOARD_DEFAULT_LOG_BAUDRATE);
    services::Shell_WriteString("\r\n");
}

void ResetCommand(int argc, const char * const argv[])
{
    (void) argc;
    (void) argv;

    services::Shell_WriteLine("resetting...");
    NVIC_SystemReset();
}

void PrintLedUsage(void)
{
    services::Shell_WriteLine("usage:");
    services::Shell_WriteLine("  led status");
    services::Shell_WriteLine("  led <1|2|3> on|off|toggle|status");
    services::Shell_WriteLine("  led all on|off|toggle|status");
}

bool ParseLedId(const char *text, board::BoardLedId *id)
{
    uint32_t led_number = 0U;
    if ((id == 0) ||
        (!ParseUint32(text, board::Board_LedCount(), &led_number)) ||
        (led_number == 0U)) {
        return false;
    }

    *id = static_cast<board::BoardLedId>(led_number - 1U);
    return true;
}

drivers::DriverStatus ApplyLedAction(board::BoardLedId id, const char *action)
{
    if (StrEqual(action, "on")) {
        return board::Board_LedOn(id);
    }
    if (StrEqual(action, "off")) {
        return board::Board_LedOff(id);
    }
    if (StrEqual(action, "toggle")) {
        return board::Board_LedToggle(id);
    }

    return drivers::DRIVER_ERROR_INVALID_ARG;
}

drivers::DriverStatus ApplyAllLedAction(const char *action)
{
    for (uint8_t index = 0U; index < board::Board_LedCount(); index++) {
        const drivers::DriverStatus status = ApplyLedAction(
            static_cast<board::BoardLedId>(index),
            action);
        if (status != drivers::DRIVER_OK) {
            return status;
        }
    }

    return drivers::DRIVER_OK;
}

void WriteLedStatus(board::BoardLedId id)
{
    services::Shell_WriteString("led ");
    services::Shell_WriteUInt32(static_cast<uint32_t>(id) + 1U);
    services::Shell_WriteString(" ready=");
    services::Shell_WriteUInt32(board::Board_LedIsReady(id) ? 1U : 0U);
    services::Shell_WriteString(" state=");
    services::Shell_WriteString(board::Board_LedIsOn(id) ? "on" : "off");
    services::Shell_WriteString("\r\n");
}

void WriteAllLedStatus(void)
{
    for (uint8_t index = 0U; index < board::Board_LedCount(); index++) {
        WriteLedStatus(static_cast<board::BoardLedId>(index));
    }
}

void LedCommand(int argc, const char * const argv[])
{
    if (argc < 2) {
        PrintLedUsage();
        return;
    }

    if (argc == 2) {
        if (StrEqual(argv[1], "status")) {
            WriteAllLedStatus();
            return;
        }

        PrintLedUsage();
        return;
    }

    if (argc != 3) {
        PrintLedUsage();
        return;
    }

    if (StrEqual(argv[1], "all")) {
        if (StrEqual(argv[2], "status")) {
            WriteAllLedStatus();
            return;
        }

        const drivers::DriverStatus status = ApplyAllLedAction(argv[2]);
        if (status == drivers::DRIVER_ERROR_INVALID_ARG) {
            PrintLedUsage();
            return;
        }

        WriteStatusLine("led all: ", status);
        return;
    }

    board::BoardLedId id = board::BOARD_LED_ID_1;
    if (!ParseLedId(argv[1], &id)) {
        PrintLedUsage();
        return;
    }

    if (StrEqual(argv[2], "status")) {
        WriteLedStatus(id);
        return;
    }

    const drivers::DriverStatus status = ApplyLedAction(id, argv[2]);
    if (status == drivers::DRIVER_ERROR_INVALID_ARG) {
        PrintLedUsage();
        return;
    }

    WriteStatusLine("led: ", status);
}

void BuzzerCommand(int argc, const char * const argv[])
{
    if (argc != 2) {
        services::Shell_WriteLine("usage: buzzer on|off");
        return;
    }

    if (!board::Board_BuzzerIsReady()) {
        services::Shell_WriteLine("buzzer: not ready");
        return;
    }

    if (StrEqual(argv[1], "on")) {
        (void) board::Board_BuzzerOn();
        services::Shell_WriteLine("buzzer on");
        return;
    }

    if (StrEqual(argv[1], "off")) {
        (void) board::Board_BuzzerOff();
        services::Shell_WriteLine("buzzer off");
        return;
    }

    services::Shell_WriteLine("usage: buzzer on|off");
}

void ButtonCommand(int argc, const char * const argv[])
{
#if FEATURE_ENABLE_BUTTONS
    if (argc != 1) {
        services::Shell_WriteLine("usage: button");
        return;
    }

    for (uint32_t i = 0U; i < (uint32_t) board::BOARD_BUTTON_COUNT; i++) {
        const board::BoardButtonId id = (board::BoardButtonId) i;
        bool raw_pressed = false;
        const drivers::DriverStatus status = board::Board_ButtonReadRaw(
            id,
            &raw_pressed);

        services::Shell_WriteString(board::Board_ButtonGetName(id));
        services::Shell_WriteString(" raw=");
        if (status == drivers::DRIVER_OK) {
            services::Shell_WriteString(raw_pressed ? "pressed" : "released");
        } else {
            services::Shell_WriteString(DriverStatusText(status));
        }

        services::Shell_WriteString(" debounced=");
        services::Shell_WriteString(
            board::Board_ButtonIsPressed(id) ? "pressed" : "released");
        services::Shell_WriteString(" press_event=");
        services::Shell_WriteUInt32(board::Board_ButtonWasPressed(id) ? 1U : 0U);
        services::Shell_WriteString(" release_event=");
        services::Shell_WriteUInt32(board::Board_ButtonWasReleased(id) ? 1U : 0U);
        services::Shell_WriteString("\r\n");
    }
#else
    (void) argc;
    (void) argv;
    services::Shell_WriteLine("button: disabled");
#endif
}

void FramCommand(int argc, const char * const argv[])
{
#if FEATURE_ENABLE_FRAM
    uint32_t address = 0U;
    uint32_t length = 0U;
    uint32_t byte_value = 0U;
    uint8_t data[kFramShellMaxReadBytes];
    const uint32_t capacity = board::Board_FramCapacityBytes();

    if ((argc < 2) || (!board::Board_FramIsReady())) {
        if (argc < 2) {
            PrintFramUsage();
        } else {
            services::Shell_WriteLine("fram: not ready");
        }
        return;
    }

    if (StrEqual(argv[1], "recover")) {
        if (argc != 2) {
            PrintFramUsage();
            return;
        }

        const drivers::DriverStatus status = board::Board_FramRecoverBus();
        WriteStatusLine("fram recover: ", status);
        return;
    }

    if (StrEqual(argv[1], "status")) {
        uint32_t controller_status = 0U;
        bool scl_high = false;
        bool sda_high = false;

        if (argc != 2) {
            PrintFramUsage();
            return;
        }

        const drivers::DriverStatus status = board::Board_FramGetBusStatus(
            &controller_status,
            &scl_high,
            &sda_high);
        if (status != drivers::DRIVER_OK) {
            WriteStatusLine("fram status: ", status);
            return;
        }

        services::Shell_WriteString("fram bus status=");
        WriteHex32(controller_status);
        services::Shell_WriteString(" scl=");
        services::Shell_WriteString(scl_high ? "H" : "L");
        services::Shell_WriteString(" sda=");
        services::Shell_WriteString(sda_high ? "H" : "L");
        services::Shell_WriteString("\r\n");
        return;
    }

    if (StrEqual(argv[1], "test")) {
        if (argc != 2) {
            PrintFramUsage();
            return;
        }

        const drivers::DriverStatus status = board::Board_FramSelfTest();
        WriteStatusLine("fram test: ", status);
        return;
    }

    if (StrEqual(argv[1], "read")) {
        if ((argc != 4) ||
            (!ParseUint32(argv[2], capacity - 1U, &address)) ||
            (!ParseUint32(argv[3], kFramShellMaxReadBytes, &length)) ||
            (length == 0U) || ((address + length) > capacity)) {
            PrintFramUsage();
            return;
        }

        const drivers::DriverStatus status = board::Board_FramRead(
            (uint16_t) address,
            data,
            (uint16_t) length);
        if (status != drivers::DRIVER_OK) {
            WriteStatusLine("fram read: ", status);
            return;
        }

        services::Shell_WriteString("fram ");
        WriteHex16((uint16_t) address);
        services::Shell_WriteString(":");
        for (uint16_t i = 0U; i < (uint16_t) length; i++) {
            services::Shell_WriteString(" ");
            WriteHex8(data[i]);
        }
        services::Shell_WriteString("\r\n");
        return;
    }

    if (StrEqual(argv[1], "write")) {
        if ((argc != 4) ||
            (!ParseUint32(argv[2], capacity - 1U, &address)) ||
            (!ParseUint32(argv[3], 0xFFU, &byte_value))) {
            PrintFramUsage();
            return;
        }

        drivers::DriverStatus status = board::Board_FramWriteByte(
            (uint16_t) address,
            (uint8_t) byte_value);
        if (status != drivers::DRIVER_OK) {
            WriteStatusLine("fram write: ", status);
            return;
        }

        status = board::Board_FramReadByte((uint16_t) address, data);
        if (status != drivers::DRIVER_OK) {
            WriteStatusLine("fram verify: ", status);
            return;
        }

        services::Shell_WriteString("fram write ok ");
        WriteHex16((uint16_t) address);
        services::Shell_WriteString(" = ");
        WriteHex8(data[0]);
        services::Shell_WriteString("\r\n");
        return;
    }

    PrintFramUsage();
#else
    (void) argc;
    (void) argv;
    services::Shell_WriteLine("fram: disabled");
#endif
}

void OledCommand(int argc, const char * const argv[])
{
#if FEATURE_ENABLE_OLED
    uint32_t value = 0U;

    if (argc < 2) {
        PrintOledUsage();
        return;
    }

    if (StrEqual(argv[1], "status")) {
        uint32_t controller_status = 0U;
        bool scl_high = false;
        bool sda_high = false;
        const drivers::DriverStatus bus_status = board::Board_OledGetBusStatus(
            &controller_status,
            &scl_high,
            &sda_high);
        const drivers::DriverStatus probe_status = board::Board_OledProbe();

        if (argc != 2) {
            PrintOledUsage();
            return;
        }

        services::Shell_WriteString("oled ready=");
        services::Shell_WriteUInt32(board::Board_OledIsReady() ? 1U : 0U);
        services::Shell_WriteString(" addr=");
        WriteHex8(board::Board_OledAddress());
        services::Shell_WriteString(" size=");
        services::Shell_WriteUInt32(board::Board_OledWidth());
        services::Shell_WriteString("x");
        services::Shell_WriteUInt32(board::Board_OledHeight());
        services::Shell_WriteString(" probe=");
        services::Shell_WriteString(DriverStatusText(probe_status));
        services::Shell_WriteString(" bus=");
        if (bus_status == drivers::DRIVER_OK) {
            WriteHex32(controller_status);
            services::Shell_WriteString(" scl=");
            services::Shell_WriteString(scl_high ? "H" : "L");
            services::Shell_WriteString(" sda=");
            services::Shell_WriteString(sda_high ? "H" : "L");
        } else {
            services::Shell_WriteString(DriverStatusText(bus_status));
        }
        services::Shell_WriteString("\r\n");
        return;
    }

    if (StrEqual(argv[1], "init")) {
        if (argc != 2) {
            PrintOledUsage();
            return;
        }

        const drivers::DriverStatus status = board::Board_OledInit();
        WriteStatusLine("oled init: ", status);
        return;
    }

    if (StrEqual(argv[1], "clear")) {
        if (argc != 2) {
            PrintOledUsage();
            return;
        }

        const drivers::DriverStatus status = board::Board_OledClear();
        WriteStatusLine("oled clear: ", status);
        return;
    }

    if (StrEqual(argv[1], "fill")) {
        if ((argc != 3) || (!ParseUint32(argv[2], 0xFFU, &value))) {
            PrintOledUsage();
            return;
        }

        const drivers::DriverStatus status =
            board::Board_OledFill(static_cast<uint8_t>(value));
        WriteStatusLine("oled fill: ", status);
        return;
    }

    if (StrEqual(argv[1], "test")) {
        if (argc != 2) {
            PrintOledUsage();
            return;
        }

        drivers::DriverStatus status = board::Board_OledFill(0xFFU);
        if (status != drivers::DRIVER_OK) {
            WriteStatusLine("oled test: ", status);
            return;
        }
        services::Time_DelayMs(250U);

        status = board::Board_OledClear();
        if (status != drivers::DRIVER_OK) {
            WriteStatusLine("oled test: ", status);
            return;
        }
        services::Time_DelayMs(250U);

        status = board::Board_OledTestPattern();
        WriteStatusLine("oled test: ", status);
        return;
    }

    if (StrEqual(argv[1], "text")) {
        uint32_t row = 0U;
        uint32_t col = 0U;

        if ((argc < 5) ||
            (!ParseUint32(argv[2], kOledTextRows - 1U, &row)) ||
            (!ParseUint32(argv[3], kOledTextCols - 1U, &col))) {
            PrintOledUsage();
            return;
        }

        drivers::DriverStatus status = drivers::DRIVER_OK;
        uint8_t write_col = static_cast<uint8_t>(col);

        for (int arg = 4; (arg < argc) && (write_col < kOledTextCols); arg++) {
            status = board::Board_OledWriteText(
                static_cast<uint8_t>(row),
                write_col,
                argv[arg]);
            if (status != drivers::DRIVER_OK) {
                WriteStatusLine("oled text: ", status);
                return;
            }

            write_col = static_cast<uint8_t>(
                write_col + CountTextCells(argv[arg],
                                           kOledTextCols - write_col));

            if ((arg + 1 < argc) && (write_col < kOledTextCols)) {
                status = board::Board_OledWriteText(
                    static_cast<uint8_t>(row),
                    write_col,
                    " ");
                if (status != drivers::DRIVER_OK) {
                    WriteStatusLine("oled text: ", status);
                    return;
                }
                write_col++;
            }
        }

        WriteStatusLine("oled text: ", status);
        return;
    }

    if (StrEqual(argv[1], "invert")) {
        if (argc != 3) {
            PrintOledUsage();
            return;
        }

        if (StrEqual(argv[2], "on")) {
            WriteStatusLine("oled invert: ", board::Board_OledSetInvert(true));
            return;
        }
        if (StrEqual(argv[2], "off")) {
            WriteStatusLine("oled invert: ", board::Board_OledSetInvert(false));
            return;
        }

        PrintOledUsage();
        return;
    }

    if (StrEqual(argv[1], "on") || StrEqual(argv[1], "off")) {
        if (argc != 2) {
            PrintOledUsage();
            return;
        }

        const bool on = StrEqual(argv[1], "on");
        WriteStatusLine("oled display: ", board::Board_OledSetDisplayOn(on));
        return;
    }

    PrintOledUsage();
#else
    (void) argc;
    (void) argv;
    services::Shell_WriteLine("oled: disabled");
#endif
}

void Ina219Command(int argc, const char * const argv[])
{
#if FEATURE_ENABLE_INA219
    uint32_t reg = 0U;
    uint32_t value = 0U;

    if ((argc < 2) || (!board::Board_Ina219IsReady())) {
        if (argc < 2) {
            PrintIna219Usage();
        } else {
            services::Shell_WriteLine("ina219: not ready");
        }
        return;
    }

    if (StrEqual(argv[1], "scan")) {
        if (argc != 2) {
            PrintIna219Usage();
            return;
        }

        for (uint8_t address = 0x40U; address <= 0x4FU; address++) {
            uint16_t config_value = 0U;
            const drivers::DriverStatus status = board::Board_Ina219ProbeAddress(
                address,
                &config_value);

            if (status != drivers::DRIVER_OK) {
                (void) board::Board_Ina219RecoverBus();
                continue;
            }

            const drivers::DriverStatus select_status =
                board::Board_Ina219SetAddress(address);

            services::Shell_WriteString("ina219 found addr=");
            WriteHex8(address);
            services::Shell_WriteString(" cfg=");
            WriteHex16(config_value);
            services::Shell_WriteString(" select=");
            services::Shell_WriteString(DriverStatusText(select_status));
            services::Shell_WriteString("\r\n");
            return;
        }

        services::Shell_WriteLine("ina219 scan: none");
        return;
    }

    if (StrEqual(argv[1], "addr")) {
        if ((argc != 3) || (!ParseUint32(argv[2], 0x4FU, &value)) ||
            (value < 0x40U)) {
            PrintIna219Usage();
            return;
        }

        const drivers::DriverStatus status = board::Board_Ina219SetAddress(
            (uint8_t) value);
        if (status != drivers::DRIVER_OK) {
            WriteStatusLine("ina219 addr: ", status);
            return;
        }

        services::Shell_WriteString("ina219 addr=");
        WriteHex8(board::Board_Ina219GetAddress());
        services::Shell_WriteString("\r\n");
        return;
    }

    if (StrEqual(argv[1], "status")) {
        uint32_t controller_status = 0U;
        bool scl_high = false;
        bool sda_high = false;

        if (argc != 2) {
            PrintIna219Usage();
            return;
        }

        const drivers::DriverStatus status = board::Board_Ina219GetBusStatus(
            &controller_status,
            &scl_high,
            &sda_high);
        if (status != drivers::DRIVER_OK) {
            WriteStatusLine("ina219 status: ", status);
            return;
        }

        services::Shell_WriteString("ina219 addr=");
        WriteHex8(board::Board_Ina219GetAddress());
        services::Shell_WriteString(" bus status=");
        WriteHex32(controller_status);
        services::Shell_WriteString(" scl=");
        services::Shell_WriteString(scl_high ? "H" : "L");
        services::Shell_WriteString(" sda=");
        services::Shell_WriteString(sda_high ? "H" : "L");
        services::Shell_WriteString(" cal=");
        WriteHex16(board::Board_Ina219Calibration());
        services::Shell_WriteString("\r\n");
        return;
    }

    if (StrEqual(argv[1], "recover")) {
        if (argc != 2) {
            PrintIna219Usage();
            return;
        }

        const drivers::DriverStatus status = board::Board_Ina219RecoverBus();
        WriteStatusLine("ina219 recover: ", status);
        return;
    }

    if (StrEqual(argv[1], "config")) {
        if (argc != 2) {
            PrintIna219Usage();
            return;
        }

        const drivers::DriverStatus status =
            board::Board_Ina219ConfigureDefault();
        if (status != drivers::DRIVER_OK) {
            WriteStatusLine("ina219 config: ", status);
            return;
        }

        services::Shell_WriteString("ina219 config ok cal=");
        WriteHex16(board::Board_Ina219Calibration());
        services::Shell_WriteString("\r\n");
        return;
    }

    if (StrEqual(argv[1], "reset")) {
        if (argc != 2) {
            PrintIna219Usage();
            return;
        }

        drivers::DriverStatus status = board::Board_Ina219Reset();
        if (status == drivers::DRIVER_OK) {
            delay_cycles(32000U);
            status = board::Board_Ina219ConfigureDefault();
        }
        WriteStatusLine("ina219 reset: ", status);
        return;
    }

    if (StrEqual(argv[1], "read")) {
        drivers::Ina219Measurement measurement;

        if (argc != 2) {
            PrintIna219Usage();
            return;
        }

        const drivers::DriverStatus status = board::Board_Ina219ReadMeasurement(
            &measurement);
        if (status != drivers::DRIVER_OK) {
            WriteStatusLine("ina219 read: ", status);
            return;
        }

        services::Shell_WriteString("ina219 bus_mV=");
        WriteInt32(measurement.bus_voltage_mv);
        services::Shell_WriteString(" shunt_uV=");
        WriteInt32(measurement.shunt_voltage_uv);
        services::Shell_WriteString(" current_uA=");
        WriteInt32(measurement.current_ua);
        services::Shell_WriteString(" power_mW=");
        WriteInt32(measurement.power_mw);
        services::Shell_WriteString(" cnvr=");
        services::Shell_WriteUInt32(measurement.conversion_ready ? 1U : 0U);
        services::Shell_WriteString(" ovf=");
        services::Shell_WriteUInt32(measurement.math_overflow ? 1U : 0U);
        services::Shell_WriteString("\r\n");
        return;
    }

    if (StrEqual(argv[1], "raw")) {
        drivers::Ina219RawRegisters raw;

        if (argc != 2) {
            PrintIna219Usage();
            return;
        }

        const drivers::DriverStatus status = board::Board_Ina219ReadRawRegisters(
            &raw);
        if (status != drivers::DRIVER_OK) {
            WriteStatusLine("ina219 raw: ", status);
            return;
        }

        services::Shell_WriteString("ina219 raw cfg=");
        WriteHex16(raw.config);
        services::Shell_WriteString(" shunt=");
        WriteHex16((uint16_t) raw.shunt_voltage);
        services::Shell_WriteString(" bus=");
        WriteHex16(raw.bus_voltage);
        services::Shell_WriteString(" power=");
        WriteHex16(raw.power);
        services::Shell_WriteString(" current=");
        WriteHex16((uint16_t) raw.current);
        services::Shell_WriteString(" cal=");
        WriteHex16(raw.calibration);
        services::Shell_WriteString("\r\n");
        return;
    }

    if (StrEqual(argv[1], "reg")) {
        if (((argc != 3) && (argc != 4)) ||
            (!ParseUint32(argv[2], 5U, &reg))) {
            PrintIna219Usage();
            return;
        }

        if (argc == 3) {
            uint16_t read_value = 0U;
            const drivers::DriverStatus status = board::Board_Ina219ReadRegister(
                (uint8_t) reg,
                &read_value);
            if (status != drivers::DRIVER_OK) {
                WriteStatusLine("ina219 reg: ", status);
                return;
            }

            services::Shell_WriteString("ina219 reg ");
            WriteHex8((uint8_t) reg);
            services::Shell_WriteString(" = ");
            WriteHex16(read_value);
            services::Shell_WriteString("\r\n");
            return;
        }

        if (!ParseUint32(argv[3], 0xFFFFU, &value)) {
            PrintIna219Usage();
            return;
        }

        const drivers::DriverStatus status = board::Board_Ina219WriteRegister(
            (uint8_t) reg,
            (uint16_t) value);
        WriteStatusLine("ina219 reg write: ", status);
        return;
    }

    PrintIna219Usage();
#else
    (void) argc;
    (void) argv;
    services::Shell_WriteLine("ina219: disabled");
#endif
}


const drivers::I2cDiagBusConfig *FindI2cBusOrPrint(const char *name)
{
    const drivers::I2cDiagBusConfig *bus = board::Board_I2cBusFind(name);

    if (bus == 0) {
        services::Shell_WriteString("i2c: unknown bus ");
        services::Shell_WriteString(name == 0 ? "" : name);
        services::Shell_WriteString("\r\n");
        services::Shell_WriteString("i2c buses:");
        for (uint32_t i = 0U; i < board::Board_I2cBusCount(); i++) {
            const drivers::I2cDiagBusConfig *item = board::Board_I2cBusGet(i);
            if (item != 0) {
                services::Shell_WriteString(" ");
                services::Shell_WriteString(item->name);
            }
        }
        services::Shell_WriteString("\r\n");
    }

    return bus;
}

bool ParseI2cRange(int argc,
                   const char * const argv[],
                   uint8_t first_optional_arg,
                   uint8_t *start,
                   uint8_t *end)
{
    uint32_t value = 0U;

    if ((start == 0) || (end == 0)) {
        return false;
    }

    *start = drivers::I2C_DIAG_MIN_7BIT_ADDRESS;
    *end = drivers::I2C_DIAG_MAX_7BIT_ADDRESS;

    if (argc == first_optional_arg) {
        return true;
    }

    if (argc != (first_optional_arg + 2)) {
        return false;
    }

    if ((!ParseUint32(argv[first_optional_arg],
                      drivers::I2C_DIAG_MAX_7BIT_ADDRESS,
                      &value)) ||
        (value < drivers::I2C_DIAG_MIN_7BIT_ADDRESS)) {
        return false;
    }
    *start = (uint8_t) value;

    if ((!ParseUint32(argv[first_optional_arg + 1],
                      drivers::I2C_DIAG_MAX_7BIT_ADDRESS,
                      &value)) ||
        (value < *start)) {
        return false;
    }
    *end = (uint8_t) value;

    return true;
}

void ImuCommand(int argc, const char * const argv[])
{
#if FEATURE_ENABLE_IMU
    uint32_t reg = 0U;
    uint8_t value = 0U;

    if ((argc < 2) || (!board::Board_ImuIsReady())) {
        if (argc < 2) {
            PrintImuUsage();
        } else {
            services::Shell_WriteLine("imu: not ready");
        }
        return;
    }

    if (StrEqual(argv[1], "status")) {
        board::BoardImuLineStatus lines = {};
        const drivers::DriverStatus status =
            board::Board_ImuGetLineStatus(&lines);

        if (status != drivers::DRIVER_OK) {
            WriteStatusLine("imu status: ", status);
            return;
        }

        services::Shell_WriteString("imu cs_icm_in=");
        services::Shell_WriteString(lines.icm_cs_high ? "H" : "L");
        services::Shell_WriteString(" cs_lis_in=");
        services::Shell_WriteString(lines.lis_cs_high ? "H" : "L");
        services::Shell_WriteString(" cs_icm_out=");
        services::Shell_WriteString(lines.icm_cs_latch_high ? "H" : "L");
        services::Shell_WriteString(lines.icm_cs_output_enabled ? "/OE" : "/HZ");
        services::Shell_WriteString(" cs_lis_out=");
        services::Shell_WriteString(lines.lis_cs_latch_high ? "H" : "L");
        services::Shell_WriteString(lines.lis_cs_output_enabled ? "/OE" : "/HZ");
        services::Shell_WriteString(" icm_int1=");
        services::Shell_WriteString(lines.icm_int1_high ? "H" : "L");
        services::Shell_WriteString(" icm_int2_fsync=");
        services::Shell_WriteString(lines.icm_int2_fsync_high ? "H" : "L");
        services::Shell_WriteString(" lis_drdy=");
        services::Shell_WriteString(lines.lis_drdy_high ? "H" : "L");
        services::Shell_WriteString("\r\n");
        return;
    }

    if (StrEqual(argv[1], "cs")) {
        drivers::DriverStatus status = drivers::DRIVER_ERROR_INVALID_ARG;

        if (argc != 3) {
            PrintImuUsage();
            return;
        }

        if (StrEqual(argv[2], "idle")) {
            status = board::Board_ImuSetChipSelectDebug(true,
                                                        true,
                                                        true,
                                                        true);
        } else if (StrEqual(argv[2], "icm")) {
            status = board::Board_ImuSetChipSelectDebug(true,
                                                        false,
                                                        true,
                                                        true);
        } else if (StrEqual(argv[2], "lis")) {
            status = board::Board_ImuSetChipSelectDebug(true,
                                                        true,
                                                        true,
                                                        false);
        } else if (StrEqual(argv[2], "float")) {
            status = board::Board_ImuSetChipSelectDebug(false,
                                                        true,
                                                        false,
                                                        true);
        } else {
            PrintImuUsage();
            return;
        }

        WriteStatusLine("imu cs: ", status);
        return;
    }

    if (StrEqual(argv[1], "pins")) {
        uint32_t loops = kImuPinWiggleDefaultLoops;

        if ((argc != 3) && (argc != 4)) {
            PrintImuUsage();
            return;
        }
        if (!StrEqual(argv[2], "wiggle")) {
            PrintImuUsage();
            return;
        }
        if ((argc == 4) &&
            ((!ParseUint32(argv[3], kImuPinWiggleMaxLoops, &loops)) ||
             (loops == 0U))) {
            PrintImuUsage();
            return;
        }

        const drivers::DriverStatus status =
            board::Board_ImuWiggleSpiPins(loops);
        WriteStatusLine("imu pins wiggle: ", status);
        return;
    }

    if (StrEqual(argv[1], "spi")) {
        uint32_t bytes = kImuSpiBurstDefaultBytes;
        uint32_t byte_value = 0xAAU;
        uint32_t mode = 0U;
        uint8_t rx[kImuSpiSampleMaxBytes];

        if ((argc < 3) || (argc > 5)) {
            PrintImuUsage();
            return;
        }
        if (StrEqual(argv[2], "mode")) {
            if ((argc != 4) || (!ParseUint32(argv[3], 3U, &mode))) {
                PrintImuUsage();
                return;
            }

            const drivers::DriverStatus status =
                board::Board_ImuSetSpiMode((uint8_t) mode);
            WriteStatusLine("imu spi mode: ", status);
            return;
        }
        if (StrEqual(argv[2], "rx")) {
            bytes = kImuSpiSampleDefaultBytes;
            byte_value = 0x00U;

            if ((argc != 3) && (argc != 4) && (argc != 5)) {
                PrintImuUsage();
                return;
            }
            if ((argc >= 4) &&
                ((!ParseUint32(argv[3], kImuSpiSampleMaxBytes, &bytes)) ||
                 (bytes == 0U))) {
                PrintImuUsage();
                return;
            }
            if ((argc == 5) && (!ParseUint32(argv[4], 0xFFU, &byte_value))) {
                PrintImuUsage();
                return;
            }

            const drivers::DriverStatus status =
                board::Board_ImuSpiSampleIcm((uint8_t) byte_value, rx, bytes);
            if (status != drivers::DRIVER_OK) {
                WriteStatusLine("imu spi rx: ", status);
                return;
            }

            services::Shell_WriteString("imu spi rx:");
            for (uint32_t i = 0U; i < bytes; i++) {
                services::Shell_WriteString(" ");
                WriteHex8(rx[i]);
            }
            services::Shell_WriteString("\r\n");
            return;
        }
        if (!StrEqual(argv[2], "burst")) {
            PrintImuUsage();
            return;
        }
        if ((argc >= 4) &&
            ((!ParseUint32(argv[3], kImuSpiBurstMaxBytes, &bytes)) ||
             (bytes == 0U))) {
            PrintImuUsage();
            return;
        }
        if ((argc == 5) && (!ParseUint32(argv[4], 0xFFU, &byte_value))) {
            PrintImuUsage();
            return;
        }

        const drivers::DriverStatus status =
            board::Board_ImuSpiBurstIcm(bytes, (uint8_t) byte_value);
        WriteStatusLine("imu spi burst: ", status);
        return;
    }

    if (StrEqual(argv[1], "lis")) {
        if ((argc == 3) && StrEqual(argv[2], "whoami")) {
            const drivers::DriverStatus status =
                board::Board_Lis3mdlReadWhoAmI(&value);
            if (status != drivers::DRIVER_OK) {
                WriteStatusLine("imu lis whoami: ", status);
                return;
            }

            services::Shell_WriteString("imu lis whoami=");
            WriteHex8(value);
            services::Shell_WriteString(" expected=0x3D\r\n");
            return;
        }

        if ((argc == 4) && StrEqual(argv[2], "reg") &&
            ParseUint32(argv[3], 0x3FU, &reg)) {
            const drivers::DriverStatus status =
                board::Board_Lis3mdlReadRegister((uint8_t) reg, &value);
            if (status != drivers::DRIVER_OK) {
                WriteStatusLine("imu lis reg: ", status);
                return;
            }

            services::Shell_WriteString("imu lis reg ");
            WriteHex8((uint8_t) reg);
            services::Shell_WriteString(" = ");
            WriteHex8(value);
            services::Shell_WriteString("\r\n");
            return;
        }

        PrintImuUsage();
        return;
    }

    if (StrEqual(argv[1], "icm")) {
        if ((argc == 4) && StrEqual(argv[2], "reg") &&
            ParseUint32(argv[3], 0x7FU, &reg)) {
            const drivers::DriverStatus status =
                board::Board_Icm45686ReadRegister((uint8_t) reg, &value);
            if (status != drivers::DRIVER_OK) {
                WriteStatusLine("imu icm reg: ", status);
                return;
            }

            services::Shell_WriteString("imu icm reg ");
            WriteHex8((uint8_t) reg);
            services::Shell_WriteString(" = ");
            WriteHex8(value);
            services::Shell_WriteString("\r\n");
            return;
        }

        PrintImuUsage();
        return;
    }

    PrintImuUsage();
#else
    (void) argc;
    (void) argv;
    services::Shell_WriteLine("imu: disabled");
#endif
}

void PrintI2cStatus(const drivers::I2cDiagBusConfig *bus)
{
    drivers::I2cDiagBusStatus status = { 0U, false, false };
    const drivers::DriverStatus driver_status = drivers::I2cDiag_GetBusStatus(
        bus,
        &status);

    if (driver_status != drivers::DRIVER_OK) {
        services::Shell_WriteString("i2c ");
        services::Shell_WriteString(bus->name);
        services::Shell_WriteString(" status: ");
        services::Shell_WriteString(DriverStatusText(driver_status));
        services::Shell_WriteString("\r\n");
        return;
    }

    services::Shell_WriteString("i2c ");
    services::Shell_WriteString(bus->name);
    services::Shell_WriteString(" status=");
    WriteHex32(status.controller_status);
    services::Shell_WriteString(" scl=");
    services::Shell_WriteString(status.scl_high ? "H" : "L");
    services::Shell_WriteString(" sda=");
    services::Shell_WriteString(status.sda_high ? "H" : "L");
    services::Shell_WriteString("\r\n");
}

uint32_t I2cScanBus(const drivers::I2cDiagBusConfig *bus,
                    uint8_t start,
                    uint8_t end,
                    bool print_each)
{
    uint32_t count = 0U;

    services::Shell_WriteString("i2c ");
    services::Shell_WriteString(bus->name);
    services::Shell_WriteString(" scan ");
    WriteHex8(start);
    services::Shell_WriteString("..");
    WriteHex8(end);
    services::Shell_WriteString("\r\n");

    for (uint8_t address = start; address <= end; address++) {
        const drivers::DriverStatus status = drivers::I2cDiag_ProbeAddress(
            bus,
            address);
        if (status == drivers::DRIVER_OK) {
            count++;
            if (print_each) {
                services::Shell_WriteString("i2c ");
                services::Shell_WriteString(bus->name);
                services::Shell_WriteString(" found ");
                WriteHex8(address);
                services::Shell_WriteString("\r\n");
            }
        }

        if (address == end) {
            break;
        }
    }

    services::Shell_WriteString("i2c ");
    services::Shell_WriteString(bus->name);
    services::Shell_WriteString(" scan count=");
    services::Shell_WriteUInt32(count);
    services::Shell_WriteString("\r\n");

    return count;
}

void I2cCommand(int argc, const char * const argv[])
{
    uint32_t value = 0U;

    if (argc < 2) {
        PrintI2cUsage();
        return;
    }

    if (StrEqual(argv[1], "list")) {
        if (argc != 2) {
            PrintI2cUsage();
            return;
        }

        services::Shell_WriteString("i2c buses:");
        for (uint32_t i = 0U; i < board::Board_I2cBusCount(); i++) {
            const drivers::I2cDiagBusConfig *bus = board::Board_I2cBusGet(i);
            if (bus != 0) {
                services::Shell_WriteString(" ");
                services::Shell_WriteString(bus->name);
            }
        }
        services::Shell_WriteString("\r\n");
        return;
    }

    if (argc < 3) {
        PrintI2cUsage();
        return;
    }

    const drivers::I2cDiagBusConfig *bus = FindI2cBusOrPrint(argv[2]);
    if (bus == 0) {
        return;
    }

    if (StrEqual(argv[1], "status")) {
        if (argc != 3) {
            PrintI2cUsage();
            return;
        }

        PrintI2cStatus(bus);
        return;
    }

    if (StrEqual(argv[1], "recover")) {
        if (argc != 3) {
            PrintI2cUsage();
            return;
        }

        const drivers::DriverStatus status = drivers::I2cDiag_RecoverBus(bus);
        services::Shell_WriteString("i2c ");
        services::Shell_WriteString(bus->name);
        services::Shell_WriteString(" recover: ");
        services::Shell_WriteString(DriverStatusText(status));
        services::Shell_WriteString("\r\n");
        return;
    }

    if (StrEqual(argv[1], "scan")) {
        uint8_t start = 0U;
        uint8_t end = 0U;

        if (!ParseI2cRange(argc, argv, 3U, &start, &end)) {
            PrintI2cUsage();
            return;
        }

        (void) I2cScanBus(bus, start, end, true);
        return;
    }

    if (StrEqual(argv[1], "probe")) {
        if ((argc != 4) ||
            (!ParseUint32(argv[3],
                          drivers::I2C_DIAG_MAX_7BIT_ADDRESS,
                          &value)) ||
            (value < drivers::I2C_DIAG_MIN_7BIT_ADDRESS)) {
            PrintI2cUsage();
            return;
        }

        const uint8_t address = (uint8_t) value;
        const drivers::DriverStatus status = drivers::I2cDiag_ProbeAddress(
            bus,
            address);
        services::Shell_WriteString("i2c ");
        services::Shell_WriteString(bus->name);
        services::Shell_WriteString(" probe ");
        WriteHex8(address);
        services::Shell_WriteString(": ");
        services::Shell_WriteString(DriverStatusText(status));
        services::Shell_WriteString("\r\n");
        return;
    }

    if (StrEqual(argv[1], "read")) {
        uint8_t data[drivers::I2C_DIAG_MAX_READ_BYTES];
        uint32_t address_value = 0U;
        uint32_t reg_value = 0U;
        uint32_t length_value = 0U;

        if ((argc != 6) ||
            (!ParseUint32(argv[3],
                          drivers::I2C_DIAG_MAX_7BIT_ADDRESS,
                          &address_value)) ||
            (address_value < drivers::I2C_DIAG_MIN_7BIT_ADDRESS) ||
            (!ParseUint32(argv[4], 0xFFU, &reg_value)) ||
            (!ParseUint32(argv[5],
                          drivers::I2C_DIAG_MAX_READ_BYTES,
                          &length_value)) ||
            (length_value == 0U)) {
            PrintI2cUsage();
            return;
        }

        const drivers::DriverStatus status = drivers::I2cDiag_ReadReg8(
            bus,
            (uint8_t) address_value,
            (uint8_t) reg_value,
            data,
            (uint16_t) length_value);
        if (status != drivers::DRIVER_OK) {
            services::Shell_WriteString("i2c read: ");
            services::Shell_WriteString(DriverStatusText(status));
            services::Shell_WriteString("\r\n");
            return;
        }

        services::Shell_WriteString("i2c ");
        services::Shell_WriteString(bus->name);
        services::Shell_WriteString(" read addr=");
        WriteHex8((uint8_t) address_value);
        services::Shell_WriteString(" reg=");
        WriteHex8((uint8_t) reg_value);
        services::Shell_WriteString(":");
        for (uint16_t i = 0U; i < (uint16_t) length_value; i++) {
            services::Shell_WriteString(" ");
            WriteHex8(data[i]);
        }
        services::Shell_WriteString("\r\n");
        return;
    }

    if (StrEqual(argv[1], "write")) {
        uint8_t data[drivers::I2C_DIAG_MAX_WRITE_BYTES];
        uint32_t address_value = 0U;
        uint32_t reg_value = 0U;
        const uint16_t length = (uint16_t) (argc - 5);

        if ((argc < 6) ||
            (length > drivers::I2C_DIAG_MAX_WRITE_BYTES) ||
            (!ParseUint32(argv[3],
                          drivers::I2C_DIAG_MAX_7BIT_ADDRESS,
                          &address_value)) ||
            (address_value < drivers::I2C_DIAG_MIN_7BIT_ADDRESS) ||
            (!ParseUint32(argv[4], 0xFFU, &reg_value))) {
            PrintI2cUsage();
            return;
        }

        for (uint16_t i = 0U; i < length; i++) {
            if (!ParseUint32(argv[5 + i], 0xFFU, &value)) {
                PrintI2cUsage();
                return;
            }
            data[i] = (uint8_t) value;
        }

        const drivers::DriverStatus status = drivers::I2cDiag_WriteReg8(
            bus,
            (uint8_t) address_value,
            (uint8_t) reg_value,
            data,
            length);
        services::Shell_WriteString("i2c write: ");
        services::Shell_WriteString(DriverStatusText(status));
        services::Shell_WriteString("\r\n");
        return;
    }

    if (StrEqual(argv[1], "test")) {
        uint8_t start = 0U;
        uint8_t end = 0U;

        if (!ParseI2cRange(argc, argv, 3U, &start, &end)) {
            PrintI2cUsage();
            return;
        }

        PrintI2cStatus(bus);
        const drivers::DriverStatus recover_status = drivers::I2cDiag_RecoverBus(
            bus);
        services::Shell_WriteString("i2c ");
        services::Shell_WriteString(bus->name);
        services::Shell_WriteString(" recover: ");
        services::Shell_WriteString(DriverStatusText(recover_status));
        services::Shell_WriteString("\r\n");
        PrintI2cStatus(bus);

        const uint32_t count = I2cScanBus(bus, start, end, true);
        services::Shell_WriteString("i2c ");
        services::Shell_WriteString(bus->name);
        services::Shell_WriteString(" test: ");
        services::Shell_WriteString(
            (recover_status == drivers::DRIVER_OK) ? "pass" : "fail");
        services::Shell_WriteString(" count=");
        services::Shell_WriteUInt32(count);
        services::Shell_WriteString("\r\n");
        return;
    }

    PrintI2cUsage();
}
drivers::DriverStatus LoraWriteArgs(int argc,
                                    const char * const argv[],
                                    bool append_newline,
                                    uint16_t *bytes_written)
{
    uint16_t count = 0U;

    if (bytes_written == 0) {
        return drivers::DRIVER_ERROR_INVALID_ARG;
    }

    for (int arg = 2; arg < argc; arg++) {
        if (arg > 2) {
            const drivers::DriverStatus status =
                board::Board_LoraWriteByte((uint8_t) ' ');
            if (status != drivers::DRIVER_OK) {
                return status;
            }
            count++;
        }

        const char *cursor = argv[arg];
        while ((cursor != 0) && (*cursor != '\0')) {
            const drivers::DriverStatus status =
                board::Board_LoraWriteByte((uint8_t) *cursor);
            if (status != drivers::DRIVER_OK) {
                return status;
            }
            count++;
            cursor++;
        }
    }

    if (append_newline) {
        drivers::DriverStatus status = board::Board_LoraWriteByte((uint8_t) '\r');
        if (status != drivers::DRIVER_OK) {
            return status;
        }
        status = board::Board_LoraWriteByte((uint8_t) '\n');
        if (status != drivers::DRIVER_OK) {
            return status;
        }
        count = (uint16_t) (count + 2U);
    }

    *bytes_written = count;
    return drivers::DRIVER_OK;
}

void LoraCommand(int argc, const char * const argv[])
{
#if FEATURE_ENABLE_LORA
    uint32_t value = 0U;

    if ((argc < 2) || (!board::Board_LoraIsReady())) {
        if (argc < 2) {
            PrintLoraUsage();
        } else {
            services::Shell_WriteLine("lora: not ready");
        }
        return;
    }

    if (StrEqual(argv[1], "status")) {
        uint32_t uart_status = 0U;

        if (argc != 2) {
            PrintLoraUsage();
            return;
        }

        const drivers::DriverStatus uart_status_result =
            board::Board_LoraGetControllerStatus(&uart_status);

        services::Shell_WriteString("lora baud=");
        services::Shell_WriteUInt32(board::Board_LoraGetBaudrate());
        services::Shell_WriteString(" rx_buf=");
        services::Shell_WriteUInt32(board::Board_LoraGetRxAvailable());
        services::Shell_WriteString(" dropped=");
        services::Shell_WriteUInt32(board::Board_LoraGetRxDroppedCount());
        services::Shell_WriteString(" uart=");
        if (uart_status_result == drivers::DRIVER_OK) {
            WriteHex32(uart_status);
            services::Shell_WriteString(" busy=");
            services::Shell_WriteUInt32(
                ((uart_status & UART_STAT_BUSY_MASK) != 0U) ? 1U : 0U);
            services::Shell_WriteString(" tx_empty=");
            services::Shell_WriteUInt32(
                ((uart_status & UART_STAT_TXFE_MASK) != 0U) ? 1U : 0U);
            services::Shell_WriteString(" tx_full=");
            services::Shell_WriteUInt32(
                ((uart_status & UART_STAT_TXFF_MASK) != 0U) ? 1U : 0U);
        } else {
            services::Shell_WriteString(DriverStatusText(uart_status_result));
        }
        services::Shell_WriteString("\r\n");
        return;
    }
    if (StrEqual(argv[1], "clear")) {
        if (argc != 2) {
            PrintLoraUsage();
            return;
        }

        const drivers::DriverStatus status = board::Board_LoraClearRxBuffer();
        WriteStatusLine("lora clear: ", status);
        return;
    }

    if (StrEqual(argv[1], "test")) {
        static const uint8_t kTestData[] = { 'p', 'i', 'n', 'g', '\r', '\n' };

        if (argc != 2) {
            PrintLoraUsage();
            return;
        }

        const drivers::DriverStatus status = board::Board_LoraWrite(
            kTestData,
            (uint16_t) sizeof(kTestData));
        WriteStatusLine("lora test: ", status);
        return;
    }

    if (StrEqual(argv[1], "send") || StrEqual(argv[1], "line")) {
        uint16_t bytes_written = 0U;

        if (argc < 3) {
            PrintLoraUsage();
            return;
        }

        const drivers::DriverStatus status = LoraWriteArgs(
            argc,
            argv,
            StrEqual(argv[1], "line"),
            &bytes_written);
        if (status != drivers::DRIVER_OK) {
            WriteStatusLine("lora send: ", status);
            return;
        }

        services::Shell_WriteString("lora tx bytes=");
        services::Shell_WriteUInt32(bytes_written);
        services::Shell_WriteString("\r\n");
        return;
    }

    if (StrEqual(argv[1], "hex")) {
        uint8_t data[16];
        uint16_t length = 0U;

        if ((argc < 3) || (argc > 18)) {
            PrintLoraUsage();
            return;
        }

        for (int arg = 2; arg < argc; arg++) {
            if (!ParseUint32(argv[arg], 0xFFU, &value)) {
                PrintLoraUsage();
                return;
            }
            data[length] = (uint8_t) value;
            length++;
        }

        const drivers::DriverStatus status = board::Board_LoraWrite(
            data,
            length);
        if (status != drivers::DRIVER_OK) {
            WriteStatusLine("lora hex: ", status);
            return;
        }

        services::Shell_WriteString("lora tx bytes=");
        services::Shell_WriteUInt32(length);
        services::Shell_WriteString("\r\n");
        return;
    }

    if (StrEqual(argv[1], "read")) {
        uint8_t data[kLoraShellMaxReadBytes];
        uint16_t length = kLoraShellMaxReadBytes;
        uint16_t actual = 0U;

        if ((argc != 2) && (argc != 3)) {
            PrintLoraUsage();
            return;
        }

        if (argc == 3) {
            if ((!ParseUint32(argv[2], kLoraShellMaxReadBytes, &value)) ||
                (value == 0U)) {
                PrintLoraUsage();
                return;
            }
            length = (uint16_t) value;
        }

        while ((actual < length) && board::Board_LoraReadByte(&data[actual])) {
            actual++;
        }

        services::Shell_WriteString("lora rx len=");
        services::Shell_WriteUInt32(actual);
        services::Shell_WriteString(" hex:");
        for (uint16_t i = 0U; i < actual; i++) {
            services::Shell_WriteString(" ");
            WriteHex8(data[i]);
        }
        services::Shell_WriteString(" ascii:");
        for (uint16_t i = 0U; i < actual; i++) {
            services::Shell_WriteChar(IsPrintableAscii(data[i]) ?
                                      (char) data[i] : '.');
        }
        services::Shell_WriteString("\r\n");
        return;
    }

    PrintLoraUsage();
#else
    (void) argc;
    (void) argv;
    services::Shell_WriteLine("lora: disabled");
#endif
}

drivers::DriverStatus MotorWriteArgs(int argc,
                                     const char * const argv[],
                                     bool append_newline,
                                     uint16_t *bytes_written)
{
    uint16_t count = 0U;

    if (bytes_written == 0) {
        return drivers::DRIVER_ERROR_INVALID_ARG;
    }

    for (int arg = 2; arg < argc; arg++) {
        if (arg > 2) {
            const drivers::DriverStatus status =
                board::Board_MotorDriverWriteByte((uint8_t) ' ');
            if (status != drivers::DRIVER_OK) {
                return status;
            }
            count++;
        }

        const char *cursor = argv[arg];
        while ((cursor != 0) && (*cursor != '\0')) {
            const drivers::DriverStatus status =
                board::Board_MotorDriverWriteByte((uint8_t) *cursor);
            if (status != drivers::DRIVER_OK) {
                return status;
            }
            count++;
            cursor++;
        }
    }

    if (append_newline) {
        drivers::DriverStatus status =
            board::Board_MotorDriverWriteByte((uint8_t) '\r');
        if (status != drivers::DRIVER_OK) {
            return status;
        }
        status = board::Board_MotorDriverWriteByte((uint8_t) '\n');
        if (status != drivers::DRIVER_OK) {
            return status;
        }
        count = (uint16_t) (count + 2U);
    }

    *bytes_written = count;
    return drivers::DRIVER_OK;
}

void MotorCommand(int argc, const char * const argv[])
{
#if FEATURE_ENABLE_MOTOR_DRIVER
    uint32_t value = 0U;

    if ((argc < 2) || (!board::Board_MotorDriverIsReady())) {
        if (argc < 2) {
            PrintMotorUsage();
        } else {
            services::Shell_WriteLine("motor: not ready");
        }
        return;
    }

    if (StrEqual(argv[1], "status")) {
        uint32_t uart_status = 0U;
        drivers::I2cDiagBusStatus i2c_status = { 0U, false, false };
        const drivers::I2cDiagBusConfig *i2c_bus = MotorI2cBus();

        if (argc != 2) {
            PrintMotorUsage();
            return;
        }

        const drivers::DriverStatus uart_status_result =
            board::Board_MotorDriverGetControllerStatus(&uart_status);

        services::Shell_WriteString("motor bus=");
        services::Shell_WriteString(MotorTransportText());
        services::Shell_WriteString(" baud=");
        services::Shell_WriteUInt32(board::Board_MotorDriverGetBaudrate());
        services::Shell_WriteString(" rx_buf=");
        services::Shell_WriteUInt32(board::Board_MotorDriverGetRxAvailable());
        services::Shell_WriteString(" dropped=");
        services::Shell_WriteUInt32(
            board::Board_MotorDriverGetRxDroppedCount());
        services::Shell_WriteString(" uart=");
        if (uart_status_result == drivers::DRIVER_OK) {
            WriteHex32(uart_status);
            services::Shell_WriteString(" busy=");
            services::Shell_WriteUInt32(
                ((uart_status & UART_STAT_BUSY_MASK) != 0U) ? 1U : 0U);
            services::Shell_WriteString(" tx_empty=");
            services::Shell_WriteUInt32(
                ((uart_status & UART_STAT_TXFE_MASK) != 0U) ? 1U : 0U);
            services::Shell_WriteString(" tx_full=");
            services::Shell_WriteUInt32(
                ((uart_status & UART_STAT_TXFF_MASK) != 0U) ? 1U : 0U);
        } else {
            services::Shell_WriteString(DriverStatusText(uart_status_result));
        }
        services::Shell_WriteString(" i2c_addr=");
        WriteHex8(g_motorI2cAddress);
        services::Shell_WriteString(" i2c_bus=");
        services::Shell_WriteString(kMotorI2cBusName);
        if ((i2c_bus != 0) &&
            (drivers::I2cDiag_GetBusStatus(i2c_bus, &i2c_status) ==
             drivers::DRIVER_OK)) {
            services::Shell_WriteString(" i2c=");
            WriteHex32(i2c_status.controller_status);
            services::Shell_WriteString(" scl=");
            services::Shell_WriteString(i2c_status.scl_high ? "H" : "L");
            services::Shell_WriteString(" sda=");
            services::Shell_WriteString(i2c_status.sda_high ? "H" : "L");
        }
        services::Shell_WriteString("\r\n");
        return;
    }

    if (StrEqual(argv[1], "bus")) {
        if (argc == 2) {
            services::Shell_WriteString("motor bus=");
            services::Shell_WriteString(MotorTransportText());
            services::Shell_WriteString("\r\n");
            return;
        }

        if (argc != 3) {
            PrintMotorUsage();
            return;
        }

        if (StrEqual(argv[2], "uart")) {
            g_motorTransport = MOTOR_TRANSPORT_UART;
        } else if (StrEqual(argv[2], "i2c")) {
            g_motorTransport = MOTOR_TRANSPORT_I2C;
        } else {
            PrintMotorUsage();
            return;
        }

        services::Shell_WriteString("motor bus=");
        services::Shell_WriteString(MotorTransportText());
        services::Shell_WriteString("\r\n");
        return;
    }

    if (StrEqual(argv[1], "i2caddr")) {
        if (argc == 2) {
            services::Shell_WriteString("motor i2caddr=");
            WriteHex8(g_motorI2cAddress);
            services::Shell_WriteString("\r\n");
            return;
        }

        if ((argc != 3) ||
            (!ParseUint32(argv[2],
                          drivers::I2C_DIAG_MAX_7BIT_ADDRESS,
                          &value)) ||
            (value < drivers::I2C_DIAG_MIN_7BIT_ADDRESS)) {
            PrintMotorUsage();
            return;
        }

        g_motorI2cAddress = static_cast<uint8_t>(value);
        services::Shell_WriteString("motor i2caddr=");
        WriteHex8(g_motorI2cAddress);
        services::Shell_WriteString("\r\n");
        return;
    }

    if (StrEqual(argv[1], "ping")) {
        MotorProtocolFrame response = {};

        if (argc != 2) {
            PrintMotorUsage();
            return;
        }

        const drivers::DriverStatus status = MotorRequest(
            kMotorCmdHeartbeat,
            0U,
            0,
            0U,
            &response);
        if (status != drivers::DRIVER_OK) {
            WriteStatusLine("motor ping: ", status);
            return;
        }

        const bool ok = (response.length == 1U) &&
                        (response.data[0] == kMotorStatusOk);
        services::Shell_WriteString("motor ping: ");
        services::Shell_WriteString(ok ? "ok" : "error");
        services::Shell_WriteString("\r\n");
        return;
    }

    if (StrEqual(argv[1], "info")) {
        MotorProtocolFrame response = {};

        if (argc != 2) {
            PrintMotorUsage();
            return;
        }

        const drivers::DriverStatus status = MotorRequest(
            kMotorCmdRead,
            kMotorRegDeviceId,
            0,
            6U,
            &response);
        if (status != drivers::DRIVER_OK) {
            WriteStatusLine("motor info: ", status);
            return;
        }
        if (response.length != 6U) {
            services::Shell_WriteLine("motor info: bad-response");
            return;
        }

        services::Shell_WriteString("motor id=");
        WriteHex8(response.data[0]);
        services::Shell_WriteString(" fw=");
        services::Shell_WriteUInt32(response.data[1]);
        services::Shell_WriteString(" status=");
        WriteHex8(response.data[2]);
        services::Shell_WriteString(" fault=");
        WriteHex8(response.data[3]);
        services::Shell_WriteString(" ctrl=");
        WriteHex8(response.data[4]);
        services::Shell_WriteString(" i2c=");
        WriteHex8(response.data[5]);
        services::Shell_WriteString("\r\n");
        return;
    }

    if (StrEqual(argv[1], "enc")) {
        MotorProtocolFrame response = {};

        if ((argc == 3) && StrEqual(argv[2], "reset")) {
            static const uint8_t kResetBothEncoders[] = { 0x03U };

            const drivers::DriverStatus status = MotorRequest(
                kMotorCmdWrite,
                kMotorRegEncoderControl,
                kResetBothEncoders,
                (uint8_t) sizeof(kResetBothEncoders),
                &response);
            if (status != drivers::DRIVER_OK) {
                WriteStatusLine("motor enc reset: ", status);
                return;
            }

            const bool ok = (response.length == 1U) &&
                            (response.data[0] == kMotorStatusOk);
            services::Shell_WriteString("motor enc reset: ");
            services::Shell_WriteString(ok ? "ok" : "error");
            services::Shell_WriteString("\r\n");
            return;
        }

        if (argc != 2) {
            PrintMotorUsage();
            return;
        }

        drivers::DriverStatus status = MotorRequest(
            kMotorCmdRead,
            kMotorRegM1Encoder,
            0,
            kMotorEncoderBlockLength,
            &response);
        if (status != drivers::DRIVER_OK) {
            WriteStatusLine("motor enc m1: ", status);
            return;
        }
        PrintMotorEncoderLine("motor enc m1", response);

        status = MotorRequest(kMotorCmdRead,
                              kMotorRegM2Encoder,
                              0,
                              kMotorEncoderBlockLength,
                              &response);
        if (status != drivers::DRIVER_OK) {
            WriteStatusLine("motor enc m2: ", status);
            return;
        }
        PrintMotorEncoderLine("motor enc m2", response);
        return;
    }

    if (StrEqual(argv[1], "rpm")) {
        MotorProtocolFrame response = {};

        if (argc != 2) {
            PrintMotorUsage();
            return;
        }

        const drivers::DriverStatus status = MotorRequest(
            kMotorCmdRead,
            kMotorRegSpeedRpmBlock,
            0,
            kMotorSpeedRpmBlockLength,
            &response);
        if (status != drivers::DRIVER_OK) {
            WriteStatusLine("motor rpm: ", status);
            return;
        }

        PrintMotorRpmBlock(response);
        return;
    }

    if (StrEqual(argv[1], "cfg")) {
        MotorProtocolFrame response = {};

        if (argc == 2) {
            const drivers::DriverStatus status = MotorRequest(
                kMotorCmdRead,
                kMotorRegCountsPerRev,
                0,
                kMotorCountsPerRevLength,
                &response);
            if (status != drivers::DRIVER_OK) {
                WriteStatusLine("motor cfg: ", status);
                return;
            }

            PrintMotorConfigBlock(response);
            return;
        }

        if (argc != 4) {
            PrintMotorUsage();
            return;
        }

        uint32_t m1_counts_per_rev = 0U;
        uint32_t m2_counts_per_rev = 0U;
        uint8_t data[kMotorCountsPerRevLength] = {
            0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U
        };

        if ((!ParseUint32(argv[2],
                          kMotorCountsPerRevMax,
                          &m1_counts_per_rev)) ||
            (!ParseUint32(argv[3],
                          kMotorCountsPerRevMax,
                          &m2_counts_per_rev))) {
            PrintMotorUsage();
            return;
        }

        EncodeMotorUint32Le(m1_counts_per_rev, &data[0]);
        EncodeMotorUint32Le(m2_counts_per_rev, &data[4]);

        const drivers::DriverStatus status = MotorRequest(
            kMotorCmdWrite,
            kMotorRegCountsPerRev,
            data,
            kMotorCountsPerRevLength,
            &response);
        if (status != drivers::DRIVER_OK) {
            WriteStatusLine("motor cfg: ", status);
            return;
        }

        services::Shell_WriteString("motor cfg: ");
        services::Shell_WriteString(MotorStatusOkResponse(response) ?
                                    "ok" : "error");
        services::Shell_WriteString("\r\n");
        return;
    }

    if (StrEqual(argv[1], "pid")) {
        MotorProtocolFrame response = {};

        if (argc == 2) {
            const drivers::DriverStatus status = MotorRequest(
                kMotorCmdRead,
                kMotorRegSpeedPid,
                0,
                kMotorSpeedPidLength,
                &response);
            if (status != drivers::DRIVER_OK) {
                WriteStatusLine("motor pid: ", status);
                return;
            }

            PrintMotorPidBlock(response);
            return;
        }

        if ((argc < 5) || (argc > 7)) {
            PrintMotorUsage();
            return;
        }

        uint8_t data[kMotorSpeedPidLength] = { 0U, 0U, 0U, 0U, 0U };
        const uint8_t length = static_cast<uint8_t>(argc - 2);
        for (uint8_t i = 0U; i < length; i++) {
            const uint32_t max_value = (i >= 3U) ? 100U : 0xFFU;
            if (!ParseUint32(argv[2U + i], max_value, &value)) {
                PrintMotorUsage();
                return;
            }
            data[i] = static_cast<uint8_t>(value);
        }

        const drivers::DriverStatus status = MotorRequest(
            kMotorCmdWrite,
            kMotorRegSpeedPid,
            data,
            length,
            &response);
        if (status != drivers::DRIVER_OK) {
            WriteStatusLine("motor pid: ", status);
            return;
        }

        services::Shell_WriteString("motor pid: ");
        services::Shell_WriteString(MotorStatusOkResponse(response) ?
                                    "ok" : "error");
        services::Shell_WriteString("\r\n");
        return;
    }

    if (StrEqual(argv[1], "pos")) {
        MotorProtocolFrame response = {};

        if (argc != 2) {
            PrintMotorUsage();
            return;
        }

        const drivers::DriverStatus status = MotorRequest(
            kMotorCmdRead,
            kMotorRegPositionBlock,
            0,
            kMotorPositionBlockLength,
            &response);
        if (status != drivers::DRIVER_OK) {
            WriteStatusLine("motor pos: ", status);
            return;
        }

        PrintMotorPositionBlock(response);
        return;
    }

    if (StrEqual(argv[1], "pospid")) {
        MotorProtocolFrame response = {};

        if (argc == 2) {
            const drivers::DriverStatus status = MotorRequest(
                kMotorCmdRead,
                kMotorRegPositionPid,
                0,
                kMotorPositionPidLength,
                &response);
            if (status != drivers::DRIVER_OK) {
                WriteStatusLine("motor pospid: ", status);
                return;
            }

            PrintMotorPositionPidBlock(response);
            return;
        }

        if ((argc < 5) || (argc > 7)) {
            PrintMotorUsage();
            return;
        }

        uint8_t data[kMotorPositionPidLength] = {
            0U, 0U, 0U, 0U, 0U, 0U, 0U
        };
        uint32_t max_rpm = 0U;
        uint32_t tolerance = 0U;
        uint8_t length = 3U;

        for (uint8_t i = 0U; i < 3U; i++) {
            if (!ParseUint32(argv[2U + i], 0xFFU, &value)) {
                PrintMotorUsage();
                return;
            }
            data[i] = static_cast<uint8_t>(value);
        }

        if (argc >= 6) {
            if (!ParseUint32(argv[5], kMotorPositionMaxRpm, &max_rpm)) {
                PrintMotorUsage();
                return;
            }
            EncodeMotorUint16Le(static_cast<uint16_t>(max_rpm), &data[3]);
            length = 5U;
        }

        if (argc == 7) {
            if (!ParseUint32(argv[6],
                             kMotorPositionToleranceMax,
                             &tolerance)) {
                PrintMotorUsage();
                return;
            }
            EncodeMotorUint16Le(static_cast<uint16_t>(tolerance), &data[5]);
            length = 7U;
        }

        const drivers::DriverStatus status = MotorRequest(
            kMotorCmdWrite,
            kMotorRegPositionPid,
            data,
            length,
            &response);
        if (status != drivers::DRIVER_OK) {
            WriteStatusLine("motor pospid: ", status);
            return;
        }

        services::Shell_WriteString("motor pospid: ");
        services::Shell_WriteString(MotorStatusOkResponse(response) ?
                                    "ok" : "error");
        services::Shell_WriteString("\r\n");
        return;
    }

    if (StrEqual(argv[1], "posctl")) {
        MotorProtocolFrame response = {};

        if (argc == 2) {
            const drivers::DriverStatus status = MotorRequest(
                kMotorCmdRead,
                kMotorRegPositionControl,
                0,
                kMotorPositionControlLength,
                &response);
            if (status != drivers::DRIVER_OK) {
                WriteStatusLine("motor posctl: ", status);
                return;
            }

            PrintMotorPositionControlBlock(response);
            return;
        }

        if ((argc < 4) || (argc > 6)) {
            PrintMotorUsage();
            return;
        }

        uint8_t data[kMotorPositionControlLength] = { 0U, 0U, 0U, 0U, 0U };
        uint32_t max_duty = 0U;
        uint32_t exit_tolerance = 0U;
        uint32_t settle_ms = 0U;
        uint8_t length = 2U;

        if ((!ParseUint32(argv[2], 100U, &value)) ||
            (!ParseUint32(argv[3], 100U, &max_duty))) {
            PrintMotorUsage();
            return;
        }
        data[0] = static_cast<uint8_t>(value);
        data[1] = static_cast<uint8_t>(max_duty);

        if (argc >= 5) {
            if (!ParseUint32(argv[4],
                             kMotorPositionToleranceMax,
                             &exit_tolerance)) {
                PrintMotorUsage();
                return;
            }
            EncodeMotorUint16Le(static_cast<uint16_t>(exit_tolerance),
                                &data[2]);
            length = 4U;
        }

        if (argc == 6) {
            if (!ParseUint32(argv[5],
                             kMotorPositionSettleMsMax,
                             &settle_ms)) {
                PrintMotorUsage();
                return;
            }
            data[4] = static_cast<uint8_t>((settle_ms + 9U) / 10U);
            length = 5U;
        }

        const drivers::DriverStatus status = MotorRequest(
            kMotorCmdWrite,
            kMotorRegPositionControl,
            data,
            length,
            &response);
        if (status != drivers::DRIVER_OK) {
            WriteStatusLine("motor posctl: ", status);
            return;
        }

        services::Shell_WriteString("motor posctl: ");
        services::Shell_WriteString(MotorStatusOkResponse(response) ?
                                    "ok" : "error");
        services::Shell_WriteString("\r\n");
        return;
    }

    if (StrEqual(argv[1], "reg")) {
        MotorProtocolFrame response = {};
        uint32_t reg = 0U;
        uint32_t length = 0U;

        if ((argc != 4) ||
            (!ParseUint32(argv[2], 0xFFU, &reg)) ||
            (!ParseUint32(argv[3], kMotorMaxPayloadLength, &length)) ||
            (length == 0U)) {
            PrintMotorUsage();
            return;
        }

        const drivers::DriverStatus status = MotorRequest(
            kMotorCmdRead,
            (uint8_t) reg,
            0,
            (uint8_t) length,
            &response);
        if (status != drivers::DRIVER_OK) {
            WriteStatusLine("motor reg: ", status);
            return;
        }

        PrintMotorFrameData("motor reg", response);
        return;
    }

    if (StrEqual(argv[1], "set")) {
        MotorProtocolFrame response = {};
        uint32_t reg = 0U;
        uint8_t data[kMotorMaxPayloadLength];
        uint8_t length = 0U;

        if ((argc < 4) || (argc > 35) ||
            (!ParseUint32(argv[2], 0xFFU, &reg))) {
            PrintMotorUsage();
            return;
        }

        for (int arg = 3; arg < argc; arg++) {
            if (!ParseUint32(argv[arg], 0xFFU, &value)) {
                PrintMotorUsage();
                return;
            }
            data[length] = (uint8_t) value;
            length++;
        }

        const drivers::DriverStatus status = MotorRequest(
            kMotorCmdWrite,
            (uint8_t) reg,
            data,
            length,
            &response);
        if (status != drivers::DRIVER_OK) {
            WriteStatusLine("motor set: ", status);
            return;
        }

        const bool ok = (response.length == 1U) &&
                        (response.data[0] == kMotorStatusOk);
        services::Shell_WriteString("motor set: ");
        services::Shell_WriteString(ok ? "ok" : "error");
        services::Shell_WriteString("\r\n");
        return;
    }

    if (StrEqual(argv[1], "stop")) {
        MotorProtocolFrame response = {};
        static const uint8_t kM1Stop[] = { 0U, 0U };
        static const uint8_t kM2Stop[] = { 0U, 0U };
        static const uint8_t kTargetZero[] = { 0U, 0U };

        if (argc != 2) {
            PrintMotorUsage();
            return;
        }

        drivers::DriverStatus status = MotorRequest(
            kMotorCmdWrite,
            kMotorRegM1Mode,
            kM1Stop,
            (uint8_t) sizeof(kM1Stop),
            &response);
        if ((status == drivers::DRIVER_OK) &&
            ((response.length != 1U) || (response.data[0] != kMotorStatusOk))) {
            status = drivers::DRIVER_ERROR;
        }
        if (status != drivers::DRIVER_OK) {
            WriteStatusLine("motor stop m1: ", status);
            return;
        }

        status = MotorRequest(kMotorCmdWrite,
                              kMotorRegM2Mode,
                              kM2Stop,
                              (uint8_t) sizeof(kM2Stop),
                              &response);
        if ((status == drivers::DRIVER_OK) &&
            ((response.length != 1U) || (response.data[0] != kMotorStatusOk))) {
            status = drivers::DRIVER_ERROR;
        }
        if (status != drivers::DRIVER_OK) {
            WriteStatusLine("motor stop m2: ", status);
            return;
        }

        status = MotorRequest(kMotorCmdWrite,
                              kMotorRegM1TargetRpm,
                              kTargetZero,
                              (uint8_t) sizeof(kTargetZero),
                              &response);
        if ((status == drivers::DRIVER_OK) &&
            ((response.length != 1U) || (response.data[0] != kMotorStatusOk))) {
            status = drivers::DRIVER_ERROR;
        }
        if (status != drivers::DRIVER_OK) {
            WriteStatusLine("motor stop m1 target: ", status);
            return;
        }

        status = MotorRequest(kMotorCmdWrite,
                              kMotorRegM2TargetRpm,
                              kTargetZero,
                              (uint8_t) sizeof(kTargetZero),
                              &response);
        if ((status == drivers::DRIVER_OK) &&
            ((response.length != 1U) || (response.data[0] != kMotorStatusOk))) {
            status = drivers::DRIVER_ERROR;
        }

        WriteStatusLine("motor stop: ", status);
        return;
    }

    if (StrEqual(argv[1], "m1") || StrEqual(argv[1], "m2")) {
        MotorProtocolFrame response = {};
        const bool motor1 = StrEqual(argv[1], "m1");
        const uint8_t mode_reg = motor1 ? kMotorRegM1Mode : kMotorRegM2Mode;
        const uint8_t target_rpm_reg =
            motor1 ? kMotorRegM1TargetRpm : kMotorRegM2TargetRpm;
        const uint8_t target_position_reg =
            motor1 ? kMotorRegM1TargetPosition : kMotorRegM2TargetPosition;
        uint8_t data[3] = { 0U, 0U, 0U };
        uint8_t length = 0U;

        if (argc < 3) {
            PrintMotorUsage();
            return;
        }

        if (StrEqual(argv[2], "coast")) {
            if (argc != 3) {
                PrintMotorUsage();
                return;
            }
            data[0] = 0U;
            data[1] = 0U;
            length = 2U;
        } else if (StrEqual(argv[2], "brake")) {
            if (argc != 3) {
                PrintMotorUsage();
                return;
            }
            data[0] = 2U;
            data[1] = 0U;
            length = 2U;
        } else if (StrEqual(argv[2], "run")) {
            uint32_t duty = 0U;

            if ((argc != 4) && (argc != 5)) {
                PrintMotorUsage();
                return;
            }
            if (!ParseUint32(argv[3], 100U, &duty)) {
                PrintMotorUsage();
                return;
            }
            data[0] = 1U;
            data[1] = (uint8_t) duty;
            data[2] = 0U;
            if (argc == 5) {
                if (StrEqual(argv[4], "rev")) {
                    data[2] = 1U;
                } else if (!StrEqual(argv[4], "fwd")) {
                    PrintMotorUsage();
                    return;
                }
            }
            length = 3U;
        } else if (StrEqual(argv[2], "speed")) {
            uint32_t rpm = 0U;
            uint8_t rpm_data[2] = { 0U, 0U };

            if ((argc != 4) && (argc != 5)) {
                PrintMotorUsage();
                return;
            }
            if (!ParseUint32(argv[3], kMotorSpeedMaxRpm, &rpm)) {
                PrintMotorUsage();
                return;
            }

            data[0] = kMotorModeSpeed;
            data[1] = 0U;
            data[2] = 0U;
            if (argc == 5) {
                if (StrEqual(argv[4], "rev")) {
                    data[2] = 1U;
                } else if (!StrEqual(argv[4], "fwd")) {
                    PrintMotorUsage();
                    return;
                }
            }
            length = 3U;

            EncodeMotorUint16Le(static_cast<uint16_t>(rpm), rpm_data);
            const drivers::DriverStatus target_status = MotorRequest(
                kMotorCmdWrite,
                target_rpm_reg,
                rpm_data,
                (uint8_t) sizeof(rpm_data),
                &response);
            if (target_status != drivers::DRIVER_OK) {
                WriteStatusLine("motor speed target: ", target_status);
                return;
            }
            if (!MotorStatusOkResponse(response)) {
                services::Shell_WriteLine("motor speed target: error");
                return;
            }
        } else if (StrEqual(argv[2], "hold") ||
                   StrEqual(argv[2], "pos") ||
                   StrEqual(argv[2], "posrel")) {
            int32_t target_count = 0;
            uint8_t position_data[4] = { 0U, 0U, 0U, 0U };

            if (StrEqual(argv[2], "hold")) {
                if (argc != 3) {
                    PrintMotorUsage();
                    return;
                }

                const drivers::DriverStatus read_status =
                    MotorReadEncoderCount(motor1, &target_count);
                if (read_status != drivers::DRIVER_OK) {
                    WriteStatusLine("motor hold encoder: ", read_status);
                    return;
                }
            } else {
                int32_t degrees = 0;
                int32_t delta_count = 0;
                uint32_t counts_per_rev = 0U;

                if (argc != 4) {
                    PrintMotorUsage();
                    return;
                }
                if (!ParseInt32(argv[3],
                                -kMotorPositionDegreeLimit,
                                kMotorPositionDegreeLimit,
                                &degrees)) {
                    PrintMotorUsage();
                    return;
                }

                drivers::DriverStatus read_status =
                    MotorReadCountsPerRev(motor1, &counts_per_rev);
                if (read_status != drivers::DRIVER_OK) {
                    WriteStatusLine("motor pos cfg: ", read_status);
                    return;
                }
                if (!MotorDegreesToCounts(degrees,
                                          counts_per_rev,
                                          &delta_count)) {
                    services::Shell_WriteLine("motor pos: range");
                    return;
                }

                if (StrEqual(argv[2], "posrel")) {
                    int32_t current_count = 0;
                    read_status = MotorReadEncoderCount(motor1, &current_count);
                    if (read_status != drivers::DRIVER_OK) {
                        WriteStatusLine("motor pos encoder: ", read_status);
                        return;
                    }

                    const int64_t target =
                        static_cast<int64_t>(current_count) +
                        static_cast<int64_t>(delta_count);
                    if ((target > 2147483647LL) ||
                        (target < (-2147483647LL - 1LL))) {
                        services::Shell_WriteLine("motor pos: range");
                        return;
                    }
                    target_count = static_cast<int32_t>(target);
                } else {
                    target_count = delta_count;
                }
            }

            EncodeMotorInt32Le(target_count, position_data);
            const drivers::DriverStatus target_status = MotorRequest(
                kMotorCmdWrite,
                target_position_reg,
                position_data,
                (uint8_t) sizeof(position_data),
                &response);
            if (target_status != drivers::DRIVER_OK) {
                WriteStatusLine("motor position target: ", target_status);
                return;
            }
            if (!MotorStatusOkResponse(response)) {
                services::Shell_WriteLine("motor position target: error");
                return;
            }

            data[0] = kMotorModePosition;
            data[1] = 0U;
            data[2] = 0U;
            length = 3U;
        } else {
            PrintMotorUsage();
            return;
        }

        const drivers::DriverStatus status = MotorRequest(
            kMotorCmdWrite,
            mode_reg,
            data,
            length,
            &response);
        if (status != drivers::DRIVER_OK) {
            WriteStatusLine("motor drive: ", status);
            return;
        }

        const bool ok = (response.length == 1U) &&
                        (response.data[0] == kMotorStatusOk);
        services::Shell_WriteString("motor ");
        services::Shell_WriteString(argv[1]);
        services::Shell_WriteString(": ");
        services::Shell_WriteString(ok ? "ok" : "error");
        services::Shell_WriteString("\r\n");
        return;
    }

    if (StrEqual(argv[1], "clear")) {
        if (argc != 2) {
            PrintMotorUsage();
            return;
        }

        const drivers::DriverStatus status =
            board::Board_MotorDriverClearRxBuffer();
        WriteStatusLine("motor clear: ", status);
        return;
    }

    if (StrEqual(argv[1], "test")) {
        static const uint8_t kTestData[] = { 'p', 'i', 'n', 'g', '\r', '\n' };

        if (argc != 2) {
            PrintMotorUsage();
            return;
        }

        const drivers::DriverStatus status = board::Board_MotorDriverWrite(
            kTestData,
            (uint16_t) sizeof(kTestData));
        WriteStatusLine("motor test: ", status);
        return;
    }

    if (StrEqual(argv[1], "send") || StrEqual(argv[1], "line")) {
        uint16_t bytes_written = 0U;

        if (argc < 3) {
            PrintMotorUsage();
            return;
        }

        const drivers::DriverStatus status = MotorWriteArgs(
            argc,
            argv,
            StrEqual(argv[1], "line"),
            &bytes_written);
        if (status != drivers::DRIVER_OK) {
            WriteStatusLine("motor send: ", status);
            return;
        }

        services::Shell_WriteString("motor tx bytes=");
        services::Shell_WriteUInt32(bytes_written);
        services::Shell_WriteString("\r\n");
        return;
    }

    if (StrEqual(argv[1], "hex")) {
        uint8_t data[16];
        uint16_t length = 0U;

        if ((argc < 3) || (argc > 18)) {
            PrintMotorUsage();
            return;
        }

        for (int arg = 2; arg < argc; arg++) {
            if (!ParseUint32(argv[arg], 0xFFU, &value)) {
                PrintMotorUsage();
                return;
            }
            data[length] = (uint8_t) value;
            length++;
        }

        const drivers::DriverStatus status =
            board::Board_MotorDriverWrite(data, length);
        if (status != drivers::DRIVER_OK) {
            WriteStatusLine("motor hex: ", status);
            return;
        }

        services::Shell_WriteString("motor tx bytes=");
        services::Shell_WriteUInt32(length);
        services::Shell_WriteString("\r\n");
        return;
    }

    if (StrEqual(argv[1], "read")) {
        uint8_t data[kMotorShellMaxReadBytes];
        uint16_t length = kMotorShellMaxReadBytes;
        uint16_t actual = 0U;

        if ((argc != 2) && (argc != 3)) {
            PrintMotorUsage();
            return;
        }

        if (argc == 3) {
            if ((!ParseUint32(argv[2], kMotorShellMaxReadBytes, &value)) ||
                (value == 0U)) {
                PrintMotorUsage();
                return;
            }
            length = (uint16_t) value;
        }

        while ((actual < length) &&
               board::Board_MotorDriverReadByte(&data[actual])) {
            actual++;
        }

        services::Shell_WriteString("motor rx len=");
        services::Shell_WriteUInt32(actual);
        services::Shell_WriteString(" hex:");
        for (uint16_t i = 0U; i < actual; i++) {
            services::Shell_WriteString(" ");
            WriteHex8(data[i]);
        }
        services::Shell_WriteString(" ascii:");
        for (uint16_t i = 0U; i < actual; i++) {
            services::Shell_WriteChar(IsPrintableAscii(data[i]) ?
                                      (char) data[i] : '.');
        }
        services::Shell_WriteString("\r\n");
        return;
    }

    PrintMotorUsage();
#else
    (void) argc;
    (void) argv;
    services::Shell_WriteLine("motor: disabled");
#endif
}

void AdcCommand(int argc, const char * const argv[])
{
    (void) argc;
    (void) argv;

    services::Shell_WriteLine("adc: no ADC driver registered");
}

void PwmCommand(int argc, const char * const argv[])
{
    uint8_t duty = 0U;

    if ((argc != 2) || (!ParsePercent(argv[1], &duty))) {
        services::Shell_WriteLine("usage: pwm 0..100");
        return;
    }

    services::Shell_WriteString("pwm duty request ");
    services::Shell_WriteUInt32(duty);
    services::Shell_WriteLine("%, no PWM driver registered");
}

} /* namespace */

void AppShell_RegisterCommands(void)
{
    (void) services::Shell_RegisterCommand("version",
                                           "Show firmware and board info",
                                           VersionCommand);
    (void) services::Shell_RegisterCommand("reset",
                                           "Reset the MCU",
                                           ResetCommand);
    (void) services::Shell_RegisterCommand("led",
                                           "Control LEDs: led <1|2|3|all> on|off|toggle|status",
                                           LedCommand);
    (void) services::Shell_RegisterCommand("buzzer",
                                           "Control buzzer: buzzer on|off",
                                           BuzzerCommand);
#if FEATURE_ENABLE_BUTTONS
    (void) services::Shell_RegisterCommand("button",
                                           "Show button states",
                                           ButtonCommand);
#endif
#if FEATURE_ENABLE_FRAM
    (void) services::Shell_RegisterCommand(
        "fram",
        "FRAM: fram status|recover|test|read <addr> <len>|write <addr> <byte>",
        FramCommand);
#endif
#if FEATURE_ENABLE_INA219
    (void) services::Shell_RegisterCommand(
        "ina219",
        "INA219: status|scan|addr|recover|config|read|raw|reg",
        Ina219Command);
#endif
#if FEATURE_ENABLE_OLED
    (void) services::Shell_RegisterCommand(
        "oled",
        "OLED: status|init|clear|fill|test|invert|on|off",
        OledCommand);
#endif
#if FEATURE_ENABLE_IMU
    (void) services::Shell_RegisterCommand(
        "imu",
        "IMU SPI: status|lis whoami|lis reg <addr>|icm reg <addr>",
        ImuCommand);
#endif
#if FEATURE_ENABLE_LORA
    (void) services::Shell_RegisterCommand(
        "lora",
        "LoRa UART: status|send|line|hex|read|clear|test",
        LoraCommand);
#endif
#if FEATURE_ENABLE_MOTOR_DRIVER
    (void) services::Shell_RegisterCommand(
        "motor",
        "MotorDriver: status|bus|ping|info|reg|set|run",
        MotorCommand);
#endif
    (void) services::Shell_RegisterCommand(
        "i2c",
        "I2C diag: list|status|recover|scan|probe|read|write|test",
        I2cCommand);
    (void) services::Shell_RegisterCommand("adc",
                                           "Read ADC placeholder",
                                           AdcCommand);
    (void) services::Shell_RegisterCommand("pwm",
                                           "Set PWM placeholder: pwm 0..100",
                                           PwmCommand);
}

} /* namespace app */
