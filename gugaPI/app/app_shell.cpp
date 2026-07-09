#include "app/app_shell.h"

#include <stdint.h>

#include "app/chassis.h"
#include "app/app_grayscale.h"
#include "app/app_imu.h"
#include "app/config_store.h"
#include "app/motor_driver_client.h"
#include "board/board_buzzer.h"
#include "board/board_button.h"
#include "board/board_config.h"
#include "board/board_fram.h"
#include "board/board_gy931.h"
#include "board/board_grayscale.h"
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
#include "services/scheduler.h"
#include "services/shell.h"
#include "services/time.h"
#include "ti_msp_dl_config.h"

namespace app {
namespace {

namespace motor = motor_driver_client;

static const uint16_t kFramShellMaxReadBytes = 32U;
#if FEATURE_ENABLE_LORA
static const uint16_t kLoraShellMaxReadBytes = 64U;
#endif
static const uint16_t kMotorShellMaxReadBytes = 64U;
#if FEATURE_ENABLE_IMU
static const uint32_t kImuPinWiggleDefaultLoops = 20000U;
static const uint32_t kImuPinWiggleMaxLoops = 1000000U;
static const uint32_t kImuSpiBurstDefaultBytes = 4096U;
static const uint32_t kImuSpiBurstMaxBytes = 1000000U;
static const uint32_t kImuSpiSampleDefaultBytes = 8U;
static const uint32_t kImuSpiSampleMaxBytes = 16U;
#endif
static const uint8_t kOledTextRows = 4U;
static const uint8_t kOledTextCols = 21U;
static const uint8_t kGy931MaxReadWords = drivers::GY931_MAX_READ_WORDS;
static const uint32_t kGy931OledTaskPeriodMs = 50U;
static const uint32_t kGy931OledDefaultPeriodMs = 200U;
static const uint32_t kGy931OledMinPeriodMs = 50U;
static const uint32_t kGy931OledMaxPeriodMs = 5000U;
static const uint32_t kIna219OledTaskPeriodMs = 50U;
static const uint32_t kIna219OledDefaultPeriodMs = 500U;
static const uint32_t kIna219OledMinPeriodMs = 100U;
static const uint32_t kIna219OledMaxPeriodMs = 5000U;
static const uint32_t kGrayOledTaskPeriodMs = 50U;
static const uint32_t kGrayOledDefaultPeriodMs = 200U;
static const uint32_t kGrayOledMinPeriodMs = 50U;
static const uint32_t kGrayOledMaxPeriodMs = 5000U;
static const int32_t kChassisLinearLimitMmS = 5000;
static const int32_t kChassisAngularLimitMdegS = 720000;
motor::Client g_motorClient = { motor::TRANSPORT_I2C,
                                motor::kI2cDefaultAddress };
#if FEATURE_ENABLE_OLED && (FEATURE_ENABLE_GY931 || FEATURE_ENABLE_INA219)
bool g_gy931OledEnabled = false;
bool g_gy931OledTaskRegistered = false;
services::SchedulerTaskId g_gy931OledTaskId = 0U;
uint32_t g_gy931OledPeriodMs = kGy931OledDefaultPeriodMs;
uint32_t g_gy931OledLastUpdateMs = 0U;
drivers::DriverStatus g_gy931OledLastStatus = drivers::DRIVER_OK;
#endif
#if FEATURE_ENABLE_INA219 && FEATURE_ENABLE_OLED
bool g_ina219OledEnabled = false;
bool g_ina219OledTaskRegistered = false;
services::SchedulerTaskId g_ina219OledTaskId = 0U;
uint32_t g_ina219OledPeriodMs = kIna219OledDefaultPeriodMs;
uint32_t g_ina219OledLastUpdateMs = 0U;
drivers::DriverStatus g_ina219OledLastStatus = drivers::DRIVER_OK;

#if FEATURE_ENABLE_IMU
static const uint32_t kImuOledTaskPeriodMs = 50U;
static const uint32_t kImuOledDefaultPeriodMs = 200U;
static const uint32_t kImuOledMinPeriodMs = 50U;
static const uint32_t kImuOledMaxPeriodMs = 5000U;

bool g_imuOledEnabled = false;
bool g_imuOledTaskRegistered = false;
services::SchedulerTaskId g_imuOledTaskId = 0U;
uint32_t g_imuOledPeriodMs = kImuOledDefaultPeriodMs;
uint32_t g_imuOledLastUpdateMs = 0U;
drivers::DriverStatus g_imuOledLastStatus = drivers::DRIVER_OK;
#endif
#endif
#if FEATURE_ENABLE_GRAYSCALE && FEATURE_ENABLE_OLED
bool g_grayOledEnabled = false;
bool g_grayOledTaskRegistered = false;
services::SchedulerTaskId g_grayOledTaskId = 0U;
uint32_t g_grayOledPeriodMs = kGrayOledDefaultPeriodMs;
uint32_t g_grayOledLastUpdateMs = 0U;
drivers::DriverStatus g_grayOledLastStatus = drivers::DRIVER_OK;
#endif

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

#if FEATURE_ENABLE_SHELL_DIAGNOSTICS
bool ParsePercent(const char *text, uint8_t *outValue)
{
    uint32_t value = 0U;

    if ((outValue == 0) || (!ParseUint32(text, 100U, &value))) {
        return false;
    }

    *outValue = (uint8_t) value;
    return true;
}
#endif

bool ParseOnOff(const char *text, bool *outValue)
{
    if ((text == 0) || (outValue == 0)) {
        return false;
    }
    if (StrEqual(text, "on") || StrEqual(text, "1")) {
        *outValue = true;
        return true;
    }
    if (StrEqual(text, "off") || StrEqual(text, "0")) {
        *outValue = false;
        return true;
    }
    return false;
}

void SetFlagValue(uint8_t *flags, uint8_t mask, bool enabled)
{
    if (flags == 0) {
        return;
    }
    if (enabled) {
        *flags = static_cast<uint8_t>(*flags | mask);
    } else {
        *flags = static_cast<uint8_t>(*flags & static_cast<uint8_t>(~mask));
    }
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

void WriteFixedMilli(int32_t value)
{
    uint32_t magnitude = 0U;

    if (value < 0) {
        services::Shell_WriteString("-");
        magnitude = (uint32_t) (-(value + 1)) + 1U;
    } else {
        magnitude = (uint32_t) value;
    }

    services::Shell_WriteUInt32(magnitude / 1000U);
    services::Shell_WriteString(".");

    const uint32_t fraction = magnitude % 1000U;
    char text[4] = { '0', '0', '0', '\0' };
    text[0] = (char) ('0' + ((fraction / 100U) % 10U));
    text[1] = (char) ('0' + ((fraction / 10U) % 10U));
    text[2] = (char) ('0' + (fraction % 10U));
    services::Shell_WriteString(text);
}

void WriteSignedVector3Milli(const char *label,
                             const int32_t values[3],
                             const char *unit)
{
    services::Shell_WriteString(label);
    services::Shell_WriteString("=");
    WriteFixedMilli(values[0]);
    services::Shell_WriteString(",");
    WriteFixedMilli(values[1]);
    services::Shell_WriteString(",");
    WriteFixedMilli(values[2]);
    if (unit != 0) {
        services::Shell_WriteString(unit);
    }
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

void PrintParamUsage(void)
{
    services::Shell_WriteLine("usage:");
    services::Shell_WriteLine("  param status");
    services::Shell_WriteLine("  param get [name]");
    services::Shell_WriteLine("  param set <name> <value>");
    services::Shell_WriteLine("  param save");
    services::Shell_WriteLine("  param load");
    services::Shell_WriteLine("  param reset");
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
    services::Shell_WriteLine("  ina219 oled on [period_ms 100..5000]|off|status|once");
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

void PrintGy931Usage(void)
{
    services::Shell_WriteLine("usage:");
    services::Shell_WriteLine("  gy931 status");
    services::Shell_WriteLine("  gy931 init");
    services::Shell_WriteLine("  gy931 recover");
    services::Shell_WriteLine("  gy931 scan [start end]");
    services::Shell_WriteLine("  gy931 addr [0x08..0x77]");
    services::Shell_WriteLine("  gy931 angle");
    services::Shell_WriteLine("  gy931 sample");
    services::Shell_WriteLine("  gy931 raw <reg> <words 1..16>");
    services::Shell_WriteLine("  gy931 oled on [period_ms 50..5000]|off|status|once");
}

#if FEATURE_ENABLE_OLED && (FEATURE_ENABLE_GY931 || FEATURE_ENABLE_INA219 || FEATURE_ENABLE_IMU || FEATURE_ENABLE_GRAYSCALE)
char *AppendChar(char *cursor, char *end, char ch)
{
    if (cursor < end) {
        *cursor = ch;
        cursor++;
    }
    return cursor;
}

char *AppendString(char *cursor, char *end, const char *text)
{
    while ((text != 0) && (*text != '\0') && (cursor < end)) {
        *cursor = *text;
        cursor++;
        text++;
    }
    return cursor;
}

char *AppendUIntDec(char *cursor, char *end, uint32_t value)
{
    char digits[10];
    uint8_t count = 0U;

    do {
        digits[count] = (char) ('0' + (value % 10U));
        value /= 10U;
        count++;
    } while ((value != 0U) && (count < sizeof(digits)));

    while ((count > 0U) && (cursor < end)) {
        count--;
        *cursor = digits[count];
        cursor++;
    }

    return cursor;
}

char *AppendIntDec(char *cursor, char *end, int32_t value)
{
    uint32_t magnitude = 0U;

    if (value < 0) {
        cursor = AppendChar(cursor, end, '-');
        magnitude = (uint32_t) (-(value + 1)) + 1U;
    } else {
        magnitude = (uint32_t) value;
    }

    return AppendUIntDec(cursor, end, magnitude);
}

char *AppendHex8Text(char *cursor, char *end, uint8_t value)
{
    cursor = AppendString(cursor, end, "0x");
    cursor = AppendChar(cursor, end, HexDigit((uint8_t) (value >> 4U)));
    return AppendChar(cursor, end, HexDigit(value));
}

char *AppendFixedMilliText(char *cursor, char *end, int32_t value)
{
    uint32_t magnitude = 0U;

    if (value < 0) {
        cursor = AppendChar(cursor, end, '-');
        magnitude = (uint32_t) (-(value + 1)) + 1U;
    } else {
        magnitude = (uint32_t) value;
    }

    cursor = AppendUIntDec(cursor, end, magnitude / 1000U);
    cursor = AppendChar(cursor, end, '.');

    const uint32_t fraction = magnitude % 1000U;
    cursor = AppendChar(cursor, end,
                        (char) ('0' + ((fraction / 100U) % 10U)));
    cursor = AppendChar(cursor, end,
                        (char) ('0' + ((fraction / 10U) % 10U)));
    return AppendChar(cursor, end, (char) ('0' + (fraction % 10U)));
}

void FinishOledLine(char *buffer, char *cursor)
{
    char *end = &buffer[kOledTextCols];
    while (cursor < end) {
        *cursor = ' ';
        cursor++;
    }
    buffer[kOledTextCols] = '\0';
}

drivers::DriverStatus OledTextWriteLine(uint8_t row, const char *text)
{
    char line[kOledTextCols + 1U];
    char *cursor = AppendString(line, &line[kOledTextCols], text);
    FinishOledLine(line, cursor);
    return board::Board_OledWriteText(row, 0U, line);
}

drivers::DriverStatus SchedulerStatusToDriverStatus(
    services::SchedulerStatus status)
{
    switch (status) {
        case services::SCHEDULER_OK:
            return drivers::DRIVER_OK;
        case services::SCHEDULER_ERROR_INVALID_ARG:
        case services::SCHEDULER_ERROR_INVALID_ID:
            return drivers::DRIVER_ERROR_INVALID_ARG;
        case services::SCHEDULER_ERROR_FULL:
            return drivers::DRIVER_ERROR_BUSY;
        default:
            return drivers::DRIVER_ERROR;
    }
}

#if FEATURE_ENABLE_GY931
drivers::DriverStatus Gy931OledWriteStatusLine(void)
{
    char line[kOledTextCols + 1U];
    char *cursor = line;

    cursor = AppendString(cursor, &line[kOledTextCols], "GY931 ");
    cursor = AppendHex8Text(cursor, &line[kOledTextCols],
                            board::Board_Gy931Address());
    cursor = AppendChar(cursor, &line[kOledTextCols], ' ');
    cursor = AppendUIntDec(cursor, &line[kOledTextCols], g_gy931OledPeriodMs);
    cursor = AppendString(cursor, &line[kOledTextCols], "ms");
    FinishOledLine(line, cursor);
    return board::Board_OledWriteText(0U, 0U, line);
}

drivers::DriverStatus Gy931OledWriteAngleLine(uint8_t row,
                                              const char *label,
                                              int32_t angle_mdeg)
{
    char line[kOledTextCols + 1U];
    char *cursor = line;

    cursor = AppendString(cursor, &line[kOledTextCols], label);
    cursor = AppendFixedMilliText(cursor, &line[kOledTextCols], angle_mdeg);
    cursor = AppendString(cursor, &line[kOledTextCols], " deg");
    FinishOledLine(line, cursor);
    return board::Board_OledWriteText(row, 0U, line);
}

drivers::DriverStatus Gy931OledShowAngles(const drivers::Gy931Angles &angles)
{
    drivers::DriverStatus status = Gy931OledWriteStatusLine();
    if (status != drivers::DRIVER_OK) {
        return status;
    }
    status = Gy931OledWriteAngleLine(1U, "Roll: ", angles.roll_mdeg);
    if (status != drivers::DRIVER_OK) {
        return status;
    }
    status = Gy931OledWriteAngleLine(2U, "Pitch:", angles.pitch_mdeg);
    if (status != drivers::DRIVER_OK) {
        return status;
    }
    return Gy931OledWriteAngleLine(3U, "Yaw:  ", angles.yaw_mdeg);
}

drivers::DriverStatus Gy931OledShowError(drivers::DriverStatus read_status)
{
    drivers::DriverStatus status = OledTextWriteLine(0U, "GY931 read error");
    if (status != drivers::DRIVER_OK) {
        return status;
    }

    char line[kOledTextCols + 1U];
    char *cursor = AppendString(line, &line[kOledTextCols], "status: ");
    cursor = AppendString(cursor, &line[kOledTextCols],
                          DriverStatusText(read_status));
    FinishOledLine(line, cursor);
    status = board::Board_OledWriteText(1U, 0U, line);
    if (status != drivers::DRIVER_OK) {
        return status;
    }

    cursor = AppendString(line, &line[kOledTextCols], "addr: ");
    cursor = AppendHex8Text(cursor, &line[kOledTextCols],
                            board::Board_Gy931Address());
    FinishOledLine(line, cursor);
    status = board::Board_OledWriteText(2U, 0U, line);
    if (status != drivers::DRIVER_OK) {
        return status;
    }
    return OledTextWriteLine(3U, "use gy931 status");
}

drivers::DriverStatus Gy931OledUpdateDisplay(void)
{
    if (!board::Board_OledIsReady()) {
        const drivers::DriverStatus init_status = board::Board_OledInit();
        if (init_status != drivers::DRIVER_OK) {
            g_gy931OledLastStatus = init_status;
            return init_status;
        }
    }

    drivers::Gy931Angles angles;
    const drivers::DriverStatus read_status =
        board::Board_Gy931ReadAngles(&angles);
    if (read_status != drivers::DRIVER_OK) {
        g_gy931OledLastStatus = read_status;
        (void) Gy931OledShowError(read_status);
        return read_status;
    }

    const drivers::DriverStatus oled_status = Gy931OledShowAngles(angles);
    g_gy931OledLastStatus = oled_status;
    return oled_status;
}

void Gy931OledTask(void)
{
    if (!g_gy931OledEnabled) {
        return;
    }

    const uint32_t now = services::Time_Millis();
    if (!services::Time_HasElapsed(g_gy931OledLastUpdateMs,
                                   g_gy931OledPeriodMs)) {
        return;
    }

    g_gy931OledLastUpdateMs = now;
    (void) Gy931OledUpdateDisplay();
}

drivers::DriverStatus Gy931OledEnsureTask(void)
{
    if (g_gy931OledTaskRegistered) {
        return drivers::DRIVER_OK;
    }

    const services::SchedulerStatus status = services::Scheduler_AddTask(
        "gy931_oled",
        Gy931OledTask,
        kGy931OledTaskPeriodMs,
        0U,
        &g_gy931OledTaskId);
    if (status != services::SCHEDULER_OK) {
        return SchedulerStatusToDriverStatus(status);
    }

    g_gy931OledTaskRegistered = true;
    return SchedulerStatusToDriverStatus(
        services::Scheduler_EnableTask(g_gy931OledTaskId, false));
}

drivers::DriverStatus Gy931OledSetEnabled(bool enabled)
{
    if (!enabled) {
        g_gy931OledEnabled = false;
        if (!g_gy931OledTaskRegistered) {
            return drivers::DRIVER_OK;
        }
        return SchedulerStatusToDriverStatus(
            services::Scheduler_EnableTask(g_gy931OledTaskId, false));
    }

    drivers::DriverStatus status = Gy931OledEnsureTask();
    if (status != drivers::DRIVER_OK) {
        return status;
    }

#if FEATURE_ENABLE_INA219
    g_ina219OledEnabled = false;
    if (g_ina219OledTaskRegistered) {
        (void) services::Scheduler_EnableTask(g_ina219OledTaskId, false);
    }
#endif
#if FEATURE_ENABLE_GRAYSCALE
    g_grayOledEnabled = false;
    if (g_grayOledTaskRegistered) {
        (void) services::Scheduler_EnableTask(g_grayOledTaskId, false);
    }
#endif
    g_gy931OledEnabled = true;
    g_gy931OledLastUpdateMs = services::Time_Millis();
    status = SchedulerStatusToDriverStatus(
        services::Scheduler_EnableTask(g_gy931OledTaskId, true));
    if (status != drivers::DRIVER_OK) {
        g_gy931OledEnabled = false;
        return status;
    }

    return Gy931OledUpdateDisplay();
}
#endif

#if FEATURE_ENABLE_INA219
drivers::DriverStatus Ina219OledWriteStatusLine(void)
{
    char line[kOledTextCols + 1U];
    char *cursor = line;

    cursor = AppendString(cursor, &line[kOledTextCols], "INA219 ");
    cursor = AppendHex8Text(cursor, &line[kOledTextCols],
                            board::Board_Ina219GetAddress());
    cursor = AppendChar(cursor, &line[kOledTextCols], ' ');
    cursor = AppendUIntDec(cursor, &line[kOledTextCols],
                           g_ina219OledPeriodMs);
    cursor = AppendString(cursor, &line[kOledTextCols], "ms");
    FinishOledLine(line, cursor);
    return board::Board_OledWriteText(0U, 0U, line);
}

drivers::DriverStatus Ina219OledShowMeasurement(
    const drivers::Ina219Measurement &measurement)
{
    drivers::DriverStatus status = Ina219OledWriteStatusLine();
    if (status != drivers::DRIVER_OK) {
        return status;
    }

    char line[kOledTextCols + 1U];
    char *cursor = line;
    cursor = AppendString(cursor, &line[kOledTextCols], "Bus: ");
    cursor = AppendFixedMilliText(cursor, &line[kOledTextCols],
                                  measurement.bus_voltage_mv);
    cursor = AppendChar(cursor, &line[kOledTextCols], 'V');
    FinishOledLine(line, cursor);
    status = board::Board_OledWriteText(1U, 0U, line);
    if (status != drivers::DRIVER_OK) {
        return status;
    }

    cursor = line;
    cursor = AppendString(cursor, &line[kOledTextCols], "Cur: ");
    cursor = AppendFixedMilliText(cursor, &line[kOledTextCols],
                                  measurement.current_ua);
    cursor = AppendString(cursor, &line[kOledTextCols], "mA");
    FinishOledLine(line, cursor);
    status = board::Board_OledWriteText(2U, 0U, line);
    if (status != drivers::DRIVER_OK) {
        return status;
    }

    cursor = line;
    cursor = AppendString(cursor, &line[kOledTextCols], "P:");
    cursor = AppendIntDec(cursor, &line[kOledTextCols],
                          measurement.power_mw);
    cursor = AppendString(cursor, &line[kOledTextCols], "mW Sh:");
    cursor = AppendIntDec(cursor, &line[kOledTextCols],
                          measurement.shunt_voltage_uv);
    cursor = AppendString(cursor, &line[kOledTextCols], "uV");
    FinishOledLine(line, cursor);
    return board::Board_OledWriteText(3U, 0U, line);
}

drivers::DriverStatus Ina219OledShowError(drivers::DriverStatus read_status)
{
    drivers::DriverStatus status =
        OledTextWriteLine(0U, "INA219 read error");
    if (status != drivers::DRIVER_OK) {
        return status;
    }

    char line[kOledTextCols + 1U];
    char *cursor = AppendString(line, &line[kOledTextCols], "status: ");
    cursor = AppendString(cursor, &line[kOledTextCols],
                          DriverStatusText(read_status));
    FinishOledLine(line, cursor);
    status = board::Board_OledWriteText(1U, 0U, line);
    if (status != drivers::DRIVER_OK) {
        return status;
    }

    cursor = AppendString(line, &line[kOledTextCols], "addr: ");
    cursor = AppendHex8Text(cursor, &line[kOledTextCols],
                            board::Board_Ina219GetAddress());
    FinishOledLine(line, cursor);
    status = board::Board_OledWriteText(2U, 0U, line);
    if (status != drivers::DRIVER_OK) {
        return status;
    }
    return OledTextWriteLine(3U, "use ina219 status");
}

drivers::DriverStatus Ina219OledUpdateDisplay(void)
{
    if (!board::Board_OledIsReady()) {
        const drivers::DriverStatus init_status = board::Board_OledInit();
        if (init_status != drivers::DRIVER_OK) {
            g_ina219OledLastStatus = init_status;
            return init_status;
        }
    }

    drivers::Ina219Measurement measurement;
    const drivers::DriverStatus read_status =
        board::Board_Ina219ReadMeasurement(&measurement);
    if (read_status != drivers::DRIVER_OK) {
        g_ina219OledLastStatus = read_status;
        (void) Ina219OledShowError(read_status);
        return read_status;
    }

    const drivers::DriverStatus oled_status =
        Ina219OledShowMeasurement(measurement);
    g_ina219OledLastStatus = oled_status;
    return oled_status;
}

void Ina219OledTask(void)
{
    if (!g_ina219OledEnabled) {
        return;
    }

    const uint32_t now = services::Time_Millis();
    if (!services::Time_HasElapsed(g_ina219OledLastUpdateMs,
                                   g_ina219OledPeriodMs)) {
        return;
    }

    g_ina219OledLastUpdateMs = now;
    (void) Ina219OledUpdateDisplay();
}

drivers::DriverStatus Ina219OledEnsureTask(void)
{
    if (g_ina219OledTaskRegistered) {
        return drivers::DRIVER_OK;
    }

    const services::SchedulerStatus status = services::Scheduler_AddTask(
        "ina219_oled",
        Ina219OledTask,
        kIna219OledTaskPeriodMs,
        0U,
        &g_ina219OledTaskId);
    if (status != services::SCHEDULER_OK) {
        return SchedulerStatusToDriverStatus(status);
    }

    g_ina219OledTaskRegistered = true;
    return SchedulerStatusToDriverStatus(
        services::Scheduler_EnableTask(g_ina219OledTaskId, false));
}

drivers::DriverStatus Ina219OledSetEnabled(bool enabled)
{
    if (!enabled) {
        g_ina219OledEnabled = false;
        if (!g_ina219OledTaskRegistered) {
            return drivers::DRIVER_OK;
        }
        return SchedulerStatusToDriverStatus(
            services::Scheduler_EnableTask(g_ina219OledTaskId, false));
    }

    drivers::DriverStatus status = Ina219OledEnsureTask();
    if (status != drivers::DRIVER_OK) {
        return status;
    }

#if FEATURE_ENABLE_GY931
    g_gy931OledEnabled = false;
    if (g_gy931OledTaskRegistered) {
        (void) services::Scheduler_EnableTask(g_gy931OledTaskId, false);
    }
#endif
#if FEATURE_ENABLE_GRAYSCALE
    g_grayOledEnabled = false;
    if (g_grayOledTaskRegistered) {
        (void) services::Scheduler_EnableTask(g_grayOledTaskId, false);
    }
#endif
    g_ina219OledEnabled = true;
    g_ina219OledLastUpdateMs = services::Time_Millis();
    status = SchedulerStatusToDriverStatus(
        services::Scheduler_EnableTask(g_ina219OledTaskId, true));
    if (status != drivers::DRIVER_OK) {
        g_ina219OledEnabled = false;
        return status;
    }

    return Ina219OledUpdateDisplay();
}
#endif

#if FEATURE_ENABLE_IMU
drivers::DriverStatus ImuOledWriteStatusLine(void)
{
    char line[kOledTextCols + 1U];
    char *cursor = line;
    cursor = AppendString(cursor, &line[kOledTextCols], "IMU ");
    cursor = AppendUIntDec(cursor, &line[kOledTextCols], g_imuOledPeriodMs);
    cursor = AppendString(cursor, &line[kOledTextCols], "ms");
    FinishOledLine(line, cursor);
    return board::Board_OledWriteText(0U, 0U, line);
}

drivers::DriverStatus ImuOledWriteAngleLine(uint8_t row,
                                            const char *label,
                                            int32_t angle_mdeg)
{
    char line[kOledTextCols + 1U];
    char *cursor = line;
    cursor = AppendString(cursor, &line[kOledTextCols], label);
    cursor = AppendFixedMilliText(cursor, &line[kOledTextCols], angle_mdeg);
    cursor = AppendString(cursor, &line[kOledTextCols], " deg");
    FinishOledLine(line, cursor);
    return board::Board_OledWriteText(row, 0U, line);
}

drivers::DriverStatus ImuOledShowData(const AppImuData *imu)
{
    drivers::DriverStatus status = ImuOledWriteStatusLine();
    if (status != drivers::DRIVER_OK) {
        return status;
    }
    status = ImuOledWriteAngleLine(1U, "Pit: ", imu->pitch_mdeg);
    if (status != drivers::DRIVER_OK) {
        return status;
    }
    status = ImuOledWriteAngleLine(2U, "Rol: ", imu->roll_mdeg);
    if (status != drivers::DRIVER_OK) {
        return status;
    }

    char line[kOledTextCols + 1U];
    char *cursor = line;
    cursor = AppendString(cursor, &line[kOledTextCols], "Gz:  ");
    cursor = AppendFixedMilliText(cursor, &line[kOledTextCols],
                                  imu->gyro_mdps[2]);
    cursor = AppendString(cursor, &line[kOledTextCols], " d/s");
    FinishOledLine(line, cursor);
    return board::Board_OledWriteText(3U, 0U, line);
}

drivers::DriverStatus ImuOledShowError(void)
{
    drivers::DriverStatus status = OledTextWriteLine(0U, "IMU no data");
    if (status != drivers::DRIVER_OK) {
        return status;
    }
    (void) OledTextWriteLine(1U, "run 'imu icm init'");
    return OledTextWriteLine(2U, "then 'imu oled on'");
}

drivers::DriverStatus ImuOledUpdateDisplay(void)
{
    if (!board::Board_OledIsReady()) {
        const drivers::DriverStatus init_status = board::Board_OledInit();
        if (init_status != drivers::DRIVER_OK) {
            g_imuOledLastStatus = init_status;
            return init_status;
        }
    }

    const AppImuData *imu = App_ImuGetData();
    if ((imu == 0) || (!imu->valid)) {
        g_imuOledLastStatus = drivers::DRIVER_ERROR_NOT_INITIALIZED;
        (void) ImuOledShowError();
        return drivers::DRIVER_ERROR_NOT_INITIALIZED;
    }

    const drivers::DriverStatus oled_status = ImuOledShowData(imu);
    g_imuOledLastStatus = oled_status;
    return oled_status;
}

void ImuOledTask(void)
{
    if (!g_imuOledEnabled) {
        return;
    }

    const uint32_t now = services::Time_Millis();
    if (!services::Time_HasElapsed(g_imuOledLastUpdateMs,
                                   g_imuOledPeriodMs)) {
        return;
    }

    g_imuOledLastUpdateMs = now;
    (void) ImuOledUpdateDisplay();
}

drivers::DriverStatus ImuOledEnsureTask(void)
{
    if (g_imuOledTaskRegistered) {
        return drivers::DRIVER_OK;
    }

    const services::SchedulerStatus status = services::Scheduler_AddTask(
        "imu_oled",
        ImuOledTask,
        kImuOledTaskPeriodMs,
        0U,
        &g_imuOledTaskId);
    if (status != services::SCHEDULER_OK) {
        return SchedulerStatusToDriverStatus(status);
    }

    g_imuOledTaskRegistered = true;
    return SchedulerStatusToDriverStatus(
        services::Scheduler_EnableTask(g_imuOledTaskId, false));
}

drivers::DriverStatus ImuOledSetEnabled(bool enabled)
{
    if (!enabled) {
        g_imuOledEnabled = false;
        if (!g_imuOledTaskRegistered) {
            return drivers::DRIVER_OK;
        }
        return SchedulerStatusToDriverStatus(
            services::Scheduler_EnableTask(g_imuOledTaskId, false));
    }

    drivers::DriverStatus status = ImuOledEnsureTask();
    if (status != drivers::DRIVER_OK) {
        return status;
    }

#if FEATURE_ENABLE_GY931
    g_gy931OledEnabled = false;
    if (g_gy931OledTaskRegistered) {
        (void) services::Scheduler_EnableTask(g_gy931OledTaskId, false);
    }
#endif
#if FEATURE_ENABLE_INA219
    g_ina219OledEnabled = false;
    if (g_ina219OledTaskRegistered) {
        (void) services::Scheduler_EnableTask(g_ina219OledTaskId, false);
    }
#endif
#if FEATURE_ENABLE_GRAYSCALE
    g_grayOledEnabled = false;
    if (g_grayOledTaskRegistered) {
        (void) services::Scheduler_EnableTask(g_grayOledTaskId, false);
    }
#endif
    g_imuOledEnabled = true;
    g_imuOledLastUpdateMs = services::Time_Millis();
    status = SchedulerStatusToDriverStatus(
        services::Scheduler_EnableTask(g_imuOledTaskId, true));
    if (status != drivers::DRIVER_OK) {
        g_imuOledEnabled = false;
        return status;
    }

    return ImuOledUpdateDisplay();
}
#endif

#if FEATURE_ENABLE_GRAYSCALE
drivers::DriverStatus GrayOledWriteStatusLine(void)
{
    char line[kOledTextCols + 1U];
    char *cursor = line;

    cursor = AppendString(cursor, &line[kOledTextCols], "GRAY ");
    cursor = AppendUIntDec(cursor, &line[kOledTextCols], g_grayOledPeriodMs);
    cursor = AppendString(cursor, &line[kOledTextCols], "ms");
    FinishOledLine(line, cursor);
    return board::Board_OledWriteText(0U, 0U, line);
}

drivers::DriverStatus GrayOledWriteRawLine(uint8_t row,
                                           uint8_t first_channel,
                                           uint8_t count,
                                           const uint16_t raw[8])
{
    char line[kOledTextCols + 1U];
    char *cursor = line;

    cursor = AppendUIntDec(cursor, &line[kOledTextCols], first_channel);
    if (count > 1U) {
        cursor = AppendChar(cursor, &line[kOledTextCols], '-');
        cursor = AppendUIntDec(cursor,
                               &line[kOledTextCols],
                               (uint32_t) (first_channel + count - 1U));
    }
    cursor = AppendChar(cursor, &line[kOledTextCols], ' ');

    for (uint8_t i = 0U; i < count; i++) {
        if (i != 0U) {
            cursor = AppendChar(cursor, &line[kOledTextCols], ' ');
        }
        cursor = AppendUIntDec(cursor,
                               &line[kOledTextCols],
                               raw[first_channel + i]);
    }

    FinishOledLine(line, cursor);
    return board::Board_OledWriteText(row, 0U, line);
}

drivers::DriverStatus GrayOledShowRaw(const uint16_t raw[8])
{
    drivers::DriverStatus status = GrayOledWriteStatusLine();
    if (status != drivers::DRIVER_OK) {
        return status;
    }
    status = GrayOledWriteRawLine(1U, 0U, 3U, raw);
    if (status != drivers::DRIVER_OK) {
        return status;
    }
    status = GrayOledWriteRawLine(2U, 3U, 3U, raw);
    if (status != drivers::DRIVER_OK) {
        return status;
    }
    return GrayOledWriteRawLine(3U, 6U, 2U, raw);
}

drivers::DriverStatus GrayOledShowError(drivers::DriverStatus read_status)
{
    drivers::DriverStatus status = OledTextWriteLine(0U, "GRAY read error");
    if (status != drivers::DRIVER_OK) {
        return status;
    }

    char line[kOledTextCols + 1U];
    char *cursor = AppendString(line, &line[kOledTextCols], "status: ");
    cursor = AppendString(cursor, &line[kOledTextCols],
                          DriverStatusText(read_status));
    FinishOledLine(line, cursor);
    status = board::Board_OledWriteText(1U, 0U, line);
    if (status != drivers::DRIVER_OK) {
        return status;
    }

    status = OledTextWriteLine(2U, "check PA15/mux");
    if (status != drivers::DRIVER_OK) {
        return status;
    }
    return OledTextWriteLine(3U, "use gray status");
}

drivers::DriverStatus GrayOledUpdateDisplay(void)
{
    if (!board::Board_OledIsReady()) {
        const drivers::DriverStatus init_status = board::Board_OledInit();
        if (init_status != drivers::DRIVER_OK) {
            g_grayOledLastStatus = init_status;
            return init_status;
        }
    }

    uint16_t raw[8] = { 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U };
    const drivers::DriverStatus read_status = board::Board_GrayscaleReadAll(raw);
    if (read_status != drivers::DRIVER_OK) {
        g_grayOledLastStatus = read_status;
        (void) GrayOledShowError(read_status);
        return read_status;
    }

    const drivers::DriverStatus oled_status = GrayOledShowRaw(raw);
    g_grayOledLastStatus = oled_status;
    return oled_status;
}

void GrayOledTask(void)
{
    if (!g_grayOledEnabled) {
        return;
    }

    const uint32_t now = services::Time_Millis();
    if (!services::Time_HasElapsed(g_grayOledLastUpdateMs,
                                   g_grayOledPeriodMs)) {
        return;
    }

    g_grayOledLastUpdateMs = now;
    (void) GrayOledUpdateDisplay();
}

drivers::DriverStatus GrayOledEnsureTask(void)
{
    if (g_grayOledTaskRegistered) {
        return drivers::DRIVER_OK;
    }

    const services::SchedulerStatus status = services::Scheduler_AddTask(
        "gray_oled",
        GrayOledTask,
        kGrayOledTaskPeriodMs,
        0U,
        &g_grayOledTaskId);
    if (status != services::SCHEDULER_OK) {
        return SchedulerStatusToDriverStatus(status);
    }

    g_grayOledTaskRegistered = true;
    return SchedulerStatusToDriverStatus(
        services::Scheduler_EnableTask(g_grayOledTaskId, false));
}

drivers::DriverStatus GrayOledSetEnabled(bool enabled)
{
    if (!enabled) {
        g_grayOledEnabled = false;
        if (!g_grayOledTaskRegistered) {
            return drivers::DRIVER_OK;
        }
        return SchedulerStatusToDriverStatus(
            services::Scheduler_EnableTask(g_grayOledTaskId, false));
    }

    drivers::DriverStatus status = GrayOledEnsureTask();
    if (status != drivers::DRIVER_OK) {
        return status;
    }

#if FEATURE_ENABLE_GY931
    g_gy931OledEnabled = false;
    if (g_gy931OledTaskRegistered) {
        (void) services::Scheduler_EnableTask(g_gy931OledTaskId, false);
    }
#endif
#if FEATURE_ENABLE_INA219
    g_ina219OledEnabled = false;
    if (g_ina219OledTaskRegistered) {
        (void) services::Scheduler_EnableTask(g_ina219OledTaskId, false);
    }
#endif
#if FEATURE_ENABLE_IMU
    g_imuOledEnabled = false;
    if (g_imuOledTaskRegistered) {
        (void) services::Scheduler_EnableTask(g_imuOledTaskId, false);
    }
#endif

    g_grayOledEnabled = true;
    g_grayOledLastUpdateMs = services::Time_Millis();
    status = SchedulerStatusToDriverStatus(
        services::Scheduler_EnableTask(g_grayOledTaskId, true));
    if (status != drivers::DRIVER_OK) {
        g_grayOledEnabled = false;
        return status;
    }

    return GrayOledUpdateDisplay();
}
#endif
#endif

#if FEATURE_ENABLE_IMU
void PrintImuUsage(void)
{
    services::Shell_WriteLine("usage:");
    services::Shell_WriteLine("  imu status");
    services::Shell_WriteLine("  imu cs idle|icm|lis|float");
    services::Shell_WriteLine("  imu pins wiggle [loops]");
    services::Shell_WriteLine("  imu spi mode <0..3>");
    services::Shell_WriteLine("  imu spi burst [bytes] [byte]");
    services::Shell_WriteLine("  imu spi rx [count 1..16] [tx]");
    services::Shell_WriteLine("  imu oled on [period_ms]|off|status|once");
    services::Shell_WriteLine("  imu lis whoami");
    services::Shell_WriteLine("  imu lis reg <addr>");
    services::Shell_WriteLine("  imu sample");
    services::Shell_WriteLine("  imu icm init");
    services::Shell_WriteLine("  imu icm whoami");
    services::Shell_WriteLine("  imu icm sample");
    services::Shell_WriteLine("  imu icm reg <addr>");
    services::Shell_WriteLine("  imu icm wreg <addr> <val>");
}
#endif

#if FEATURE_ENABLE_LORA
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
#endif

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
    services::Shell_WriteLine("  motor invert");
    services::Shell_WriteLine("  motor invert m1|m2 on|off");
    services::Shell_WriteLine("  motor invert enc m1|m2 on|off");
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

void PrintChassisUsage(void)
{
    services::Shell_WriteLine("usage:");
    services::Shell_WriteLine("  chassis status");
    services::Shell_WriteLine("  chassis stop");
    services::Shell_WriteLine("  chassis wheel <left_rpm> <right_rpm>");
    services::Shell_WriteLine(
        "  chassis vel <linear_mm_s> <angular_mdeg_s>");
}

void PrintMotorFrameData(const char *prefix, const motor::Frame &frame)
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

void PrintMotorEncoderLine(const char *prefix, const motor::EncoderData &encoder)
{
    services::Shell_WriteString(prefix);
    services::Shell_WriteString(" count=");
    WriteInt32(encoder.count);
    services::Shell_WriteString(" cps=");
    WriteInt32(encoder.counts_per_second);
    services::Shell_WriteString(" state=");
    WriteHex8(encoder.state);
    services::Shell_WriteString("\r\n");
}

void PrintMotorRpmBlock(const motor::RpmData &rpm)
{
    services::Shell_WriteString("motor rpm m1 target=");
    services::Shell_WriteUInt32(rpm.target_m1);
    services::Shell_WriteString(" actual=");
    WriteInt32(rpm.actual_m1);
    services::Shell_WriteString("\r\n");

    services::Shell_WriteString("motor rpm m2 target=");
    services::Shell_WriteUInt32(rpm.target_m2);
    services::Shell_WriteString(" actual=");
    WriteInt32(rpm.actual_m2);
    services::Shell_WriteString("\r\n");
}

void PrintMotorConfigBlock(const motor::CountsPerRev &counts)
{
    services::Shell_WriteString("motor cfg m1_counts_per_rev=");
    services::Shell_WriteUInt32(counts.m1);
    services::Shell_WriteString(" m2_counts_per_rev=");
    services::Shell_WriteUInt32(counts.m2);
    services::Shell_WriteString("\r\n");
}

void WriteOnOff(bool enabled)
{
    services::Shell_WriteString(enabled ? "on" : "off");
}

void PrintMotorInvertBlock(const motor::InvertConfig &config)
{
    services::Shell_WriteString("motor invert output=");
    WriteHex8(config.output_flags);
    services::Shell_WriteString(" encoder=");
    WriteHex8(config.encoder_flags);
    services::Shell_WriteString(" m1=");
    WriteOnOff((config.output_flags & motor::kInvertM1) != 0U);
    services::Shell_WriteString(" m2=");
    WriteOnOff((config.output_flags & motor::kInvertM2) != 0U);
    services::Shell_WriteString(" enc_m1=");
    WriteOnOff((config.encoder_flags & motor::kInvertM1) != 0U);
    services::Shell_WriteString(" enc_m2=");
    WriteOnOff((config.encoder_flags & motor::kInvertM2) != 0U);
    services::Shell_WriteString("\r\n");
}

void PrintMotorPidBlock(const motor::SpeedPid &pid)
{
    services::Shell_WriteString("motor pid kp=");
    services::Shell_WriteUInt32(pid.kp_q4_4);
    services::Shell_WriteString(" ki=");
    services::Shell_WriteUInt32(pid.ki_q4_4);
    services::Shell_WriteString(" kd=");
    services::Shell_WriteUInt32(pid.kd_q4_4);
    services::Shell_WriteString(" max=");
    services::Shell_WriteUInt32(pid.max_duty);
    services::Shell_WriteString(" min=");
    services::Shell_WriteUInt32(pid.min_duty);
    services::Shell_WriteString("\r\n");
}

void PrintMotorPositionPidFields(const motor::PositionPid &pid)
{
    services::Shell_WriteString(" kp=");
    services::Shell_WriteUInt32(pid.kp_q4_4);
    services::Shell_WriteString(" ki=");
    services::Shell_WriteUInt32(pid.ki_q4_4);
    services::Shell_WriteString(" kd=");
    services::Shell_WriteUInt32(pid.kd_q4_4);
    services::Shell_WriteString(" max_rpm=");
    services::Shell_WriteUInt32(pid.max_rpm);
    services::Shell_WriteString(" tol_counts=");
    services::Shell_WriteUInt32(pid.tolerance_counts);
}

void PrintMotorPositionControlFields(const motor::PositionControl &control)
{
    services::Shell_WriteString(" min_duty=");
    services::Shell_WriteUInt32(control.min_duty);
    services::Shell_WriteString(" max_duty=");
    services::Shell_WriteUInt32(control.max_duty);
    services::Shell_WriteString(" exit_tol_counts=");
    services::Shell_WriteUInt32(control.exit_tolerance_counts);
    services::Shell_WriteString(" settle_ms=");
    services::Shell_WriteUInt32(control.settle_ms);
}

void PrintMotorPositionBlock(const motor::PositionData &position)
{
    services::Shell_WriteString("motor pos m1 target_count=");
    WriteInt32(position.m1_target_count);
    services::Shell_WriteString(" error_count=");
    WriteInt32(position.m1_error_count);
    services::Shell_WriteString("\r\n");

    services::Shell_WriteString("motor pos m2 target_count=");
    WriteInt32(position.m2_target_count);
    services::Shell_WriteString(" error_count=");
    WriteInt32(position.m2_error_count);
    services::Shell_WriteString("\r\n");

    services::Shell_WriteString("motor pospid");
    PrintMotorPositionPidFields(position.pid);
    services::Shell_WriteString(" status=");
    WriteHex8(position.status);
    services::Shell_WriteString("\r\n");

    services::Shell_WriteString("motor posctl");
    PrintMotorPositionControlFields(position.control);
    services::Shell_WriteString("\r\n");
}

void PrintMotorPositionPidBlock(const motor::PositionPid &pid)
{
    services::Shell_WriteString("motor pospid");
    PrintMotorPositionPidFields(pid);
    services::Shell_WriteString("\r\n");
}

void PrintMotorPositionControlBlock(const motor::PositionControl &control)
{
    services::Shell_WriteString("motor posctl");
    PrintMotorPositionControlFields(control);
    services::Shell_WriteString("\r\n");
}

void PrintChassisWheelState(const char *prefix,
                            const ChassisWheelState &wheel)
{
    services::Shell_WriteString(prefix);
    services::Shell_WriteString(" target_rpm=");
    WriteInt32(wheel.target_rpm);
    services::Shell_WriteString(" actual_rpm=");
    WriteInt32(wheel.actual_rpm);
    services::Shell_WriteString(" enc=");
    WriteInt32(wheel.encoder_count);
    services::Shell_WriteString(" cps=");
    WriteInt32(wheel.encoder_counts_per_second);
    services::Shell_WriteString(" state=");
    WriteHex8(wheel.encoder_state);
    services::Shell_WriteString("\r\n");
}

void PrintChassisState(const ChassisState &state)
{
    services::Shell_WriteString("chassis initialized=");
    services::Shell_WriteUInt32(state.initialized ? 1U : 0U);
    services::Shell_WriteString(" linear_mm_s=");
    WriteInt32(state.target_linear_mm_s);
    services::Shell_WriteString(" angular_mdeg_s=");
    WriteInt32(state.target_angular_mdeg_s);
    services::Shell_WriteString(" last=");
    services::Shell_WriteString(DriverStatusText(state.last_status));
    services::Shell_WriteString("\r\n");

    services::Shell_WriteString("chassis cfg radius_mm=");
    services::Shell_WriteUInt32(state.config.wheel_radius_mm);
    services::Shell_WriteString(" track_mm=");
    services::Shell_WriteUInt32(state.config.wheel_track_mm);
    services::Shell_WriteString(" left_cpr=");
    services::Shell_WriteUInt32(state.config.left_counts_per_rev);
    services::Shell_WriteString(" right_cpr=");
    services::Shell_WriteUInt32(state.config.right_counts_per_rev);
    services::Shell_WriteString(" max_rpm=");
    services::Shell_WriteUInt32(state.config.max_wheel_rpm);
    services::Shell_WriteString("\r\n");

    PrintChassisWheelState("chassis left", state.left);
    PrintChassisWheelState("chassis right", state.right);
}

void PrintParamLine(const char *name)
{
    int32_t value = 0;
    int32_t min_value = 0;
    int32_t max_value = 0;

    if (!ConfigStore_GetValue(name, &value, &min_value, &max_value)) {
        services::Shell_WriteLine("param: unknown");
        return;
    }

    services::Shell_WriteString("param ");
    services::Shell_WriteString(name);
    services::Shell_WriteString("=");
    WriteInt32(value);
    services::Shell_WriteString(" range=");
    WriteInt32(min_value);
    services::Shell_WriteString("..");
    WriteInt32(max_value);
    services::Shell_WriteString("\r\n");
}

void PrintAllParams(void)
{
    const uint8_t count = ConfigStore_ParamCount();
    for (uint8_t i = 0U; i < count; i++) {
        const char *name = ConfigStore_ParamName(i);
        if (name != 0) {
            PrintParamLine(name);
        }
    }
}

void PrintParamStatus(void)
{
    const ConfigStoreStatus *status = ConfigStore_GetStatus();

    services::Shell_WriteString("param loaded=");
    services::Shell_WriteUInt32(status->loaded_from_fram ? 1U : 0U);
    services::Shell_WriteString(" dirty=");
    services::Shell_WriteUInt32(status->dirty ? 1U : 0U);
    services::Shell_WriteString(" len=");
    services::Shell_WriteUInt32(status->stored_length);
    services::Shell_WriteString(" crc=");
    WriteHex32(status->stored_crc);
    services::Shell_WriteString(" load=");
    services::Shell_WriteString(DriverStatusText(status->last_load_status));
    services::Shell_WriteString(" save=");
    services::Shell_WriteString(DriverStatusText(status->last_save_status));
    services::Shell_WriteString("\r\n");
}


#if FEATURE_ENABLE_SHELL_DIAGNOSTICS
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
#endif
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

#if FEATURE_ENABLE_STATUS_LED
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
#endif

#if FEATURE_ENABLE_BUZZER
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
#endif

#if FEATURE_ENABLE_BUTTONS
void PrintButtonUsage(void)
{
    services::Shell_WriteLine("usage:");
    services::Shell_WriteLine("  button");
    services::Shell_WriteLine("  button watch [duration_ms 100..30000]");
    services::Shell_WriteLine("  button scan [duration_ms 100..30000]");
}

void PrintButtonGpioSnapshot(void)
{
    services::Shell_WriteString("gpioB din=");
    WriteHex32(GPIO_BUTTON_B_PORT->DIN31_0);
    services::Shell_WriteString(" pb20=");
    services::Shell_WriteString(
        (DL_GPIO_readPins(BOARD_BUTTON2_PORT, BOARD_BUTTON2_PIN) != 0U) ?
        "H" : "L");
    services::Shell_WriteString(" pb23=");
    services::Shell_WriteString(
        (DL_GPIO_readPins(BOARD_BUTTON3_PORT, BOARD_BUTTON3_PIN) != 0U) ?
        "H" : "L");
    services::Shell_WriteString(" gpioC din=");
    WriteHex32(GPIO_BUTTON_C_PORT->DIN31_0);
    services::Shell_WriteString(" pc9=");
    services::Shell_WriteString(
        (DL_GPIO_readPins(BOARD_BUTTON1_PORT, BOARD_BUTTON1_PIN) != 0U) ?
        "H" : "L");
    services::Shell_WriteString(" iomux23=");
    WriteHex32(IOMUX->SECCFG.PINCM[GPIO_BUTTON_B_BUTTON3_IOMUX]);
    services::Shell_WriteString("\r\n");
}

void ButtonWatchCommand(uint32_t duration_ms)
{
    const uint32_t start_ms = services::Time_Millis();
    uint32_t last_gpio_b = GPIO_BUTTON_B_PORT->DIN31_0;
    uint32_t last_gpio_c = GPIO_BUTTON_C_PORT->DIN31_0;

    services::Shell_WriteLine("button watch: press/release buttons now");
    PrintButtonGpioSnapshot();

    while (!services::Time_HasElapsed(start_ms, duration_ms)) {
        const uint32_t gpio_b = GPIO_BUTTON_B_PORT->DIN31_0;
        const uint32_t gpio_c = GPIO_BUTTON_C_PORT->DIN31_0;
        if ((gpio_b != last_gpio_b) || (gpio_c != last_gpio_c)) {
            services::Shell_WriteString("t=");
            services::Shell_WriteUInt32(services::Time_Millis() - start_ms);
            services::Shell_WriteString(" ");
            PrintButtonGpioSnapshot();
            last_gpio_b = gpio_b;
            last_gpio_c = gpio_c;
        }
    }

    services::Shell_WriteLine("button watch: done");
}

void PrintChangedBits(const char *port_name, uint32_t before, uint32_t after)
{
    const uint32_t changed = before ^ after;
    if (changed == 0U) {
        return;
    }

    services::Shell_WriteString(port_name);
    services::Shell_WriteString(" before=");
    WriteHex32(before);
    services::Shell_WriteString(" after=");
    WriteHex32(after);
    services::Shell_WriteString(" changed:");
    for (uint8_t bit = 0U; bit < 32U; bit++) {
        const uint32_t mask = (uint32_t) 1U << bit;
        if ((changed & mask) != 0U) {
            services::Shell_WriteString(" ");
            services::Shell_WriteUInt32(bit);
            services::Shell_WriteString((after & mask) != 0U ? "=H" : "=L");
        }
    }
    services::Shell_WriteString("\r\n");
}

void ButtonScanCommand(uint32_t duration_ms)
{
    const uint32_t start_ms = services::Time_Millis();
    uint32_t last_gpio_a = GPIOA->DIN31_0;
    uint32_t last_gpio_b = GPIOB->DIN31_0;
    uint32_t last_gpio_c = GPIOC->DIN31_0;

    services::Shell_WriteLine("button scan: press/release button3 now");
    services::Shell_WriteString("initial A=");
    WriteHex32(last_gpio_a);
    services::Shell_WriteString(" B=");
    WriteHex32(last_gpio_b);
    services::Shell_WriteString(" C=");
    WriteHex32(last_gpio_c);
    services::Shell_WriteString("\r\n");

    while (!services::Time_HasElapsed(start_ms, duration_ms)) {
        const uint32_t gpio_a = GPIOA->DIN31_0;
        const uint32_t gpio_b = GPIOB->DIN31_0;
        const uint32_t gpio_c = GPIOC->DIN31_0;
        if ((gpio_a != last_gpio_a) ||
            (gpio_b != last_gpio_b) ||
            (gpio_c != last_gpio_c)) {
            services::Shell_WriteString("t=");
            services::Shell_WriteUInt32(services::Time_Millis() - start_ms);
            services::Shell_WriteString("\r\n");
            PrintChangedBits("GPIOA", last_gpio_a, gpio_a);
            PrintChangedBits("GPIOB", last_gpio_b, gpio_b);
            PrintChangedBits("GPIOC", last_gpio_c, gpio_c);
            last_gpio_a = gpio_a;
            last_gpio_b = gpio_b;
            last_gpio_c = gpio_c;
        }
    }

    services::Shell_WriteLine("button scan: done");
}
#endif

void ButtonCommand(int argc, const char * const argv[])
{
#if FEATURE_ENABLE_BUTTONS
    if ((argc >= 2) && (StrEqual(argv[1], "watch") ||
                        StrEqual(argv[1], "scan"))) {
        uint32_t duration_ms = 5000U;
        if (argc == 3) {
            if ((!ParseUint32(argv[2], 30000U, &duration_ms)) ||
                (duration_ms < 100U)) {
                PrintButtonUsage();
                return;
            }
        } else if (argc != 2) {
            PrintButtonUsage();
            return;
        }

        if (StrEqual(argv[1], "watch")) {
            ButtonWatchCommand(duration_ms);
        } else {
            ButtonScanCommand(duration_ms);
        }
        return;
    }

    if (argc != 1) {
        PrintButtonUsage();
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

    services::Shell_WriteString("button gpioB din=");
    WriteHex32(GPIO_BUTTON_B_PORT->DIN31_0);
    services::Shell_WriteString(" pb20=");
    services::Shell_WriteString(
        (DL_GPIO_readPins(BOARD_BUTTON2_PORT, BOARD_BUTTON2_PIN) != 0U) ?
        "H" : "L");
    services::Shell_WriteString(" pb23=");
    services::Shell_WriteString(
        (DL_GPIO_readPins(BOARD_BUTTON3_PORT, BOARD_BUTTON3_PIN) != 0U) ?
        "H" : "L");
    services::Shell_WriteString(" iomux23=");
    WriteHex32(IOMUX->SECCFG.PINCM[GPIO_BUTTON_B_BUTTON3_IOMUX]);
    services::Shell_WriteString("\r\n");
#else
    (void) argc;
    (void) argv;
    services::Shell_WriteLine("button: disabled");
#endif
}

void ParamCommand(int argc, const char * const argv[])
{
    if (argc < 2) {
        PrintParamUsage();
        return;
    }

    if (StrEqual(argv[1], "status")) {
        if (argc != 2) {
            PrintParamUsage();
            return;
        }
        PrintParamStatus();
        return;
    }

    if (StrEqual(argv[1], "get")) {
        if (argc == 2) {
            PrintAllParams();
            return;
        }
        if (argc == 3) {
            PrintParamLine(argv[2]);
            return;
        }
        PrintParamUsage();
        return;
    }

    if (StrEqual(argv[1], "set")) {
        int32_t value = 0;
        int32_t min_value = 0;
        int32_t max_value = 0;

        if ((argc != 4) ||
            (!ConfigStore_GetValue(argv[2], 0, &min_value, &max_value)) ||
            (!ParseInt32(argv[3], min_value, max_value, &value))) {
            PrintParamUsage();
            return;
        }

        const drivers::DriverStatus status = ConfigStore_Set(argv[2], value);
        WriteStatusLine("param set: ", status);
        return;
    }

    if (StrEqual(argv[1], "save")) {
        if (argc != 2) {
            PrintParamUsage();
            return;
        }
        const drivers::DriverStatus status = ConfigStore_Save();
        WriteStatusLine("param save: ", status);
        return;
    }

    if (StrEqual(argv[1], "load")) {
        if (argc != 2) {
            PrintParamUsage();
            return;
        }
        const drivers::DriverStatus status = ConfigStore_Load();
        WriteStatusLine("param load: ", status);
        return;
    }

    if (StrEqual(argv[1], "reset")) {
        if (argc != 2) {
            PrintParamUsage();
            return;
        }
        ConfigStore_ResetDefaults();
        services::Shell_WriteLine("param reset: ok");
        return;
    }

    PrintParamUsage();
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

void Gy931Command(int argc, const char * const argv[])
{
#if FEATURE_ENABLE_GY931
    uint32_t value = 0U;

    if (argc < 2) {
        PrintGy931Usage();
        return;
    }

    if (StrEqual(argv[1], "status")) {
        bool scl_high = false;
        bool sda_high = false;

        if (argc != 2) {
            PrintGy931Usage();
            return;
        }

        const drivers::DriverStatus bus_status =
            board::Board_Gy931GetBusStatus(&scl_high, &sda_high);
        const drivers::DriverStatus probe_status = board::Board_Gy931Probe();

        services::Shell_WriteString("gy931 ready=");
        services::Shell_WriteUInt32(board::Board_Gy931IsReady() ? 1U : 0U);
        services::Shell_WriteString(" addr=");
        WriteHex8(board::Board_Gy931Address());
        services::Shell_WriteString(" probe=");
        services::Shell_WriteString(DriverStatusText(probe_status));
        services::Shell_WriteString(" bus=");
        if (bus_status == drivers::DRIVER_OK) {
            services::Shell_WriteString("ok scl=");
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
            PrintGy931Usage();
            return;
        }

        WriteStatusLine("gy931 init: ", board::Board_Gy931Init());
        return;
    }

    if (StrEqual(argv[1], "recover")) {
        if (argc != 2) {
            PrintGy931Usage();
            return;
        }

        WriteStatusLine("gy931 recover: ", board::Board_Gy931RecoverBus());
        return;
    }

    if (StrEqual(argv[1], "scan")) {
        uint32_t start = 0x08U;
        uint32_t end = 0x77U;

        if ((argc != 2) && (argc != 4)) {
            PrintGy931Usage();
            return;
        }
        if (argc == 4) {
            if ((!ParseUint32(argv[2], 0x77U, &start)) ||
                (!ParseUint32(argv[3], 0x77U, &end)) ||
                (start < 0x08U) || (end < start)) {
                PrintGy931Usage();
                return;
            }
        }

        services::Shell_WriteString("gy931 scan ");
        WriteHex8((uint8_t) start);
        services::Shell_WriteString("..");
        WriteHex8((uint8_t) end);
        services::Shell_WriteString("\r\n");

        uint32_t count = 0U;
        for (uint8_t address = (uint8_t) start;
             address <= (uint8_t) end;
             address++) {
            const drivers::DriverStatus status =
                board::Board_Gy931ProbeAddress(address);
            if (status == drivers::DRIVER_OK) {
                services::Shell_WriteString("gy931 found ");
                WriteHex8(address);
                services::Shell_WriteString("\r\n");
                count++;
            }
        }

        services::Shell_WriteString("gy931 scan count=");
        services::Shell_WriteUInt32(count);
        services::Shell_WriteString("\r\n");
        return;
    }

    if (StrEqual(argv[1], "addr")) {
        if ((argc != 2) && (argc != 3)) {
            PrintGy931Usage();
            return;
        }

        if (argc == 2) {
            services::Shell_WriteString("gy931 addr=");
            WriteHex8(board::Board_Gy931Address());
            services::Shell_WriteString("\r\n");
            return;
        }

        if ((!ParseUint32(argv[2], 0x77U, &value)) || (value < 0x08U)) {
            PrintGy931Usage();
            return;
        }

        const drivers::DriverStatus status =
            board::Board_Gy931SetAddress((uint8_t) value);
        services::Shell_WriteString("gy931 addr: ");
        services::Shell_WriteString(DriverStatusText(status));
        services::Shell_WriteString(" addr=");
        WriteHex8(board::Board_Gy931Address());
        services::Shell_WriteString("\r\n");
        return;
    }

    if (StrEqual(argv[1], "oled")) {
#if FEATURE_ENABLE_OLED
        if (argc < 3) {
            PrintGy931Usage();
            return;
        }

        if (StrEqual(argv[2], "on")) {
            if ((argc != 3) && (argc != 4)) {
                PrintGy931Usage();
                return;
            }
            if (argc == 4) {
                if ((!ParseUint32(argv[3],
                                  kGy931OledMaxPeriodMs,
                                  &value)) ||
                    (value < kGy931OledMinPeriodMs)) {
                    PrintGy931Usage();
                    return;
                }
                g_gy931OledPeriodMs = value;
            }

            const drivers::DriverStatus status = Gy931OledSetEnabled(true);
            services::Shell_WriteString("gy931 oled: ");
            services::Shell_WriteString(DriverStatusText(status));
            services::Shell_WriteString(" period_ms=");
            services::Shell_WriteUInt32(g_gy931OledPeriodMs);
            services::Shell_WriteString("\r\n");
            return;
        }

        if (StrEqual(argv[2], "off")) {
            if (argc != 3) {
                PrintGy931Usage();
                return;
            }

            const drivers::DriverStatus status = Gy931OledSetEnabled(false);
            WriteStatusLine("gy931 oled: ", status);
            return;
        }

        if (StrEqual(argv[2], "status")) {
            if (argc != 3) {
                PrintGy931Usage();
                return;
            }

            services::Shell_WriteString("gy931 oled enabled=");
            services::Shell_WriteUInt32(g_gy931OledEnabled ? 1U : 0U);
            services::Shell_WriteString(" registered=");
            services::Shell_WriteUInt32(g_gy931OledTaskRegistered ? 1U : 0U);
            services::Shell_WriteString(" period_ms=");
            services::Shell_WriteUInt32(g_gy931OledPeriodMs);
            services::Shell_WriteString(" last=");
            services::Shell_WriteString(DriverStatusText(g_gy931OledLastStatus));
            services::Shell_WriteString("\r\n");
            return;
        }

        if (StrEqual(argv[2], "once")) {
            if (argc != 3) {
                PrintGy931Usage();
                return;
            }

            WriteStatusLine("gy931 oled once: ", Gy931OledUpdateDisplay());
            return;
        }

        PrintGy931Usage();
        return;
#else
        (void) value;
        services::Shell_WriteLine("gy931 oled: oled disabled");
        return;
#endif
    }

    if (StrEqual(argv[1], "angle")) {
        drivers::Gy931Angles angles;

        if (argc != 2) {
            PrintGy931Usage();
            return;
        }

        const drivers::DriverStatus status =
            board::Board_Gy931ReadAngles(&angles);
        if (status != drivers::DRIVER_OK) {
            WriteStatusLine("gy931 angle: ", status);
            return;
        }

        services::Shell_WriteString("gy931 angle raw=");
        WriteInt32(angles.roll_raw);
        services::Shell_WriteString(",");
        WriteInt32(angles.pitch_raw);
        services::Shell_WriteString(",");
        WriteInt32(angles.yaw_raw);
        services::Shell_WriteString(" deg=");
        WriteFixedMilli(angles.roll_mdeg);
        services::Shell_WriteString(",");
        WriteFixedMilli(angles.pitch_mdeg);
        services::Shell_WriteString(",");
        WriteFixedMilli(angles.yaw_mdeg);
        services::Shell_WriteString("\r\n");
        return;
    }

    if (StrEqual(argv[1], "sample")) {
        drivers::Gy931Sample sample;

        if (argc != 2) {
            PrintGy931Usage();
            return;
        }

        const drivers::DriverStatus status =
            board::Board_Gy931ReadSample(&sample);
        if (status != drivers::DRIVER_OK) {
            WriteStatusLine("gy931 sample: ", status);
            return;
        }

        services::Shell_WriteString("gy931 ");
        WriteSignedVector3Milli("acc_g", sample.acc_mg, "g");
        services::Shell_WriteString(" ");
        WriteSignedVector3Milli("gyro_dps", sample.gyro_mdps, "dps");
        services::Shell_WriteString(" mag_raw=");
        WriteInt32(sample.mag_raw[0]);
        services::Shell_WriteString(",");
        WriteInt32(sample.mag_raw[1]);
        services::Shell_WriteString(",");
        WriteInt32(sample.mag_raw[2]);
        services::Shell_WriteString(" ");
        WriteSignedVector3Milli("angle_deg", sample.angle_mdeg, "deg");
        services::Shell_WriteString("\r\n");
        return;
    }

    if (StrEqual(argv[1], "raw")) {
        uint32_t reg = 0U;
        uint32_t word_count = 0U;
        int16_t words[kGy931MaxReadWords];

        if ((argc != 4) ||
            (!ParseUint32(argv[2], 0xFFU, &reg)) ||
            (!ParseUint32(argv[3], kGy931MaxReadWords, &word_count)) ||
            (word_count == 0U) ||
            (((uint16_t) reg + ((uint16_t) word_count * 2U)) > 256U)) {
            PrintGy931Usage();
            return;
        }

        const drivers::DriverStatus status =
            board::Board_Gy931ReadRawRegisters((uint8_t) reg,
                                               words,
                                               (uint8_t) word_count);
        if (status != drivers::DRIVER_OK) {
            WriteStatusLine("gy931 raw: ", status);
            return;
        }

        services::Shell_WriteString("gy931 raw reg=");
        WriteHex8((uint8_t) reg);
        services::Shell_WriteString(" words=");
        services::Shell_WriteUInt32(word_count);
        services::Shell_WriteString(" data:");
        for (uint8_t i = 0U; i < (uint8_t) word_count; i++) {
            services::Shell_WriteString(" ");
            WriteInt32(words[i]);
            services::Shell_WriteString("(");
            WriteHex16((uint16_t) words[i]);
            services::Shell_WriteString(")");
        }
        services::Shell_WriteString("\r\n");
        return;
    }

    PrintGy931Usage();
#else
    (void) argc;
    (void) argv;
    services::Shell_WriteLine("gy931: disabled");
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

    if (StrEqual(argv[1], "oled")) {
#if FEATURE_ENABLE_OLED
        if (argc < 3) {
            PrintIna219Usage();
            return;
        }

        if (StrEqual(argv[2], "on")) {
            if ((argc != 3) && (argc != 4)) {
                PrintIna219Usage();
                return;
            }
            if (argc == 4) {
                if ((!ParseUint32(argv[3],
                                  kIna219OledMaxPeriodMs,
                                  &value)) ||
                    (value < kIna219OledMinPeriodMs)) {
                    PrintIna219Usage();
                    return;
                }
                g_ina219OledPeriodMs = value;
            }

            const drivers::DriverStatus status = Ina219OledSetEnabled(true);
            services::Shell_WriteString("ina219 oled: ");
            services::Shell_WriteString(DriverStatusText(status));
            services::Shell_WriteString(" period_ms=");
            services::Shell_WriteUInt32(g_ina219OledPeriodMs);
            services::Shell_WriteString("\r\n");
            return;
        }

        if (StrEqual(argv[2], "off")) {
            if (argc != 3) {
                PrintIna219Usage();
                return;
            }

            const drivers::DriverStatus status = Ina219OledSetEnabled(false);
            WriteStatusLine("ina219 oled: ", status);
            return;
        }

        if (StrEqual(argv[2], "status")) {
            if (argc != 3) {
                PrintIna219Usage();
                return;
            }

            services::Shell_WriteString("ina219 oled enabled=");
            services::Shell_WriteUInt32(g_ina219OledEnabled ? 1U : 0U);
            services::Shell_WriteString(" registered=");
            services::Shell_WriteUInt32(g_ina219OledTaskRegistered ? 1U : 0U);
            services::Shell_WriteString(" period_ms=");
            services::Shell_WriteUInt32(g_ina219OledPeriodMs);
            services::Shell_WriteString(" last=");
            services::Shell_WriteString(
                DriverStatusText(g_ina219OledLastStatus));
            services::Shell_WriteString("\r\n");
            return;
        }

        if (StrEqual(argv[2], "once")) {
            if (argc != 3) {
                PrintIna219Usage();
                return;
            }

            WriteStatusLine("ina219 oled once: ", Ina219OledUpdateDisplay());
            return;
        }

        PrintIna219Usage();
        return;
#else
        (void) value;
        services::Shell_WriteLine("ina219 oled: oled disabled");
        return;
#endif
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


#if FEATURE_ENABLE_SHELL_DIAGNOSTICS
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
#endif

#if FEATURE_ENABLE_IMU
void ImuCommand(int argc, const char * const argv[])
{
    uint32_t reg = 0U;
    uint8_t value = 0U;
    uint32_t val_u32 = 0U;

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

    if (StrEqual(argv[1], "oled")) {
#if FEATURE_ENABLE_OLED
        if (argc < 3) {
            PrintImuUsage();
            return;
        }

        if (StrEqual(argv[2], "on")) {
            if ((argc != 3) && (argc != 4)) {
                PrintImuUsage();
                return;
            }
            if (argc == 4) {
                if ((!ParseUint32(argv[3], kImuOledMaxPeriodMs, &val_u32)) ||
                    (val_u32 < kImuOledMinPeriodMs)) {
                    PrintImuUsage();
                    return;
                }
                g_imuOledPeriodMs = val_u32;
            }

            const drivers::DriverStatus status = ImuOledSetEnabled(true);
            services::Shell_WriteString("imu oled: ");
            services::Shell_WriteString(DriverStatusText(status));
            services::Shell_WriteString(" period_ms=");
            services::Shell_WriteUInt32(g_imuOledPeriodMs);
            services::Shell_WriteString("\r\n");
            return;
        }

        if (StrEqual(argv[2], "off")) {
            if (argc != 3) {
                PrintImuUsage();
                return;
            }
            WriteStatusLine("imu oled: ", ImuOledSetEnabled(false));
            return;
        }

        if (StrEqual(argv[2], "status")) {
            if (argc != 3) {
                PrintImuUsage();
                return;
            }
            services::Shell_WriteString("imu oled enabled=");
            services::Shell_WriteUInt32(g_imuOledEnabled ? 1U : 0U);
            services::Shell_WriteString(" period_ms=");
            services::Shell_WriteUInt32(g_imuOledPeriodMs);
            services::Shell_WriteString(" last=");
            services::Shell_WriteString(DriverStatusText(g_imuOledLastStatus));
            services::Shell_WriteString("\r\n");
            return;
        }

        if (StrEqual(argv[2], "once")) {
            if (argc != 3) {
                PrintImuUsage();
                return;
            }
            WriteStatusLine("imu oled once: ", ImuOledUpdateDisplay());
            return;
        }

        PrintImuUsage();
        return;
#else
        services::Shell_WriteLine("imu oled: oled disabled");
        return;
#endif
    }

    if (StrEqual(argv[1], "sample")) {
        const AppImuData *imu = App_ImuGetData();
        if (!imu->valid) {
            services::Shell_WriteLine("imu sample: no data (run 'imu icm init')");
            return;
        }
        services::Shell_WriteString("imu acc=");
        WriteInt32(imu->accel_mg[0]);
        services::Shell_WriteString(",");
        WriteInt32(imu->accel_mg[1]);
        services::Shell_WriteString(",");
        WriteInt32(imu->accel_mg[2]);
        services::Shell_WriteString(" mg gyr=");
        WriteInt32(imu->gyro_mdps[0]);
        services::Shell_WriteString(",");
        WriteInt32(imu->gyro_mdps[1]);
        services::Shell_WriteString(",");
        WriteInt32(imu->gyro_mdps[2]);
        services::Shell_WriteString(" mdps t=");
        WriteInt32(imu->temp_centi_c);
        services::Shell_WriteString(" cC\r\n");
        return;
    }

    if (StrEqual(argv[1], "icm")) {
        if ((argc == 3) && StrEqual(argv[2], "whoami")) {
            const drivers::DriverStatus status =
                board::Board_Icm45686ReadWhoAmI(&value);
            if (status != drivers::DRIVER_OK) {
                WriteStatusLine("imu icm whoami: ", status);
                return;
            }
            services::Shell_WriteString("imu icm whoami=");
            WriteHex8(value);
            services::Shell_WriteString(" expected=0xE9\r\n");
            return;
        }

        if ((argc == 3) && StrEqual(argv[2], "init")) {
            const drivers::DriverStatus status = board::Board_Icm45686Init();
            WriteStatusLine("imu icm init: ", status);
            return;
        }

        if ((argc == 3) && StrEqual(argv[2], "sample")) {
            drivers::Icm45686SensorData raw = {};
            const drivers::DriverStatus status =
                board::Board_Icm45686ReadSensors(&raw);
            if (status != drivers::DRIVER_OK) {
                WriteStatusLine("imu icm sample: ", status);
                return;
            }
            services::Shell_WriteString("imu icm acc=");
            WriteInt32(raw.accel_x);
            services::Shell_WriteString(",");
            WriteInt32(raw.accel_y);
            services::Shell_WriteString(",");
            WriteInt32(raw.accel_z);
            services::Shell_WriteString(" gyr=");
            WriteInt32(raw.gyro_x);
            services::Shell_WriteString(",");
            WriteInt32(raw.gyro_y);
            services::Shell_WriteString(",");
            WriteInt32(raw.gyro_z);
            services::Shell_WriteString(" t=");
            WriteInt32(raw.temp);
            services::Shell_WriteString("\r\n");
            return;
        }

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

        if ((argc == 5) && StrEqual(argv[2], "wreg") &&
            ParseUint32(argv[3], 0x7FU, &reg) &&
            ParseUint32(argv[4], 0xFFU, &val_u32)) {
            const drivers::DriverStatus status =
                board::Board_Icm45686WriteRegister((uint8_t) reg,
                                                   (uint8_t) val_u32);
            WriteStatusLine("imu icm wreg: ", status);
            return;
        }

        PrintImuUsage();
        return;
    }

    PrintImuUsage();
}
#endif

#if FEATURE_ENABLE_SHELL_DIAGNOSTICS
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
#endif

#if FEATURE_ENABLE_LORA
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
}
#endif

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
                motor::RawWriteByte((uint8_t) ' ');
            if (status != drivers::DRIVER_OK) {
                return status;
            }
            count++;
        }

        const char *cursor = argv[arg];
        while ((cursor != 0) && (*cursor != '\0')) {
            const drivers::DriverStatus status =
                motor::RawWriteByte((uint8_t) *cursor);
            if (status != drivers::DRIVER_OK) {
                return status;
            }
            count++;
            cursor++;
        }
    }

    if (append_newline) {
        drivers::DriverStatus status =
            motor::RawWriteByte((uint8_t) '\r');
        if (status != drivers::DRIVER_OK) {
            return status;
        }
        status = motor::RawWriteByte((uint8_t) '\n');
        if (status != drivers::DRIVER_OK) {
            return status;
        }
        count = (uint16_t) (count + 2U);
    }

    *bytes_written = count;
    return drivers::DRIVER_OK;
}

void ChassisCommand(int argc, const char * const argv[])
{
#if FEATURE_ENABLE_MOTOR_DRIVER
    if (argc < 2) {
        PrintChassisUsage();
        return;
    }

    if (StrEqual(argv[1], "status")) {
        if (argc != 2) {
            PrintChassisUsage();
            return;
        }

        const drivers::DriverStatus status = Chassis_Update();
        if ((status != drivers::DRIVER_OK) &&
            (status != drivers::DRIVER_ERROR_NOT_INITIALIZED)) {
            WriteStatusLine("chassis update: ", status);
        }
        PrintChassisState(*Chassis_GetState());
        return;
    }

    if (StrEqual(argv[1], "stop")) {
        if (argc != 2) {
            PrintChassisUsage();
            return;
        }

        const drivers::DriverStatus status = Chassis_Stop();
        WriteStatusLine("chassis stop: ", status);
        return;
    }

    if (StrEqual(argv[1], "wheel")) {
        int32_t left_rpm = 0;
        int32_t right_rpm = 0;
        const ChassisState *state = Chassis_GetState();
        const int32_t max_rpm =
            static_cast<int32_t>(state->config.max_wheel_rpm);

        if ((argc != 4) ||
            (!ParseInt32(argv[2], -max_rpm, max_rpm, &left_rpm)) ||
            (!ParseInt32(argv[3], -max_rpm, max_rpm, &right_rpm))) {
            PrintChassisUsage();
            return;
        }

        const drivers::DriverStatus status =
            Chassis_SetWheelRpm(left_rpm, right_rpm);
        WriteStatusLine("chassis wheel: ", status);
        return;
    }

    if (StrEqual(argv[1], "vel")) {
        int32_t linear_mm_s = 0;
        int32_t angular_mdeg_s = 0;

        if ((argc != 4) ||
            (!ParseInt32(argv[2],
                         -kChassisLinearLimitMmS,
                         kChassisLinearLimitMmS,
                         &linear_mm_s)) ||
            (!ParseInt32(argv[3],
                         -kChassisAngularLimitMdegS,
                         kChassisAngularLimitMdegS,
                         &angular_mdeg_s))) {
            PrintChassisUsage();
            return;
        }

        const drivers::DriverStatus status =
            Chassis_SetVelocity(linear_mm_s, angular_mdeg_s);
        WriteStatusLine("chassis vel: ", status);
        return;
    }

    PrintChassisUsage();
#else
    (void) argc;
    (void) argv;
    services::Shell_WriteLine("chassis: disabled");
#endif
}

void MotorCommand(int argc, const char * const argv[])
{
#if FEATURE_ENABLE_MOTOR_DRIVER
    uint32_t value = 0U;

    if ((argc < 2) || (!motor::IsReady())) {
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
        const drivers::I2cDiagBusConfig *i2c_bus = motor::GetI2cBus();

        if (argc != 2) {
            PrintMotorUsage();
            return;
        }

        const drivers::DriverStatus uart_status_result =
            motor::GetControllerStatus(&uart_status);

        services::Shell_WriteString("motor bus=");
        services::Shell_WriteString(motor::TransportText(&g_motorClient));
        services::Shell_WriteString(" baud=");
        services::Shell_WriteUInt32(motor::GetBaudrate());
        services::Shell_WriteString(" rx_buf=");
        services::Shell_WriteUInt32(motor::GetRxAvailable());
        services::Shell_WriteString(" dropped=");
        services::Shell_WriteUInt32(
            motor::GetRxDroppedCount());
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
        WriteHex8(motor::GetI2cAddress(&g_motorClient));
        services::Shell_WriteString(" i2c_bus=");
        services::Shell_WriteString(motor::kI2cBusName);
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
            services::Shell_WriteString(motor::TransportText(&g_motorClient));
            services::Shell_WriteString("\r\n");
            return;
        }

        if (argc != 3) {
            PrintMotorUsage();
            return;
        }

        if (StrEqual(argv[2], "uart")) {
            (void) motor::SetBus(&g_motorClient, motor::TRANSPORT_UART);
        } else if (StrEqual(argv[2], "i2c")) {
            (void) motor::SetBus(&g_motorClient, motor::TRANSPORT_I2C);
        } else {
            PrintMotorUsage();
            return;
        }

        services::Shell_WriteString("motor bus=");
        services::Shell_WriteString(motor::TransportText(&g_motorClient));
        services::Shell_WriteString("\r\n");
        return;
    }

    if (StrEqual(argv[1], "i2caddr")) {
        if (argc == 2) {
            services::Shell_WriteString("motor i2caddr=");
            WriteHex8(motor::GetI2cAddress(&g_motorClient));
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

        (void) motor::SetI2cAddress(&g_motorClient, static_cast<uint8_t>(value));
        services::Shell_WriteString("motor i2caddr=");
        WriteHex8(motor::GetI2cAddress(&g_motorClient));
        services::Shell_WriteString("\r\n");
        return;
    }

    if (StrEqual(argv[1], "ping")) {
        bool ok = false;

        if (argc != 2) {
            PrintMotorUsage();
            return;
        }

        const drivers::DriverStatus status = motor::Ping(&g_motorClient, &ok);
        if (status != drivers::DRIVER_OK) {
            WriteStatusLine("motor ping: ", status);
            return;
        }

        services::Shell_WriteString("motor ping: ");
        services::Shell_WriteString(ok ? "ok" : "error");
        services::Shell_WriteString("\r\n");
        return;
    }

    if (StrEqual(argv[1], "info")) {
        motor::DeviceInfo info = {};

        if (argc != 2) {
            PrintMotorUsage();
            return;
        }

        const drivers::DriverStatus status =
            motor::ReadInfo(&g_motorClient, &info);
        if (status != drivers::DRIVER_OK) {
            WriteStatusLine("motor info: ", status);
            return;
        }

        services::Shell_WriteString("motor id=");
        WriteHex8(info.device_id);
        services::Shell_WriteString(" fw=");
        services::Shell_WriteUInt32(info.firmware_version);
        services::Shell_WriteString(" status=");
        WriteHex8(info.status);
        services::Shell_WriteString(" fault=");
        WriteHex8(info.fault_flags);
        services::Shell_WriteString(" ctrl=");
        WriteHex8(info.control_flags);
        services::Shell_WriteString(" i2c=");
        WriteHex8(info.i2c_address);
        services::Shell_WriteString("\r\n");
        return;
    }

    if (StrEqual(argv[1], "enc")) {
        motor::EncoderData encoder = {};

        if ((argc == 3) && StrEqual(argv[2], "reset")) {
            const drivers::DriverStatus status =
                motor::ResetEncoders(&g_motorClient);
            if (status != drivers::DRIVER_OK) {
                WriteStatusLine("motor enc reset: ", status);
                return;
            }

            services::Shell_WriteLine("motor enc reset: ok");
            return;
        }

        if (argc != 2) {
            PrintMotorUsage();
            return;
        }

        drivers::DriverStatus status =
            motor::ReadEncoder(&g_motorClient, true, &encoder);
        if (status != drivers::DRIVER_OK) {
            WriteStatusLine("motor enc m1: ", status);
            return;
        }
        PrintMotorEncoderLine("motor enc m1", encoder);

        status = motor::ReadEncoder(&g_motorClient, false, &encoder);
        if (status != drivers::DRIVER_OK) {
            WriteStatusLine("motor enc m2: ", status);
            return;
        }
        PrintMotorEncoderLine("motor enc m2", encoder);
        return;
    }

    if (StrEqual(argv[1], "rpm")) {
        motor::RpmData rpm = {};

        if (argc != 2) {
            PrintMotorUsage();
            return;
        }

        const drivers::DriverStatus status =
            motor::ReadRpm(&g_motorClient, &rpm);
        if (status != drivers::DRIVER_OK) {
            WriteStatusLine("motor rpm: ", status);
            return;
        }

        PrintMotorRpmBlock(rpm);
        return;
    }

    if (StrEqual(argv[1], "cfg")) {
        motor::CountsPerRev counts = {};

        if (argc == 2) {
            const drivers::DriverStatus status =
                motor::ReadCountsPerRev(&g_motorClient, &counts);
            if (status != drivers::DRIVER_OK) {
                WriteStatusLine("motor cfg: ", status);
                return;
            }

            PrintMotorConfigBlock(counts);
            return;
        }

        if (argc != 4) {
            PrintMotorUsage();
            return;
        }

        uint32_t m1_counts_per_rev = 0U;
        uint32_t m2_counts_per_rev = 0U;

        if ((!ParseUint32(argv[2],
                          motor::kCountsPerRevMax,
                          &m1_counts_per_rev)) ||
            (!ParseUint32(argv[3],
                          motor::kCountsPerRevMax,
                          &m2_counts_per_rev))) {
            PrintMotorUsage();
            return;
        }

        const drivers::DriverStatus status =
            motor::SetCountsPerRev(&g_motorClient,
                                   m1_counts_per_rev,
                                   m2_counts_per_rev);
        if (status != drivers::DRIVER_OK) {
            WriteStatusLine("motor cfg: ", status);
            return;
        }

        services::Shell_WriteLine("motor cfg: ok");
        return;
    }

    if (StrEqual(argv[1], "invert")) {
        motor::InvertConfig config = {};
        const drivers::DriverStatus read_status =
            motor::ReadInvertConfig(&g_motorClient, &config);
        if (read_status != drivers::DRIVER_OK) {
            WriteStatusLine("motor invert: ", read_status);
            return;
        }

        if (argc == 2) {
            PrintMotorInvertBlock(config);
            return;
        }

        bool encoder_invert = false;
        const char *motor_name = 0;
        const char *state_name = 0;

        if (argc == 4) {
            motor_name = argv[2];
            state_name = argv[3];
        } else if ((argc == 5) && StrEqual(argv[2], "enc")) {
            encoder_invert = true;
            motor_name = argv[3];
            state_name = argv[4];
        } else {
            PrintMotorUsage();
            return;
        }

        uint8_t mask = 0U;
        bool enabled = false;
        if (StrEqual(motor_name, "m1")) {
            mask = motor::kInvertM1;
        } else if (StrEqual(motor_name, "m2")) {
            mask = motor::kInvertM2;
        } else {
            PrintMotorUsage();
            return;
        }

        if (!ParseOnOff(state_name, &enabled)) {
            PrintMotorUsage();
            return;
        }

        if (encoder_invert) {
            SetFlagValue(&config.encoder_flags, mask, enabled);
        } else {
            SetFlagValue(&config.output_flags, mask, enabled);
        }

        const drivers::DriverStatus status =
            motor::SetInvertConfig(&g_motorClient, config);
        if (status != drivers::DRIVER_OK) {
            WriteStatusLine("motor invert: ", status);
            return;
        }

        const drivers::DriverStatus param_status =
            ConfigStore_Set(encoder_invert ? "motor_encoder_invert_flags" :
                                             "motor_output_invert_flags",
                            encoder_invert ? config.encoder_flags :
                                             config.output_flags);
        if (param_status != drivers::DRIVER_OK) {
            WriteStatusLine("motor invert param: ", param_status);
            return;
        }

        services::Shell_WriteLine("motor invert: ok");
        return;
    }

    if (StrEqual(argv[1], "pid")) {
        motor::SpeedPid pid = {};

        if (argc == 2) {
            const drivers::DriverStatus status =
                motor::ReadPid(&g_motorClient, &pid);
            if (status != drivers::DRIVER_OK) {
                WriteStatusLine("motor pid: ", status);
                return;
            }

            PrintMotorPidBlock(pid);
            return;
        }

        if ((argc < 5) || (argc > 7)) {
            PrintMotorUsage();
            return;
        }

        uint8_t data[motor::kSpeedPidLength] = { 0U, 0U, 0U, 0U, 0U };
        const uint8_t length = static_cast<uint8_t>(argc - 2);
        for (uint8_t i = 0U; i < length; i++) {
            const uint32_t max_value = (i >= 3U) ? 100U : 0xFFU;
            if (!ParseUint32(argv[2U + i], max_value, &value)) {
                PrintMotorUsage();
                return;
            }
            data[i] = static_cast<uint8_t>(value);
        }

        const drivers::DriverStatus status =
            motor::SetPid(&g_motorClient, data, length);
        if (status != drivers::DRIVER_OK) {
            WriteStatusLine("motor pid: ", status);
            return;
        }

        services::Shell_WriteLine("motor pid: ok");
        return;
    }

    if (StrEqual(argv[1], "pos")) {
        motor::PositionData position = {};

        if (argc != 2) {
            PrintMotorUsage();
            return;
        }

        const drivers::DriverStatus status =
            motor::ReadPosition(&g_motorClient, &position);
        if (status != drivers::DRIVER_OK) {
            WriteStatusLine("motor pos: ", status);
            return;
        }

        PrintMotorPositionBlock(position);
        return;
    }

    if (StrEqual(argv[1], "pospid")) {
        motor::PositionPid pid = {};

        if (argc == 2) {
            const drivers::DriverStatus status =
                motor::ReadPositionPid(&g_motorClient, &pid);
            if (status != drivers::DRIVER_OK) {
                WriteStatusLine("motor pospid: ", status);
                return;
            }

            PrintMotorPositionPidBlock(pid);
            return;
        }

        if ((argc < 5) || (argc > 7)) {
            PrintMotorUsage();
            return;
        }

        uint8_t data[motor::kPositionPidLength] = {
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
            if (!ParseUint32(argv[5], motor::kPositionMaxRpm, &max_rpm)) {
                PrintMotorUsage();
                return;
            }
            motor::EncodeUint16Le(static_cast<uint16_t>(max_rpm), &data[3]);
            length = 5U;
        }

        if (argc == 7) {
            if (!ParseUint32(argv[6],
                             motor::kPositionToleranceMax,
                             &tolerance)) {
                PrintMotorUsage();
                return;
            }
            motor::EncodeUint16Le(static_cast<uint16_t>(tolerance), &data[5]);
            length = 7U;
        }

        const drivers::DriverStatus status =
            motor::SetPositionPid(&g_motorClient, data, length);
        if (status != drivers::DRIVER_OK) {
            WriteStatusLine("motor pospid: ", status);
            return;
        }

        services::Shell_WriteLine("motor pospid: ok");
        return;
    }

    if (StrEqual(argv[1], "posctl")) {
        motor::PositionControl control = {};

        if (argc == 2) {
            const drivers::DriverStatus status =
                motor::ReadPositionControl(&g_motorClient, &control);
            if (status != drivers::DRIVER_OK) {
                WriteStatusLine("motor posctl: ", status);
                return;
            }

            PrintMotorPositionControlBlock(control);
            return;
        }

        if ((argc < 4) || (argc > 6)) {
            PrintMotorUsage();
            return;
        }

        uint8_t data[motor::kPositionControlLength] = { 0U, 0U, 0U, 0U, 0U };
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
                             motor::kPositionToleranceMax,
                             &exit_tolerance)) {
                PrintMotorUsage();
                return;
            }
            motor::EncodeUint16Le(static_cast<uint16_t>(exit_tolerance),
                                  &data[2]);
            length = 4U;
        }

        if (argc == 6) {
            if (!ParseUint32(argv[5],
                             motor::kPositionSettleMsMax,
                             &settle_ms)) {
                PrintMotorUsage();
                return;
            }
            data[4] = static_cast<uint8_t>((settle_ms + 9U) / 10U);
            length = 5U;
        }

        const drivers::DriverStatus status =
            motor::SetPositionControl(&g_motorClient, data, length);
        if (status != drivers::DRIVER_OK) {
            WriteStatusLine("motor posctl: ", status);
            return;
        }

        services::Shell_WriteLine("motor posctl: ok");
        return;
    }

    if (StrEqual(argv[1], "reg")) {
        motor::Frame response = {};
        uint32_t reg = 0U;
        uint32_t length = 0U;

        if ((argc != 4) ||
            (!ParseUint32(argv[2], 0xFFU, &reg)) ||
            (!ParseUint32(argv[3], motor::kMaxPayloadLength, &length)) ||
            (length == 0U)) {
            PrintMotorUsage();
            return;
        }

        const drivers::DriverStatus status =
            motor::ReadRegisters(&g_motorClient,
                                 (uint8_t) reg,
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
        motor::Frame response = {};
        uint32_t reg = 0U;
        uint8_t data[motor::kMaxPayloadLength];
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

        const drivers::DriverStatus status =
            motor::WriteRegisters(&g_motorClient,
                                  (uint8_t) reg,
                                  data,
                                  length,
                                  &response);
        if (status != drivers::DRIVER_OK) {
            WriteStatusLine("motor set: ", status);
            return;
        }

        const bool ok = motor::StatusOkResponse(response);
        services::Shell_WriteString("motor set: ");
        services::Shell_WriteString(ok ? "ok" : "error");
        services::Shell_WriteString("\r\n");
        return;
    }

    if (StrEqual(argv[1], "stop")) {
        motor::StopResult stop_result = {};
        drivers::DriverStatus final_status = drivers::DRIVER_OK;

        if (argc != 2) {
            PrintMotorUsage();
            return;
        }

        (void) motor::Stop(&g_motorClient, &stop_result);
        if (stop_result.m1_status != drivers::DRIVER_OK) {
            WriteStatusLine("motor stop m1: ", stop_result.m1_status);
            final_status = stop_result.m1_status;
        }
        if (stop_result.m2_status != drivers::DRIVER_OK) {
            WriteStatusLine("motor stop m2: ", stop_result.m2_status);
            if (final_status == drivers::DRIVER_OK) {
                final_status = stop_result.m2_status;
            }
        }

        WriteStatusLine("motor stop: ", final_status);
        return;
    }

    if (StrEqual(argv[1], "m1") || StrEqual(argv[1], "m2")) {
        const bool motor1 = StrEqual(argv[1], "m1");
        motor::MotionResult motion = {};

        if (argc < 3) {
            PrintMotorUsage();
            return;
        }

        if (StrEqual(argv[2], "coast")) {
            if (argc != 3) {
                PrintMotorUsage();
                return;
            }
            (void) motor::SetCoast(&g_motorClient, motor1, &motion);
        } else if (StrEqual(argv[2], "brake")) {
            if (argc != 3) {
                PrintMotorUsage();
                return;
            }
            (void) motor::SetBrake(&g_motorClient, motor1, &motion);
        } else if (StrEqual(argv[2], "run")) {
            uint32_t duty = 0U;
            bool reverse = false;

            if ((argc != 4) && (argc != 5)) {
                PrintMotorUsage();
                return;
            }
            if (!ParseUint32(argv[3], 100U, &duty)) {
                PrintMotorUsage();
                return;
            }
            if (argc == 5) {
                if (StrEqual(argv[4], "rev")) {
                    reverse = true;
                } else if (!StrEqual(argv[4], "fwd")) {
                    PrintMotorUsage();
                    return;
                }
            }
            (void) motor::SetRun(&g_motorClient,
                                 motor1,
                                 static_cast<uint8_t>(duty),
                                 reverse,
                                 &motion);
        } else if (StrEqual(argv[2], "speed")) {
            uint32_t rpm = 0U;
            bool reverse = false;

            if ((argc != 4) && (argc != 5)) {
                PrintMotorUsage();
                return;
            }
            if (!ParseUint32(argv[3], motor::kSpeedMaxRpm, &rpm)) {
                PrintMotorUsage();
                return;
            }
            if (argc == 5) {
                if (StrEqual(argv[4], "rev")) {
                    reverse = true;
                } else if (!StrEqual(argv[4], "fwd")) {
                    PrintMotorUsage();
                    return;
                }
            }

            (void) motor::SetSpeed(&g_motorClient,
                                   motor1,
                                   static_cast<uint16_t>(rpm),
                                   reverse,
                                   &motion);
            if (motion.target_status != drivers::DRIVER_OK) {
                WriteStatusLine("motor speed target: ", motion.target_status);
                return;
            }
            if (!motion.target_ack) {
                services::Shell_WriteLine("motor speed target: error");
                return;
            }
        } else if (StrEqual(argv[2], "hold") ||
                   StrEqual(argv[2], "pos") ||
                   StrEqual(argv[2], "posrel")) {
            if (StrEqual(argv[2], "hold")) {
                if (argc != 3) {
                    PrintMotorUsage();
                    return;
                }
                (void) motor::Hold(&g_motorClient, motor1, &motion);
                if (motion.read_status != drivers::DRIVER_OK) {
                    WriteStatusLine("motor hold encoder: ", motion.read_status);
                    return;
                }
            } else {
                int32_t degrees = 0;

                if (argc != 4) {
                    PrintMotorUsage();
                    return;
                }
                if (!ParseInt32(argv[3],
                                -motor::kPositionDegreeLimit,
                                motor::kPositionDegreeLimit,
                                &degrees)) {
                    PrintMotorUsage();
                    return;
                }

                if (StrEqual(argv[2], "posrel")) {
                    (void) motor::SetPositionRelative(&g_motorClient,
                                                      motor1,
                                                      degrees,
                                                      &motion);
                } else {
                    (void) motor::SetPosition(&g_motorClient,
                                              motor1,
                                              degrees,
                                              &motion);
                }

                if (motion.config_status != drivers::DRIVER_OK) {
                    WriteStatusLine("motor pos cfg: ", motion.config_status);
                    return;
                }
                if (motion.range_error) {
                    services::Shell_WriteLine("motor pos: range");
                    return;
                }
                if (motion.read_status != drivers::DRIVER_OK) {
                    WriteStatusLine("motor pos encoder: ", motion.read_status);
                    return;
                }
            }

            if (motion.target_status != drivers::DRIVER_OK) {
                WriteStatusLine("motor position target: ",
                                motion.target_status);
                return;
            }
            if (!motion.target_ack) {
                services::Shell_WriteLine("motor position target: error");
                return;
            }
        } else {
            PrintMotorUsage();
            return;
        }

        if (motion.mode_status != drivers::DRIVER_OK) {
            WriteStatusLine("motor drive: ", motion.mode_status);
            return;
        }

        services::Shell_WriteString("motor ");
        services::Shell_WriteString(argv[1]);
        services::Shell_WriteString(": ");
        services::Shell_WriteString(motion.mode_ack ? "ok" : "error");
        services::Shell_WriteString("\r\n");
        return;
    }

    if (StrEqual(argv[1], "clear")) {
        if (argc != 2) {
            PrintMotorUsage();
            return;
        }

        const drivers::DriverStatus status = motor::ClearRxBuffer();
        WriteStatusLine("motor clear: ", status);
        return;
    }

    if (StrEqual(argv[1], "test")) {
        static const uint8_t kTestData[] = { 'p', 'i', 'n', 'g', '\r', '\n' };

        if (argc != 2) {
            PrintMotorUsage();
            return;
        }

        const drivers::DriverStatus status = motor::RawWrite(
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

        const drivers::DriverStatus status = motor::RawWrite(data, length);
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
               motor::RawReadByte(&data[actual])) {
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

#if FEATURE_ENABLE_SHELL_DIAGNOSTICS
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
#endif

#if FEATURE_ENABLE_SCHEDULER_STATS
void PrintSchedulerUsage(void)
{
    services::Shell_WriteLine("usage:");
    services::Shell_WriteLine("  sched");
    services::Shell_WriteLine("  sched reset");
}

void SchedulerCommand(int argc, const char * const argv[])
{
    if (argc == 2) {
        if (StrEqual(argv[1], "reset")) {
            services::Scheduler_ResetAllStats();
            services::Shell_WriteLine("sched reset: ok");
            return;
        }

        PrintSchedulerUsage();
        return;
    }

    if (argc != 1) {
        PrintSchedulerUsage();
        return;
    }

    for (uint32_t i = 0U; i < BOARD_SCHEDULER_MAX_TASKS; i++) {
        services::SchedulerTaskInfo info = {};
        if (services::Scheduler_GetTaskInfo((services::SchedulerTaskId) i,
                                            &info) != services::SCHEDULER_OK) {
            continue;
        }

        services::Shell_WriteString("sched ");
        services::Shell_WriteUInt32(i);
        services::Shell_WriteString(" ");
        services::Shell_WriteString(info.name == 0 ? "-" : info.name);
        services::Shell_WriteString(" en=");
        services::Shell_WriteUInt32(info.enabled ? 1U : 0U);
        services::Shell_WriteString(" per_ms=");
        services::Shell_WriteUInt32(info.period_ms);
        services::Shell_WriteString(" run=");
        services::Shell_WriteUInt32(info.run_count);
        services::Shell_WriteString(" last_us=");
        services::Shell_WriteUInt32(info.last_runtime_us);
        services::Shell_WriteString(" max_us=");
        services::Shell_WriteUInt32(info.max_runtime_us);
        services::Shell_WriteString(" late_ms=");
        services::Shell_WriteUInt32(info.last_late_ms);
        services::Shell_WriteString(" max_late_ms=");
        services::Shell_WriteUInt32(info.max_late_ms);
        services::Shell_WriteString(" timeout=");
        services::Shell_WriteUInt32(info.timeout_count);
        services::Shell_WriteString("\r\n");
    }
}
#endif

#if FEATURE_ENABLE_GRAYSCALE
void PrintGrayUsage(void)
{
    services::Shell_WriteLine("usage:");
    services::Shell_WriteLine("  gray status");
    services::Shell_WriteLine("  gray read <0..7>");
    services::Shell_WriteLine("  gray all");
    services::Shell_WriteLine("  gray data");
    services::Shell_WriteLine("  gray oled on [period_ms 50..5000]|off|status|once");
}

void GrayCommand(int argc, const char * const argv[])
{
    if ((argc < 2) || (!board::Board_GrayscaleIsReady())) {
        if (argc < 2) {
            PrintGrayUsage();
        } else {
            services::Shell_WriteLine("gray: not ready");
        }
        return;
    }

    if (StrEqual(argv[1], "status")) {
        const AppGrayscaleData *data = App_GrayscaleGetData();
        services::Shell_WriteString("gray ready=1 valid=");
        services::Shell_WriteUInt32(data->valid ? 1U : 0U);
        services::Shell_WriteString("\r\n");
        return;
    }

    if (StrEqual(argv[1], "oled")) {
#if FEATURE_ENABLE_OLED
        uint32_t value = 0U;

        if (argc < 3) {
            PrintGrayUsage();
            return;
        }

        if (StrEqual(argv[2], "on")) {
            if ((argc != 3) && (argc != 4)) {
                PrintGrayUsage();
                return;
            }
            if (argc == 4) {
                if ((!ParseUint32(argv[3],
                                  kGrayOledMaxPeriodMs,
                                  &value)) ||
                    (value < kGrayOledMinPeriodMs)) {
                    PrintGrayUsage();
                    return;
                }
                g_grayOledPeriodMs = value;
            }

            const drivers::DriverStatus status = GrayOledSetEnabled(true);
            services::Shell_WriteString("gray oled: ");
            services::Shell_WriteString(DriverStatusText(status));
            services::Shell_WriteString(" period_ms=");
            services::Shell_WriteUInt32(g_grayOledPeriodMs);
            services::Shell_WriteString("\r\n");
            return;
        }

        if (StrEqual(argv[2], "off")) {
            if (argc != 3) {
                PrintGrayUsage();
                return;
            }

            const drivers::DriverStatus status = GrayOledSetEnabled(false);
            WriteStatusLine("gray oled: ", status);
            return;
        }

        if (StrEqual(argv[2], "status")) {
            if (argc != 3) {
                PrintGrayUsage();
                return;
            }

            services::Shell_WriteString("gray oled enabled=");
            services::Shell_WriteUInt32(g_grayOledEnabled ? 1U : 0U);
            services::Shell_WriteString(" registered=");
            services::Shell_WriteUInt32(g_grayOledTaskRegistered ? 1U : 0U);
            services::Shell_WriteString(" period_ms=");
            services::Shell_WriteUInt32(g_grayOledPeriodMs);
            services::Shell_WriteString(" last=");
            services::Shell_WriteString(DriverStatusText(g_grayOledLastStatus));
            services::Shell_WriteString("\r\n");
            return;
        }

        if (StrEqual(argv[2], "once")) {
            if (argc != 3) {
                PrintGrayUsage();
                return;
            }

            WriteStatusLine("gray oled once: ", GrayOledUpdateDisplay());
            return;
        }

        PrintGrayUsage();
        return;
#else
        services::Shell_WriteLine("gray oled: oled disabled");
        return;
#endif
    }

    if (StrEqual(argv[1], "data")) {
        const AppGrayscaleData *data = App_GrayscaleGetData();
        if (!data->valid) {
            services::Shell_WriteLine("gray: no data");
            return;
        }
        services::Shell_WriteString("gray:");
        for (uint8_t i = 0U; i < 8U; i++) {
            services::Shell_WriteString(" ");
            services::Shell_WriteUInt32(data->raw[i]);
        }
        services::Shell_WriteString("\r\n");
        return;
    }

    if (StrEqual(argv[1], "read")) {
        uint32_t channel = 0U;
        uint16_t raw = 0U;
        if ((argc != 3) || (!ParseUint32(argv[2], 7U, &channel))) {
            PrintGrayUsage();
            return;
        }
        const drivers::DriverStatus status =
            board::Board_GrayscaleReadChannel((uint8_t) channel, &raw);
        if (status != drivers::DRIVER_OK) {
            WriteStatusLine("gray read: ", status);
            return;
        }
        services::Shell_WriteString("gray ch");
        services::Shell_WriteUInt32(channel);
        services::Shell_WriteString("=");
        services::Shell_WriteUInt32(raw);
        services::Shell_WriteString("\r\n");
        return;
    }

    if (StrEqual(argv[1], "all")) {
        uint16_t raw[8] = { 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U };
        const drivers::DriverStatus status = board::Board_GrayscaleReadAll(raw);
        if (status != drivers::DRIVER_OK) {
            WriteStatusLine("gray all: ", status);
            return;
        }
        services::Shell_WriteString("gray:");
        for (uint8_t i = 0U; i < 8U; i++) {
            services::Shell_WriteString(" ");
            services::Shell_WriteUInt32(raw[i]);
        }
        services::Shell_WriteString("\r\n");
        return;
    }

    PrintGrayUsage();
}
#else
void GrayCommand(int argc, const char * const argv[])
{
    (void) argc;
    (void) argv;
    services::Shell_WriteLine("gray: disabled");
}
#endif

} /* namespace */

drivers::DriverStatus AppShell_EnableIna219Oled(uint32_t period_ms)
{
#if FEATURE_ENABLE_INA219 && FEATURE_ENABLE_OLED
    if ((period_ms < kIna219OledMinPeriodMs) ||
        (period_ms > kIna219OledMaxPeriodMs)) {
        return drivers::DRIVER_ERROR_INVALID_ARG;
    }

    g_ina219OledPeriodMs = period_ms;
    return Ina219OledSetEnabled(true);
#else
    (void) period_ms;
    return drivers::DRIVER_ERROR_UNSUPPORTED;
#endif
}

drivers::DriverStatus AppShell_EnableGy931Oled(uint32_t period_ms)
{
#if FEATURE_ENABLE_GY931 && FEATURE_ENABLE_OLED
    if ((period_ms < kGy931OledMinPeriodMs) ||
        (period_ms > kGy931OledMaxPeriodMs)) {
        return drivers::DRIVER_ERROR_INVALID_ARG;
    }

    g_gy931OledPeriodMs = period_ms;
    return Gy931OledSetEnabled(true);
#else
    (void) period_ms;
    return drivers::DRIVER_ERROR_UNSUPPORTED;
#endif
}

drivers::DriverStatus AppShell_EnableGrayOled(uint32_t period_ms)
{
#if FEATURE_ENABLE_GRAYSCALE && FEATURE_ENABLE_OLED
    if ((period_ms < kGrayOledMinPeriodMs) ||
        (period_ms > kGrayOledMaxPeriodMs)) {
        return drivers::DRIVER_ERROR_INVALID_ARG;
    }

    g_grayOledPeriodMs = period_ms;
    return GrayOledSetEnabled(true);
#else
    (void) period_ms;
    return drivers::DRIVER_ERROR_UNSUPPORTED;
#endif
}

void AppShell_RegisterCommands(void)
{
    (void) services::Shell_RegisterCommand("version",
                                           "Show firmware and board info",
                                           VersionCommand);
    (void) services::Shell_RegisterCommand("reset",
                                           "Reset the MCU",
                                           ResetCommand);
#if FEATURE_ENABLE_SCHEDULER_STATS
    (void) services::Shell_RegisterCommand("sched",
                                           "Scheduler runtime stats",
                                           SchedulerCommand);
#endif
#if FEATURE_ENABLE_STATUS_LED
    (void) services::Shell_RegisterCommand("led",
                                           "Control LEDs: led <1|2|3|all> on|off|toggle|status",
                                           LedCommand);
#endif
#if FEATURE_ENABLE_BUZZER
    (void) services::Shell_RegisterCommand("buzzer",
                                           "Control buzzer: buzzer on|off",
                                           BuzzerCommand);
#endif
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
    (void) services::Shell_RegisterCommand(
        "param",
        "Params: status|get|set|save|load|reset",
        ParamCommand);
#endif
#if FEATURE_ENABLE_INA219
    (void) services::Shell_RegisterCommand(
        "ina219",
        "INA219: status|scan|addr|recover|config|read|raw|reg|oled",
        Ina219Command);
#endif
#if FEATURE_ENABLE_OLED
    (void) services::Shell_RegisterCommand(
        "oled",
        "OLED: status|init|clear|fill|test|invert|on|off",
        OledCommand);
#endif
#if FEATURE_ENABLE_GY931
    (void) services::Shell_RegisterCommand(
        "gy931",
        "GY931: status|init|recover|scan|addr|angle|sample|raw|oled",
        Gy931Command);
#endif
#if FEATURE_ENABLE_IMU
    (void) services::Shell_RegisterCommand(
        "imu",
        "IMU SPI: status|sample|oled on|off|once|icm init|icm whoami|icm sample|icm reg <a>|icm wreg <a> <v>",
        ImuCommand);
#endif
#if FEATURE_ENABLE_GRAYSCALE
    (void) services::Shell_RegisterCommand(
        "gray",
        "Grayscale: status|read <0..7>|all|data|oled",
        GrayCommand);
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
    (void) services::Shell_RegisterCommand(
        "chassis",
        "Chassis: status|stop|wheel <l_rpm> <r_rpm>|vel <mm_s> <mdeg_s>",
        ChassisCommand);
#endif
#if FEATURE_ENABLE_SHELL_DIAGNOSTICS
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
#endif
}

} /* namespace app */
