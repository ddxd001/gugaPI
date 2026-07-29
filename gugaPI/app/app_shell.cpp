#include "app/app_shell.h"

#include <stdint.h>

#include "app/app_main.h"
#include "app/app_can_bus.h"
#include "app/chassis.h"
#include "app/app_grayscale.h"
#include "app/app_imu.h"
#include "app/app_ina219.h"
#include "app/app_jyme02_can.h"
#include "app/dm_g6220_controller.h"
#include "app/battery_monitor.h"
#include "app/app_lora.h"
#include "app/action.h"
#include "app/config_store.h"
#include "app/heading.h"
#include "app/linefollow.h"
#include "app/motor_driver_client.h"
#include "app/road_event_controller.h"
#include "app/seq_store.h"
#include "board/board_buzzer.h"
#include "board/board_button.h"
#include "board/board_can.h"
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
#include "config/debug_config.h"
#include "drivers/common/driver_status.h"
#include "drivers/i2c_diag/i2c_diag.h"
#include "services/debug_uart.h"
#include "services/fault.h"
#include "services/scheduler.h"
#include "services/shell.h"
#include "services/time.h"
#include "ti_msp_dl_config.h"

namespace app {
namespace {

namespace motor = motor_driver_client;

#if FEATURE_ENABLE_CAN
static bool g_canWatchEnabled = false;
#endif

static const uint16_t kFramShellMaxReadBytes = 32U;
static const uint8_t kParamExportDefaultCount = 16U;
static const uint8_t kParamExportMaxCount = 16U;
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
#if FEATURE_ENABLE_OLED
static const uint8_t kOledTextRows = 4U;
static const uint8_t kOledTextCols = 21U;
#endif
#if FEATURE_ENABLE_GY931
static const uint8_t kGy931MaxReadWords = drivers::GY931_MAX_READ_WORDS;
#endif
#if FEATURE_ENABLE_GY931 && FEATURE_ENABLE_OLED
static const uint32_t kGy931OledTaskPeriodMs = 50U;
static const uint32_t kGy931OledDefaultPeriodMs = 200U;
static const uint32_t kGy931OledMinPeriodMs = 50U;
static const uint32_t kGy931OledMaxPeriodMs = 5000U;
#endif
#if FEATURE_ENABLE_INA219 && FEATURE_ENABLE_OLED
static const uint32_t kIna219OledTaskPeriodMs = 50U;
static const uint32_t kIna219OledDefaultPeriodMs = 500U;
static const uint32_t kIna219OledMinPeriodMs = 100U;
static const uint32_t kIna219OledMaxPeriodMs = 5000U;
#endif
#if FEATURE_ENABLE_INA219
static const uint32_t kBatteryLogTaskPeriodMs = 50U;
static const uint32_t kBatteryLogDefaultPeriodMs = 500U;
static const uint32_t kBatteryLogMinPeriodMs = 100U;
static const uint32_t kBatteryLogMaxPeriodMs = 5000U;
#endif
#if FEATURE_ENABLE_GRAYSCALE && FEATURE_ENABLE_OLED
static const uint32_t kGrayOledTaskPeriodMs = 50U;
static const uint32_t kGrayOledDefaultPeriodMs = 200U;
static const uint32_t kGrayOledMinPeriodMs = 50U;
static const uint32_t kGrayOledMaxPeriodMs = 5000U;
#endif
static const int32_t kChassisLinearLimitMmS = 5000;
static const int32_t kChassisAngularLimitMdegS = 720000;
motor::Client g_motorClient = { motor::TRANSPORT_I2C,
                                motor::kI2cDefaultAddress };
#if FEATURE_ENABLE_GY931 && FEATURE_ENABLE_OLED
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
#endif

#if FEATURE_ENABLE_IMU && FEATURE_ENABLE_OLED
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
#if FEATURE_ENABLE_INA219
bool g_batteryLogEnabled = false;
bool g_batteryLogTaskRegistered = false;
services::SchedulerTaskId g_batteryLogTaskId = 0U;
uint32_t g_batteryLogPeriodMs = kBatteryLogDefaultPeriodMs;
uint32_t g_batteryLogLastUpdateMs = 0U;
#endif
#if FEATURE_ENABLE_GRAYSCALE && FEATURE_ENABLE_OLED
bool g_grayOledEnabled = false;
bool g_grayOledTaskRegistered = false;
services::SchedulerTaskId g_grayOledTaskId = 0U;
uint32_t g_grayOledPeriodMs = kGrayOledDefaultPeriodMs;
uint32_t g_grayOledLastUpdateMs = 0U;
drivers::DriverStatus g_grayOledLastStatus = drivers::DRIVER_OK;
#endif

/* FireWater telemetry (VOFA+ protocol). Outputs comma-separated ASCII data
 * at a configurable rate for real-time plotting. */
bool g_telemEnabled = false;
bool g_telemTaskRegistered = false;
services::SchedulerTaskId g_telemTaskId = 0U;
uint32_t g_telemPeriodMs = 100U;
uint32_t g_telemLastUpdateMs = 0U;
bool g_telemHeaderSent = false;
enum TelemProfile {
    TELEM_PROFILE_FULL = 0,
    TELEM_PROFILE_RUNTIME,
    TELEM_PROFILE_COMPETITION,
    TELEM_PROFILE_MOTOR,
    TELEM_PROFILE_HEADING,
    TELEM_PROFILE_HEADING_OUTPUT,
    TELEM_PROFILE_CHASSIS,
    TELEM_PROFILE_FEEDBACK_STATE,
    TELEM_PROFILE_FEEDBACK_AGE,
    /* Legacy combined line profile remains available for compatibility. */
    TELEM_PROFILE_LINE,
    TELEM_PROFILE_LINE_POSITION,
    TELEM_PROFILE_LINE_OUTPUT,
    TELEM_PROFILE_LINE_QUALITY,
    TELEM_PROFILE_LINE_STATE,
    TELEM_PROFILE_LINE_FAULTS,
    TELEM_PROFILE_ROAD_EVENT,
    TELEM_PROFILE_ROAD_SEQUENCE,
    TELEM_PROFILE_ROAD_PHASE,
    TELEM_PROFILE_TURN_PHASE,
    TELEM_PROFILE_TURN_RATE,
    TELEM_PROFILE_TURN_ANGLE,
    TELEM_PROFILE_TURN_TIMING,
    TELEM_PROFILE_TURN_SPEED,
    TELEM_PROFILE_ACCEL,
    TELEM_PROFILE_GYRO,
    TELEM_PROFILE_ATTITUDE,
    TELEM_PROFILE_IMU_TEMPERATURE,
    TELEM_PROFILE_IMU_STATE,
    TELEM_PROFILE_IMU_AGE,
    TELEM_PROFILE_GRAY_RAW,
    TELEM_PROFILE_GRAY_HEALTH,
    TELEM_PROFILE_GRAY_AGE,
    TELEM_PROFILE_FAULT,
    TELEM_PROFILE_UART
};
TelemProfile g_telemProfile = TELEM_PROFILE_FULL;

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

#if FEATURE_ENABLE_OLED
bool CompetitionOwnsOled(void)
{
    const AppMode mode = App_GetState()->mode;
    return (mode == APP_MODE_COMPETITION_ARMED) ||
           (mode == APP_MODE_COMPETITION_RUNNING);
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
#endif

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

bool ParseHexUint32(const char *text,
                    uint32_t max_value,
                    uint32_t *out_value)
{
    uint32_t value = 0U;
    if ((text == 0) || (out_value == 0) || (*text == '\0')) {
        return false;
    }
    if ((text[0] == '0') && ((text[1] == 'x') || (text[1] == 'X'))) {
        text += 2;
        if (*text == '\0') {
            return false;
        }
    }
    while (*text != '\0') {
        uint32_t digit = 0U;
        if ((*text >= '0') && (*text <= '9')) {
            digit = static_cast<uint32_t>(*text - '0');
        } else if ((*text >= 'a') && (*text <= 'f')) {
            digit = 10U + static_cast<uint32_t>(*text - 'a');
        } else if ((*text >= 'A') && (*text <= 'F')) {
            digit = 10U + static_cast<uint32_t>(*text - 'A');
        } else {
            return false;
        }
        if ((digit > max_value) ||
            (value > ((max_value - digit) / 16U))) {
            return false;
        }
        value = (value * 16U) + digit;
        text++;
    }
    *out_value = value;
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

#if FEATURE_ENABLE_GY931
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
#endif

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
        case drivers::DRIVER_ERROR_NACK:
            return "nack";
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
    services::Shell_WriteLine("  param export [start [count 1..16]]");
    services::Shell_WriteLine("  param set <name> <value>");
    services::Shell_WriteLine("  param save");
    services::Shell_WriteLine("  param load");
    services::Shell_WriteLine("  param reset");
}

#if FEATURE_ENABLE_INA219
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
    services::Shell_WriteLine("  ina219 protect status|clear");
    services::Shell_WriteLine("  ina219 oled on [period_ms 100..5000]|off|status|once");
}

void PrintBatteryUsage(void)
{
    services::Shell_WriteLine("usage:");
    services::Shell_WriteLine("  battery status");
    services::Shell_WriteLine("  battery full");
    services::Shell_WriteLine("  battery reset");
    services::Shell_WriteLine("  battery log on [period_ms 100..5000]");
    services::Shell_WriteLine("  battery log off|status");
}
#endif

#if FEATURE_ENABLE_OLED
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
#endif

#if FEATURE_ENABLE_GY931
void PrintGy931Usage(void)
{
    services::Shell_WriteLine("usage:");
    services::Shell_WriteLine("  gy931 status");
    services::Shell_WriteLine("  gy931 init");
    services::Shell_WriteLine("  gy931 recover");
    services::Shell_WriteLine("  gy931 scan [start end]");
    services::Shell_WriteLine("  gy931 addr [0x08..0x77]");
    services::Shell_WriteLine("  gy931 angle");
    services::Shell_WriteLine("  gy931 algorithm [6axis|9axis]");
    services::Shell_WriteLine("  gy931 sample");
    services::Shell_WriteLine("  gy931 raw <reg> <words 1..16>");
    services::Shell_WriteLine("  gy931 oled on [period_ms 50..5000]|off|status|once");
}
#endif

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

#if FEATURE_ENABLE_INA219 || \
    (FEATURE_ENABLE_GRAYSCALE && FEATURE_ENABLE_OLED)
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
#endif

#if FEATURE_ENABLE_GY931 || FEATURE_ENABLE_INA219
char *AppendHex8Text(char *cursor, char *end, uint8_t value)
{
    cursor = AppendString(cursor, end, "0x");
    cursor = AppendChar(cursor, end, HexDigit((uint8_t) (value >> 4U)));
    return AppendChar(cursor, end, HexDigit(value));
}
#endif

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
    if (CompetitionOwnsOled()) {
        return drivers::DRIVER_ERROR_BUSY;
    }
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
    if ((!g_gy931OledEnabled) || services::Fault_HasFault()) {
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
    if (enabled && CompetitionOwnsOled()) {
        return drivers::DRIVER_ERROR_BUSY;
    }
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
    if (CompetitionOwnsOled()) {
        return drivers::DRIVER_ERROR_BUSY;
    }
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
    if ((!g_ina219OledEnabled) || services::Fault_HasFault()) {
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
    if (enabled && CompetitionOwnsOled()) {
        return drivers::DRIVER_ERROR_BUSY;
    }
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

    return ImuOledWriteAngleLine(3U, "Yaw: ", imu->yaw_mdeg);
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
    if (CompetitionOwnsOled()) {
        return drivers::DRIVER_ERROR_BUSY;
    }
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
    if ((!g_imuOledEnabled) || services::Fault_HasFault()) {
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
    if (enabled && CompetitionOwnsOled()) {
        return drivers::DRIVER_ERROR_BUSY;
    }
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
drivers::DriverStatus GrayOledWritePositionLine(
    const AppGrayscaleData *data)
{
    char line[kOledTextCols + 1U];
    char *cursor = line;

    cursor = AppendString(cursor, &line[kOledTextCols], "P:");
    cursor = AppendIntDec(cursor, &line[kOledTextCols], data->line_position);
    cursor = AppendString(cursor, &line[kOledTextCols], " V:");
    cursor = AppendUIntDec(cursor,
                           &line[kOledTextCols],
                           data->position_valid ? 1U : 0U);
    cursor = AppendString(cursor, &line[kOledTextCols], " C:");
    cursor = AppendUIntDec(cursor,
                           &line[kOledTextCols],
                           data->position_confidence);
    FinishOledLine(line, cursor);
    return board::Board_OledWriteText(0U, 0U, line);
}

drivers::DriverStatus GrayOledWriteQualityLine(
    const AppGrayscaleData *data)
{
    char line[kOledTextCols + 1U];
    char *cursor = line;

    cursor = AppendString(cursor, &line[kOledTextCols], "S:");
    cursor = AppendUIntDec(cursor,
                           &line[kOledTextCols],
                           data->line_strength);
    cursor = AppendString(cursor, &line[kOledTextCols], " W:");
    cursor = AppendUIntDec(cursor,
                           &line[kOledTextCols],
                           data->weak_tracking_frames);
    cursor = AppendString(cursor, &line[kOledTextCols], " I:");
    cursor = AppendUIntDec(cursor,
                           &line[kOledTextCols],
                           data->invalid_frames);
    FinishOledLine(line, cursor);
    return board::Board_OledWriteText(1U, 0U, line);
}

drivers::DriverStatus GrayOledWriteWheelLine(uint8_t row,
                                             const char *label,
                                             int32_t left_rpm,
                                             int32_t right_rpm)
{
    char line[kOledTextCols + 1U];
    char *cursor = AppendString(line, &line[kOledTextCols], label);
    cursor = AppendString(cursor, &line[kOledTextCols], " L:");
    cursor = AppendIntDec(cursor, &line[kOledTextCols], left_rpm);
    cursor = AppendString(cursor, &line[kOledTextCols], " R:");
    cursor = AppendIntDec(cursor, &line[kOledTextCols], right_rpm);
    FinishOledLine(line, cursor);
    return board::Board_OledWriteText(row, 0U, line);
}

drivers::DriverStatus GrayOledShowTracking(const AppGrayscaleData *data,
                                           const ChassisState *chassis)
{
    drivers::DriverStatus status = GrayOledWritePositionLine(data);
    if (status != drivers::DRIVER_OK) {
        return status;
    }
    status = GrayOledWriteQualityLine(data);
    if (status != drivers::DRIVER_OK) {
        return status;
    }
    status = GrayOledWriteWheelLine(2U,
                                    "T",
                                    chassis->left.target_rpm,
                                    chassis->right.target_rpm);
    if (status != drivers::DRIVER_OK) {
        return status;
    }
    return GrayOledWriteWheelLine(3U,
                                  "A",
                                  chassis->left.actual_rpm,
                                  chassis->right.actual_rpm);
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
    if (CompetitionOwnsOled()) {
        return drivers::DRIVER_ERROR_BUSY;
    }
    if (!board::Board_OledIsReady()) {
        const drivers::DriverStatus init_status = board::Board_OledInit();
        if (init_status != drivers::DRIVER_OK) {
            g_grayOledLastStatus = init_status;
            return init_status;
        }
    }

    const AppGrayscaleData *data = App_GrayscaleGetData();
    if ((data == 0) || (!data->valid) || (!data->processed_valid)) {
        const drivers::DriverStatus data_status = (data != 0)
            ? data->processing_status
            : drivers::DRIVER_ERROR;
        g_grayOledLastStatus = data_status;
        (void) GrayOledShowError(data_status);
        return data_status;
    }

    const ChassisState *chassis = Chassis_GetState();
    if (chassis == 0) {
        g_grayOledLastStatus = drivers::DRIVER_ERROR;
        return g_grayOledLastStatus;
    }

    /* Use the existing application snapshots. This page adds no ADC read and
     * no MotorDriver transaction, so OLED refresh cannot disturb the 5 ms
     * grayscale pipeline or the 20 ms chassis-feedback cadence. */
    const drivers::DriverStatus oled_status =
        GrayOledShowTracking(data, chassis);
    g_grayOledLastStatus = oled_status;
    return oled_status;
}

void GrayOledTask(void)
{
    if ((!g_grayOledEnabled) || services::Fault_HasFault()) {
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
    if (enabled && CompetitionOwnsOled()) {
        return drivers::DRIVER_ERROR_BUSY;
    }
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
    services::Shell_WriteLine("  imu bias status");
    services::Shell_WriteLine("  imu bias calibrate");
    services::Shell_WriteLine("  imu bias auto on|off");
    services::Shell_WriteLine("  imu bias save");
    services::Shell_WriteLine("  imu bias reset");
    services::Shell_WriteLine("  imu lis whoami");
    services::Shell_WriteLine("  imu lis reg <addr>");
    services::Shell_WriteLine("  imu lis status|init|sample");
    services::Shell_WriteLine("  imu lis scale <0..3>|odr <0..7>|mode <0..2>");
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
    services::Shell_WriteLine("  lora proto on|off|status|reset|recv");
    services::Shell_WriteLine("  lora proto send <type> <ack|noack> [text...]");
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
    services::Shell_WriteLine("  motor ramp [accel_rpm_s decel_rpm_s]");
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
    services::Shell_WriteLine("  chassis stat");
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

void PrintMotorRpmBlock(const motor::RpmData &rpm,
                        const motor::SpeedControlTelemetry *telemetry)
{
    services::Shell_WriteString("motor rpm m1 target=");
    services::Shell_WriteUInt32(rpm.target_m1);
    if (telemetry != 0) {
        services::Shell_WriteString(" control=");
        services::Shell_WriteUInt32(telemetry->control_m1);
    }
    services::Shell_WriteString(" actual=");
    WriteInt32(rpm.actual_m1);
    if (telemetry != 0) {
        services::Shell_WriteString(" error=");
        WriteInt32(telemetry->error_m1);
        services::Shell_WriteString(" integral_q4=");
        WriteInt32(telemetry->integral_m1_q4);
        services::Shell_WriteString(" duty=");
        services::Shell_WriteUInt32(telemetry->duty_m1);
    }
    services::Shell_WriteString("\r\n");

    services::Shell_WriteString("motor rpm m2 target=");
    services::Shell_WriteUInt32(rpm.target_m2);
    if (telemetry != 0) {
        services::Shell_WriteString(" control=");
        services::Shell_WriteUInt32(telemetry->control_m2);
    }
    services::Shell_WriteString(" actual=");
    WriteInt32(rpm.actual_m2);
    if (telemetry != 0) {
        services::Shell_WriteString(" error=");
        WriteInt32(telemetry->error_m2);
        services::Shell_WriteString(" integral_q4=");
        WriteInt32(telemetry->integral_m2_q4);
        services::Shell_WriteString(" duty=");
        services::Shell_WriteUInt32(telemetry->duty_m2);
    }
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
    services::Shell_WriteString(" fb=");
    services::Shell_WriteString(DriverStatusText(state.last_feedback_status));
    services::Shell_WriteString(" fb_seq=");
    services::Shell_WriteUInt32(state.feedback_sequence);
    services::Shell_WriteString(" fb_ms=");
    services::Shell_WriteUInt32(state.last_feedback_ms);
    services::Shell_WriteString("\r\n");

    services::Shell_WriteString("chassis cfg radius_mm=");
    WriteFixedMilli(static_cast<int32_t>(state.config.wheel_radius_um));
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

void PrintParamExportPage(uint8_t start, uint8_t requested_count)
{
    const uint8_t total = ConfigStore_ParamCount();
    uint8_t count = static_cast<uint8_t>(total - start);
    if (count > requested_count) {
        count = requested_count;
    }

    services::Shell_WriteString("param export start=");
    services::Shell_WriteUInt32(start);
    services::Shell_WriteString(" count=");
    services::Shell_WriteUInt32(count);
    services::Shell_WriteString(" total=");
    services::Shell_WriteUInt32(total);
    services::Shell_WriteString("\r\n");

    for (uint8_t offset = 0U; offset < count; offset++) {
        const char *name = ConfigStore_ParamName(
            static_cast<uint8_t>(start + offset));
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
#if FEATURE_ENABLE_CAN
void WriteCanFrame(const drivers::CanFrame &frame)
{
    services::Shell_WriteString("can rx ");
    services::Shell_WriteString(frame.extended ? "ext id=" : "std id=");
    WriteHex32(frame.id);
    services::Shell_WriteString(" dlc=");
    services::Shell_WriteUInt32(frame.length);
    services::Shell_WriteString(" data=");
    for (uint8_t i = 0U; i < frame.length; i++) {
        if (i != 0U) {
            services::Shell_WriteString(" ");
        }
        WriteHex8(frame.data[i]);
    }
    services::Shell_WriteString("\r\n");
}

void PrintCanUsage(void)
{
    services::Shell_WriteLine("usage:");
    services::Shell_WriteLine("  can status");
    services::Shell_WriteLine("  can mode normal|standby");
    services::Shell_WriteLine("  can send std|ext <hex_id> [hex_byte ...]");
    services::Shell_WriteLine("  can read [count 1..32]");
    services::Shell_WriteLine("  can watch on|off");
    services::Shell_WriteLine("  can clear|cancel|recover");
}

void WriteCanStatus(void)
{
    drivers::CanStatus status = {};
    const drivers::DriverStatus result =
        board::Board_CanGetStatus(&status);
    if (result != drivers::DRIVER_OK) {
        WriteStatusLine("can status: ", result);
        return;
    }

    services::Shell_WriteString("can ready=1 mode=");
    services::Shell_WriteString(
        (status.mode == drivers::CAN_TRANSCEIVER_NORMAL) ?
        "normal" : "standby");
    services::Shell_WriteString(" bitrate=");
    services::Shell_WriteUInt32(status.bitrate);
    services::Shell_WriteString(" watch=");
    services::Shell_WriteUInt32(g_canWatchEnabled ? 1U : 0U);
    services::Shell_WriteString(" rx/queued/drop/fifo_lost=");
    services::Shell_WriteUInt32(status.rx_count);
    services::Shell_WriteString("/");
    services::Shell_WriteUInt32(status.rx_available);
    services::Shell_WriteString("/");
    services::Shell_WriteUInt32(status.rx_dropped_count);
    services::Shell_WriteString("/");
    services::Shell_WriteUInt32(status.rx_fifo_lost_count);
    services::Shell_WriteString(" app_queue/overwrite=");
    services::Shell_WriteUInt32(AppCanBus_GetRawAvailable());
    services::Shell_WriteString("/");
    services::Shell_WriteUInt32(AppCanBus_GetRawDropped());
    services::Shell_WriteString(" tx_req/done/cancel=");
    services::Shell_WriteUInt32(status.tx_request_count);
    services::Shell_WriteString("/");
    services::Shell_WriteUInt32(status.tx_complete_count);
    services::Shell_WriteString("/");
    services::Shell_WriteUInt32(status.tx_cancel_count);
    services::Shell_WriteString(" pending=");
    services::Shell_WriteUInt32(status.tx_pending ? 1U : 0U);
    services::Shell_WriteString(" err_evt=");
    services::Shell_WriteUInt32(status.error_event_count);
    services::Shell_WriteString(" TEC/REC=");
    services::Shell_WriteUInt32(status.tx_error_count);
    services::Shell_WriteString("/");
    services::Shell_WriteUInt32(status.rx_error_count);
    services::Shell_WriteString(" LEC=");
    services::Shell_WriteUInt32(status.last_error_code);
    services::Shell_WriteString(" BO/EP/EW=");
    services::Shell_WriteUInt32(status.bus_off ? 1U : 0U);
    services::Shell_WriteString("/");
    services::Shell_WriteUInt32(status.error_passive ? 1U : 0U);
    services::Shell_WriteString("/");
    services::Shell_WriteUInt32(status.warning ? 1U : 0U);
    services::Shell_WriteString(" irq=");
    WriteHex32(status.last_interrupt_status);
    services::Shell_WriteString("\r\n");
}

void CanCommand(int argc, const char * const argv[])
{
    if (argc < 2) {
        PrintCanUsage();
        return;
    }

    if (StrEqual(argv[1], "status") && (argc == 2)) {
        WriteCanStatus();
        return;
    }

    if (StrEqual(argv[1], "mode") && (argc == 3)) {
        drivers::CanTransceiverMode mode;
        if (StrEqual(argv[2], "normal")) {
            mode = drivers::CAN_TRANSCEIVER_NORMAL;
        } else if (StrEqual(argv[2], "standby")) {
            mode = drivers::CAN_TRANSCEIVER_STANDBY;
        } else {
            PrintCanUsage();
            return;
        }
        WriteStatusLine("can mode: ", board::Board_CanSetMode(mode));
        return;
    }

    if (StrEqual(argv[1], "send") &&
        (argc >= 4) && (argc <= 12)) {
#if FEATURE_ENABLE_DM_G6220_CAN
        if ((App_GetState()->mode != APP_MODE_RUNNING) ||
            DmG6220Controller_IsTxReserved()) {
            services::Shell_WriteLine(
                "can send: busy (DM control owns CAN TX)");
            return;
        }
#endif
        drivers::CanFrame frame = {};
        if (StrEqual(argv[2], "std")) {
            frame.extended = false;
        } else if (StrEqual(argv[2], "ext")) {
            frame.extended = true;
        } else {
            PrintCanUsage();
            return;
        }
        const uint32_t max_id =
            frame.extended ? 0x1FFFFFFFU : 0x7FFU;
        if (!ParseHexUint32(argv[3], max_id, &frame.id)) {
            PrintCanUsage();
            return;
        }
        frame.length = static_cast<uint8_t>(argc - 4);
        for (uint8_t i = 0U; i < frame.length; i++) {
            uint32_t value = 0U;
            if (!ParseHexUint32(argv[4 + i], 0xFFU, &value)) {
                PrintCanUsage();
                return;
            }
            frame.data[i] = static_cast<uint8_t>(value);
        }
        WriteStatusLine("can send: ", board::Board_CanSend(&frame));
        return;
    }

    if (StrEqual(argv[1], "read") &&
        ((argc == 2) || (argc == 3))) {
        uint32_t count = 1U;
        if ((argc == 3) &&
            ((!ParseUint32(argv[2], BOARD_CAN_RX_QUEUE_SIZE, &count)) ||
             (count == 0U))) {
            PrintCanUsage();
            return;
        }
        uint32_t read_count = 0U;
        drivers::CanFrame frame = {};
        while ((read_count < count) && AppCanBus_ReadRaw(&frame)) {
            WriteCanFrame(frame);
            read_count++;
        }
        if (read_count == 0U) {
            services::Shell_WriteLine("can rx: none");
        }
        return;
    }

    if (StrEqual(argv[1], "watch") && (argc == 3)) {
        if (StrEqual(argv[2], "on")) {
            g_canWatchEnabled = true;
        } else if (StrEqual(argv[2], "off")) {
            g_canWatchEnabled = false;
        } else {
            PrintCanUsage();
            return;
        }
        services::Shell_WriteString("can watch ");
        services::Shell_WriteLine(g_canWatchEnabled ? "on" : "off");
        return;
    }

    if (StrEqual(argv[1], "clear") && (argc == 2)) {
        AppCanBus_Clear();
        WriteStatusLine("can clear: ", board::Board_CanClear());
        return;
    }
    if (StrEqual(argv[1], "cancel") && (argc == 2)) {
        WriteStatusLine("can cancel: ", board::Board_CanCancelTx());
        return;
    }
    if (StrEqual(argv[1], "recover") && (argc == 2)) {
        WriteStatusLine("can recover: ", board::Board_CanRecover());
        return;
    }

    PrintCanUsage();
}

#if FEATURE_ENABLE_DM_G6220_CAN
void PrintDmUsage(void)
{
    services::Shell_WriteLine("usage:");
    services::Shell_WriteLine("  dm status");
    services::Shell_WriteLine("  dm probe");
    services::Shell_WriteLine("  dm enable");
    services::Shell_WriteLine(
        "  dm position absolute|relative <target_mrad> <max_velocity_mrad_s> <timeout_ms>");
    services::Shell_WriteLine("  dm speed <velocity_mrad_s>");
    services::Shell_WriteLine("  dm hold");
    services::Shell_WriteLine("  dm disable");
    services::Shell_WriteLine("  dm clear");
    services::Shell_WriteLine("  dm zero confirm");
}

bool DmManualCommandAllowed(void)
{
    if (App_GetState()->mode != APP_MODE_RUNNING) {
        services::Shell_WriteLine(
            "dm: manual commands require dev-running mode");
        return false;
    }
    return true;
}

void WriteDmStatus(void)
{
    const DmG6220ControlState *state = DmG6220Controller_GetState();
    const drivers::DmG6220Feedback *feedback =
        DmG6220Controller_GetFeedback();
    const uint32_t now_ms = services::Time_Millis();
    services::Shell_WriteString("dm mode=");
    services::Shell_WriteString(DmG6220Controller_ModeText(state->mode));
    services::Shell_WriteString(" result=");
    services::Shell_WriteUInt32(
        static_cast<uint32_t>(state->operation_result));
    services::Shell_WriteString(" enabled=");
    services::Shell_WriteUInt32(state->enabled ? 1U : 0U);
    services::Shell_WriteString(" online=");
    services::Shell_WriteUInt32((feedback != 0) && feedback->valid ? 1U : 0U);
    services::Shell_WriteString(" fresh=");
    services::Shell_WriteUInt32(
        DmG6220Controller_IsFeedbackFresh(now_ms) ? 1U : 0U);
    services::Shell_WriteString(" state=");
    services::Shell_WriteUInt32(
        (feedback != 0) ? feedback->state : 0U);
    services::Shell_WriteString(" p_mrad=");
    WriteInt32((feedback != 0) ? feedback->position_mrad : 0);
    services::Shell_WriteString(" v_mrad_s=");
    WriteInt32((feedback != 0) ? feedback->velocity_mrad_s : 0);
    services::Shell_WriteString(" torque_mNm=");
    WriteInt32((feedback != 0) ? feedback->torque_mnm : 0);
    services::Shell_WriteString(" temp_mos/coil=");
    services::Shell_WriteUInt32(
        (feedback != 0) ? feedback->mos_temperature_c : 0U);
    services::Shell_WriteString("/");
    services::Shell_WriteUInt32(
        (feedback != 0) ? feedback->coil_temperature_c : 0U);
    services::Shell_WriteString(" age_ms=");
    services::Shell_WriteUInt32(
        ((feedback != 0) && feedback->valid) ?
            static_cast<uint32_t>(now_ms - feedback->last_update_ms) :
            0xFFFFFFFFU);
    services::Shell_WriteString(" ref=");
    WriteInt32(state->reference_position_mrad);
    services::Shell_WriteString(" target=");
    WriteInt32(state->target_position_mrad);
    services::Shell_WriteString(" tx/busy/error=");
    services::Shell_WriteUInt32(state->tx_count);
    services::Shell_WriteString("/");
    services::Shell_WriteUInt32(state->tx_busy_count);
    services::Shell_WriteString("/");
    services::Shell_WriteUInt32(state->tx_error_count);
    services::Shell_WriteString(" status=");
    services::Shell_WriteString(DriverStatusText(state->last_status));
    services::Shell_WriteString("\r\n");
}

void DmCommand(int argc, const char * const argv[])
{
    if ((argc == 2) && StrEqual(argv[1], "status")) {
        WriteDmStatus();
        return;
    }
    if ((argc < 2) || !DmManualCommandAllowed()) {
        PrintDmUsage();
        return;
    }
    if ((argc == 2) && StrEqual(argv[1], "probe")) {
        WriteStatusLine("dm probe: ", DmG6220Controller_Probe());
        return;
    }
    if ((argc == 2) && StrEqual(argv[1], "enable")) {
        WriteStatusLine("dm enable: ", DmG6220Controller_EnableHold());
        return;
    }
    if ((argc == 6) && StrEqual(argv[1], "position")) {
        DmG6220PositionFrame frame = DM_POSITION_ABSOLUTE;
        if (StrEqual(argv[2], "relative")) {
            frame = DM_POSITION_RELATIVE;
        } else if (!StrEqual(argv[2], "absolute")) {
            PrintDmUsage();
            return;
        }
        int32_t target_mrad = 0;
        int32_t max_velocity_mrad_s = 0;
        uint32_t timeout_ms = 0U;
        if ((!ParseInt32(argv[3],
                         -drivers::DM_G6220_POSITION_LIMIT_MRAD,
                         drivers::DM_G6220_POSITION_LIMIT_MRAD,
                         &target_mrad)) ||
            (!ParseInt32(argv[4], 1, 20000, &max_velocity_mrad_s)) ||
            (!ParseUint32(argv[5], 30000U, &timeout_ms)) ||
            (timeout_ms < 50U)) {
            PrintDmUsage();
            return;
        }
        WriteStatusLine(
            "dm position: ",
            DmG6220Controller_StartPosition(frame,
                                            target_mrad,
                                            max_velocity_mrad_s,
                                            timeout_ms));
        return;
    }
    if ((argc == 3) && StrEqual(argv[1], "speed")) {
        int32_t velocity_mrad_s = 0;
        if ((!ParseInt32(argv[2], -20000, 20000, &velocity_mrad_s)) ||
            (velocity_mrad_s == 0)) {
            PrintDmUsage();
            return;
        }
        WriteStatusLine("dm speed: ",
                        DmG6220Controller_StartSpeed(velocity_mrad_s));
        return;
    }
    if ((argc == 2) && StrEqual(argv[1], "hold")) {
        WriteStatusLine("dm hold: ", DmG6220Controller_HoldCurrent());
        return;
    }
    if ((argc == 2) && StrEqual(argv[1], "disable")) {
        WriteStatusLine("dm disable: ", DmG6220Controller_Disable());
        return;
    }
    if ((argc == 2) && StrEqual(argv[1], "clear")) {
        WriteStatusLine("dm clear: ", DmG6220Controller_ClearError());
        return;
    }
    if ((argc == 3) && StrEqual(argv[1], "zero") &&
        StrEqual(argv[2], "confirm")) {
        WriteStatusLine("dm zero: ", DmG6220Controller_SetZero());
        return;
    }
    PrintDmUsage();
}
#endif

#if FEATURE_ENABLE_JYME02_CAN
void PrintJyme02Usage(void)
{
    services::Shell_WriteLine("usage:");
    services::Shell_WriteLine("  jyme02 status");
    services::Shell_WriteLine("  jyme02 readreg <hex_reg>");
    services::Shell_WriteLine("  jyme02 regs");
    services::Shell_WriteLine("  jyme02 address <hex_id>       (parser only)");
    services::Shell_WriteLine("  jyme02 sampletime <100us>     (parser only)");
    services::Shell_WriteLine("  jyme02 clear");
}

void WriteJyme02Status(void)
{
    const drivers::JYME02CanData *data = AppJyme02Can_GetData();
    if (data == 0) {
        services::Shell_WriteLine("jyme02: not initialized");
        return;
    }

    const uint32_t now_ms = services::Time_Millis();
    services::Shell_WriteString("jyme02 id=");
    WriteHex16(data->address);
    services::Shell_WriteString(" sample_100us=");
    services::Shell_WriteUInt32(data->sample_time_100us);
    services::Shell_WriteString(" fresh(meas/temp)=");
    services::Shell_WriteUInt32(
        AppJyme02Can_IsMeasurementFresh(now_ms) ? 1U : 0U);
    services::Shell_WriteString("/");
    services::Shell_WriteUInt32(
        AppJyme02Can_IsTemperatureFresh(now_ms) ? 1U : 0U);
    services::Shell_WriteString(" count(meas/temp/reg/invalid)=");
    services::Shell_WriteUInt32(data->measurement_count);
    services::Shell_WriteString("/");
    services::Shell_WriteUInt32(data->temperature_count);
    services::Shell_WriteString("/");
    services::Shell_WriteUInt32(data->register_count);
    services::Shell_WriteString("/");
    services::Shell_WriteUInt32(data->invalid_count);
    services::Shell_WriteString("\r\n");

    if (data->measurement_valid) {
        services::Shell_WriteString("  angle_raw/mdeg=");
        services::Shell_WriteUInt32(data->angle_raw);
        services::Shell_WriteString("/");
        services::Shell_WriteUInt32(data->angle_mdeg);
        services::Shell_WriteString(" velocity_raw/mdeg_s=");
        WriteInt32(data->angular_velocity_raw);
        services::Shell_WriteString("/");
        WriteInt32(data->angular_velocity_mdeg_s);
        services::Shell_WriteString(" revolutions=");
        WriteInt32(data->revolutions);
        services::Shell_WriteString(" age_ms=");
        services::Shell_WriteUInt32(
            (uint32_t) (now_ms - data->last_measurement_ms));
        services::Shell_WriteString("\r\n");
    }
    if (data->temperature_valid) {
        services::Shell_WriteString("  temperature_raw/mdeg_c=");
        WriteInt32(data->temperature_raw);
        services::Shell_WriteString("/");
        WriteInt32(data->temperature_mdeg_c);
        services::Shell_WriteString(" age_ms=");
        services::Shell_WriteUInt32(
            (uint32_t) (now_ms - data->last_temperature_ms));
        services::Shell_WriteString("\r\n");
    }
}

void WriteJyme02Registers(void)
{
    const drivers::JYME02CanData *data = AppJyme02Can_GetData();
    if ((data == 0) || (!data->register_valid)) {
        services::Shell_WriteLine("jyme02 regs: none");
        return;
    }

    services::Shell_WriteString("jyme02 regs start=");
    WriteHex8(data->register_start);
    services::Shell_WriteString(" values=");
    for (uint8_t i = 0U; i < 3U; i++) {
        if (i != 0U) {
            services::Shell_WriteString(" ");
        }
        WriteHex16(data->register_values[i]);
    }
    services::Shell_WriteString("\r\n");
}

void JYME02Command(int argc, const char * const argv[])
{
    if ((argc == 2) && StrEqual(argv[1], "status")) {
        WriteJyme02Status();
        return;
    }
    if ((argc == 2) && StrEqual(argv[1], "regs")) {
        WriteJyme02Registers();
        return;
    }
    if ((argc == 2) && StrEqual(argv[1], "clear")) {
        AppJyme02Can_Clear();
        services::Shell_WriteLine("jyme02 clear: ok");
        return;
    }
    if ((argc == 3) && StrEqual(argv[1], "readreg")) {
        uint32_t value = 0U;
        if (!ParseHexUint32(argv[2], 0xFFU, &value)) {
            PrintJyme02Usage();
            return;
        }
        WriteStatusLine(
            "jyme02 readreg: ",
            AppJyme02Can_ReadRegister(static_cast<uint8_t>(value)));
        return;
    }
    if ((argc == 3) && StrEqual(argv[1], "address")) {
        uint32_t value = 0U;
        if (!ParseHexUint32(argv[2], 0x7FFU, &value)) {
            PrintJyme02Usage();
            return;
        }
        WriteStatusLine(
            "jyme02 address: ",
            AppJyme02Can_SetAddress(static_cast<uint16_t>(value)));
        return;
    }
    if ((argc == 3) && StrEqual(argv[1], "sampletime")) {
        uint32_t value = 0U;
        if ((!ParseUint32(argv[2], 0xFFFFU, &value)) || (value == 0U)) {
            PrintJyme02Usage();
            return;
        }
        WriteStatusLine(
            "jyme02 sampletime: ",
            AppJyme02Can_SetSampleTime(static_cast<uint16_t>(value)));
        return;
    }

    PrintJyme02Usage();
}
#endif
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
        if (app::ActionRunner_GetState()->running) {
            services::Shell_WriteLine("led all: busy");
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
    if (app::ActionRunner_GetState()->running &&
        (id != board::BOARD_LED_ID_1)) {
        services::Shell_WriteLine("led: busy");
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
        services::Shell_WriteLine("usage: buzzer on|off|toggle|status");
        return;
    }

    if (!board::Board_BuzzerIsReady()) {
        services::Shell_WriteLine("buzzer: not ready");
        return;
    }

    if (StrEqual(argv[1], "status")) {
        services::Shell_WriteString("buzzer ready=1 state=");
        services::Shell_WriteLine(
            board::Board_BuzzerIsOn() ? "on" : "off");
        return;
    }

    if (app::ActionRunner_GetState()->running) {
        services::Shell_WriteLine("buzzer: busy");
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

    if (StrEqual(argv[1], "toggle")) {
        (void) board::Board_BuzzerToggle();
        services::Shell_WriteLine("buzzer toggled");
        return;
    }

    services::Shell_WriteLine("usage: buzzer on|off|toggle|status");
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
        const uint32_t events = board::Board_ButtonTakeEvents(
            id,
            drivers::BUTTON_EVENT_ALL);
        services::Shell_WriteString(" held_ms=");
        services::Shell_WriteUInt32(board::Board_ButtonGetPressDurationMs(id));
        services::Shell_WriteString(" events=");
        services::Shell_WriteString(
            ((events & drivers::BUTTON_EVENT_PRESSED) != 0U) ? "D" : "-");
        services::Shell_WriteString(
            ((events & drivers::BUTTON_EVENT_RELEASED) != 0U) ? "U" : "-");
        services::Shell_WriteString(
            ((events & drivers::BUTTON_EVENT_SHORT_PRESSED) != 0U) ? "S" : "-");
        services::Shell_WriteString(
            ((events & drivers::BUTTON_EVENT_LONG_PRESSED) != 0U) ? "L" : "-");
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

    if (StrEqual(argv[1], "export")) {
        uint32_t start = 0U;
        uint32_t count = kParamExportDefaultCount;
        const uint32_t total = ConfigStore_ParamCount();

        if ((argc > 4) ||
            ((argc >= 3) && (!ParseUint32(argv[2], total, &start))) ||
            ((argc == 4) &&
             ((!ParseUint32(argv[3], kParamExportMaxCount, &count)) ||
              (count == 0U)))) {
            PrintParamUsage();
            return;
        }

        PrintParamExportPage(static_cast<uint8_t>(start),
                             static_cast<uint8_t>(count));
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

#if FEATURE_ENABLE_OLED
void OledCommand(int argc, const char * const argv[])
{
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
        services::Shell_WriteString(" pending=");
        services::Shell_WriteUInt32(
            board::Board_OledHasPendingFlush() ? 1U : 0U);
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

    if (CompetitionOwnsOled()) {
        WriteStatusLine("oled: ", drivers::DRIVER_ERROR_BUSY);
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
}
#endif

#if FEATURE_ENABLE_GY931
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

    if (StrEqual(argv[1], "algorithm")) {
        if ((argc != 2) && (argc != 3)) {
            PrintGy931Usage();
            return;
        }

        drivers::Gy931Algorithm algorithm =
            drivers::GY931_ALGORITHM_9_AXIS;
        drivers::DriverStatus status = drivers::DRIVER_OK;
        if (argc == 3) {
            if (StrEqual(argv[2], "6axis")) {
                algorithm = drivers::GY931_ALGORITHM_6_AXIS;
            } else if (StrEqual(argv[2], "9axis")) {
                algorithm = drivers::GY931_ALGORITHM_9_AXIS;
            } else {
                PrintGy931Usage();
                return;
            }
            status = board::Board_Gy931SetAlgorithmTemporary(algorithm);
            if (status != drivers::DRIVER_OK) {
                WriteStatusLine("gy931 algorithm: ", status);
                return;
            }
        }

        status = board::Board_Gy931ReadAlgorithm(&algorithm);
        if (status != drivers::DRIVER_OK) {
            WriteStatusLine("gy931 algorithm: ", status);
            return;
        }
        services::Shell_WriteString("gy931 algorithm=");
        services::Shell_WriteString(
            (algorithm == drivers::GY931_ALGORITHM_6_AXIS) ?
                "6axis" : "9axis");
        services::Shell_WriteLine(" temporary=1");
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
#endif

#if FEATURE_ENABLE_INA219
void Ina219Command(int argc, const char * const argv[])
{
#if FEATURE_ENABLE_INA219
    uint32_t reg = 0U;
    uint32_t value = 0U;

    if (argc < 2) {
        PrintIna219Usage();
        return;
    }

    if (StrEqual(argv[1], "protect")) {
        if ((argc == 3) && StrEqual(argv[2], "clear")) {
            App_Ina219ClearLatchedFaults();
            services::Shell_WriteLine("ina219 protect clear: ok");
            return;
        }
        if ((argc != 3) || (!StrEqual(argv[2], "status"))) {
            PrintIna219Usage();
            return;
        }

        const AppIna219Data *data = App_Ina219GetData();
        const Ina219ProtectionState &state = data->protection;
        services::Shell_WriteString("ina219 protect initialized=");
        services::Shell_WriteUInt32(data->initialized ? 1U : 0U);
        services::Shell_WriteString(" active=");
        WriteHex8(state.active_fault_mask);
        services::Shell_WriteString(" latched=");
        WriteHex8(state.latched_fault_mask);
        services::Shell_WriteString(" inhibit=");
        services::Shell_WriteUInt32(state.motion_inhibit_requested ? 1U : 0U);
        services::Shell_WriteString(" last=");
        services::Shell_WriteString(DriverStatusText(state.last_read_status));
        services::Shell_WriteString(" samples=");
        services::Shell_WriteUInt32(state.successful_sample_count);
        services::Shell_WriteString(" errors=");
        services::Shell_WriteUInt32(state.read_error_count);
        services::Shell_WriteString("\r\n");
        services::Shell_WriteString("ina219 protect uv=");
        services::Shell_WriteUInt32(state.config.undervoltage_trip_mv);
        services::Shell_WriteString("/");
        services::Shell_WriteUInt32(state.config.undervoltage_release_mv);
        services::Shell_WriteString("mV oc=");
        services::Shell_WriteUInt32(state.config.overcurrent_trip_ma);
        services::Shell_WriteString("/");
        services::Shell_WriteUInt32(state.config.overcurrent_release_ma);
        services::Shell_WriteString("mA trip/release/comm=");
        services::Shell_WriteUInt32(state.config.trip_samples);
        services::Shell_WriteString("/");
        services::Shell_WriteUInt32(state.config.release_samples);
        services::Shell_WriteString("/");
        services::Shell_WriteUInt32(state.config.communication_fail_samples);
        services::Shell_WriteString("\r\n");
        return;
    }

    if (!board::Board_Ina219IsReady()) {
        services::Shell_WriteLine("ina219: not ready");
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
            services::Time_DelayUs(800U);
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
#endif

#if FEATURE_ENABLE_INA219
const char *BatterySocSourceText(BatterySocSource source)
{
    switch (source) {
        case BATTERY_SOC_SOURCE_ESTIMATING:
            return "estimating";
        case BATTERY_SOC_SOURCE_VOLTAGE:
            return "voltage";
        case BATTERY_SOC_SOURCE_MANUAL_FULL:
            return "full";
        default:
            return "unknown";
    }
}

void WriteBatterySampleLine(void)
{
    const BatteryMonitorStatus *status = BatteryMonitor_GetStatus();
    services::Shell_WriteString("battery pack_mV=");
    services::Shell_WriteUInt32(status->pack_voltage_mv);
    services::Shell_WriteString(" cell_mV=");
    services::Shell_WriteUInt32(status->average_cell_voltage_mv);
    services::Shell_WriteString(" current_mA=");
    WriteInt32(status->current_ua / 1000);
    services::Shell_WriteString(" power_mW=");
    WriteInt32(status->power_mw);
    services::Shell_WriteString(" soc=");
    services::Shell_WriteUInt32(status->soc_percent);
    services::Shell_WriteString(" ready=");
    services::Shell_WriteUInt32(status->soc_ready ? 1U : 0U);
    services::Shell_WriteString(" valid=");
    services::Shell_WriteUInt32(status->sample_valid ? 1U : 0U);
    services::Shell_WriteString(" charging=");
    services::Shell_WriteUInt32(status->charging ? 1U : 0U);
    services::Shell_WriteString(" low=");
    services::Shell_WriteUInt32(status->low_battery ? 1U : 0U);
    services::Shell_WriteString(" critical=");
    services::Shell_WriteUInt32(status->critical_battery ? 1U : 0U);
    services::Shell_WriteString("\r\n");
}

void WriteBatteryStatus(void)
{
    const BatteryMonitorStatus *status = BatteryMonitor_GetStatus();
    services::Shell_WriteString("battery initialized=");
    services::Shell_WriteUInt32(status->initialized ? 1U : 0U);
    services::Shell_WriteString(" source=");
    services::Shell_WriteString(BatterySocSourceText(status->soc_source));
    services::Shell_WriteString(" cells=");
    services::Shell_WriteUInt32(status->series_cells);
    services::Shell_WriteString(" capacity_mAh=");
    services::Shell_WriteUInt32(status->rated_capacity_mah);
    services::Shell_WriteString(" remaining_uAh=");
    services::Shell_WriteUInt32(status->remaining_uah);
    services::Shell_WriteString(" consumed_uAh=");
    services::Shell_WriteUInt32(status->consumed_uah);
    services::Shell_WriteString(" peak_mA=");
    services::Shell_WriteUInt32(status->peak_current_ma);
    services::Shell_WriteString("\r\n");
    WriteBatterySampleLine();
    services::Shell_WriteString("battery samples=");
    services::Shell_WriteUInt32(status->successful_sample_count);
    services::Shell_WriteString(" overflow=");
    services::Shell_WriteUInt32(status->overflow_count);
    services::Shell_WriteString(" errors=");
    services::Shell_WriteUInt32(status->read_error_count);
    services::Shell_WriteString(" last=");
    services::Shell_WriteString(DriverStatusText(status->last_read_status));
    services::Shell_WriteString(" last_update_ms=");
    services::Shell_WriteUInt32(status->last_update_ms);
    services::Shell_WriteString(" log=");
    services::Shell_WriteUInt32(g_batteryLogEnabled ? 1U : 0U);
    services::Shell_WriteString(" log_period_ms=");
    services::Shell_WriteUInt32(g_batteryLogPeriodMs);
    services::Shell_WriteString("\r\n");
}

void BatteryLogTask(void)
{
    if (!g_batteryLogEnabled) {
        return;
    }
    if (!services::Time_HasElapsed(g_batteryLogLastUpdateMs,
                                   g_batteryLogPeriodMs)) {
        return;
    }

    g_batteryLogLastUpdateMs = services::Time_Millis();
    if (services::DebugUart_GetTxPending() > 3000U) {
        return;
    }
    WriteBatterySampleLine();
}

drivers::DriverStatus BatteryLogEnsureTask(void)
{
    if (g_batteryLogTaskRegistered) {
        return drivers::DRIVER_OK;
    }

    const services::SchedulerStatus status = services::Scheduler_AddTask(
        "battery_log",
        BatteryLogTask,
        kBatteryLogTaskPeriodMs,
        0U,
        &g_batteryLogTaskId);
    if (status != services::SCHEDULER_OK) {
        return SchedulerStatusToDriverStatus(status);
    }

    g_batteryLogTaskRegistered = true;
    return SchedulerStatusToDriverStatus(
        services::Scheduler_EnableTask(g_batteryLogTaskId, false));
}

drivers::DriverStatus BatteryLogSetEnabled(bool enabled)
{
    if (!enabled) {
        g_batteryLogEnabled = false;
        if (!g_batteryLogTaskRegistered) {
            return drivers::DRIVER_OK;
        }
        return SchedulerStatusToDriverStatus(
            services::Scheduler_EnableTask(g_batteryLogTaskId, false));
    }

    const drivers::DriverStatus status = BatteryLogEnsureTask();
    if (status != drivers::DRIVER_OK) {
        return status;
    }
    g_batteryLogLastUpdateMs = services::Time_Millis();
    g_batteryLogEnabled = true;
    const drivers::DriverStatus enable_status = SchedulerStatusToDriverStatus(
        services::Scheduler_EnableTask(g_batteryLogTaskId, true));
    if (enable_status != drivers::DRIVER_OK) {
        g_batteryLogEnabled = false;
    }
    return enable_status;
}

void BatteryCommand(int argc, const char * const argv[])
{
    if (argc < 2) {
        PrintBatteryUsage();
        return;
    }

    if (StrEqual(argv[1], "status")) {
        if (argc != 2) {
            PrintBatteryUsage();
            return;
        }
        WriteBatteryStatus();
        return;
    }

    if (StrEqual(argv[1], "full")) {
        if (argc != 2) {
            PrintBatteryUsage();
            return;
        }
        BatteryMonitor_SetFull();
        services::Shell_WriteLine("battery full: runtime SOC set to 100%");
        return;
    }

    if (StrEqual(argv[1], "reset")) {
        if (argc != 2) {
            PrintBatteryUsage();
            return;
        }
        BatteryMonitor_ResetEstimate();
        services::Shell_WriteLine("battery reset: estimating from voltage");
        return;
    }

    if (StrEqual(argv[1], "log")) {
        if ((argc >= 3) && StrEqual(argv[2], "on")) {
            if (argc == 4) {
                uint32_t period_ms = 0U;
                if ((!ParseUint32(argv[3], kBatteryLogMaxPeriodMs,
                                  &period_ms)) ||
                    (period_ms < kBatteryLogMinPeriodMs)) {
                    PrintBatteryUsage();
                    return;
                }
                g_batteryLogPeriodMs = period_ms;
            } else if (argc != 3) {
                PrintBatteryUsage();
                return;
            }

            const drivers::DriverStatus status = BatteryLogSetEnabled(true);
            services::Shell_WriteString("battery log: ");
            services::Shell_WriteString(DriverStatusText(status));
            services::Shell_WriteString(" period_ms=");
            services::Shell_WriteUInt32(g_batteryLogPeriodMs);
            services::Shell_WriteString("\r\n");
            return;
        }

        if ((argc == 3) && StrEqual(argv[2], "off")) {
            WriteStatusLine("battery log off: ",
                            BatteryLogSetEnabled(false));
            return;
        }

        if ((argc == 3) && StrEqual(argv[2], "status")) {
            services::Shell_WriteString("battery log enabled=");
            services::Shell_WriteUInt32(g_batteryLogEnabled ? 1U : 0U);
            services::Shell_WriteString(" registered=");
            services::Shell_WriteUInt32(
                g_batteryLogTaskRegistered ? 1U : 0U);
            services::Shell_WriteString(" period_ms=");
            services::Shell_WriteUInt32(g_batteryLogPeriodMs);
            services::Shell_WriteString("\r\n");
            return;
        }
    }

    PrintBatteryUsage();
}
#endif


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
void PrintImuBiasStatus(void)
{
    const ImuBiasEstimatorStatus *bias = App_ImuGetBiasStatus();
    const ConfigStoreParams *params = ConfigStore_Get();
    const AppImuData *imu = App_ImuGetData();
    if ((bias == 0) || (params == 0)) {
        services::Shell_WriteLine("imu bias: not initialized");
        return;
    }

    const int32_t fixed = params->imu_gyro_bias_z_mdps;
    services::Shell_WriteString("imu bias state=");
    services::Shell_WriteString(ImuBiasEstimator_StateText(bias->state));
    services::Shell_WriteString(" reject=");
    services::Shell_WriteString(
        ImuBiasEstimator_RejectReasonText(bias->reject_reason));
    services::Shell_WriteString(" auto=");
    services::Shell_WriteUInt32(bias->auto_enabled ? 1U : 0U);
    services::Shell_WriteString(" manual=");
    services::Shell_WriteUInt32(bias->manual_requested ? 1U : 0U);
    services::Shell_WriteString(" valid=");
    services::Shell_WriteUInt32(bias->estimate_valid ? 1U : 0U);
    services::Shell_WriteString(" freeze=");
    services::Shell_WriteUInt32(bias->freeze_yaw ? 1U : 0U);
    services::Shell_WriteString("\r\n");

    services::Shell_WriteString("imu bias fixed_z_mdps=");
    WriteInt32(fixed);
    services::Shell_WriteString(" runtime_z_mdps=");
    WriteInt32(bias->runtime_bias_z_mdps);
    services::Shell_WriteString(" total_z_mdps=");
    WriteInt32(fixed + bias->runtime_bias_z_mdps);
    services::Shell_WriteString(" corrected_z_mdps=");
    WriteInt32((imu != 0) ? imu->gyro_mdps[2] : 0);
    services::Shell_WriteString(" temp_centi_c=");
    WriteInt32((imu != 0) ? imu->temp_centi_c : 0);
    services::Shell_WriteString("\r\n");

    services::Shell_WriteString("imu bias samples=");
    services::Shell_WriteUInt32(bias->sample_count);
    services::Shell_WriteString("/");
    services::Shell_WriteUInt32(IMU_BIAS_REQUIRED_SAMPLES);
    services::Shell_WriteString(" elapsed_ms=");
    services::Shell_WriteUInt32(bias->collection_elapsed_ms);
    services::Shell_WriteString(" mean_mdps=");
    WriteInt32(bias->last_mean_z_mdps);
    services::Shell_WriteString(" stddev_mdps=");
    services::Shell_WriteUInt32(bias->last_stddev_z_mdps);
    services::Shell_WriteString(" adjust_mdps=");
    WriteInt32(bias->last_adjustment_z_mdps);
    services::Shell_WriteString(" accepted=");
    services::Shell_WriteUInt32(bias->accepted_windows);
    services::Shell_WriteString(" rejected=");
    services::Shell_WriteUInt32(bias->rejected_windows);
    services::Shell_WriteString("\r\n");
}

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

    if (StrEqual(argv[1], "bias")) {
        if ((argc == 3) && StrEqual(argv[2], "status")) {
            PrintImuBiasStatus();
            return;
        }
        if ((argc == 3) && StrEqual(argv[2], "calibrate")) {
            App_ImuBiasRequestCalibration();
            services::Shell_WriteLine(
                "imu bias calibrate: waiting for 2 s stationary window");
            return;
        }
        if ((argc == 4) && StrEqual(argv[2], "auto")) {
            if (StrEqual(argv[3], "on")) {
                App_ImuBiasSetAutoEnabled(true);
            } else if (StrEqual(argv[3], "off")) {
                App_ImuBiasSetAutoEnabled(false);
            } else {
                PrintImuUsage();
                return;
            }
            services::Shell_WriteString("imu bias auto=");
            services::Shell_WriteUInt32(
                App_ImuGetBiasStatus()->auto_enabled ? 1U : 0U);
            services::Shell_WriteString("\r\n");
            return;
        }
        if ((argc == 3) && StrEqual(argv[2], "save")) {
            WriteStatusLine("imu bias save: ", App_ImuBiasSave());
            return;
        }
        if ((argc == 3) && StrEqual(argv[2], "reset")) {
            App_ImuBiasResetRuntime();
            services::Shell_WriteLine("imu bias reset: runtime only");
            return;
        }
        PrintImuUsage();
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
        if ((argc == 3) && StrEqual(argv[2], "status")) {
            services::Shell_WriteString("imu lis ready=");
            services::Shell_WriteUInt32(board::Board_Lis3mdlIsReady() ? 1U : 0U);
            services::Shell_WriteString(" init=");
            services::Shell_WriteString(
                DriverStatusText(board::Board_Lis3mdlGetInitStatus()));
            services::Shell_WriteString(" scale=");
            services::Shell_WriteUInt32(board::Board_Lis3mdlGetFullScale());
            services::Shell_WriteString(" odr=");
            services::Shell_WriteUInt32(board::Board_Lis3mdlGetOutputDataRate());
            services::Shell_WriteString(" mode=");
            services::Shell_WriteUInt32(board::Board_Lis3mdlGetOperatingMode());
            services::Shell_WriteString("\r\n");
            return;
        }
        if ((argc == 3) && StrEqual(argv[2], "init")) {
            WriteStatusLine("imu lis init: ", board::Board_Lis3mdlInit());
            return;
        }
        if ((argc == 3) && StrEqual(argv[2], "sample")) {
            drivers::Lis3mdlRawData sample = {};
            const drivers::DriverStatus status =
                board::Board_Lis3mdlReadRaw(&sample);
            if (status != drivers::DRIVER_OK) {
                WriteStatusLine("imu lis sample: ", status);
                return;
            }
            const uint8_t scale = board::Board_Lis3mdlGetFullScale();
            services::Shell_WriteString("imu lis raw=");
            WriteInt32(sample.x);
            services::Shell_WriteString(",");
            WriteInt32(sample.y);
            services::Shell_WriteString(",");
            WriteInt32(sample.z);
            services::Shell_WriteString(" mG=");
            WriteInt32(drivers::Lis3mdl_RawToMilliGauss(sample.x, scale));
            services::Shell_WriteString(",");
            WriteInt32(drivers::Lis3mdl_RawToMilliGauss(sample.y, scale));
            services::Shell_WriteString(",");
            WriteInt32(drivers::Lis3mdl_RawToMilliGauss(sample.z, scale));
            services::Shell_WriteString("\r\n");
            return;
        }
        if ((argc == 4) && StrEqual(argv[2], "scale") &&
            ParseUint32(argv[3], 3U, &reg)) {
            WriteStatusLine("imu lis scale: ",
                board::Board_Lis3mdlSetFullScale((uint8_t) reg));
            return;
        }
        if ((argc == 4) && StrEqual(argv[2], "odr") &&
            ParseUint32(argv[3], 7U, &reg)) {
            WriteStatusLine("imu lis odr: ",
                board::Board_Lis3mdlSetOutputDataRate((uint8_t) reg));
            return;
        }
        if ((argc == 4) && StrEqual(argv[2], "mode") &&
            ParseUint32(argv[3], 2U, &reg)) {
            WriteStatusLine("imu lis mode: ",
                board::Board_Lis3mdlSetOperatingMode((uint8_t) reg));
            return;
        }
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
        services::Shell_WriteString(" cC yaw=");
        WriteFixedMilli(imu->yaw_mdeg);
        services::Shell_WriteString(" deg\r\n");
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

drivers::DriverStatus BuildLoraProtocolTextPayload(
    int argc,
    const char * const argv[],
    uint8_t payload[LORA_PROTOCOL_MAX_PAYLOAD_LENGTH],
    uint16_t *length)
{
    if ((payload == 0) || (length == 0)) {
        return drivers::DRIVER_ERROR_INVALID_ARG;
    }

    uint16_t count = 0U;
    for (int arg = 5; arg < argc; arg++) {
        if (arg > 5) {
            if (count >= LORA_PROTOCOL_MAX_PAYLOAD_LENGTH) {
                return drivers::DRIVER_ERROR_INVALID_ARG;
            }
            payload[count++] = static_cast<uint8_t>(' ');
        }

        const char *cursor = argv[arg];
        while ((cursor != 0) && (*cursor != '\0')) {
            if (count >= LORA_PROTOCOL_MAX_PAYLOAD_LENGTH) {
                return drivers::DRIVER_ERROR_INVALID_ARG;
            }
            payload[count++] = static_cast<uint8_t>(*cursor);
            cursor++;
        }
    }

    *length = count;
    return drivers::DRIVER_OK;
}

void PrintLoraProtocolStatus(void)
{
    const AppLoraProtocolState *state = App_LoraProtocolGetState();
    const LoraProtocolContext &protocol = state->protocol;
    const LoraProtocolStatistics &stats = protocol.statistics;

    services::Shell_WriteString("lora proto initialized=");
    services::Shell_WriteUInt32(state->initialized ? 1U : 0U);
    services::Shell_WriteString(" enabled=");
    services::Shell_WriteUInt32(state->enabled ? 1U : 0U);
    services::Shell_WriteString(" board_ready=");
    services::Shell_WriteUInt32(board::Board_LoraIsReady() ? 1U : 0U);
    services::Shell_WriteString(" queued=");
    services::Shell_WriteUInt32(
        LoraProtocol_GetQueuedFrameCount(&protocol));
    services::Shell_WriteString(" process_errors=");
    services::Shell_WriteUInt32(state->process_error_count);
    services::Shell_WriteString(" last_process=");
    services::Shell_WriteString(DriverStatusText(state->last_process_status));
    services::Shell_WriteString("\r\n");

    services::Shell_WriteString("lora proto next_seq=");
    services::Shell_WriteUInt32(protocol.next_tx_sequence);
    services::Shell_WriteString(" awaiting_ack=");
    services::Shell_WriteUInt32(protocol.awaiting_ack ? 1U : 0U);
    services::Shell_WriteString(" pending_seq=");
    services::Shell_WriteUInt32(protocol.pending_sequence);
    services::Shell_WriteString(" retries=");
    services::Shell_WriteUInt32(protocol.pending_retry_count);
    services::Shell_WriteString("/");
    services::Shell_WriteUInt32(protocol.config.max_retries);
    services::Shell_WriteString(" last_tx=");
    services::Shell_WriteString(DriverStatusText(protocol.last_tx_status));
    services::Shell_WriteString("\r\n");

    services::Shell_WriteString("lora proto rx valid=");
    services::Shell_WriteUInt32(stats.rx_valid_frames);
    services::Shell_WriteString(" delivered=");
    services::Shell_WriteUInt32(stats.rx_delivered_frames);
    services::Shell_WriteString(" crc_err=");
    services::Shell_WriteUInt32(stats.rx_crc_errors);
    services::Shell_WriteString(" format_err=");
    services::Shell_WriteUInt32(stats.rx_format_errors);
    services::Shell_WriteString(" length_err=");
    services::Shell_WriteUInt32(stats.rx_length_errors);
    services::Shell_WriteString(" dup=");
    services::Shell_WriteUInt32(stats.rx_duplicates);
    services::Shell_WriteString(" drop=");
    services::Shell_WriteUInt32(stats.rx_queue_drops);
    services::Shell_WriteString("\r\n");

    services::Shell_WriteString("lora proto tx frames=");
    services::Shell_WriteUInt32(stats.tx_frames);
    services::Shell_WriteString(" ack=");
    services::Shell_WriteUInt32(stats.tx_ack_frames);
    services::Shell_WriteString(" retries=");
    services::Shell_WriteUInt32(stats.tx_retries);
    services::Shell_WriteString(" errors=");
    services::Shell_WriteUInt32(stats.tx_errors);
    services::Shell_WriteString(" exhausted=");
    services::Shell_WriteUInt32(stats.tx_retry_exhausted);
    services::Shell_WriteString(" unexpected_ack=");
    services::Shell_WriteUInt32(stats.rx_unexpected_acks);
    services::Shell_WriteString("\r\n");
}

void PrintLoraProtocolFrame(const LoraProtocolFrame &frame)
{
    services::Shell_WriteString("lora proto rx type=");
    WriteHex8(frame.type);
    services::Shell_WriteString(" flags=");
    WriteHex8(frame.flags);
    services::Shell_WriteString(" seq=");
    services::Shell_WriteUInt32(frame.sequence);
    services::Shell_WriteString(" len=");
    services::Shell_WriteUInt32(frame.length);
    services::Shell_WriteString(" hex:");
    for (uint16_t i = 0U; i < frame.length; i++) {
        services::Shell_WriteString(" ");
        WriteHex8(frame.payload[i]);
    }
    services::Shell_WriteString(" ascii:");
    for (uint16_t i = 0U; i < frame.length; i++) {
        services::Shell_WriteChar(IsPrintableAscii(frame.payload[i]) ?
                                  static_cast<char>(frame.payload[i]) : '.');
    }
    services::Shell_WriteString("\r\n");
}

void LoraProtocolCommand(int argc, const char * const argv[])
{
    if (argc < 3) {
        PrintLoraUsage();
        return;
    }
    if (StrEqual(argv[2], "status") && (argc == 3)) {
        PrintLoraProtocolStatus();
        return;
    }
    if (StrEqual(argv[2], "on") && (argc == 3)) {
        WriteStatusLine("lora proto on: ",
                        App_LoraProtocolSetEnabled(true));
        return;
    }
    if (StrEqual(argv[2], "off") && (argc == 3)) {
        WriteStatusLine("lora proto off: ",
                        App_LoraProtocolSetEnabled(false));
        return;
    }
    if (StrEqual(argv[2], "reset") && (argc == 3)) {
        WriteStatusLine("lora proto reset: ", App_LoraProtocolReset());
        return;
    }
    if (StrEqual(argv[2], "recv") && (argc == 3)) {
        LoraProtocolFrame frame = {};
        if (!App_LoraProtocolReadFrame(&frame)) {
            services::Shell_WriteLine("lora proto recv: empty");
            return;
        }
        PrintLoraProtocolFrame(frame);
        return;
    }
    if (StrEqual(argv[2], "send") && (argc >= 5)) {
        uint32_t type = 0U;
        if ((!ParseUint32(argv[3], 0xFFU, &type)) || (type == 0U) ||
            (type == LORA_PROTOCOL_TYPE_ACK)) {
            PrintLoraUsage();
            return;
        }
        const bool acknowledgment_required = StrEqual(argv[4], "ack");
        if ((!acknowledgment_required) && (!StrEqual(argv[4], "noack"))) {
            PrintLoraUsage();
            return;
        }

        uint8_t payload[LORA_PROTOCOL_MAX_PAYLOAD_LENGTH];
        uint16_t length = 0U;
        drivers::DriverStatus status = BuildLoraProtocolTextPayload(
            argc,
            argv,
            payload,
            &length);
        uint8_t sequence = 0U;
        if (status == drivers::DRIVER_OK) {
            status = App_LoraProtocolSend(static_cast<uint8_t>(type),
                                          payload,
                                          length,
                                          acknowledgment_required,
                                          &sequence);
        }
        if (status != drivers::DRIVER_OK) {
            WriteStatusLine("lora proto send: ", status);
            return;
        }
        services::Shell_WriteString("lora proto tx seq=");
        services::Shell_WriteUInt32(sequence);
        services::Shell_WriteString(" len=");
        services::Shell_WriteUInt32(length);
        services::Shell_WriteString(" ack=");
        services::Shell_WriteUInt32(acknowledgment_required ? 1U : 0U);
        services::Shell_WriteString("\r\n");
        return;
    }

    PrintLoraUsage();
}

void LoraCommand(int argc, const char * const argv[])
{
    uint32_t value = 0U;

    if (argc < 2) {
        PrintLoraUsage();
        return;
    }

    if (StrEqual(argv[1], "proto")) {
        LoraProtocolCommand(argc, argv);
        return;
    }

    if (!board::Board_LoraIsReady()) {
        services::Shell_WriteLine("lora: not ready");
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

        const AppLoraProtocolState *protocol_state =
            App_LoraProtocolGetState();
        const drivers::DriverStatus status = protocol_state->enabled ?
            App_LoraProtocolReset() : board::Board_LoraClearRxBuffer();
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
        if (App_LoraProtocolGetState()->enabled) {
            services::Shell_WriteLine(
                "lora read: protocol enabled; use 'lora proto recv'");
            return;
        }

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

    if (StrEqual(argv[1], "stat")) {
        /* One-line summary from the cached state (refreshed by the chassis_fb
         * periodic task). Does not force an I2C round-trip, so it is useful to
         * confirm the feedback task is keeping actual_rpm fresh. */
        if (argc != 2) {
            PrintChassisUsage();
            return;
        }

        const ChassisState *state = Chassis_GetState();
        if (state == 0) {
            services::Shell_WriteLine("chassis stat: no state");
            return;
        }

        services::Shell_WriteString("chassis stat: L tgt=");
        WriteInt32(state->left.target_rpm);
        services::Shell_WriteString(" act=");
        WriteInt32(state->left.actual_rpm);
        services::Shell_WriteString(" R tgt=");
        WriteInt32(state->right.target_rpm);
        services::Shell_WriteString(" act=");
        WriteInt32(state->right.actual_rpm);
        services::Shell_WriteString(" last=");
        services::Shell_WriteString(DriverStatusText(state->last_status));
        services::Shell_WriteString("\r\n");
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

void PrintHeadingUsage(void)
{
    services::Shell_WriteLine("usage:");
    services::Shell_WriteLine("  heading status");
    services::Shell_WriteLine("  heading hold <base_rpm>");
    services::Shell_WriteLine("  heading lock");
    services::Shell_WriteLine("  heading lockcfg");
    services::Shell_WriteLine(
        "  heading lockcfg set <key> <value>");
    services::Shell_WriteLine("  heading lockcfg save");
    services::Shell_WriteLine("  heading turn <deg -180..180>");
    services::Shell_WriteLine("  heading turncfg");
    services::Shell_WriteLine(
        "  heading turncfg set <brake_ms> <margin_mdeg> <settle_mdps> <settle_rpm>");
    services::Shell_WriteLine("  heading turncfg save");
    services::Shell_WriteLine(
        "  heading distance <mm -10000..10000> <max_rpm> [timeout_ms]");
    services::Shell_WriteLine("  heading profile");
    services::Shell_WriteLine("  heading profile mode <legacy|trapezoid>");
    services::Shell_WriteLine(
        "  heading profile <accel|decel|creep|latency|margin|settle|tolerance> <value>");
    services::Shell_WriteLine("  heading profile save");
    services::Shell_WriteLine("  heading stop");
}

const char *HeadingModeText(app::HeadingMode mode)
{
    switch (mode) {
    case app::HEADING_HOLD:
        return "hold";
    case app::HEADING_TURN:
        return "turn";
    case app::HEADING_DISTANCE:
        return "distance";
    case app::HEADING_LOCK:
        return "lock";
    case app::HEADING_ARC_TURN:
        return "arc_turn";
    default:
        return "idle";
    }
}

const char *DistanceSpeedModeText(uint8_t mode)
{
    return (mode == DISTANCE_SPEED_MODE_TRAPEZOID)
        ? "trapezoid"
        : "legacy";
}

const char *DistancePhaseText(app::DistanceProfilePhase phase)
{
    switch (phase) {
    case app::DISTANCE_PHASE_LEGACY: return "legacy";
    case app::DISTANCE_PHASE_ACCEL: return "accel";
    case app::DISTANCE_PHASE_CRUISE: return "cruise";
    case app::DISTANCE_PHASE_BRAKE: return "brake";
    case app::DISTANCE_PHASE_CREEP: return "creep";
    case app::DISTANCE_PHASE_SETTLE: return "settle";
    default: return "idle";
    }
}

const char *HeadingTurnPhaseText(app::HeadingTurnPhase phase)
{
    switch (phase) {
    case app::HEADING_TURN_PHASE_DRIVE: return "drive";
    case app::HEADING_TURN_PHASE_BRAKE: return "brake";
    case app::HEADING_TURN_PHASE_SETTLE: return "settle";
    default: return "idle";
    }
}

const char *HeadingLockPhaseText(app::HeadingLockPhase phase)
{
    switch (phase) {
    case app::HEADING_LOCK_PHASE_LOCKED: return "locked";
    case app::HEADING_LOCK_PHASE_RECOVER: return "recover";
    case app::HEADING_LOCK_PHASE_SETTLE: return "settle";
    case app::HEADING_LOCK_PHASE_FAILED: return "failed";
    default: return "idle";
    }
}

void PrintHeadingTurnConfig(void)
{
    const ConfigStoreParams *params = ConfigStore_Get();
    services::Shell_WriteString("heading turncfg brake_ms=");
    services::Shell_WriteUInt32(params->heading_turn_brake_ms);
    services::Shell_WriteString(" margin_mdeg=");
    services::Shell_WriteUInt32(params->heading_turn_brake_margin_mdeg);
    services::Shell_WriteString(" settle_mdps=");
    services::Shell_WriteUInt32(params->heading_turn_settle_rate_mdps);
    services::Shell_WriteString(" settle_rpm=");
    services::Shell_WriteUInt32(params->heading_turn_settle_rpm);
    services::Shell_WriteString("\r\n");
}

void PrintHeadingLockConfig(void)
{
    const ConfigStoreParams *params = ConfigStore_Get();
    services::Shell_WriteString("heading lockcfg kp=");
    WriteInt32(params->heading_lock_kp);
    services::Shell_WriteString(" kd=");
    WriteInt32(params->heading_lock_kd);
    services::Shell_WriteString(" wake_mdeg=");
    services::Shell_WriteUInt32(params->heading_lock_wake_mdeg);
    services::Shell_WriteString(" settle_mdeg=");
    services::Shell_WriteUInt32(params->heading_lock_settle_mdeg);
    services::Shell_WriteString(" min_rpm=");
    services::Shell_WriteUInt32(params->heading_lock_min_rpm);
    services::Shell_WriteString(" max_rpm=");
    services::Shell_WriteUInt32(params->heading_lock_max_rpm);
    services::Shell_WriteString(" rate_mdps=");
    services::Shell_WriteUInt32(params->heading_lock_settle_rate_mdps);
    services::Shell_WriteString(" wheel_rpm=");
    services::Shell_WriteUInt32(params->heading_lock_settle_rpm);
    services::Shell_WriteString(" settle_ms=");
    services::Shell_WriteUInt32(params->heading_lock_settle_ms);
    services::Shell_WriteString(" timeout_ms=");
    services::Shell_WriteUInt32(params->heading_lock_timeout_ms);
    services::Shell_WriteString("\r\n");
}

void PrintDistanceProfile(void)
{
    const ConfigStoreParams *params = ConfigStore_Get();
    services::Shell_WriteString("heading profile mode=");
    services::Shell_WriteString(DistanceSpeedModeText(
        params->distance_speed_mode));
    services::Shell_WriteString(" accel_rpm_s=");
    services::Shell_WriteUInt32(params->distance_accel_rpm_s);
    services::Shell_WriteString(" decel_rpm_s=");
    services::Shell_WriteUInt32(params->distance_decel_rpm_s);
    services::Shell_WriteString(" creep_rpm=");
    services::Shell_WriteUInt32(params->distance_creep_rpm);
    services::Shell_WriteString(" latency_ms=");
    services::Shell_WriteUInt32(params->distance_stop_latency_ms);
    services::Shell_WriteString(" margin_mm=");
    services::Shell_WriteUInt32(params->distance_brake_margin_mm);
    services::Shell_WriteString(" settle_rpm=");
    services::Shell_WriteUInt32(params->distance_settle_rpm);
    services::Shell_WriteString(" tolerance_mm=");
    services::Shell_WriteUInt32(params->distance_tolerance_mm);
    services::Shell_WriteString("\r\n");
}

void HeadingCommand(int argc, const char * const argv[])
{
#if FEATURE_ENABLE_IMU && FEATURE_ENABLE_MOTOR_DRIVER
    if (argc < 2) {
        PrintHeadingUsage();
        return;
    }

    if (StrEqual(argv[1], "status")) {
        const app::HeadingState *st = app::Heading_GetState();
        services::Shell_WriteString("heading mode=");
        services::Shell_WriteString(HeadingModeText(st->mode));
        services::Shell_WriteString(" target=");
        WriteInt32(st->target_yaw_mdeg / 1000);
        services::Shell_WriteString("deg error=");
        WriteInt32(st->error_mdeg / 1000);
        services::Shell_WriteString("deg corr=");
        WriteInt32(st->correction_rpm);
        services::Shell_WriteString("rpm at_target=");
        services::Shell_WriteUInt32(st->at_target ? 1U : 0U);
        services::Shell_WriteString(" arc_lr=");
        WriteInt32(st->arc_left_command_rpm);
        services::Shell_WriteString("/");
        WriteInt32(st->arc_right_command_rpm);
        services::Shell_WriteString(" last=");
        services::Shell_WriteString(DriverStatusText(st->last_status));
        services::Shell_WriteString(" profile=");
        services::Shell_WriteString(DistancePhaseText(st->distance_phase));
        services::Shell_WriteString(" profile_rpm=");
        WriteInt32(st->profile_command_rpm);
        services::Shell_WriteString(" brake_mm=");
        WriteInt32(st->brake_distance_mm);
        services::Shell_WriteString(" turn_phase=");
        services::Shell_WriteString(HeadingTurnPhaseText(st->turn_phase));
        services::Shell_WriteString(" turn_rate_mdps=");
        WriteInt32(st->turn_rate_mdps);
        services::Shell_WriteString(" turn_brake_mdeg=");
        WriteInt32(st->turn_brake_angle_mdeg);
        services::Shell_WriteString(" lock_phase=");
        services::Shell_WriteString(HeadingLockPhaseText(st->lock_phase));
        services::Shell_WriteString(" lock_rate_mdps=");
        WriteInt32(st->lock_rate_mdps);
        services::Shell_WriteString(" lock_elapsed_ms=");
        services::Shell_WriteUInt32(st->lock_recover_elapsed_ms);
        services::Shell_WriteString(" lock_result=");
        services::Shell_WriteString(DriverStatusText(st->lock_result));
        services::Shell_WriteString(" bias_valid=");
        services::Shell_WriteUInt32(
            App_ImuGetBiasStatus()->estimate_valid ? 1U : 0U);
        services::Shell_WriteString("\r\n");
        if ((st->mode == app::HEADING_DISTANCE) ||
            (st->target_distance_mm != 0)) {
            services::Shell_WriteString("distance target_mm=");
            WriteInt32(st->target_distance_mm);
            services::Shell_WriteString(" traveled_mm=");
            WriteInt32(st->traveled_distance_mm);
            services::Shell_WriteString(" remaining_mm=");
            WriteInt32(st->remaining_distance_mm);
            services::Shell_WriteString(" max_rpm=");
            WriteInt32(st->distance_max_rpm);
            services::Shell_WriteString(" target_counts=");
            WriteInt32(st->left_target_delta_counts);
            services::Shell_WriteString("/");
            WriteInt32(st->right_target_delta_counts);
            services::Shell_WriteString(" timeout_ms=");
            services::Shell_WriteUInt32(st->distance_timeout_ms);
            services::Shell_WriteString("\r\n");
        }
        return;
    }

    if (StrEqual(argv[1], "lockcfg")) {
        if (argc == 2) {
            PrintHeadingLockConfig();
            return;
        }
        if ((argc == 3) && StrEqual(argv[2], "save")) {
            WriteStatusLine("heading lockcfg save: ", ConfigStore_Save());
            return;
        }
        if ((argc == 5) && StrEqual(argv[2], "set")) {
            const char *param_name = 0;
            uint32_t maximum = 0U;
            if (StrEqual(argv[3], "kp")) {
                param_name = "heading_lock_kp";
                maximum = 100000U;
            } else if (StrEqual(argv[3], "kd")) {
                param_name = "heading_lock_kd";
                maximum = 100000U;
            } else if (StrEqual(argv[3], "wake")) {
                param_name = "heading_lock_wake_mdeg";
                maximum = 30000U;
            } else if (StrEqual(argv[3], "settle")) {
                param_name = "heading_lock_settle_mdeg";
                maximum = 29999U;
            } else if (StrEqual(argv[3], "minrpm")) {
                param_name = "heading_lock_min_rpm";
                maximum = 100U;
            } else if (StrEqual(argv[3], "maxrpm")) {
                param_name = "heading_lock_max_rpm";
                maximum = 200U;
            } else if (StrEqual(argv[3], "rate")) {
                param_name = "heading_lock_settle_rate_mdps";
                maximum = 60000U;
            } else if (StrEqual(argv[3], "wheelrpm")) {
                param_name = "heading_lock_settle_rpm";
                maximum = 100U;
            } else if (StrEqual(argv[3], "settlems")) {
                param_name = "heading_lock_settle_ms";
                maximum = 5000U;
            } else if (StrEqual(argv[3], "timeout")) {
                param_name = "heading_lock_timeout_ms";
                maximum = 10000U;
            }

            uint32_t value = 0U;
            if ((param_name == 0) ||
                (!ParseUint32(argv[4], maximum, &value))) {
                PrintHeadingUsage();
                return;
            }
            const drivers::DriverStatus status = ConfigStore_Set(
                param_name, static_cast<int32_t>(value));
            WriteStatusLine("heading lockcfg set: ", status);
            if (status == drivers::DRIVER_OK) {
                PrintHeadingLockConfig();
            }
            return;
        }
        PrintHeadingUsage();
        return;
    }

    if (StrEqual(argv[1], "turncfg")) {
        if (argc == 2) {
            PrintHeadingTurnConfig();
            return;
        }
        if ((argc == 3) && StrEqual(argv[2], "save")) {
            WriteStatusLine("heading turncfg save: ", ConfigStore_Save());
            return;
        }
        if ((argc == 7) && StrEqual(argv[2], "set")) {
            uint32_t brake_ms = 0U;
            uint32_t margin_mdeg = 0U;
            uint32_t settle_mdps = 0U;
            uint32_t settle_rpm = 0U;
            if ((!ParseUint32(argv[3], 500U, &brake_ms)) ||
                (!ParseUint32(argv[4], 30000U, &margin_mdeg)) ||
                (!ParseUint32(argv[5], 60000U, &settle_mdps)) ||
                (!ParseUint32(argv[6], 100U, &settle_rpm))) {
                PrintHeadingUsage();
                return;
            }
            drivers::DriverStatus status = ConfigStore_Set(
                "heading_turn_brake_ms", static_cast<int32_t>(brake_ms));
            if (status == drivers::DRIVER_OK) {
                status = ConfigStore_Set(
                    "heading_turn_brake_margin_mdeg",
                    static_cast<int32_t>(margin_mdeg));
            }
            if (status == drivers::DRIVER_OK) {
                status = ConfigStore_Set(
                    "heading_turn_settle_rate_mdps",
                    static_cast<int32_t>(settle_mdps));
            }
            if (status == drivers::DRIVER_OK) {
                status = ConfigStore_Set(
                    "heading_turn_settle_rpm",
                    static_cast<int32_t>(settle_rpm));
            }
            WriteStatusLine("heading turncfg set: ", status);
            if (status == drivers::DRIVER_OK) {
                PrintHeadingTurnConfig();
            }
            return;
        }
        PrintHeadingUsage();
        return;
    }

    if (StrEqual(argv[1], "profile")) {
        if (argc == 2) {
            PrintDistanceProfile();
            return;
        }
        if (app::Heading_GetState()->mode == app::HEADING_DISTANCE) {
            WriteStatusLine("heading profile: ", drivers::DRIVER_ERROR_BUSY);
            return;
        }
        if ((argc == 3) && StrEqual(argv[2], "save")) {
            WriteStatusLine("heading profile save: ", ConfigStore_Save());
            return;
        }
        if ((argc == 4) && StrEqual(argv[2], "mode")) {
            int32_t mode = -1;
            if (StrEqual(argv[3], "legacy")) {
                mode = DISTANCE_SPEED_MODE_LEGACY;
            } else if (StrEqual(argv[3], "trapezoid")) {
                mode = DISTANCE_SPEED_MODE_TRAPEZOID;
            }
            if (mode < 0) {
                PrintHeadingUsage();
                return;
            }
            const drivers::DriverStatus status =
                ConfigStore_Set("distance_speed_mode", mode);
            WriteStatusLine("heading profile mode: ", status);
            if (status == drivers::DRIVER_OK) {
                PrintDistanceProfile();
            }
            return;
        }
        if (argc == 4) {
            const char *param_name = 0;
            uint32_t maximum = 0U;
            uint32_t minimum = 0U;
            if (StrEqual(argv[2], "accel")) {
                param_name = "distance_accel_rpm_s";
                minimum = 1U;
                maximum = 5000U;
            } else if (StrEqual(argv[2], "decel")) {
                param_name = "distance_decel_rpm_s";
                minimum = 1U;
                maximum = 5000U;
            } else if (StrEqual(argv[2], "creep")) {
                param_name = "distance_creep_rpm";
                minimum = 1U;
                maximum = 500U;
            } else if (StrEqual(argv[2], "latency")) {
                param_name = "distance_stop_latency_ms";
                maximum = 2000U;
            } else if (StrEqual(argv[2], "margin")) {
                param_name = "distance_brake_margin_mm";
                maximum = 1000U;
            } else if (StrEqual(argv[2], "settle")) {
                param_name = "distance_settle_rpm";
                maximum = 100U;
            } else if (StrEqual(argv[2], "tolerance")) {
                param_name = "distance_tolerance_mm";
                minimum = 1U;
                maximum = 100U;
            }

            uint32_t value = 0U;
            if ((param_name == 0) ||
                (!ParseUint32(argv[3], maximum, &value)) ||
                (value < minimum)) {
                PrintHeadingUsage();
                return;
            }
            const drivers::DriverStatus status = ConfigStore_Set(
                param_name, static_cast<int32_t>(value));
            WriteStatusLine("heading profile set: ", status);
            if (status == drivers::DRIVER_OK) {
                PrintDistanceProfile();
            }
            return;
        }
        PrintHeadingUsage();
        return;
    }

    if (StrEqual(argv[1], "stop")) {
        WriteStatusLine("heading stop: ", app::Heading_Stop());
        return;
    }

    if (StrEqual(argv[1], "lock")) {
        if (argc != 2) {
            PrintHeadingUsage();
            return;
        }
        if (app::ActionRunner_GetState()->running) {
            WriteStatusLine("heading lock: ", drivers::DRIVER_ERROR_BUSY);
            return;
        }
#if FEATURE_ENABLE_GRAYSCALE
        if (app::LF_GetState()->mode != app::LF_IDLE) {
            WriteStatusLine("heading lock: ", drivers::DRIVER_ERROR_BUSY);
            return;
        }
#endif
        WriteStatusLine("heading lock: ", app::Heading_LockStart());
        return;
    }

    if (StrEqual(argv[1], "hold")) {
        int32_t base_rpm = 0;
        const app::ChassisState *cs = app::Chassis_GetState();
        const int32_t max_rpm =
            static_cast<int32_t>(cs->config.max_wheel_rpm);
        if ((argc != 3) ||
            (!ParseInt32(argv[2], -max_rpm, max_rpm, &base_rpm))) {
            PrintHeadingUsage();
            return;
        }
        WriteStatusLine("heading hold: ", app::Heading_HoldStart(base_rpm));
        return;
    }

    if (StrEqual(argv[1], "turn")) {
        int32_t deg = 0;
        if ((argc != 3) || (!ParseInt32(argv[2], -180, 180, &deg))) {
            PrintHeadingUsage();
            return;
        }
        WriteStatusLine("heading turn: ", app::Heading_TurnStart(deg));
        return;
    }

    if (StrEqual(argv[1], "distance")) {
        int32_t distance_mm = 0;
        int32_t max_rpm = 0;
        uint32_t timeout_ms = 0U;
        const app::ChassisState *cs = app::Chassis_GetState();
        const int32_t wheel_max_rpm =
            static_cast<int32_t>(cs->config.max_wheel_rpm);
        if (((argc != 4) && (argc != 5)) ||
            (!ParseInt32(argv[2], -10000, 10000, &distance_mm)) ||
            (distance_mm == 0) ||
            (!ParseInt32(argv[3], 1, wheel_max_rpm, &max_rpm)) ||
            ((argc == 5) &&
             ((!ParseUint32(argv[4], 60000U, &timeout_ms)) ||
              (timeout_ms < 500U)))) {
            PrintHeadingUsage();
            return;
        }
        WriteStatusLine("heading distance: ",
                        app::Heading_DistanceStart(distance_mm,
                                                   max_rpm,
                                                   timeout_ms));
        return;
    }

    PrintHeadingUsage();
#else
    (void) argc;
    (void) argv;
    services::Shell_WriteLine("heading: disabled");
#endif
}

void PrintRunUsage(void)
{
    services::Shell_WriteLine("usage:");
    services::Shell_WriteLine("  run add <op> <p1> <p2> <until> <onsuccess> <ontimeout>");
    services::Shell_WriteLine(
        "    op: drive|drive_mm|turn|follow|wait|stop|branch|loop|road_nav|end");
    services::Shell_WriteLine(
        "        dm_position|dm_speed|dm_disable");
    services::Shell_WriteLine(
        "        led_on|led_off|led_toggle|buzzer_on|buzzer_off|buzzer_toggle");
    services::Shell_WriteLine(
        "  run add condition <source> <cmp> <value> <instant|wait>");
    services::Shell_WriteLine(
        "    <timeout_ms> <stable_ms> <ontrue> <onfalse>");
    services::Shell_WriteLine(
        "  run add drive_if|follow_if <rpm> <source> <cmp> <value>");
    services::Shell_WriteLine(
        "    <timeout_ms> <stable_ms> <onsuccess> <onfailure>");
    services::Shell_WriteLine(
        "  run add loop <count> 0 immediate <body_index> <done_index>");
    services::Shell_WriteLine(
        "  run add road_nav <route> <rpm> <timeout_ms> <onsuccess> <onfailure>");
    services::Shell_WriteLine(
        "    route: left|straight|right|uturn_left_arc|uturn_right_arc");
    services::Shell_WriteLine(
        "           uturn_left_pivot|uturn_right_pivot");
    services::Shell_WriteLine(
        "  run add dm_position absolute|relative <target_mrad> <max_mrad_s> <timeout_ms> <onsuccess> <onfailure>");
    services::Shell_WriteLine(
        "  run add dm_speed <velocity_mrad_s> <duration_ms> <onsuccess> <onfailure>");
    services::Shell_WriteLine(
        "  run add dm_disable <onsuccess> <onfailure>");
    services::Shell_WriteLine(
        "    until: timeout|heading_reached|distance_reached|line_detected|line_lost|button|immediate");
    services::Shell_WriteLine(
        "    drive_mm: p1=signed mm, p2=max rpm, until=distance_reached");
    services::Shell_WriteLine(
        "    LED: p1=0(both)|2|3; buzzer: p1=0; p2=0 or auto-off 50..30000");
    services::Shell_WriteLine(
        "    loop: p1=count 1..1000, p2=0, immediate, both targets explicit");
    services::Shell_WriteLine("    output actions require until=immediate; off requires p2=0");
    services::Shell_WriteLine("    onsuccess/ontimeout: index 0..63, or 'next'/'abort'");
    services::Shell_WriteLine("  run clear|validate|start|cancel|status|dump");
}

bool ParseActionOp(const char *t, app::ActionOp *op)
{
    if (StrEqual(t, "drive")) { *op = app::ACT_OP_DRIVE; return true; }
    if (StrEqual(t, "drive_mm")) { *op = app::ACT_OP_DRIVE_MM; return true; }
    if (StrEqual(t, "turn")) { *op = app::ACT_OP_TURN; return true; }
    if (StrEqual(t, "follow")) { *op = app::ACT_OP_FOLLOW; return true; }
    if (StrEqual(t, "wait")) { *op = app::ACT_OP_WAIT; return true; }
    if (StrEqual(t, "stop")) { *op = app::ACT_OP_STOP; return true; }
    if (StrEqual(t, "branch")) { *op = app::ACT_OP_BRANCH; return true; }
    if (StrEqual(t, "end")) { *op = app::ACT_OP_END; return true; }
    if (StrEqual(t, "led_on")) { *op = app::ACT_OP_LED_ON; return true; }
    if (StrEqual(t, "led_off")) { *op = app::ACT_OP_LED_OFF; return true; }
    if (StrEqual(t, "led_toggle")) { *op = app::ACT_OP_LED_TOGGLE; return true; }
    if (StrEqual(t, "buzzer_on")) { *op = app::ACT_OP_BUZZER_ON; return true; }
    if (StrEqual(t, "buzzer_off")) { *op = app::ACT_OP_BUZZER_OFF; return true; }
    if (StrEqual(t, "buzzer_toggle")) { *op = app::ACT_OP_BUZZER_TOGGLE; return true; }
    if (StrEqual(t, "condition")) { *op = app::ACT_OP_CONDITION; return true; }
    if (StrEqual(t, "drive_if")) { *op = app::ACT_OP_DRIVE_IF; return true; }
    if (StrEqual(t, "follow_if")) { *op = app::ACT_OP_FOLLOW_IF; return true; }
    if (StrEqual(t, "loop")) { *op = app::ACT_OP_LOOP; return true; }
    if (StrEqual(t, "road_nav")) { *op = app::ACT_OP_ROAD_NAV; return true; }
    if (StrEqual(t, "dm_position")) { *op = app::ACT_OP_DM_POSITION; return true; }
    if (StrEqual(t, "dm_speed")) { *op = app::ACT_OP_DM_SPEED; return true; }
    if (StrEqual(t, "dm_disable")) { *op = app::ACT_OP_DM_DISABLE; return true; }
    return false;
}

bool ParseRoadRoute(const char *text, app::RoadRoute *route)
{
    if ((text == 0) || (route == 0)) {
        return false;
    }
    for (uint8_t raw = 0U;
         raw < static_cast<uint8_t>(app::ROAD_ROUTE_COUNT);
         raw++) {
        const app::RoadRoute candidate =
            static_cast<app::RoadRoute>(raw);
        if (StrEqual(text,
                     app::RoadEventController_RouteText(candidate))) {
            *route = candidate;
            return true;
        }
    }
    return false;
}

bool ParseActionConditionSource(const char *text,
                                app::ActionConditionSource *source)
{
    if ((text == 0) || (source == 0)) {
        return false;
    }
    for (uint8_t raw = 0U;
         raw < static_cast<uint8_t>(app::ACT_SOURCE_COUNT);
         raw++) {
        const app::ActionConditionSource candidate =
            static_cast<app::ActionConditionSource>(raw);
        if (StrEqual(text, app::ActionCondition_SourceText(candidate))) {
            *source = candidate;
            return true;
        }
    }
    return false;
}

bool ParseActionCompare(const char *text, app::ActionCompareOp *compare)
{
    if ((text == 0) || (compare == 0)) {
        return false;
    }
    for (uint8_t raw = 0U;
         raw <= static_cast<uint8_t>(app::ACT_COMPARE_NOT_CONTAINS);
         raw++) {
        const app::ActionCompareOp candidate =
            static_cast<app::ActionCompareOp>(raw);
        if (StrEqual(text, app::ActionCondition_CompareText(candidate))) {
            *compare = candidate;
            return true;
        }
    }
    return false;
}

bool ParseActionCond(const char *t, app::ActionCond *c)
{
    if (StrEqual(t, "timeout")) { *c = app::ACT_COND_TIMEOUT; return true; }
    if (StrEqual(t, "heading_reached")) { *c = app::ACT_COND_HEADING_REACHED; return true; }
    if (StrEqual(t, "line_detected")) { *c = app::ACT_COND_LINE_DETECTED; return true; }
    if (StrEqual(t, "line_lost")) { *c = app::ACT_COND_LINE_LOST; return true; }
    if (StrEqual(t, "button")) { *c = app::ACT_COND_BUTTON; return true; }
    if (StrEqual(t, "immediate")) { *c = app::ACT_COND_IMMEDIATE; return true; }
    if (StrEqual(t, "distance_reached")) { *c = app::ACT_COND_DISTANCE_REACHED; return true; }
    if (StrEqual(t, "absolute")) { *c = app::ACT_COND_DM_ABSOLUTE; return true; }
    if (StrEqual(t, "relative")) { *c = app::ACT_COND_DM_RELATIVE; return true; }
    return false;
}

bool ParseTarget(const char *t, uint8_t *out)
{
    if (StrEqual(t, "next") || StrEqual(t, "abort")) {
        *out = app::ACT_NEXT;
        return true;
    }
    uint32_t v = 0U;
    if (!ParseUint32(t, 63U, &v)) {
        return false;
    }
    *out = static_cast<uint8_t>(v);
    return true;
}

const char *OpText(app::ActionOp op)
{
    switch (op) {
    case app::ACT_OP_DRIVE: return "drive";
    case app::ACT_OP_DRIVE_MM: return "drive_mm";
    case app::ACT_OP_TURN: return "turn";
    case app::ACT_OP_FOLLOW: return "follow";
    case app::ACT_OP_WAIT: return "wait";
    case app::ACT_OP_STOP: return "stop";
    case app::ACT_OP_BRANCH: return "branch";
    case app::ACT_OP_END: return "end";
    case app::ACT_OP_LED_ON: return "led_on";
    case app::ACT_OP_LED_OFF: return "led_off";
    case app::ACT_OP_LED_TOGGLE: return "led_toggle";
    case app::ACT_OP_BUZZER_ON: return "buzzer_on";
    case app::ACT_OP_BUZZER_OFF: return "buzzer_off";
    case app::ACT_OP_BUZZER_TOGGLE: return "buzzer_toggle";
    case app::ACT_OP_CONDITION: return "condition";
    case app::ACT_OP_DRIVE_IF: return "drive_if";
    case app::ACT_OP_FOLLOW_IF: return "follow_if";
    case app::ACT_OP_LOOP: return "loop";
    case app::ACT_OP_ROAD_NAV: return "road_nav";
    case app::ACT_OP_DM_POSITION: return "dm_position";
    case app::ACT_OP_DM_SPEED: return "dm_speed";
    case app::ACT_OP_DM_DISABLE: return "dm_disable";
    default: return "none";
    }
}

const char *CondText(app::ActionCond c)
{
    switch (c) {
    case app::ACT_COND_TIMEOUT: return "timeout";
    case app::ACT_COND_HEADING_REACHED: return "heading_reached";
    case app::ACT_COND_LINE_DETECTED: return "line_detected";
    case app::ACT_COND_LINE_LOST: return "line_lost";
    case app::ACT_COND_BUTTON: return "button";
    case app::ACT_COND_IMMEDIATE: return "immediate";
    case app::ACT_COND_DISTANCE_REACHED: return "distance_reached";
    case app::ACT_COND_DM_ABSOLUTE: return "absolute";
    case app::ACT_COND_DM_RELATIVE: return "relative";
    default: return "?";
    }
}

const char *ActionRunResultText(app::ActionRunResult result)
{
    switch (result) {
    case app::ACT_RUN_IDLE: return "idle";
    case app::ACT_RUN_RUNNING: return "running";
    case app::ACT_RUN_SUCCESS: return "success";
    case app::ACT_RUN_ABORTED: return "aborted";
    case app::ACT_RUN_CANCELLED: return "cancelled";
    case app::ACT_RUN_FAULT: return "fault";
    case app::ACT_RUN_TIMEOUT: return "timeout";
    case app::ACT_RUN_INVALID: return "invalid";
    default: return "unknown";
    }
}

const char *ActionFailureText(app::ActionFailureReason reason)
{
    switch (reason) {
    case app::ACT_FAIL_NONE: return "none";
    case app::ACT_FAIL_START: return "start_failed";
    case app::ACT_FAIL_INSTR_TIMEOUT: return "instruction_timeout";
    case app::ACT_FAIL_SEQUENCE_TIMEOUT: return "sequence_timeout";
    case app::ACT_FAIL_FAULT: return "fault";
    case app::ACT_FAIL_CANCELLED: return "cancelled";
    case app::ACT_FAIL_INVALID: return "invalid_table";
    case app::ACT_FAIL_CONDITION_FALSE: return "condition_false";
    case app::ACT_FAIL_CONDITION_TIMEOUT: return "condition_timeout";
    case app::ACT_FAIL_CONDITION_UNAVAILABLE:
        return "condition_unavailable";
    case app::ACT_FAIL_ROUTE_UNAVAILABLE:
        return "route_unavailable";
    case app::ACT_FAIL_ROUTE_REACQUIRE_FAILED:
        return "route_reacquire_failed";
    default: return "unknown";
    }
}

void WriteActionInstr(const app::Instr &instr, bool include_index,
                      uint8_t index)
{
    if (include_index) {
        services::Shell_WriteUInt32(index);
        services::Shell_WriteString(" ");
    }
    services::Shell_WriteString(OpText(instr.op));
    services::Shell_WriteString(" ");
    if (instr.op == app::ACT_OP_ROAD_NAV) {
        services::Shell_WriteString(
            app::RoadEventController_RouteText(
                static_cast<app::RoadRoute>(instr.condition_value)));
        services::Shell_WriteString(" ");
        WriteInt32(instr.param1);
        services::Shell_WriteString(" ");
        WriteInt32(instr.param2);
        services::Shell_WriteString(" ");
        services::Shell_WriteUInt32(instr.on_success);
        services::Shell_WriteString(" ");
        services::Shell_WriteUInt32(instr.on_timeout);
        services::Shell_WriteString("\r\n");
        return;
    }
    if (instr.op == app::ACT_OP_DM_POSITION) {
        services::Shell_WriteString(
            (instr.until == app::ACT_COND_DM_RELATIVE) ?
                "relative " : "absolute ");
        WriteInt32(instr.param1);
        services::Shell_WriteString(" ");
        WriteInt32(instr.param2);
        services::Shell_WriteString(" ");
        WriteInt32(instr.condition_value);
        services::Shell_WriteString(" ");
        services::Shell_WriteUInt32(instr.on_success);
        services::Shell_WriteString(" ");
        services::Shell_WriteUInt32(instr.on_timeout);
        services::Shell_WriteString("\r\n");
        return;
    }
    if (instr.op == app::ACT_OP_DM_SPEED) {
        WriteInt32(instr.param1);
        services::Shell_WriteString(" ");
        WriteInt32(instr.param2);
        services::Shell_WriteString(" ");
        services::Shell_WriteUInt32(instr.on_success);
        services::Shell_WriteString(" ");
        services::Shell_WriteUInt32(instr.on_timeout);
        services::Shell_WriteString("\r\n");
        return;
    }
    if (instr.op == app::ACT_OP_DM_DISABLE) {
        services::Shell_WriteUInt32(instr.on_success);
        services::Shell_WriteString(" ");
        services::Shell_WriteUInt32(instr.on_timeout);
        services::Shell_WriteString("\r\n");
        return;
    }
    if (app::ActionCondition_IsCompareOp(instr.op)) {
        app::ActionConditionConfig condition;
        if (!app::ActionCondition_Decode(&instr, &condition)) {
            services::Shell_WriteLine("invalid");
            return;
        }
        if ((instr.op == app::ACT_OP_DRIVE_IF) ||
            (instr.op == app::ACT_OP_FOLLOW_IF)) {
            WriteInt32(instr.param1);
            services::Shell_WriteString(" ");
        }
        services::Shell_WriteString(
            app::ActionCondition_SourceText(condition.source));
        services::Shell_WriteString(" ");
        services::Shell_WriteString(
            app::ActionCondition_CompareText(condition.compare));
        services::Shell_WriteString(" ");
        WriteInt32(condition.value);
        services::Shell_WriteString(" ");
        if (instr.op == app::ACT_OP_CONDITION) {
            services::Shell_WriteString(
                (condition.mode == app::ACT_CONDITION_WAIT) ?
                    "wait" : "instant");
            services::Shell_WriteString(" ");
        }
        services::Shell_WriteUInt32(condition.timeout_ms);
        services::Shell_WriteString(" ");
        services::Shell_WriteUInt32(condition.stable_ms);
        services::Shell_WriteString(" ");
        services::Shell_WriteUInt32(instr.on_success);
        services::Shell_WriteString(" ");
        services::Shell_WriteUInt32(instr.on_timeout);
        services::Shell_WriteString("\r\n");
        return;
    }
    WriteInt32(instr.param1);
    services::Shell_WriteString(" ");
    WriteInt32(instr.param2);
    services::Shell_WriteString(" ");
    services::Shell_WriteString(CondText(instr.until));
    services::Shell_WriteString(" ");
    services::Shell_WriteUInt32(instr.on_success);
    services::Shell_WriteString(" ");
    services::Shell_WriteUInt32(instr.on_timeout);
    services::Shell_WriteString("\r\n");
}

const char *ValidationFieldText(app::ActionValidationField field)
{
    switch (field) {
    case app::ACT_VALID_FIELD_TABLE: return "table";
    case app::ACT_VALID_FIELD_OP: return "op";
    case app::ACT_VALID_FIELD_PARAM1: return "param1";
    case app::ACT_VALID_FIELD_PARAM2: return "param2";
    case app::ACT_VALID_FIELD_CONDITION: return "condition";
    case app::ACT_VALID_FIELD_ON_SUCCESS: return "on_success";
    case app::ACT_VALID_FIELD_ON_TIMEOUT: return "on_timeout";
    case app::ACT_VALID_FIELD_NONE:
    default: return "none";
    }
}

const char *ValidationReasonText(app::ActionValidationReason reason)
{
    switch (reason) {
    case app::ACT_VALID_EMPTY: return "empty";
    case app::ACT_VALID_TOO_MANY: return "too_many";
    case app::ACT_VALID_UNKNOWN_OP: return "unknown_op";
    case app::ACT_VALID_OUT_OF_RANGE: return "out_of_range";
    case app::ACT_VALID_MUST_BE_ZERO: return "must_be_zero";
    case app::ACT_VALID_MUST_BE_NONZERO: return "must_be_nonzero";
    case app::ACT_VALID_WRONG_CONDITION: return "wrong_condition";
    case app::ACT_VALID_BAD_TARGET: return "bad_target";
    case app::ACT_VALID_OK:
    default: return "ok";
    }
}

void RunCommand(int argc, const char * const argv[])
{
#if FEATURE_ENABLE_IMU && FEATURE_ENABLE_MOTOR_DRIVER
    if (argc < 2) {
        PrintRunUsage();
        return;
    }

    if (StrEqual(argv[1], "status")) {
        if (argc != 2) {
            PrintRunUsage();
            return;
        }
        const app::ActionRunnerState *st = app::ActionRunner_GetState();
        services::Shell_WriteString("run ");
        services::Shell_WriteUInt32(st->current);
        services::Shell_WriteString("/");
        services::Shell_WriteUInt32(st->count);
        services::Shell_WriteString(" running=");
        services::Shell_WriteUInt32(st->running ? 1U : 0U);
        services::Shell_WriteString(" last=");
        services::Shell_WriteUInt32(st->last_success ? 1U : 0U);
        if (st->running && (st->current < st->count)) {
            services::Shell_WriteString(" cur=");
            services::Shell_WriteString(OpText(st->instrs[st->current].op));
        }
        const app::ChassisState *chassis = app::Chassis_GetState();
        services::Shell_WriteString(" result=");
        services::Shell_WriteString(ActionRunResultText(st->result));
        services::Shell_WriteString(" status=");
        services::Shell_WriteString(DriverStatusText(st->last_status));
        services::Shell_WriteString(" reason=");
        services::Shell_WriteString(ActionFailureText(st->failure_reason));
        services::Shell_WriteString(" fail_index=");
        services::Shell_WriteUInt32(st->failure_index);
        services::Shell_WriteString(" drive=");
        WriteInt32(chassis->left.target_rpm);
        services::Shell_WriteString("/");
        WriteInt32(chassis->right.target_rpm);
        services::Shell_WriteString("\r\n");
        return;
    }

    if (StrEqual(argv[1], "validate")) {
        if ((argc != 2) &&
            ((argc != 3) || !StrEqual(argv[2], "competition"))) {
            PrintRunUsage();
            return;
        }
        app::ActionValidationResult validation;
        const drivers::DriverStatus status = (argc == 3) ?
            app::ActionRunner_ValidateCompetition(&validation) :
            app::ActionRunner_Validate(&validation);
        services::Shell_WriteString("run validate ");
        if (status == drivers::DRIVER_OK) {
            services::Shell_WriteString("ok count=");
            services::Shell_WriteUInt32(app::ActionRunner_GetState()->count);
        } else {
            services::Shell_WriteString("error index=");
            services::Shell_WriteUInt32(validation.index);
            services::Shell_WriteString(" field=");
            services::Shell_WriteString(ValidationFieldText(validation.field));
            services::Shell_WriteString(" reason=");
            services::Shell_WriteString(ValidationReasonText(validation.reason));
        }
        services::Shell_WriteString("\r\n");
        return;
    }

    if (StrEqual(argv[1], "dump")) {
        if (argc != 2) {
            PrintRunUsage();
            return;
        }
        const app::ActionRunnerState *st = app::ActionRunner_GetState();
        services::Shell_WriteString("seq ");
        services::Shell_WriteUInt32(st->count);
        services::Shell_WriteString("\r\n");
        for (uint8_t i = 0U; i < st->count; i++) {
            WriteActionInstr(st->instrs[i], true, i);
        }
        return;
    }

    if (StrEqual(argv[1], "clear")) {
        if (argc != 2) {
            PrintRunUsage();
            return;
        }
        WriteStatusLine("run clear: ", app::ActionRunner_Clear());
        return;
    }

    if (StrEqual(argv[1], "start")) {
        if (argc != 2) {
            PrintRunUsage();
            return;
        }
        WriteStatusLine("run start: ", app::ActionRunner_Start());
        return;
    }

    if (StrEqual(argv[1], "cancel")) {
        if (argc != 2) {
            PrintRunUsage();
            return;
        }
        WriteStatusLine("run cancel: ", app::ActionRunner_Cancel());
        return;
    }

    if (StrEqual(argv[1], "add")) {
        if ((argc >= 3) && StrEqual(argv[2], "condition")) {
            if (argc != 11) {
                PrintRunUsage();
                return;
            }
            app::ActionConditionConfig condition = {
                app::ACT_SOURCE_CONSTANT,
                app::ACT_COMPARE_EQ,
                0,
                app::ACT_CONDITION_IMMEDIATE,
                0U,
                0U
            };
            int32_t timeout_ms = 0;
            int32_t stable_ms = 0;
            uint8_t ons = 0U;
            uint8_t ont = 0U;
            if ((!ParseActionConditionSource(argv[3],
                                             &condition.source)) ||
                (!ParseActionCompare(argv[4], &condition.compare)) ||
                (!ParseInt32(argv[5], -2000000, 2000000,
                             &condition.value)) ||
                ((!StrEqual(argv[6], "instant")) &&
                 (!StrEqual(argv[6], "wait"))) ||
                (!ParseInt32(argv[7], 0, 30000, &timeout_ms)) ||
                (!ParseInt32(argv[8], 0, 1000, &stable_ms)) ||
                (!ParseTarget(argv[9], &ons)) ||
                (!ParseTarget(argv[10], &ont))) {
                PrintRunUsage();
                return;
            }
            condition.mode = StrEqual(argv[6], "wait") ?
                app::ACT_CONDITION_WAIT :
                app::ACT_CONDITION_IMMEDIATE;
            condition.timeout_ms = static_cast<uint16_t>(timeout_ms);
            condition.stable_ms = static_cast<uint16_t>(stable_ms);
            WriteStatusLine(
                "run add: ",
                app::ActionRunner_AddCompareInstr(
                    app::ACT_OP_CONDITION, 0, &condition, ons, ont));
            return;
        }
        if ((argc >= 3) &&
            (StrEqual(argv[2], "drive_if") ||
             StrEqual(argv[2], "follow_if"))) {
            if (argc != 11) {
                PrintRunUsage();
                return;
            }
            const app::ChassisState *cs = app::Chassis_GetState();
            const int32_t max_rpm =
                static_cast<int32_t>(cs->config.max_wheel_rpm);
            int32_t rpm = 0;
            int32_t timeout_ms = 0;
            int32_t stable_ms = 0;
            uint8_t ons = 0U;
            uint8_t ont = 0U;
            app::ActionConditionConfig condition = {
                app::ACT_SOURCE_CONSTANT,
                app::ACT_COMPARE_EQ,
                0,
                app::ACT_CONDITION_WAIT,
                0U,
                0U
            };
            if ((!ParseInt32(argv[3], -max_rpm, max_rpm, &rpm)) ||
                (!ParseActionConditionSource(argv[4],
                                             &condition.source)) ||
                (!ParseActionCompare(argv[5], &condition.compare)) ||
                (!ParseInt32(argv[6], -2000000, 2000000,
                             &condition.value)) ||
                (!ParseInt32(argv[7], 50, 30000, &timeout_ms)) ||
                (!ParseInt32(argv[8], 0, 1000, &stable_ms)) ||
                (!ParseTarget(argv[9], &ons)) ||
                (!ParseTarget(argv[10], &ont))) {
                PrintRunUsage();
                return;
            }
            condition.timeout_ms = static_cast<uint16_t>(timeout_ms);
            condition.stable_ms = static_cast<uint16_t>(stable_ms);
            const app::ActionOp op = StrEqual(argv[2], "drive_if") ?
                app::ACT_OP_DRIVE_IF : app::ACT_OP_FOLLOW_IF;
            WriteStatusLine(
                "run add: ",
                app::ActionRunner_AddCompareInstr(
                    op, rpm, &condition, ons, ont));
            return;
        }
        if ((argc >= 3) && StrEqual(argv[2], "dm_position")) {
            if (argc != 9) {
                PrintRunUsage();
                return;
            }
            bool relative = false;
            if (StrEqual(argv[3], "relative")) {
                relative = true;
            } else if (!StrEqual(argv[3], "absolute")) {
                PrintRunUsage();
                return;
            }
            int32_t target_mrad = 0;
            int32_t max_velocity_mrad_s = 0;
            int32_t timeout_ms = 0;
            uint8_t ons = 0U;
            uint8_t onf = 0U;
            if ((!ParseInt32(argv[4],
                             -drivers::DM_G6220_POSITION_LIMIT_MRAD,
                             drivers::DM_G6220_POSITION_LIMIT_MRAD,
                             &target_mrad)) ||
                (!ParseInt32(argv[5], 1, 20000,
                             &max_velocity_mrad_s)) ||
                (!ParseInt32(argv[6], 50, 30000, &timeout_ms)) ||
                ((timeout_ms % 50) != 0) ||
                (!ParseTarget(argv[7], &ons)) ||
                (!ParseTarget(argv[8], &onf))) {
                PrintRunUsage();
                return;
            }
            WriteStatusLine(
                "run add: ",
                app::ActionRunner_AddDmPosition(relative,
                                                target_mrad,
                                                max_velocity_mrad_s,
                                                timeout_ms,
                                                ons,
                                                onf));
            return;
        }
        if ((argc >= 3) && StrEqual(argv[2], "dm_speed")) {
            int32_t velocity_mrad_s = 0;
            int32_t duration_ms = 0;
            uint8_t ons = 0U;
            uint8_t onf = 0U;
            if ((argc != 7) ||
                (!ParseInt32(argv[3], -20000, 20000,
                             &velocity_mrad_s)) ||
                (velocity_mrad_s == 0) ||
                (!ParseInt32(argv[4], 50, 30000, &duration_ms)) ||
                ((duration_ms % 50) != 0) ||
                (!ParseTarget(argv[5], &ons)) ||
                (!ParseTarget(argv[6], &onf))) {
                PrintRunUsage();
                return;
            }
            WriteStatusLine(
                "run add: ",
                app::ActionRunner_AddInstr(app::ACT_OP_DM_SPEED,
                                           velocity_mrad_s,
                                           duration_ms,
                                           app::ACT_COND_IMMEDIATE,
                                           ons,
                                           onf));
            return;
        }
        if ((argc >= 3) && StrEqual(argv[2], "dm_disable")) {
            uint8_t ons = 0U;
            uint8_t onf = 0U;
            if ((argc != 5) ||
                (!ParseTarget(argv[3], &ons)) ||
                (!ParseTarget(argv[4], &onf))) {
                PrintRunUsage();
                return;
            }
            WriteStatusLine(
                "run add: ",
                app::ActionRunner_AddInstr(app::ACT_OP_DM_DISABLE,
                                           0,
                                           0,
                                           app::ACT_COND_IMMEDIATE,
                                           ons,
                                           onf));
            return;
        }
        if ((argc >= 3) && StrEqual(argv[2], "road_nav")) {
            if (argc != 8) {
                PrintRunUsage();
                return;
            }
            app::RoadRoute route = app::ROAD_ROUTE_STRAIGHT;
            const app::ChassisState *cs = app::Chassis_GetState();
            const int32_t max_rpm =
                static_cast<int32_t>(cs->config.max_wheel_rpm);
            int32_t rpm = 0;
            int32_t timeout_ms = 0;
            uint8_t ons = 0U;
            uint8_t onf = 0U;
            if ((!ParseRoadRoute(argv[3], &route)) ||
                (!ParseInt32(argv[4], 1, max_rpm, &rpm)) ||
                (!ParseInt32(argv[5], 50, 30000, &timeout_ms)) ||
                ((timeout_ms % 50) != 0) ||
                (!ParseTarget(argv[6], &ons)) ||
                (!ParseTarget(argv[7], &onf))) {
                PrintRunUsage();
                return;
            }
            WriteStatusLine(
                "run add: ",
                app::ActionRunner_AddRoadNav(
                    static_cast<int32_t>(route),
                    rpm,
                    timeout_ms,
                    ons,
                    onf));
            return;
        }
        if (argc != 8) {
            PrintRunUsage();
            return;
        }
        app::ActionOp op = app::ACT_OP_NONE;
        int32_t p1 = 0;
        int32_t p2 = 0;
        app::ActionCond until = app::ACT_COND_TIMEOUT;
        uint8_t ons = 0U;
        uint8_t ont = 0U;
        const app::ChassisState *cs = app::Chassis_GetState();
        const int32_t max_rpm =
            static_cast<int32_t>(cs->config.max_wheel_rpm);
        if (!ParseActionOp(argv[2], &op)) {
            PrintRunUsage();
            return;
        }
        bool params_ok = false;
        if ((op == app::ACT_OP_DRIVE) || (op == app::ACT_OP_FOLLOW)) {
            params_ok = ParseInt32(argv[3], -max_rpm, max_rpm, &p1) &&
                        ParseInt32(argv[4], 1, 30000, &p2);
        } else if (op == app::ACT_OP_DRIVE_MM) {
            params_ok = ParseInt32(argv[3], -10000, 10000, &p1) &&
                        (p1 != 0) &&
                        ParseInt32(argv[4], 1, max_rpm, &p2);
        } else if (op == app::ACT_OP_TURN) {
            params_ok = ParseInt32(argv[3], -180, 180, &p1) &&
                        ParseInt32(argv[4], 1, 30000, &p2);
        } else if (op == app::ACT_OP_WAIT) {
            params_ok = ParseInt32(argv[3], 0, 0, &p1) &&
                        ParseInt32(argv[4], 0, 30000, &p2);
        } else if ((op == app::ACT_OP_STOP) ||
                   (op == app::ACT_OP_BRANCH) ||
                   (op == app::ACT_OP_END)) {
            params_ok = ParseInt32(argv[3], 0, 0, &p1) &&
                        ParseInt32(argv[4], 0, 0, &p2);
        } else if (op == app::ACT_OP_LOOP) {
            params_ok = ParseInt32(argv[3], 1, 1000, &p1) &&
                        ParseInt32(argv[4], 0, 0, &p2);
        } else if ((op >= app::ACT_OP_LED_ON) &&
                   (op <= app::ACT_OP_LED_TOGGLE)) {
            params_ok = ParseInt32(argv[3], 0, 3, &p1) &&
                        ((p1 == 0) || (p1 == 2) || (p1 == 3)) &&
                        ParseInt32(argv[4], 0, 30000, &p2);
        } else if ((op >= app::ACT_OP_BUZZER_ON) &&
                   (op <= app::ACT_OP_BUZZER_TOGGLE)) {
            params_ok = ParseInt32(argv[3], 0, 0, &p1) &&
                        ParseInt32(argv[4], 0, 30000, &p2);
        }
        if ((!params_ok) ||
            (!ParseActionCond(argv[5], &until)) ||
            (!ParseTarget(argv[6], &ons)) ||
            (!ParseTarget(argv[7], &ont))) {
            PrintRunUsage();
            return;
        }
        WriteStatusLine("run add: ",
                        app::ActionRunner_AddInstr(op, p1, p2, until, ons, ont));
        return;
    }

    PrintRunUsage();
#else
    (void) argc;
    (void) argv;
    services::Shell_WriteLine("run: disabled");
#endif
}

void PrintLFUsage(void)
{
    services::Shell_WriteLine("usage:");
    services::Shell_WriteLine("  lf status");
    services::Shell_WriteLine("  lf cal");
    services::Shell_WriteLine("  lf start <rpm> <ms>");
    services::Shell_WriteLine("  lf stop");
    services::Shell_WriteLine("  lf kp <val>");
    services::Shell_WriteLine("  lf kd <val>");
    services::Shell_WriteLine("  lf maxcorr <val>");
    services::Shell_WriteLine("  lf slew <permille_per_s 1..65535>");
    services::Shell_WriteLine("  lf losthold <ms> (compatibility only)");
    services::Shell_WriteLine("  lf losttimeout <ms> (compatibility only)");
}

const char *GrayscalePositionSourceText(
    drivers::GrayscalePositionSource source)
{
    switch (source) {
    case drivers::GRAYSCALE_POSITION_CORE:
        return "core";
    case drivers::GRAYSCALE_POSITION_LEFT_EDGE:
        return "left_edge";
    case drivers::GRAYSCALE_POSITION_RIGHT_EDGE:
        return "right_edge";
    case drivers::GRAYSCALE_POSITION_HELD:
        return "held";
    case drivers::GRAYSCALE_POSITION_NONE:
    default:
        return "none";
    }
}

const char *GrayscaleTrackStateText(drivers::GrayscaleTrackState state)
{
    switch (state) {
    case drivers::GRAYSCALE_TRACK_VALID:
        return "valid";
    case drivers::GRAYSCALE_TRACK_LOST:
        return "lost";
    case drivers::GRAYSCALE_TRACK_MULTIPLE:
        return "multiple";
    case drivers::GRAYSCALE_TRACK_WIDE:
        return "wide";
    case drivers::GRAYSCALE_TRACK_SENSOR_FAULT:
        return "sensor_fault";
    case drivers::GRAYSCALE_TRACK_UNKNOWN:
    default:
        return "unknown";
    }
}

void LFCommand(int argc, const char * const argv[])
{
#if FEATURE_ENABLE_GRAYSCALE && FEATURE_ENABLE_MOTOR_DRIVER
    if (argc < 2) {
        PrintLFUsage();
        return;
    }

    if (StrEqual(argv[1], "status")) {
        const app::LFState *st = app::LF_GetState();
        services::Shell_WriteString("lf mode=");
        services::Shell_WriteString(st->mode == app::LF_FOLLOW ? "follow" :
                                    (st->mode == app::LF_CAL ? "cal" : "idle"));
        services::Shell_WriteString(" cal=");
        services::Shell_WriteUInt32(st->calibrated ? 1U : 0U);
        services::Shell_WriteString(" error=");
        WriteInt32(st->error_mpos);
        services::Shell_WriteString(" corr=");
        WriteInt32(st->correction_rpm);
        services::Shell_WriteString(" lost=");
        services::Shell_WriteUInt32(st->lost ? 1U : 0U);
        services::Shell_WriteString(" kp=");
        WriteInt32(st->kp);
        services::Shell_WriteString(" kd=");
        WriteInt32(st->kd);
        services::Shell_WriteString(" maxcorr=");
        WriteInt32(st->max_correction_rpm);
        services::Shell_WriteString(" slew=");
        services::Shell_WriteUInt32(
            st->correction_slew_permille_per_second);
        services::Shell_WriteString(" lost_hold=");
        services::Shell_WriteUInt32(st->lost_hold_ms);
        services::Shell_WriteString(" lost_stop=");
        services::Shell_WriteUInt32(st->lost_timeout_ms);
        services::Shell_WriteString(" seq=");
        services::Shell_WriteUInt32(st->last_sequence);
        services::Shell_WriteString(" road=");
        services::Shell_WriteString(app::GrayscaleRoad_TypeText(st->road_type));
        services::Shell_WriteString(" pos_valid=");
        services::Shell_WriteUInt32(st->position_valid ? 1U : 0U);
        services::Shell_WriteString(" selected=");
        WriteHex8(st->selected_mask);
        services::Shell_WriteString(" confidence=");
        services::Shell_WriteUInt32(st->position_confidence);
        services::Shell_WriteString(" source=");
        services::Shell_WriteString(
            GrayscalePositionSourceText(st->position_source));
        services::Shell_WriteString(" track_state=");
        services::Shell_WriteString(GrayscaleTrackStateText(st->track_state));
        services::Shell_WriteString(" weak_frames=");
        services::Shell_WriteUInt32(st->weak_tracking_frames);
        services::Shell_WriteString(" invalid_frames=");
        services::Shell_WriteUInt32(st->invalid_frames);
        services::Shell_WriteString(" invalid_policy=confirm");
        services::Shell_WriteUInt32(
            static_cast<uint32_t>(app::LF_INVALID_TRACK_STOP_FRAMES));
        services::Shell_WriteString(" ref_rpm=");
        services::Shell_WriteUInt32(
            static_cast<uint32_t>(app::LF_REFERENCE_RPM));
        services::Shell_WriteString(" max_ratio_permille=");
        services::Shell_WriteUInt32(app::LF_MAX_STEERING_PERMILLE);
        services::Shell_WriteString(" deadband=");
        services::Shell_WriteUInt32(
            static_cast<uint32_t>(app::LF_ERROR_DEADBAND_MPOS));
        services::Shell_WriteString("\r\n");
        return;
    }

    if (StrEqual(argv[1], "cal")) {
        if (argc != 2) {
            PrintLFUsage();
            return;
        }
        WriteStatusLine("lf cal: ", app::LF_CalibrateStart());
        return;
    }

    if (StrEqual(argv[1], "stop")) {
        if (argc != 2) {
            PrintLFUsage();
            return;
        }
#if FEATURE_ENABLE_IMU
        const app::RoadControlPhase road_phase =
            app::RoadEventController_GetState()->phase;
        if (app::RoadEventController_IsRouteActive() ||
            ((road_phase != app::ROAD_CONTROL_PHASE_IDLE) &&
             (road_phase != app::ROAD_CONTROL_PHASE_STOPPED))) {
            WriteStatusLine("lf stop: ",
                            app::RoadEventController_Cancel());
            return;
        }
#endif
        WriteStatusLine("lf stop: ", app::LF_Stop());
        return;
    }

    if (StrEqual(argv[1], "start")) {
        int32_t rpm = 0;
        uint32_t ms = 0U;
        const app::ChassisState *cs = app::Chassis_GetState();
        const int32_t max_rpm =
            static_cast<int32_t>(cs->config.max_wheel_rpm);
        if ((argc != 4) ||
            (!ParseInt32(argv[2], -max_rpm, max_rpm, &rpm)) ||
            (!ParseUint32(argv[3], 30000U, &ms))) {
            PrintLFUsage();
            return;
        }
        WriteStatusLine("lf start: ", app::LF_Start(rpm, ms));
        return;
    }

    if (StrEqual(argv[1], "kp")) {
        int32_t v = 0;
        if ((argc != 3) || (!ParseInt32(argv[2], 0, 1000000, &v))) {
            PrintLFUsage();
            return;
        }
        app::LF_SetKp(v);
        services::Shell_WriteLine("lf kp: ok");
        return;
    }

    if (StrEqual(argv[1], "maxcorr")) {
        int32_t v = 0;
        if ((argc != 3) || (!ParseInt32(argv[2], 0, 500, &v))) {
            PrintLFUsage();
            return;
        }
        app::LF_SetMaxCorrection(v);
        services::Shell_WriteLine("lf maxcorr: ok");
        return;
    }

    if (StrEqual(argv[1], "kd")) {
        int32_t value = 0;
        if ((argc != 3) || (!ParseInt32(argv[2], 0, 1000000, &value))) {
            PrintLFUsage();
            return;
        }
        app::LF_SetKd(value);
        services::Shell_WriteLine("lf kd: ok");
        return;
    }

    if (StrEqual(argv[1], "slew")) {
        uint32_t value = 0U;
        if ((argc != 3) ||
            (!ParseUint32(argv[2], UINT16_MAX, &value)) ||
            (value == 0U)) {
            PrintLFUsage();
            return;
        }
        app::LF_SetCorrectionSlew(value);
        services::Shell_WriteLine("lf slew: ok");
        return;
    }

    if (StrEqual(argv[1], "losthold")) {
        uint32_t value = 0U;
        if ((argc != 3) || (!ParseUint32(argv[2], 10000U, &value))) {
            PrintLFUsage();
            return;
        }
        if (value > app::LF_GetState()->lost_timeout_ms) {
            services::Shell_WriteLine("lf losthold: invalid_arg");
            return;
        }
        app::LF_SetLostHold(value);
        services::Shell_WriteLine("lf losthold: ok");
        return;
    }

    if (StrEqual(argv[1], "losttimeout")) {
        uint32_t v = 0U;
        if ((argc != 3) || (!ParseUint32(argv[2], 10000U, &v))) {
            PrintLFUsage();
            return;
        }
        if (v < app::LF_GetState()->lost_hold_ms) {
            services::Shell_WriteLine("lf losttimeout: invalid_arg");
            return;
        }
        app::LF_SetLostTimeout(v);
        services::Shell_WriteLine("lf losttimeout: ok");
        return;
    }

    PrintLFUsage();
#else
    (void) argc;
    (void) argv;
    services::Shell_WriteLine("lf: disabled");
#endif
}

void PrintRoadUsage(void)
{
    services::Shell_WriteLine("usage:");
    services::Shell_WriteLine("  road status|event|clear");
    services::Shell_WriteLine("  road mode detect|corner");
    services::Shell_WriteLine("  road auto on|off");
    services::Shell_WriteLine("  road align show");
    services::Shell_WriteLine("  road align set <distance_mm 0..300> <rpm 1..300>");
    services::Shell_WriteLine("  road turn show");
    services::Shell_WriteLine(
        "  road turn set <left_deg> <right_deg> <align_mm> <rpm> <reacquire_ms>");
}

void PrintRoadEvent(void)
{
    const app::AppGrayscaleData *data = app::App_GrayscaleGetData();
    services::Shell_WriteString("road event seq=");
    services::Shell_WriteUInt32(data->road_event_sequence);
    services::Shell_WriteString(" type=");
    services::Shell_WriteString(
        app::GrayscaleRoad_TypeText(data->road_event_type));
    services::Shell_WriteString(" paths=");
    WriteHex8(data->road_event_paths);
    services::Shell_WriteString(" confidence=");
    services::Shell_WriteUInt32(data->road_event_confidence);
    services::Shell_WriteString(" entry=");
    WriteHex8(data->road_event_entry_mask);
    services::Shell_WriteString(" peak=");
    WriteHex8(data->road_event_peak_mask);
    services::Shell_WriteString(" exit=");
    WriteHex8(data->road_event_exit_mask);
    services::Shell_WriteString("\r\n");
}

void RoadCommand(int argc, const char * const argv[])
{
#if FEATURE_ENABLE_GRAYSCALE && FEATURE_ENABLE_MOTOR_DRIVER && \
    FEATURE_ENABLE_IMU
    if (argc < 2) {
        PrintRoadUsage();
        return;
    }

    if ((argc == 2) && StrEqual(argv[1], "status")) {
        const app::AppGrayscaleData *data = app::App_GrayscaleGetData();
        const app::RoadControlState *state =
            app::RoadEventController_GetState();
        services::Shell_WriteString("road mode=");
        services::Shell_WriteString(
            app::RoadEventController_ModeText(state->mode));
        services::Shell_WriteString(" phase=");
        services::Shell_WriteString(
            app::RoadEventController_PhaseText(state->phase));
        services::Shell_WriteString(" route=");
        services::Shell_WriteString(
            app::RoadEventController_RouteText(state->route));
        services::Shell_WriteString(" route_result=");
        services::Shell_WriteString(
            app::RoadEventController_RouteResultText(
                state->route_result));
        services::Shell_WriteString(" detector=");
        services::Shell_WriteString(
            app::GrayscaleRoad_PhaseText(data->road_phase));
        services::Shell_WriteString(" current=");
        services::Shell_WriteString(
            app::GrayscaleRoad_TypeText(data->road_type));
        services::Shell_WriteString(" observed=");
        WriteHex8(data->road_observed_paths);
        services::Shell_WriteString(" handled=");
        services::Shell_WriteUInt32(state->handled_event_sequence);
        services::Shell_WriteString(" align_mm=");
        WriteInt32(state->config.align_distance_mm);
        services::Shell_WriteString(" align_rpm=");
        services::Shell_WriteUInt32(state->config.align_rpm);
        services::Shell_WriteString(" active_rpm=");
        WriteInt32(state->road_base_rpm);
        services::Shell_WriteString(" reacq_frames=");
        services::Shell_WriteUInt32(state->reacquire_valid_frames);
        services::Shell_WriteString(" policy=");
        services::Shell_WriteString(
            app::RoadEventController_PolicyText(state->last_policy));
        services::Shell_WriteString(" last=");
        services::Shell_WriteString(DriverStatusText(state->last_status));
        services::Shell_WriteString("\r\n");
        return;
    }

    if ((argc == 2) && StrEqual(argv[1], "event")) {
        PrintRoadEvent();
        return;
    }

    if ((argc == 2) && StrEqual(argv[1], "clear")) {
        app::RoadEventController_ClearEvent();
        services::Shell_WriteLine("road clear: ok");
        return;
    }

    if ((argc == 3) && StrEqual(argv[1], "mode")) {
        app::RoadControlMode mode = app::ROAD_CONTROL_DETECT_ONLY;
        if (StrEqual(argv[2], "corner")) {
            mode = app::ROAD_CONTROL_AUTO_CORNERS;
        } else if (!StrEqual(argv[2], "detect")) {
            PrintRoadUsage();
            return;
        }
        WriteStatusLine("road mode: ",
                        app::RoadEventController_SetMode(mode));
        return;
    }

    if ((argc == 3) && StrEqual(argv[1], "auto")) {
        app::RoadControlMode mode = app::ROAD_CONTROL_DETECT_ONLY;
        if (StrEqual(argv[2], "on")) {
            mode = app::ROAD_CONTROL_AUTO_CORNERS;
        } else if (!StrEqual(argv[2], "off")) {
            PrintRoadUsage();
            return;
        }
        WriteStatusLine("road auto: ",
                        app::RoadEventController_SetMode(mode));
        return;
    }

    if ((argc == 3) && StrEqual(argv[1], "turn") &&
        StrEqual(argv[2], "show")) {
        const app::RoadControlConfig &config =
            app::RoadEventController_GetState()->config;
        services::Shell_WriteString("road turn left_deg=");
        WriteInt32(config.left_turn_deg);
        services::Shell_WriteString(" right_deg=");
        WriteInt32(config.right_turn_deg);
        services::Shell_WriteString(" align_mm=");
        WriteInt32(config.align_distance_mm);
        services::Shell_WriteString(" align_rpm=");
        services::Shell_WriteUInt32(config.align_rpm);
        services::Shell_WriteString(" reacquire_ms=");
        services::Shell_WriteUInt32(config.reacquire_timeout_ms);
        services::Shell_WriteString("\r\n");
        return;
    }

    if ((argc == 3) && StrEqual(argv[1], "align") &&
        StrEqual(argv[2], "show")) {
        const app::RoadControlConfig &config =
            app::RoadEventController_GetState()->config;
        services::Shell_WriteString("road align distance_mm=");
        WriteInt32(config.align_distance_mm);
        services::Shell_WriteString(" rpm=");
        services::Shell_WriteUInt32(config.align_rpm);
        services::Shell_WriteString("\r\n");
        return;
    }

    if ((argc == 5) && StrEqual(argv[1], "align") &&
        StrEqual(argv[2], "set")) {
        int32_t distance_mm = 0;
        uint32_t rpm = 0U;
        if ((!ParseInt32(argv[3], 0, 300, &distance_mm)) ||
            (!ParseUint32(argv[4], 300U, &rpm)) || (rpm == 0U)) {
            PrintRoadUsage();
            return;
        }
        WriteStatusLine(
            "road align: ",
            app::RoadEventController_SetAlignConfig(distance_mm, rpm));
        return;
    }

    if ((argc == 8) && StrEqual(argv[1], "turn") &&
        StrEqual(argv[2], "set")) {
        int32_t left_deg = 0;
        int32_t right_deg = 0;
        int32_t align_mm = 0;
        uint32_t align_rpm = 0U;
        uint32_t reacquire_ms = 0U;
        if ((!ParseInt32(argv[3], 1, 180, &left_deg)) ||
            (!ParseInt32(argv[4], -180, -1, &right_deg)) ||
            (!ParseInt32(argv[5], 0, 300, &align_mm)) ||
            (!ParseUint32(argv[6], 300U, &align_rpm)) ||
            (!ParseUint32(argv[7], 5000U, &reacquire_ms)) ||
            (align_rpm == 0U) || (reacquire_ms == 0U)) {
            PrintRoadUsage();
            return;
        }
        WriteStatusLine(
            "road turn: ",
            app::RoadEventController_SetTurnConfig(left_deg,
                                                    right_deg,
                                                    align_mm,
                                                    align_rpm,
                                                    reacquire_ms));
        return;
    }

    PrintRoadUsage();
#else
    (void) argc;
    (void) argv;
    services::Shell_WriteLine("road: disabled");
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
        motor::SpeedControlTelemetry telemetry = {};

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

        const drivers::DriverStatus telemetry_status =
            motor::ReadSpeedControlTelemetry(&g_motorClient, &telemetry);
        PrintMotorRpmBlock(rpm,
                           (telemetry_status == drivers::DRIVER_OK) ?
                               &telemetry : 0);
        return;
    }

    if (StrEqual(argv[1], "ramp")) {
        motor::SpeedRamp ramp = {};

        if (argc == 2) {
            const drivers::DriverStatus status =
                motor::ReadSpeedRamp(&g_motorClient, &ramp);
            if (status != drivers::DRIVER_OK) {
                WriteStatusLine("motor ramp: ", status);
                return;
            }

            services::Shell_WriteString("motor ramp accel_rpm_s=");
            services::Shell_WriteUInt32(ramp.accel_rpm_per_s);
            services::Shell_WriteString(" decel_rpm_s=");
            services::Shell_WriteUInt32(ramp.decel_rpm_per_s);
            services::Shell_WriteString("\r\n");
            return;
        }

        uint32_t accel_rpm_per_s = 0U;
        uint32_t decel_rpm_per_s = 0U;
        if ((argc != 4) ||
            (!ParseUint32(argv[2],
                          motor::kSpeedRampMaxRpmPerSecond,
                          &accel_rpm_per_s)) ||
            (!ParseUint32(argv[3],
                          motor::kSpeedRampMaxRpmPerSecond,
                          &decel_rpm_per_s))) {
            PrintMotorUsage();
            return;
        }

        ramp.accel_rpm_per_s = static_cast<uint16_t>(accel_rpm_per_s);
        ramp.decel_rpm_per_s = static_cast<uint16_t>(decel_rpm_per_s);
        const drivers::DriverStatus status =
            motor::SetSpeedRamp(&g_motorClient, ramp);
        if (status != drivers::DRIVER_OK) {
            WriteStatusLine("motor ramp: ", status);
            return;
        }

        drivers::DriverStatus param_status =
            ConfigStore_Set("speed_accel_rpm_s",
                            static_cast<int32_t>(accel_rpm_per_s));
        if (param_status == drivers::DRIVER_OK) {
            param_status =
                ConfigStore_Set("speed_decel_rpm_s",
                                static_cast<int32_t>(decel_rpm_per_s));
        }
        WriteStatusLine("motor ramp: ", param_status);
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

        static const char *const kPidParamNames[motor::kSpeedPidLength] = {
            "speed_kp",
            "speed_ki",
            "speed_kd",
            "speed_max_duty",
            "speed_min_duty",
        };
        for (uint8_t i = 0U; i < length; i++) {
            const drivers::DriverStatus param_status =
                ConfigStore_Set(kPidParamNames[i], data[i]);
            if (param_status != drivers::DRIVER_OK) {
                WriteStatusLine("motor pid param: ", param_status);
                return;
            }
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
        Chassis_ReleaseAllMotorCommands();
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
        bool track_position = false;
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
                track_position = true;

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

        if (motion.mode_ack) {
            if (track_position) {
                const drivers::DriverStatus lease_status =
                    Chassis_TrackMotorPosition(motor1);
                if (lease_status != drivers::DRIVER_OK) {
                    (void) motor::SetCoast(&g_motorClient, motor1, &motion);
                    Chassis_ReleaseMotorCommand(motor1);
                    WriteStatusLine("motor position lease: ", lease_status);
                    return;
                }
            } else {
                Chassis_ReleaseMotorCommand(motor1);
            }
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

#if FEATURE_ENABLE_DEBUG_UART
void TxStatCommand(int argc, const char * const argv[])
{
    (void) argc;
    (void) argv;

    services::Shell_WriteString("UART TX queued=");
    services::Shell_WriteUInt32(services::DebugUart_GetTxPending());
    services::Shell_WriteString(" dropped=");
    services::Shell_WriteUInt32(services::DebugUart_GetTxDroppedCount());
    services::Shell_WriteString(" capacity=");
    services::Shell_WriteUInt32(DEBUG_UART_TX_BUFFER_SIZE - 1U);
    services::Shell_WriteString(" dma_active=");
    services::Shell_WriteUInt32(
        services::DebugUart_IsTxDmaActive() ? 1U : 0U);
    services::Shell_WriteString(" dma_blocks=");
    services::Shell_WriteUInt32(
        services::DebugUart_GetTxDmaBlockCount());
    services::Shell_WriteString(" dma_errors=");
    services::Shell_WriteUInt32(
        services::DebugUart_GetTxDmaErrorCount());
    services::Shell_WriteString(" dma_block_size=");
    services::Shell_WriteUInt32(DEBUG_UART_TX_DMA_BLOCK_SIZE);
    services::Shell_WriteString(" rx_avail=");
    services::Shell_WriteUInt32(services::DebugUart_GetRxAvailable());
    services::Shell_WriteString(" rx_dropped=");
    services::Shell_WriteUInt32(services::DebugUart_GetRxDroppedCount());
    services::Shell_WriteString("\r\n");
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
    services::Shell_WriteLine("  gray process");
    services::Shell_WriteLine("  gray calib show|status|reload|sweep [ms]");
    services::Shell_WriteLine("  gray calib white [frames]|black [frames]|commit|cancel");
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
        services::Shell_WriteString(" processed=");
        services::Shell_WriteUInt32(data->processed_valid ? 1U : 0U);
        services::Shell_WriteString(" seq=");
        services::Shell_WriteUInt32(data->sequence);
        services::Shell_WriteString(" errors=");
        services::Shell_WriteUInt32(data->error_count);
        services::Shell_WriteString(" process_errors=");
        services::Shell_WriteUInt32(data->processing_error_count);
        services::Shell_WriteString(" frame_ms=");
        services::Shell_WriteUInt32(data->frame_period_ms);
        services::Shell_WriteString(" road=");
        services::Shell_WriteString(app::GrayscaleRoad_TypeText(data->road_type));
        services::Shell_WriteString(" road_phase=");
        services::Shell_WriteString(
            app::GrayscaleRoad_PhaseText(data->road_phase));
        services::Shell_WriteString(" event_seq=");
        services::Shell_WriteUInt32(data->road_event_sequence);
        services::Shell_WriteString(" event=");
        services::Shell_WriteString(
            app::GrayscaleRoad_TypeText(data->road_event_type));
        services::Shell_WriteString("\r\n");
        return;
    }

    if (StrEqual(argv[1], "process") && (argc == 2)) {
        const AppGrayscaleData *data = App_GrayscaleGetData();
        services::Shell_WriteString("gray normalized:");
        for (uint8_t i = 0U; i < drivers::GRAYSCALE_CHANNEL_COUNT; i++) {
            services::Shell_WriteString(" ");
            services::Shell_WriteUInt32(data->normalized[i]);
        }
        services::Shell_WriteString(" mask=");
        WriteHex8(data->active_mask);
        services::Shell_WriteString(" usable=");
        WriteHex8(data->usable_mask);
        services::Shell_WriteString(" track=");
        WriteHex8(data->track_mask);
        services::Shell_WriteString(" selected=");
        WriteHex8(data->selected_mask);
        services::Shell_WriteString(" calib_fault=");
        WriteHex8(data->calibration_fault_mask);
        services::Shell_WriteString(" saturation=");
        WriteHex8(data->saturation_mask);
        services::Shell_WriteString(" anomaly=");
        WriteHex8(data->channel_anomaly_mask);
        services::Shell_WriteString(" line=");
        services::Shell_WriteUInt32(data->line_detected ? 1U : 0U);
        services::Shell_WriteString(" pos_valid=");
        services::Shell_WriteUInt32(data->position_valid ? 1U : 0U);
        services::Shell_WriteString(" pos=");
        WriteInt32(data->line_position);
        services::Shell_WriteString(" strength=");
        services::Shell_WriteUInt32(data->line_strength);
        services::Shell_WriteString(" confidence=");
        services::Shell_WriteUInt32(data->position_confidence);
        services::Shell_WriteString(" source=");
        services::Shell_WriteString(
            GrayscalePositionSourceText(data->position_source));
        services::Shell_WriteString(" track_state=");
        services::Shell_WriteString(
            GrayscaleTrackStateText(data->track_state));
        services::Shell_WriteString(" weak_frames=");
        services::Shell_WriteUInt32(data->weak_tracking_frames);
        services::Shell_WriteString(" invalid_frames=");
        services::Shell_WriteUInt32(data->invalid_frames);
        services::Shell_WriteString(" road=");
        services::Shell_WriteString(app::GrayscaleRoad_TypeText(data->road_type));
        services::Shell_WriteString(" road_phase=");
        services::Shell_WriteString(
            app::GrayscaleRoad_PhaseText(data->road_phase));
        services::Shell_WriteString(" road_paths=");
        WriteHex8(data->road_observed_paths);
        services::Shell_WriteString(" event_seq=");
        services::Shell_WriteUInt32(data->road_event_sequence);
        services::Shell_WriteString(" event=");
        services::Shell_WriteString(
            app::GrayscaleRoad_TypeText(data->road_event_type));
        services::Shell_WriteString(" on=");
        services::Shell_WriteUInt32(data->threshold_on);
        services::Shell_WriteString(" off=");
        services::Shell_WriteUInt32(data->threshold_off);
        services::Shell_WriteString("\r\n");
        return;
    }

    if (StrEqual(argv[1], "calib")) {
        if ((argc == 3) && StrEqual(argv[2], "status")) {
            const AppGrayscaleCalibrationStatus *status =
                App_GrayscaleGetCalibrationStatus();
            services::Shell_WriteString("gray calib running=");
            services::Shell_WriteUInt32(status->running ? 1U : 0U);
            services::Shell_WriteString(" mode=");
            services::Shell_WriteUInt32(status->mode);
            services::Shell_WriteString(" samples=");
            services::Shell_WriteUInt32(status->sample_count);
            services::Shell_WriteString("/");
            services::Shell_WriteUInt32(status->target_samples);
            services::Shell_WriteString(" white=");
            services::Shell_WriteUInt32(status->white_ready ? 1U : 0U);
            services::Shell_WriteString(" black=");
            services::Shell_WriteUInt32(status->black_ready ? 1U : 0U);
            services::Shell_WriteString(" fault=");
            WriteHex8(status->fault_mask);
            services::Shell_WriteString(" last=");
            services::Shell_WriteString(DriverStatusText(status->last_status));
            services::Shell_WriteString("\r\n");
            return;
        }
        if ((argc == 3) && StrEqual(argv[2], "reload")) {
            WriteStatusLine("gray calib reload: ",
                            App_GrayscaleReloadCalibration());
            return;
        }
        if ((argc == 3) && StrEqual(argv[2], "show")) {
            const drivers::GrayscaleCalibration *calibration =
                App_GrayscaleGetCalibration();
            services::Shell_WriteString("gray white:");
            for (uint8_t i = 0U; i < drivers::GRAYSCALE_CHANNEL_COUNT; i++) {
                services::Shell_WriteString(" ");
                services::Shell_WriteUInt32(calibration->white[i]);
            }
            services::Shell_WriteString("\r\ngray black:");
            for (uint8_t i = 0U; i < drivers::GRAYSCALE_CHANNEL_COUNT; i++) {
                services::Shell_WriteString(" ");
                services::Shell_WriteUInt32(calibration->black[i]);
            }
            services::Shell_WriteString("\r\ngray threshold=");
            services::Shell_WriteUInt32(calibration->threshold);
            services::Shell_WriteString(" hysteresis=");
            services::Shell_WriteUInt32(calibration->hysteresis);
            services::Shell_WriteString(" floor=");
            services::Shell_WriteUInt32(calibration->position_floor);
            services::Shell_WriteString(" min_strength=");
            services::Shell_WriteUInt32(calibration->min_line_strength);
            services::Shell_WriteString(" track_mask=");
            WriteHex8(calibration->track_mask);
            services::Shell_WriteString("\r\n");
            return;
        }
        if ((argc >= 3) && StrEqual(argv[2], "sweep")) {
            uint32_t duration_ms = 2000U;
            if (((argc != 3) && (argc != 4)) ||
                ((argc == 4) &&
                 (!ParseUint32(argv[3], 10000U, &duration_ms)))) {
                PrintGrayUsage();
                return;
            }
            WriteStatusLine("gray calib sweep: ",
                            App_GrayscaleStartSweepCalibration(duration_ms));
            return;
        }
        if ((argc >= 3) &&
            (StrEqual(argv[2], "white") || StrEqual(argv[2], "black"))) {
            uint32_t frames = 16U;
            if (((argc != 3) && (argc != 4)) ||
                ((argc == 4) && (!ParseUint32(argv[3], 128U, &frames))) ||
                (frames == 0U)) {
                PrintGrayUsage();
                return;
            }
            const drivers::DriverStatus capture_status =
                StrEqual(argv[2], "white")
                ? App_GrayscaleStartWhiteCalibration(
                    static_cast<uint16_t>(frames))
                : App_GrayscaleStartBlackCalibration(
                    static_cast<uint16_t>(frames));
            WriteStatusLine("gray calib capture: ", capture_status);
            return;
        }
        if ((argc == 3) && StrEqual(argv[2], "commit")) {
            WriteStatusLine("gray calib commit: ",
                            App_GrayscaleCommitCalibration());
            return;
        }
        if ((argc == 3) && StrEqual(argv[2], "cancel")) {
            App_GrayscaleCancelCalibration();
            services::Shell_WriteLine("gray calib cancel: ok");
            return;
        }
        PrintGrayUsage();
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

void AppShell_CanWatchUpdate(void)
{
#if FEATURE_ENABLE_CAN
    if (!g_canWatchEnabled) {
        return;
    }

    /* Bound each cooperative invocation so sustained bus traffic cannot
     * monopolize the main loop or overflow the debug UART TX queue. */
    drivers::CanFrame frame = {};
    for (uint8_t count = 0U; count < 4U; count++) {
        if (!AppCanBus_ReadRaw(&frame)) {
            break;
        }
        WriteCanFrame(frame);
    }
#endif
}

void AppShell_DisableOledStreams(void)
{
#if FEATURE_ENABLE_GY931 && FEATURE_ENABLE_OLED
    (void) Gy931OledSetEnabled(false);
#endif
#if FEATURE_ENABLE_INA219 && FEATURE_ENABLE_OLED
    (void) Ina219OledSetEnabled(false);
#endif
#if FEATURE_ENABLE_IMU && FEATURE_ENABLE_OLED
    (void) ImuOledSetEnabled(false);
#endif
#if FEATURE_ENABLE_GRAYSCALE && FEATURE_ENABLE_OLED
    (void) GrayOledSetEnabled(false);
#endif
}

void PrintCompUsage(void)
{
    services::Shell_WriteLine("usage:");
    services::Shell_WriteLine("  comp arm");
    services::Shell_WriteLine("  comp select <0..7>");
    services::Shell_WriteLine("  comp start [seq 0..7]");
    services::Shell_WriteLine("  comp stop");
    services::Shell_WriteLine("  comp status");
}

const char *AppModeText(app::AppMode mode)
{
    switch (mode) {
    case app::APP_MODE_COMPETITION_ARMED: return "armed";
    case app::APP_MODE_COMPETITION_RUNNING: return "running";
    case app::APP_MODE_FAULT: return "fault";
    case app::APP_MODE_RUNNING: return "dev-running";
    default: return "idle";
    }
}

const char *CompetitionResultText(app::CompetitionResult result)
{
    switch (result) {
    case app::COMP_RESULT_DONE: return "done";
    case app::COMP_RESULT_FAILED: return "failed";
    case app::COMP_RESULT_STOPPED: return "stopped";
    case app::COMP_RESULT_LOAD_ERROR: return "load-error";
    case app::COMP_RESULT_NONE:
    default:
        return "none";
    }
}

void CompCommand(int argc, const char * const argv[])
{
    if (argc < 2) {
        PrintCompUsage();
        return;
    }

    if (StrEqual(argv[1], "arm")) {
        if (argc != 2) {
            PrintCompUsage();
            return;
        }
        WriteStatusLine("comp arm: ", app::App_CompetitionArm());
        return;
    }

    if (StrEqual(argv[1], "select")) {
        uint32_t slot = 0U;
        if ((argc != 3) || (!ParseUint32(argv[2], 7U, &slot))) {
            PrintCompUsage();
            return;
        }
        WriteStatusLine(
            "comp select: ",
            app::App_CompetitionSelect(static_cast<uint8_t>(slot)));
        return;
    }

    if (StrEqual(argv[1], "start")) {
        if (argc == 3) {
            uint32_t slot = 0U;
            if (!ParseUint32(argv[2], 7U, &slot)) {
                PrintCompUsage();
                return;
            }
            const drivers::DriverStatus select_status =
                app::App_CompetitionSelect(static_cast<uint8_t>(slot));
            if (select_status != drivers::DRIVER_OK) {
                WriteStatusLine("comp start select: ", select_status);
                return;
            }
        } else if (argc != 2) {
            PrintCompUsage();
            return;
        }
        WriteStatusLine("comp start: ", app::App_CompetitionStart());
        return;
    }

    if (StrEqual(argv[1], "stop")) {
        if (argc != 2) {
            PrintCompUsage();
            return;
        }
        WriteStatusLine("comp stop: ", app::App_CompetitionStop());
        return;
    }

    if (StrEqual(argv[1], "status")) {
        if (argc != 2) {
            PrintCompUsage();
            return;
        }
        const app::AppState *st = app::App_GetState();
        const app::CompetitionState *competition =
            app::App_CompetitionGetState();
        const app::ActionRunnerState *runner = app::ActionRunner_GetState();
        services::Shell_WriteString("comp mode=");
        services::Shell_WriteString(AppModeText(st->mode));
        services::Shell_WriteString(" slot=");
        services::Shell_WriteUInt32(competition->selected_slot);
        services::Shell_WriteString(" valid=");
        services::Shell_WriteUInt32(competition->slot_valid ? 1U : 0U);
        services::Shell_WriteString(" any_valid=");
        services::Shell_WriteUInt32(competition->any_valid_slot ? 1U : 0U);
        services::Shell_WriteString(" count=");
        services::Shell_WriteUInt32(competition->instruction_count);
        services::Shell_WriteString(" step=");
        services::Shell_WriteUInt32(runner->current);
        services::Shell_WriteString(" result=");
        services::Shell_WriteString(
            CompetitionResultText(competition->result));
        services::Shell_WriteString(" last=");
        services::Shell_WriteString(
            DriverStatusText(competition->last_status));
        services::Shell_WriteString("\r\n");
        return;
    }

    PrintCompUsage();
}

/* ===== Sequence store (FRAM persistence) ===== */

void PrintSeqUsage(void)
{
    services::Shell_WriteLine("usage:");
    services::Shell_WriteLine("  seq list");
    services::Shell_WriteLine("  seq dump <0..7>");
    services::Shell_WriteLine("  seq save <0..7>");
    services::Shell_WriteLine("  seq load <0..7>");
    services::Shell_WriteLine("  seq del <0..7>");
    services::Shell_WriteLine("  seq run <0..7>");
}

void SeqCommand(int argc, const char * const argv[])
{
    if (argc < 2) {
        PrintSeqUsage();
        return;
    }

    if (StrEqual(argv[1], "list")) {
        if (argc != 2) {
            PrintSeqUsage();
            return;
        }
        for (uint8_t i = 0; i < app::SEQ_SLOT_COUNT; i++) {
            app::SeqSlotInfo info = { false, 0U };
            const drivers::DriverStatus info_status =
                app::SeqStore_GetInfo(i, &info);
            services::Shell_WriteString("seq ");
            services::Shell_WriteUInt32(i);
            services::Shell_WriteString(" ");
            services::Shell_WriteString(
                (info_status != drivers::DRIVER_OK) ?
                    "error" : (info.valid ? "ok" : "empty"));
            services::Shell_WriteString(" count=");
            services::Shell_WriteUInt32(info.count);
            services::Shell_WriteString("\r\n");
        }
        return;
    }

    if (StrEqual(argv[1], "dump")) {
        uint32_t dump_slot = 0U;
        if ((argc != 3) || (!ParseUint32(argv[2], 7U, &dump_slot))) {
            PrintSeqUsage();
            return;
        }
        app::Instr instrs[64];
        uint8_t count = 0U;
        const drivers::DriverStatus status =
            app::SeqStore_Read(static_cast<uint8_t>(dump_slot), instrs, &count);
        if (status != drivers::DRIVER_OK) {
            WriteStatusLine("seq dump: ", status);
            return;
        }
        services::Shell_WriteString("SEQ ");
        services::Shell_WriteUInt32(dump_slot);
        services::Shell_WriteString(" ");
        services::Shell_WriteUInt32(count);
        services::Shell_WriteString("\r\n");
        for (uint8_t i = 0; i < count; i++) {
            WriteActionInstr(instrs[i], false, i);
        }
        services::Shell_WriteLine("END");
        return;
    }

    /* All remaining subcommands take a slot number */
    uint32_t slot = 0U;
    if ((argc != 3) || (!ParseUint32(argv[2], 7U, &slot))) {
        PrintSeqUsage();
        return;
    }

    if (StrEqual(argv[1], "save")) {
        const drivers::DriverStatus status =
            app::SeqStore_Save(static_cast<uint8_t>(slot));
        if ((status == drivers::DRIVER_OK) &&
            (app::App_GetState()->mode == app::APP_MODE_COMPETITION_ARMED) &&
            (app::App_CompetitionGetState()->selected_slot == slot)) {
            (void) app::App_CompetitionRefreshSelection();
        }
        WriteStatusLine("seq save: ", status);
        return;
    }

    if (StrEqual(argv[1], "load")) {
        WriteStatusLine("seq load: ", app::SeqStore_Load(static_cast<uint8_t>(slot)));
        return;
    }

    if (StrEqual(argv[1], "del")) {
        const drivers::DriverStatus status =
            app::SeqStore_Delete(static_cast<uint8_t>(slot));
        if ((status == drivers::DRIVER_OK) &&
            (app::App_GetState()->mode == app::APP_MODE_COMPETITION_ARMED) &&
            (app::App_CompetitionGetState()->selected_slot == slot)) {
            (void) app::App_CompetitionRefreshSelection();
        }
        WriteStatusLine("seq del: ", status);
        return;
    }

    if (StrEqual(argv[1], "run")) {
        const drivers::DriverStatus load_status =
            app::SeqStore_Load(static_cast<uint8_t>(slot));
        if (load_status != drivers::DRIVER_OK) {
            WriteStatusLine("seq run load: ", load_status);
            return;
        }
        WriteStatusLine("seq run: ", app::ActionRunner_Start());
        return;
    }

    PrintSeqUsage();
}

/* ===== FireWater telemetry (VOFA+ protocol) ===== */

const char *TelemProfileText(TelemProfile profile)
{
    switch (profile) {
        case TELEM_PROFILE_RUNTIME: return "runtime";
        case TELEM_PROFILE_COMPETITION: return "competition";
        case TELEM_PROFILE_MOTOR: return "motor";
        case TELEM_PROFILE_HEADING: return "heading";
        case TELEM_PROFILE_HEADING_OUTPUT: return "heading_output";
        case TELEM_PROFILE_CHASSIS: return "chassis";
        case TELEM_PROFILE_FEEDBACK_STATE: return "feedback_state";
        case TELEM_PROFILE_FEEDBACK_AGE: return "feedback_age";
        case TELEM_PROFILE_LINE: return "line";
        case TELEM_PROFILE_LINE_POSITION: return "line_position";
        case TELEM_PROFILE_LINE_OUTPUT: return "line_output";
        case TELEM_PROFILE_LINE_QUALITY: return "line_quality";
        case TELEM_PROFILE_LINE_STATE: return "line_state";
        case TELEM_PROFILE_LINE_FAULTS: return "line_faults";
        case TELEM_PROFILE_ROAD_EVENT: return "road_event";
        case TELEM_PROFILE_ROAD_SEQUENCE: return "road_sequence";
        case TELEM_PROFILE_ROAD_PHASE: return "road_phase";
        case TELEM_PROFILE_TURN_PHASE: return "turn_phase";
        case TELEM_PROFILE_TURN_RATE: return "turn_rate";
        case TELEM_PROFILE_TURN_ANGLE: return "turn_angle";
        case TELEM_PROFILE_TURN_TIMING: return "turn_timing";
        case TELEM_PROFILE_TURN_SPEED: return "turn_speed";
        case TELEM_PROFILE_ACCEL: return "accel";
        case TELEM_PROFILE_GYRO: return "gyro";
        case TELEM_PROFILE_ATTITUDE: return "attitude";
        case TELEM_PROFILE_IMU_TEMPERATURE: return "imu_temperature";
        case TELEM_PROFILE_IMU_STATE: return "imu_state";
        case TELEM_PROFILE_IMU_AGE: return "imu_age";
        case TELEM_PROFILE_GRAY_RAW: return "gray_raw";
        case TELEM_PROFILE_GRAY_HEALTH: return "gray_health";
        case TELEM_PROFILE_GRAY_AGE: return "gray_age";
        case TELEM_PROFILE_FAULT: return "fault";
        case TELEM_PROFILE_UART: return "uart";
        case TELEM_PROFILE_FULL:
        default: return "full";
    }
}

bool ParseTelemProfile(const char *text, TelemProfile *profile)
{
    if ((text == 0) || (profile == 0)) {
        return false;
    }
    if (StrEqual(text, "runtime")) {
        *profile = TELEM_PROFILE_RUNTIME;
    } else if (StrEqual(text, "competition")) {
        *profile = TELEM_PROFILE_COMPETITION;
    } else if (StrEqual(text, "motor")) {
        *profile = TELEM_PROFILE_MOTOR;
    } else if (StrEqual(text, "heading")) {
        *profile = TELEM_PROFILE_HEADING;
    } else if (StrEqual(text, "heading_output")) {
        *profile = TELEM_PROFILE_HEADING_OUTPUT;
    } else if (StrEqual(text, "chassis")) {
        *profile = TELEM_PROFILE_CHASSIS;
    } else if (StrEqual(text, "feedback_state")) {
        *profile = TELEM_PROFILE_FEEDBACK_STATE;
    } else if (StrEqual(text, "feedback_age")) {
        *profile = TELEM_PROFILE_FEEDBACK_AGE;
    } else if (StrEqual(text, "line")) {
        *profile = TELEM_PROFILE_LINE;
    } else if (StrEqual(text, "line_position")) {
        *profile = TELEM_PROFILE_LINE_POSITION;
    } else if (StrEqual(text, "line_output")) {
        *profile = TELEM_PROFILE_LINE_OUTPUT;
    } else if (StrEqual(text, "line_quality")) {
        *profile = TELEM_PROFILE_LINE_QUALITY;
    } else if (StrEqual(text, "line_state")) {
        *profile = TELEM_PROFILE_LINE_STATE;
    } else if (StrEqual(text, "line_faults")) {
        *profile = TELEM_PROFILE_LINE_FAULTS;
    } else if (StrEqual(text, "road_event")) {
        *profile = TELEM_PROFILE_ROAD_EVENT;
    } else if (StrEqual(text, "road_sequence")) {
        *profile = TELEM_PROFILE_ROAD_SEQUENCE;
    } else if (StrEqual(text, "road_phase")) {
        *profile = TELEM_PROFILE_ROAD_PHASE;
    } else if (StrEqual(text, "turn_phase")) {
        *profile = TELEM_PROFILE_TURN_PHASE;
    } else if (StrEqual(text, "turn_rate")) {
        *profile = TELEM_PROFILE_TURN_RATE;
    } else if (StrEqual(text, "turn_angle")) {
        *profile = TELEM_PROFILE_TURN_ANGLE;
    } else if (StrEqual(text, "turn_timing")) {
        *profile = TELEM_PROFILE_TURN_TIMING;
    } else if (StrEqual(text, "turn_speed")) {
        *profile = TELEM_PROFILE_TURN_SPEED;
    } else if (StrEqual(text, "accel")) {
        *profile = TELEM_PROFILE_ACCEL;
    } else if (StrEqual(text, "gyro")) {
        *profile = TELEM_PROFILE_GYRO;
    } else if (StrEqual(text, "attitude")) {
        *profile = TELEM_PROFILE_ATTITUDE;
    } else if (StrEqual(text, "imu_temperature")) {
        *profile = TELEM_PROFILE_IMU_TEMPERATURE;
    } else if (StrEqual(text, "imu_state")) {
        *profile = TELEM_PROFILE_IMU_STATE;
    } else if (StrEqual(text, "imu_age")) {
        *profile = TELEM_PROFILE_IMU_AGE;
    } else if (StrEqual(text, "gray_raw")) {
        *profile = TELEM_PROFILE_GRAY_RAW;
    } else if (StrEqual(text, "gray_health")) {
        *profile = TELEM_PROFILE_GRAY_HEALTH;
    } else if (StrEqual(text, "gray_age")) {
        *profile = TELEM_PROFILE_GRAY_AGE;
    } else if (StrEqual(text, "fault")) {
        *profile = TELEM_PROFILE_FAULT;
    } else if (StrEqual(text, "uart")) {
        *profile = TELEM_PROFILE_UART;
    } else {
        return false;
    }
    return true;
}

void TelemSendHeader(void)
{
    switch (g_telemProfile) {
        case TELEM_PROFILE_RUNTIME:
            services::DebugUart_WriteString("#t,mode,step\n");
            return;
        case TELEM_PROFILE_COMPETITION:
            services::DebugUart_WriteString(
                "#t,comp_slot,comp_slot_valid,comp_count\n");
            return;
        case TELEM_PROFILE_MOTOR:
            services::DebugUart_WriteString(
                "#t,L_tgt,L_act,R_tgt,R_act\n");
            return;
        case TELEM_PROFILE_HEADING:
            services::DebugUart_WriteString(
                "#t,yaw_tgt,yaw,head_err\n");
            return;
        case TELEM_PROFILE_HEADING_OUTPUT:
            services::DebugUart_WriteString("#t,head_corr\n");
            return;
        case TELEM_PROFILE_CHASSIS:
            services::DebugUart_WriteString(
                "#t,chassis_init,chassis_status\n");
            return;
        case TELEM_PROFILE_FEEDBACK_STATE:
            services::DebugUart_WriteString(
                "#t,feedback_status,feedback_valid\n");
            return;
        case TELEM_PROFILE_FEEDBACK_AGE:
            services::DebugUart_WriteString("#t,feedback_age_ms\n");
            return;
        case TELEM_PROFILE_LINE:
            services::DebugUart_WriteString(
                "#t,gray_pos,lf_err,lf_corr\n");
            return;
        case TELEM_PROFILE_LINE_POSITION:
            services::DebugUart_WriteString("#t,gray_pos,lf_err\n");
            return;
        case TELEM_PROFILE_LINE_OUTPUT:
            services::DebugUart_WriteString("#t,lf_corr\n");
            return;
        case TELEM_PROFILE_LINE_QUALITY:
            services::DebugUart_WriteString(
                "#t,gray_strength,gray_conf\n");
            return;
        case TELEM_PROFILE_LINE_STATE:
            services::DebugUart_WriteString(
                "#t,gray_valid,gray_state\n");
            return;
        case TELEM_PROFILE_LINE_FAULTS:
            services::DebugUart_WriteString("#t,lf_weak,lf_invalid\n");
            return;
        case TELEM_PROFILE_ROAD_EVENT:
            services::DebugUart_WriteString(
                "#t,road_type,road_event_type,road_paths\n");
            return;
        case TELEM_PROFILE_ROAD_SEQUENCE:
            services::DebugUart_WriteString("#t,road_event_seq\n");
            return;
        case TELEM_PROFILE_ROAD_PHASE:
            services::DebugUart_WriteString(
                "#t,road_phase,road_ctrl_phase\n");
            return;
        case TELEM_PROFILE_TURN_PHASE:
            services::DebugUart_WriteString("#t,head_turn_phase\n");
            return;
        case TELEM_PROFILE_TURN_RATE:
            services::DebugUart_WriteString(
                "#t,head_turn_rate_mdps,head_turn_settle_mdps\n");
            return;
        case TELEM_PROFILE_TURN_ANGLE:
            services::DebugUart_WriteString(
                "#t,head_turn_brake_mdeg,head_turn_margin_mdeg\n");
            return;
        case TELEM_PROFILE_TURN_TIMING:
            services::DebugUart_WriteString("#t,head_turn_brake_ms\n");
            return;
        case TELEM_PROFILE_TURN_SPEED:
            services::DebugUart_WriteString("#t,head_turn_settle_rpm\n");
            return;
        case TELEM_PROFILE_ACCEL:
            services::DebugUart_WriteString(
                "#t,acc_x_mg,acc_y_mg,acc_z_mg\n");
            return;
        case TELEM_PROFILE_GYRO:
            services::DebugUart_WriteString(
                "#t,gyro_x_mdps,gyro_y_mdps,gyro_z_mdps\n");
            return;
        case TELEM_PROFILE_ATTITUDE:
            services::DebugUart_WriteString("#t,pitch,roll\n");
            return;
        case TELEM_PROFILE_IMU_TEMPERATURE:
            services::DebugUart_WriteString("#t,imu_temp_cc\n");
            return;
        case TELEM_PROFILE_IMU_STATE:
            services::DebugUart_WriteString(
                "#t,imu_valid,imu_error_count\n");
            return;
        case TELEM_PROFILE_IMU_AGE:
            services::DebugUart_WriteString("#t,imu_age_ms\n");
            return;
        case TELEM_PROFILE_GRAY_RAW:
            services::DebugUart_WriteString(
                "#t,gray0,gray1,gray2,gray3,gray4,gray5,gray6,gray7\n");
            return;
        case TELEM_PROFILE_GRAY_HEALTH:
            services::DebugUart_WriteString(
                "#t,gray_sample_valid,gray_error_count\n");
            return;
        case TELEM_PROFILE_GRAY_AGE:
            services::DebugUart_WriteString("#t,gray_age_ms\n");
            return;
        case TELEM_PROFILE_FAULT:
            services::DebugUart_WriteString("#t,fault_code,fault_count\n");
            return;
        case TELEM_PROFILE_UART:
            services::DebugUart_WriteString("#t,tx_pending,tx_dropped\n");
            return;
        case TELEM_PROFILE_FULL:
        default:
            break;
    }
    services::DebugUart_WriteString(
        "#t,mode,step,L_tgt,L_act,R_tgt,R_act,yaw_tgt,yaw,head_err,"
        "head_corr,gray_pos,gray_strength,gray_conf,gray_valid,"
        "gray_state,lf_err,lf_corr,lf_weak,lf_invalid,"
        "road_type,road_event_seq,road_event_type,road_paths,road_phase,"
        "road_ctrl_phase,head_turn_phase,head_turn_rate_mdps,"
        "head_turn_brake_mdeg,head_turn_brake_ms,"
        "head_turn_margin_mdeg,head_turn_settle_mdps,"
        "head_turn_settle_rpm,fault_code,fault_count,comp_slot,"
        "comp_slot_valid,comp_count,chassis_init,chassis_status,"
        "feedback_status,feedback_valid,feedback_age_ms,tx_pending,"
        "tx_dropped,imu_valid,imu_age_ms,imu_error_count,acc_x_mg,"
        "acc_y_mg,acc_z_mg,gyro_x_mdps,gyro_y_mdps,gyro_z_mdps,"
        "pitch,roll,imu_temp_cc,gray_sample_valid,gray_age_ms,"
        "gray_error_count,gray0,gray1,gray2,gray3,gray4,gray5,"
        "gray6,gray7\n");
}

void TelemWriteInt32(int32_t value)
{
    services::Shell_WriteString(",");
    WriteInt32(value);
}

void TelemWriteUInt32(uint32_t value)
{
    services::Shell_WriteString(",");
    services::Shell_WriteUInt32(value);
}

void TelemWriteFixedMilli(int32_t value)
{
    services::Shell_WriteString(",");
    WriteFixedMilli(value);
}

void TelemSendSelectedData(void)
{
    const uint32_t now = services::Time_Millis();
    const app::AppState *app_state = app::App_GetState();
    const app::ActionRunnerState *action = app::ActionRunner_GetState();
    const app::CompetitionState *competition =
        app::App_CompetitionGetState();
    const app::ChassisState *chassis = app::Chassis_GetState();
    const app::HeadingState *heading = app::Heading_GetState();
    const app::LFState *line = app::LF_GetState();
    const app::AppImuData *imu = app::App_ImuGetData();
    const app::AppGrayscaleData *gray = app::App_GrayscaleGetData();
    const app::ConfigStoreParams *params = app::ConfigStore_Get();

    services::Shell_WriteUInt32(now);
    switch (g_telemProfile) {
        case TELEM_PROFILE_RUNTIME:
            TelemWriteInt32(static_cast<int32_t>(app_state->mode));
            TelemWriteInt32(action->running
                ? static_cast<int32_t>(action->current) : -1);
            break;
        case TELEM_PROFILE_COMPETITION:
            TelemWriteUInt32(competition->selected_slot);
            TelemWriteUInt32(competition->slot_valid ? 1U : 0U);
            TelemWriteUInt32(competition->instruction_count);
            break;
        case TELEM_PROFILE_MOTOR:
            TelemWriteInt32(chassis->left.target_rpm);
            TelemWriteInt32(chassis->left.actual_rpm);
            TelemWriteInt32(chassis->right.target_rpm);
            TelemWriteInt32(chassis->right.actual_rpm);
            break;
        case TELEM_PROFILE_HEADING:
            TelemWriteFixedMilli(heading->target_yaw_mdeg);
            TelemWriteFixedMilli((imu != 0) ? imu->yaw_mdeg : 0);
            TelemWriteFixedMilli(heading->error_mdeg);
            break;
        case TELEM_PROFILE_HEADING_OUTPUT:
            TelemWriteInt32(heading->correction_rpm);
            break;
        case TELEM_PROFILE_CHASSIS:
            TelemWriteUInt32(chassis->initialized ? 1U : 0U);
            TelemWriteUInt32(static_cast<uint32_t>(chassis->last_status));
            break;
        case TELEM_PROFILE_FEEDBACK_STATE:
            TelemWriteUInt32(
                static_cast<uint32_t>(chassis->last_feedback_status));
            TelemWriteUInt32((chassis->feedback_sequence != 0U) ? 1U : 0U);
            break;
        case TELEM_PROFILE_FEEDBACK_AGE:
            TelemWriteUInt32((chassis->feedback_sequence != 0U)
                ? static_cast<uint32_t>(now - chassis->last_feedback_ms)
                : 0U);
            break;
        case TELEM_PROFILE_LINE:
            TelemWriteInt32((gray != 0) ? gray->line_position : 0);
            TelemWriteInt32((line != 0) ? line->error_mpos : 0);
            TelemWriteInt32((line != 0) ? line->correction_rpm : 0);
            break;
        case TELEM_PROFILE_LINE_POSITION:
            TelemWriteInt32((gray != 0) ? gray->line_position : 0);
            TelemWriteInt32((line != 0) ? line->error_mpos : 0);
            break;
        case TELEM_PROFILE_LINE_OUTPUT:
            TelemWriteInt32((line != 0) ? line->correction_rpm : 0);
            break;
        case TELEM_PROFILE_LINE_QUALITY:
            TelemWriteUInt32((gray != 0) ? gray->line_strength : 0U);
            TelemWriteUInt32((gray != 0) ? gray->position_confidence : 0U);
            break;
        case TELEM_PROFILE_LINE_STATE:
            TelemWriteUInt32(((gray != 0) && gray->position_valid) ? 1U : 0U);
            TelemWriteUInt32((gray != 0)
                ? static_cast<uint32_t>(gray->track_state) : 0U);
            break;
        case TELEM_PROFILE_LINE_FAULTS:
            TelemWriteUInt32((line != 0) ? line->weak_tracking_frames : 0U);
            TelemWriteUInt32((line != 0) ? line->invalid_frames : 0U);
            break;
        case TELEM_PROFILE_ROAD_EVENT:
            TelemWriteUInt32((gray != 0)
                ? static_cast<uint32_t>(gray->road_type) : 0U);
            TelemWriteUInt32((gray != 0)
                ? static_cast<uint32_t>(gray->road_event_type) : 0U);
            TelemWriteUInt32((gray != 0) ? gray->road_event_paths : 0U);
            break;
        case TELEM_PROFILE_ROAD_SEQUENCE:
            TelemWriteUInt32((gray != 0) ? gray->road_event_sequence : 0U);
            break;
        case TELEM_PROFILE_ROAD_PHASE:
            TelemWriteUInt32((gray != 0)
                ? static_cast<uint32_t>(gray->road_phase) : 0U);
            TelemWriteUInt32(static_cast<uint32_t>(
                app::RoadEventController_GetState()->phase));
            break;
        case TELEM_PROFILE_TURN_PHASE:
            TelemWriteUInt32(static_cast<uint32_t>(heading->turn_phase));
            break;
        case TELEM_PROFILE_TURN_RATE:
            TelemWriteInt32(heading->turn_rate_mdps);
            TelemWriteUInt32(params->heading_turn_settle_rate_mdps);
            break;
        case TELEM_PROFILE_TURN_ANGLE:
            TelemWriteInt32(heading->turn_brake_angle_mdeg);
            TelemWriteUInt32(params->heading_turn_brake_margin_mdeg);
            break;
        case TELEM_PROFILE_TURN_TIMING:
            TelemWriteUInt32(params->heading_turn_brake_ms);
            break;
        case TELEM_PROFILE_TURN_SPEED:
            TelemWriteUInt32(params->heading_turn_settle_rpm);
            break;
        case TELEM_PROFILE_ACCEL:
        case TELEM_PROFILE_GYRO: {
            const int32_t *values = (g_telemProfile == TELEM_PROFILE_ACCEL)
                ? ((imu != 0) ? imu->accel_mg : 0)
                : ((imu != 0) ? imu->gyro_mdps : 0);
            for (uint32_t axis = 0U; axis < 3U; axis++) {
                TelemWriteInt32((values != 0) ? values[axis] : 0);
            }
            break;
        }
        case TELEM_PROFILE_ATTITUDE:
            TelemWriteFixedMilli((imu != 0) ? imu->pitch_mdeg : 0);
            TelemWriteFixedMilli((imu != 0) ? imu->roll_mdeg : 0);
            break;
        case TELEM_PROFILE_IMU_TEMPERATURE:
            TelemWriteInt32((imu != 0) ? imu->temp_centi_c : 0);
            break;
        case TELEM_PROFILE_IMU_STATE:
            TelemWriteUInt32(((imu != 0) && imu->valid) ? 1U : 0U);
            TelemWriteUInt32((imu != 0) ? imu->error_count : 0U);
            break;
        case TELEM_PROFILE_IMU_AGE:
            TelemWriteUInt32(((imu != 0) && (imu->sequence != 0U))
                ? static_cast<uint32_t>(now - imu->last_update_ms) : 0U);
            break;
        case TELEM_PROFILE_GRAY_RAW:
            for (uint32_t channel = 0U;
                 channel < drivers::GRAYSCALE_CHANNEL_COUNT;
                 channel++) {
                TelemWriteUInt32((gray != 0) ? gray->raw[channel] : 0U);
            }
            break;
        case TELEM_PROFILE_GRAY_HEALTH:
            TelemWriteUInt32(((gray != 0) && gray->valid) ? 1U : 0U);
            TelemWriteUInt32((gray != 0) ? gray->error_count : 0U);
            break;
        case TELEM_PROFILE_GRAY_AGE:
            TelemWriteUInt32(((gray != 0) && (gray->sequence != 0U))
                ? static_cast<uint32_t>(now - gray->last_update_ms) : 0U);
            break;
        case TELEM_PROFILE_FAULT:
            TelemWriteUInt32(static_cast<uint32_t>(services::Fault_Get()));
            TelemWriteUInt32(services::Fault_GetCount());
            break;
        case TELEM_PROFILE_UART:
            TelemWriteUInt32(services::DebugUart_GetTxPending());
            TelemWriteUInt32(services::DebugUart_GetTxDroppedCount());
            break;
        case TELEM_PROFILE_FULL:
        default:
            break;
    }
    services::Shell_WriteString("\n");
}

void TelemSendData(void)
{
    if (g_telemProfile != TELEM_PROFILE_FULL) {
        TelemSendSelectedData();
        return;
    }
    const uint32_t now = services::Time_Millis();
    const app::AppState *app = app::App_GetState();
    const app::ChassisState *cs = app::Chassis_GetState();
    const app::HeadingState *hs = app::Heading_GetState();
    const app::ActionRunnerState *as = app::ActionRunner_GetState();
    const app::CompetitionState *competition =
        app::App_CompetitionGetState();
    const app::AppImuData *imu = app::App_ImuGetData();
    const app::AppGrayscaleData *gray = app::App_GrayscaleGetData();
    const app::LFState *lf = app::LF_GetState();
    const app::ConfigStoreParams *params = app::ConfigStore_Get();

    /* t */
    services::Shell_WriteUInt32(now);
    /* mode */
    services::Shell_WriteString(",");
    WriteInt32(static_cast<int32_t>(app->mode));
    /* step */
    services::Shell_WriteString(",");
    WriteInt32(as->running ? static_cast<int32_t>(as->current) : -1);
    /* L_tgt, L_act */
    services::Shell_WriteString(",");
    WriteInt32(cs->left.target_rpm);
    services::Shell_WriteString(",");
    WriteInt32(cs->left.actual_rpm);
    /* R_tgt, R_act */
    services::Shell_WriteString(",");
    WriteInt32(cs->right.target_rpm);
    services::Shell_WriteString(",");
    WriteInt32(cs->right.actual_rpm);
    /* yaw_tgt, yaw, err (milli-deg -> deg) */
    services::Shell_WriteString(",");
    WriteFixedMilli(hs->target_yaw_mdeg);
    services::Shell_WriteString(",");
    WriteFixedMilli((imu != 0) ? imu->yaw_mdeg : 0);
    services::Shell_WriteString(",");
    WriteFixedMilli(hs->error_mdeg);
    /* corr */
    services::Shell_WriteString(",");
    WriteInt32(hs->correction_rpm);
    /* Grayscale interpolation and line-follow control diagnostics. */
    services::Shell_WriteString(",");
    WriteInt32((gray != 0) ? gray->line_position : 0);
    services::Shell_WriteString(",");
    services::Shell_WriteUInt32((gray != 0) ? gray->line_strength : 0U);
    services::Shell_WriteString(",");
    services::Shell_WriteUInt32(
        (gray != 0) ? gray->position_confidence : 0U);
    services::Shell_WriteString(",");
    services::Shell_WriteUInt32(
        ((gray != 0) && gray->position_valid) ? 1U : 0U);
    services::Shell_WriteString(",");
    services::Shell_WriteUInt32((gray != 0)
        ? static_cast<uint32_t>(gray->track_state)
        : 0U);
    services::Shell_WriteString(",");
    WriteInt32((lf != 0) ? lf->error_mpos : 0);
    services::Shell_WriteString(",");
    WriteInt32((lf != 0) ? lf->correction_rpm : 0);
    services::Shell_WriteString(",");
    services::Shell_WriteUInt32((lf != 0) ? lf->weak_tracking_frames : 0U);
    services::Shell_WriteString(",");
    services::Shell_WriteUInt32((lf != 0) ? lf->invalid_frames : 0U);
    services::Shell_WriteString(",");
    services::Shell_WriteUInt32((gray != 0)
        ? static_cast<uint32_t>(gray->road_type)
        : 0U);
    services::Shell_WriteString(",");
    services::Shell_WriteUInt32((gray != 0)
        ? gray->road_event_sequence
        : 0U);
    services::Shell_WriteString(",");
    services::Shell_WriteUInt32((gray != 0)
        ? static_cast<uint32_t>(gray->road_event_type)
        : 0U);
    services::Shell_WriteString(",");
    services::Shell_WriteUInt32((gray != 0)
        ? gray->road_event_paths
        : 0U);
    services::Shell_WriteString(",");
    services::Shell_WriteUInt32((gray != 0)
        ? static_cast<uint32_t>(gray->road_phase)
        : 0U);
    services::Shell_WriteString(",");
    services::Shell_WriteUInt32(static_cast<uint32_t>(
        app::RoadEventController_GetState()->phase));
    services::Shell_WriteString(",");
    services::Shell_WriteUInt32(static_cast<uint32_t>(hs->turn_phase));
    services::Shell_WriteString(",");
    WriteInt32(hs->turn_rate_mdps);
    services::Shell_WriteString(",");
    WriteInt32(hs->turn_brake_angle_mdeg);
    services::Shell_WriteString(",");
    services::Shell_WriteUInt32(params->heading_turn_brake_ms);
    services::Shell_WriteString(",");
    services::Shell_WriteUInt32(params->heading_turn_brake_margin_mdeg);
    services::Shell_WriteString(",");
    services::Shell_WriteUInt32(params->heading_turn_settle_rate_mdps);
    services::Shell_WriteString(",");
    services::Shell_WriteUInt32(params->heading_turn_settle_rpm);
    /* Dashboard health and raw-sensor extension. Existing VOFA+ channels
     * above retain their original names and order. */
    services::Shell_WriteString(",");
    services::Shell_WriteUInt32(
        static_cast<uint32_t>(services::Fault_Get()));
    services::Shell_WriteString(",");
    services::Shell_WriteUInt32(services::Fault_GetCount());
    services::Shell_WriteString(",");
    services::Shell_WriteUInt32(competition->selected_slot);
    services::Shell_WriteString(",");
    services::Shell_WriteUInt32(competition->slot_valid ? 1U : 0U);
    services::Shell_WriteString(",");
    services::Shell_WriteUInt32(competition->instruction_count);
    services::Shell_WriteString(",");
    services::Shell_WriteUInt32(cs->initialized ? 1U : 0U);
    services::Shell_WriteString(",");
    services::Shell_WriteUInt32(
        static_cast<uint32_t>(cs->last_status));
    services::Shell_WriteString(",");
    services::Shell_WriteUInt32(
        static_cast<uint32_t>(cs->last_feedback_status));
    services::Shell_WriteString(",");
    services::Shell_WriteUInt32(
        (cs->feedback_sequence != 0U) ? 1U : 0U);
    services::Shell_WriteString(",");
    services::Shell_WriteUInt32((cs->feedback_sequence != 0U)
        ? static_cast<uint32_t>(now - cs->last_feedback_ms)
        : 0U);
    services::Shell_WriteString(",");
    services::Shell_WriteUInt32(services::DebugUart_GetTxPending());
    services::Shell_WriteString(",");
    services::Shell_WriteUInt32(services::DebugUart_GetTxDroppedCount());
    services::Shell_WriteString(",");
    services::Shell_WriteUInt32(
        ((imu != 0) && imu->valid) ? 1U : 0U);
    services::Shell_WriteString(",");
    services::Shell_WriteUInt32(
        ((imu != 0) && (imu->sequence != 0U))
            ? static_cast<uint32_t>(now - imu->last_update_ms)
            : 0U);
    services::Shell_WriteString(",");
    services::Shell_WriteUInt32((imu != 0) ? imu->error_count : 0U);
    for (uint32_t axis = 0U; axis < 3U; axis++) {
        services::Shell_WriteString(",");
        WriteInt32((imu != 0) ? imu->accel_mg[axis] : 0);
    }
    for (uint32_t axis = 0U; axis < 3U; axis++) {
        services::Shell_WriteString(",");
        WriteInt32((imu != 0) ? imu->gyro_mdps[axis] : 0);
    }
    services::Shell_WriteString(",");
    WriteFixedMilli((imu != 0) ? imu->pitch_mdeg : 0);
    services::Shell_WriteString(",");
    WriteFixedMilli((imu != 0) ? imu->roll_mdeg : 0);
    services::Shell_WriteString(",");
    WriteInt32((imu != 0) ? imu->temp_centi_c : 0);
    services::Shell_WriteString(",");
    services::Shell_WriteUInt32(
        ((gray != 0) && gray->valid) ? 1U : 0U);
    services::Shell_WriteString(",");
    services::Shell_WriteUInt32(
        ((gray != 0) && (gray->sequence != 0U))
            ? static_cast<uint32_t>(now - gray->last_update_ms)
            : 0U);
    services::Shell_WriteString(",");
    services::Shell_WriteUInt32(
        (gray != 0) ? gray->error_count : 0U);
    for (uint32_t channel = 0U;
         channel < drivers::GRAYSCALE_CHANNEL_COUNT;
         channel++) {
        services::Shell_WriteString(",");
        services::Shell_WriteUInt32(
            (gray != 0) ? gray->raw[channel] : 0U);
    }
    services::Shell_WriteString("\n");
}

void TelemTask(void)
{
    if (!g_telemEnabled) {
        return;
    }

    const uint32_t now = services::Time_Millis();
    if (!services::Time_HasElapsed(g_telemLastUpdateMs, g_telemPeriodMs)) {
        return;
    }
    g_telemLastUpdateMs = now;

    /* Drop data if TX ring is nearly full to avoid blocking */
    if (services::DebugUart_GetTxPending() > 3000U) {
        return;
    }

    if (!g_telemHeaderSent) {
        TelemSendHeader();
        g_telemHeaderSent = true;
    }

    TelemSendData();
}

drivers::DriverStatus TelemEnsureTask(void)
{
    if (g_telemTaskRegistered) {
        return drivers::DRIVER_OK;
    }

    const services::SchedulerStatus status = services::Scheduler_AddTask(
        "telem",
        TelemTask,
        10U,
        0U,
        &g_telemTaskId);
    if (status != services::SCHEDULER_OK) {
        return SchedulerStatusToDriverStatus(status);
    }

    g_telemTaskRegistered = true;
    return SchedulerStatusToDriverStatus(
        services::Scheduler_EnableTask(g_telemTaskId, false));
}

drivers::DriverStatus TelemSetEnabled(bool enabled)
{
    if (!enabled) {
        g_telemEnabled = false;
        if (g_telemTaskRegistered) {
            (void) services::Scheduler_EnableTask(g_telemTaskId, false);
        }
        return drivers::DRIVER_OK;
    }

    const drivers::DriverStatus status = TelemEnsureTask();
    if (status != drivers::DRIVER_OK) {
        return status;
    }

    const drivers::DriverStatus enable_status = SchedulerStatusToDriverStatus(
        services::Scheduler_EnableTask(g_telemTaskId, true));
    if (enable_status != drivers::DRIVER_OK) {
        g_telemEnabled = false;
        return enable_status;
    }

    g_telemHeaderSent = false;
    g_telemLastUpdateMs = services::Time_Millis();
    g_telemEnabled = true;
    return drivers::DRIVER_OK;
}

drivers::DriverStatus TelemStart(TelemProfile profile, uint32_t period)
{
    const TelemProfile previous_profile = g_telemProfile;
    const uint32_t previous_period = g_telemPeriodMs;
    g_telemProfile = profile;
    g_telemPeriodMs = period;
    const drivers::DriverStatus status = TelemSetEnabled(true);
    if (status != drivers::DRIVER_OK) {
        g_telemProfile = previous_profile;
        g_telemPeriodMs = previous_period;
    }
    return status;
}

void PrintTelemUsage(void)
{
    services::Shell_WriteLine("usage:");
    services::Shell_WriteLine("  telem on [period_ms 50..5000]");
    services::Shell_WriteLine("  telem on <profile> [period_ms 50..5000]");
    services::Shell_WriteLine(
        "  profiles: runtime competition motor heading heading_output");
    services::Shell_WriteLine(
        "    chassis feedback_state feedback_age line line_position");
    services::Shell_WriteLine(
        "    line_output line_quality line_state line_faults road_event");
    services::Shell_WriteLine(
        "    road_sequence road_phase turn_phase turn_rate turn_angle");
    services::Shell_WriteLine(
        "    turn_timing turn_speed accel gyro attitude imu_temperature");
    services::Shell_WriteLine(
        "    imu_state imu_age gray_raw gray_health gray_age fault uart");
    services::Shell_WriteLine("  telem off");
    services::Shell_WriteLine("  telem status");
}

void TelemCommand(int argc, const char * const argv[])
{
    if (argc < 2) {
        PrintTelemUsage();
        return;
    }

    if (StrEqual(argv[1], "on")) {
        TelemProfile profile = TELEM_PROFILE_FULL;
        uint32_t period = g_telemPeriodMs;
        if ((argc < 2) || (argc > 4)) {
            PrintTelemUsage();
            return;
        }
        if (argc == 3) {
            if (!ParseTelemProfile(argv[2], &profile)) {
                if ((!ParseUint32(argv[2], 5000U, &period)) ||
                    (period < 50U)) {
                    PrintTelemUsage();
                    return;
                }
            }
        } else if (argc == 4) {
            if ((!ParseTelemProfile(argv[2], &profile)) ||
                (!ParseUint32(argv[3], 5000U, &period)) ||
                (period < 50U)) {
                PrintTelemUsage();
                return;
            }
        }
        const drivers::DriverStatus status = TelemStart(profile, period);
        services::Shell_WriteString("telem: ");
        services::Shell_WriteString(DriverStatusText(status));
        services::Shell_WriteString(" profile=");
        services::Shell_WriteString(TelemProfileText(g_telemProfile));
        services::Shell_WriteString(" period_ms=");
        services::Shell_WriteUInt32(g_telemPeriodMs);
        services::Shell_WriteString("\r\n");
        return;
    }

    if (StrEqual(argv[1], "off")) {
        if (argc != 2) {
            PrintTelemUsage();
            return;
        }
        const drivers::DriverStatus status = TelemSetEnabled(false);
        WriteStatusLine("telem off: ", status);
        return;
    }

    if (StrEqual(argv[1], "status")) {
        if (argc != 2) {
            PrintTelemUsage();
            return;
        }
        services::Shell_WriteString("telem enabled=");
        services::Shell_WriteUInt32(g_telemEnabled ? 1U : 0U);
        services::Shell_WriteString(" profile=");
        services::Shell_WriteString(TelemProfileText(g_telemProfile));
        services::Shell_WriteString(" period_ms=");
        services::Shell_WriteUInt32(g_telemPeriodMs);
        services::Shell_WriteString("\r\n");
        return;
    }

    PrintTelemUsage();
}

void EstopCommand(int argc, const char * const argv[])
{
    (void) argv;
    if (argc != 1) {
        services::Shell_WriteLine("usage: estop");
        return;
    }
    WriteStatusLine("estop: ", app::App_EmergencyStop());
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
#if FEATURE_ENABLE_DEBUG_UART
    (void) services::Shell_RegisterCommand("txstat",
                                           "Debug UART TX/RX queue stats",
                                           TxStatCommand);
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
    (void) services::Shell_RegisterCommand(
        "battery",
        "3S1P runtime battery SOC: status|full|reset|log",
        BatteryCommand);
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
        "LoRa UART and framed protocol diagnostics",
        LoraCommand);
#endif
#if FEATURE_ENABLE_CAN
    (void) services::Shell_RegisterCommand(
        "can",
        "CANFD1 classic CAN diagnostics: status|mode|send|read|watch",
        CanCommand);
#if FEATURE_ENABLE_DM_G6220_CAN
    (void) services::Shell_RegisterCommand(
        "dm",
        "DM-G6220: status|probe|enable|position|speed|hold|disable|clear|zero",
        DmCommand);
#endif
#endif
#if FEATURE_ENABLE_MOTOR_DRIVER
    (void) services::Shell_RegisterCommand(
        "motor",
        "MotorDriver: status|bus|ping|info|reg|set|run",
        MotorCommand);
    (void) services::Shell_RegisterCommand(
        "chassis",
        "Chassis: status|stat|stop|wheel <l_rpm> <r_rpm>|vel <mm_s> <mdeg_s>",
        ChassisCommand);
#endif
#if FEATURE_ENABLE_IMU && FEATURE_ENABLE_MOTOR_DRIVER
    (void) services::Shell_RegisterCommand(
        "heading",
        "Heading: status|hold|turn|distance|profile|stop",
        HeadingCommand);
    (void) services::Shell_RegisterCommand(
        "run",
        "ActionRunner: add <op> <p1> <p2> <until> <ons> <ont>|clear|start|cancel|status|dump",
        RunCommand);
#endif
#if FEATURE_ENABLE_GRAYSCALE && FEATURE_ENABLE_MOTOR_DRIVER
    (void) services::Shell_RegisterCommand(
        "lf",
        "LineFollow: status|cal|start|stop|kp|kd|maxcorr|slew|losthold|losttimeout",
        LFCommand);
#endif
#if FEATURE_ENABLE_GRAYSCALE && FEATURE_ENABLE_MOTOR_DRIVER && \
    FEATURE_ENABLE_IMU
    (void) services::Shell_RegisterCommand(
        "road",
        "Road events: status|event|clear|mode|auto|turn",
        RoadCommand);
#endif
    (void) services::Shell_RegisterCommand(
        "comp",
        "Competition: arm|select <n>|start [n]|stop|status",
        CompCommand);
    (void) services::Shell_RegisterCommand(
        "estop",
        "Software-wide motion stop",
        EstopCommand);
    (void) services::Shell_RegisterCommand(
        "telem",
        "Telemetry (FireWater/VOFA+): on [period_ms]|off|status",
        TelemCommand);
    (void) services::Shell_RegisterCommand(
        "seq",
        "Sequence: list|save <n>|load <n>|del <n>|run <n>",
        SeqCommand);
#if FEATURE_ENABLE_SHELL_DIAGNOSTICS
    (void) services::Shell_RegisterCommand(
        "i2c",
        "I2C diag: list|status|recover|scan|probe|read|write|test",
        I2cCommand);
#endif
}

} /* namespace app */
