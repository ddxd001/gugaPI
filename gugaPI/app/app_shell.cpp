#include "app/app_shell.h"

#include <stdint.h>

#include "board/board_buzzer.h"
#include "board/board_button.h"
#include "board/board_config.h"
#include "board/board_fram.h"
#include "board/board_led.h"
#include "config/feature_config.h"
#include "drivers/common/driver_status.h"
#include "services/shell.h"
#include "ti_msp_dl_config.h"

namespace app {
namespace {

static const uint16_t kFramShellMaxReadBytes = 32U;

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

void LedCommand(int argc, const char * const argv[])
{
    if (argc != 2) {
        services::Shell_WriteLine("usage: led on|off");
        return;
    }

    if (!board::Board_StatusLedIsReady()) {
        services::Shell_WriteLine("led: not ready");
        return;
    }

    if (StrEqual(argv[1], "on")) {
        (void) board::Board_StatusLedOn();
        services::Shell_WriteLine("led on");
        return;
    }

    if (StrEqual(argv[1], "off")) {
        (void) board::Board_StatusLedOff();
        services::Shell_WriteLine("led off");
        return;
    }

    services::Shell_WriteLine("usage: led on|off");
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
                                           "Control status LED: led on|off",
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
    (void) services::Shell_RegisterCommand("adc",
                                           "Read ADC placeholder",
                                           AdcCommand);
    (void) services::Shell_RegisterCommand("pwm",
                                           "Set PWM placeholder: pwm 0..100",
                                           PwmCommand);
}

} /* namespace app */