#include "app/app_shell.h"

#include <stdint.h>

#include "board/board_buzzer.h"
#include "board/board_config.h"
#include "board/board_led.h"
#include "services/shell.h"
#include "ti_msp_dl_config.h"

namespace app {
namespace {

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

bool ParsePercent(const char *text, uint8_t *outValue)
{
    uint32_t value = 0U;

    if ((text == 0) || (outValue == 0) || (*text == '\0')) {
        return false;
    }

    while (*text != '\0') {
        if ((*text < '0') || (*text > '9')) {
            return false;
        }

        value = (value * 10U) + (uint32_t) (*text - '0');
        if (value > 100U) {
            return false;
        }
        text++;
    }

    *outValue = (uint8_t) value;
    return true;
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
    (void) services::Shell_RegisterCommand("adc",
                                           "Read ADC placeholder",
                                           AdcCommand);
    (void) services::Shell_RegisterCommand("pwm",
                                           "Set PWM placeholder: pwm 0..100",
                                           PwmCommand);
}

} /* namespace app */