#include "app/app_shell.h"

#include <stdint.h>

#include "board/board_buzzer.h"
#include "board/board_button.h"
#include "board/board_config.h"
#include "board/board_fram.h"
#include "board/board_ina219.h"
#include "board/board_led.h"
#include "board/board_lora.h"
#include "config/feature_config.h"
#include "drivers/common/driver_status.h"
#include "services/shell.h"
#include "ti_msp_dl_config.h"

namespace app {
namespace {

static const uint16_t kFramShellMaxReadBytes = 32U;
static const uint16_t kLoraShellMaxReadBytes = 64U;

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
#if FEATURE_ENABLE_INA219
    (void) services::Shell_RegisterCommand(
        "ina219",
        "INA219: status|scan|addr|recover|config|read|raw|reg",
        Ina219Command);
#endif
#if FEATURE_ENABLE_LORA
    (void) services::Shell_RegisterCommand(
        "lora",
        "LoRa UART: status|send|line|hex|read|clear|test",
        LoraCommand);
#endif
    (void) services::Shell_RegisterCommand("adc",
                                           "Read ADC placeholder",
                                           AdcCommand);
    (void) services::Shell_RegisterCommand("pwm",
                                           "Set PWM placeholder: pwm 0..100",
                                           PwmCommand);
}

} /* namespace app */