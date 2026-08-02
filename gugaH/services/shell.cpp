#include "services/shell.h"

#include <stdint.h>

#include "services/debug_uart.h"

namespace services {
namespace {

static const uint8_t kMaximumCommands = 16U;
static const uint8_t kMaximumArguments = 8U;
static const uint8_t kLineSize = 96U;

struct Command {
    const char *name;
    const char *help;
    ShellHandler handler;
};

Command g_commands[kMaximumCommands] = {};
char g_line[kLineSize] = {};
uint8_t g_length = 0U;

bool Equal(const char *left, const char *right)
{
    while ((*left != '\0') && (*right != '\0') && (*left == *right)) {
        left++;
        right++;
    }
    return (*left == '\0') && (*right == '\0');
}

void Help(void)
{
    Shell_WriteLine("commands:");
    for (uint8_t i = 0U; i < kMaximumCommands; i++) {
        if (g_commands[i].handler != 0) {
            Shell_Write("  ");
            Shell_Write(g_commands[i].name);
            Shell_Write(" - ");
            Shell_WriteLine(g_commands[i].help);
        }
    }
}

void Execute(void)
{
    const char *argv[kMaximumArguments] = {};
    int argc = 0;
    char *cursor = g_line;
    while ((*cursor != '\0') && (argc < kMaximumArguments)) {
        while ((*cursor == ' ') || (*cursor == '\t')) {
            *cursor++ = '\0';
        }
        if (*cursor == '\0') {
            break;
        }
        argv[argc++] = cursor;
        while ((*cursor != '\0') &&
               (*cursor != ' ') && (*cursor != '\t')) {
            cursor++;
        }
    }
    if (argc == 0) {
        return;
    }
    if (Equal(argv[0], "help")) {
        Help();
        return;
    }
    for (uint8_t i = 0U; i < kMaximumCommands; i++) {
        if ((g_commands[i].handler != 0) &&
            Equal(argv[0], g_commands[i].name)) {
            g_commands[i].handler(argc, argv);
            return;
        }
    }
    Shell_WriteLine("ERR unknown");
}

} /* namespace */

void Shell_Init(void)
{
    for (uint8_t i = 0U; i < kMaximumCommands; i++) {
        g_commands[i] = {};
    }
    g_length = 0U;
    Shell_WriteLine("\r\ngugaH H2-H7 ready; type help");
}

bool Shell_Register(const char *name,
                    const char *help,
                    ShellHandler handler)
{
    if ((name == 0) || (help == 0) || (handler == 0)) {
        return false;
    }
    for (uint8_t i = 0U; i < kMaximumCommands; i++) {
        if (g_commands[i].handler == 0) {
            g_commands[i] = { name, help, handler };
            return true;
        }
    }
    return false;
}

void Shell_Process(void)
{
    uint8_t value = 0U;
    while (DebugUart_ReadByte(&value)) {
        if ((value == '\r') || (value == '\n')) {
            if (g_length != 0U) {
                g_line[g_length] = '\0';
                Execute();
                g_length = 0U;
            }
        } else if ((value == 0x08U) || (value == 0x7FU)) {
            if (g_length != 0U) {
                g_length--;
            }
        } else if ((value >= 32U) && (value <= 126U) &&
                   (g_length < (kLineSize - 1U))) {
            g_line[g_length++] = static_cast<char>(value);
        }
    }
}

void Shell_Write(const char *text)
{
    DebugUart_WriteString(text);
}

void Shell_WriteLine(const char *text)
{
    DebugUart_WriteString(text);
    DebugUart_WriteString("\r\n");
}

void Shell_WriteInt(int32_t value)
{
    char buffer[12] = {};
    uint8_t count = 0U;
    uint32_t magnitude = (value < 0)
        ? static_cast<uint32_t>(-(static_cast<int64_t>(value)))
        : static_cast<uint32_t>(value);
    do {
        buffer[count++] =
            static_cast<char>('0' + (magnitude % 10U));
        magnitude /= 10U;
    } while ((magnitude != 0U) && (count < sizeof(buffer)));
    if (value < 0) {
        DebugUart_WriteChar('-');
    }
    while (count != 0U) {
        DebugUart_WriteChar(buffer[--count]);
    }
}

} /* namespace services */
