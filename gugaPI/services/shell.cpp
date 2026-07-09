#include "services/shell.h"

#include "config/debug_config.h"
#include "config/feature_config.h"
#include "services/debug_uart.h"

namespace services {
namespace {

struct ShellCommandSlot {
    const char *name;
    const char *help;
    ShellCommandHandler handler;
    bool used;
};

static ShellCommandSlot g_commands[DEBUG_SHELL_MAX_COMMANDS];
static char g_lineBuffer[DEBUG_SHELL_LINE_BUFFER_SIZE];
static uint32_t g_lineLength = 0U;
static bool g_ignoreNextLf = false;

bool IsSpace(char ch)
{
    return (ch == ' ') || (ch == '\t');
}

bool IsPrintable(uint8_t value)
{
    return (value >= 32U) && (value <= 126U);
}

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

void PrintHelp(void)
{
    Shell_WriteLine("commands:");
    Shell_WriteLine("  help                 Show this help");

    for (uint32_t i = 0U; i < DEBUG_SHELL_MAX_COMMANDS; i++) {
        if (!g_commands[i].used) {
            continue;
        }

        Shell_WriteString("  ");
        Shell_WriteString(g_commands[i].name);
        if (g_commands[i].help != 0) {
            Shell_WriteString(" - ");
            Shell_WriteString(g_commands[i].help);
        }
        Shell_WriteString("\r\n");
    }
}

int TokenizeLine(char *line, const char **argv, int maxArgs)
{
    int argc = 0;
    char *cursor = line;

    while ((*cursor != '\0') && (argc < maxArgs)) {
        while (IsSpace(*cursor)) {
            *cursor = '\0';
            cursor++;
        }

        if (*cursor == '\0') {
            break;
        }

        argv[argc] = cursor;
        argc++;

        while ((*cursor != '\0') && (!IsSpace(*cursor))) {
            cursor++;
        }
    }

    return argc;
}

void ExecuteLine(char *line)
{
    const char *argv[DEBUG_SHELL_MAX_ARGS];
    const int argc = TokenizeLine(line, argv, DEBUG_SHELL_MAX_ARGS);

    if (argc == 0) {
        return;
    }

    if (StrEqual(argv[0], "help")) {
        PrintHelp();
        return;
    }

    for (uint32_t i = 0U; i < DEBUG_SHELL_MAX_COMMANDS; i++) {
        if (g_commands[i].used && StrEqual(argv[0], g_commands[i].name)) {
            g_commands[i].handler(argc, argv);
            return;
        }
    }

    Shell_WriteString("unknown command: ");
    Shell_WriteLine(argv[0]);
}

void HandleLineEnd(void)
{
#if DEBUG_SHELL_ECHO
    Shell_WriteString("\r\n");
#endif

    g_lineBuffer[g_lineLength] = '\0';
    ExecuteLine(g_lineBuffer);
    g_lineLength = 0U;
    Shell_PrintPrompt();
}

void HandleBackspace(void)
{
    if (g_lineLength == 0U) {
        return;
    }

    g_lineLength--;
#if DEBUG_SHELL_ECHO
    Shell_WriteString("\b \b");
#endif
}

void HandleInputByte(uint8_t value)
{
    if (value == '\r') {
        g_ignoreNextLf = true;
        HandleLineEnd();
        return;
    }

    if (value == '\n') {
        if (g_ignoreNextLf) {
            g_ignoreNextLf = false;
            return;
        }
        HandleLineEnd();
        return;
    }

    g_ignoreNextLf = false;

    if ((value == 0x08U) || (value == 0x7FU)) {
        HandleBackspace();
        return;
    }

    if (!IsPrintable(value)) {
        return;
    }

    if (g_lineLength >= (DEBUG_SHELL_LINE_BUFFER_SIZE - 1U)) {
        Shell_WriteChar('\a');
        return;
    }

    g_lineBuffer[g_lineLength] = (char) value;
    g_lineLength++;

#if DEBUG_SHELL_ECHO
    Shell_WriteChar((char) value);
#endif
}

} /* namespace */

void Shell_Init(void)
{
#if FEATURE_ENABLE_SHELL
    for (uint32_t i = 0U; i < DEBUG_SHELL_MAX_COMMANDS; i++) {
        g_commands[i].name = 0;
        g_commands[i].help = 0;
        g_commands[i].handler = 0;
        g_commands[i].used = false;
    }

    g_lineLength = 0U;
    g_ignoreNextLf = false;

#if FEATURE_ENABLE_SHELL_BANNER
    if (DebugUart_IsReady()) {
        Shell_WriteString("\r\ngugaPI debug shell\r\n");
        Shell_PrintPrompt();
    }
#endif
#endif
}

bool Shell_RegisterCommand(const char *name,
                           const char *help,
                           ShellCommandHandler handler)
{
#if FEATURE_ENABLE_SHELL
    if ((name == 0) || (handler == 0)) {
        return false;
    }

    for (uint32_t i = 0U; i < DEBUG_SHELL_MAX_COMMANDS; i++) {
        if (!g_commands[i].used) {
            g_commands[i].name = name;
            g_commands[i].help = help;
            g_commands[i].handler = handler;
            g_commands[i].used = true;
            return true;
        }
    }
#else
    (void) name;
    (void) help;
    (void) handler;
#endif

    return false;
}

void Shell_Process(void)
{
#if FEATURE_ENABLE_SHELL
    uint8_t byte = 0U;

    while (DebugUart_ReadByte(&byte)) {
        HandleInputByte(byte);
    }
#endif
}

void Shell_WriteChar(char ch)
{
    DebugUart_WriteChar(ch);
}

void Shell_WriteString(const char *text)
{
    DebugUart_WriteString(text);
}

void Shell_WriteLine(const char *text)
{
    Shell_WriteString(text);
    Shell_WriteString("\r\n");
}

void Shell_WriteUInt32(uint32_t value)
{
    DebugUart_WriteUInt32(value);
}

void Shell_PrintPrompt(void)
{
#if FEATURE_ENABLE_SHELL && FEATURE_ENABLE_SHELL_PROMPT
    Shell_WriteString("> ");
#endif
}

} /* namespace services */
