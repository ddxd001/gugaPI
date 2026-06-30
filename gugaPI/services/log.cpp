#include "services/log.h"

#include "config/feature_config.h"
#include "services/debug_uart.h"

namespace services {

void Log_Init(void)
{
}

void Log_WriteChar(char ch)
{
#if FEATURE_ENABLE_LOG
    DebugUart_WriteChar(ch);
#else
    (void) ch;
#endif
}

void Log_WriteString(const char *text)
{
#if FEATURE_ENABLE_LOG
    DebugUart_WriteString(text);
#else
    (void) text;
#endif
}

void Log_Write(LogLevel level, const char *text)
{
#if FEATURE_ENABLE_LOG
    if ((text == 0) || (!DebugUart_IsReady())) {
        return;
    }

    switch (level) {
    case LOG_LEVEL_DEBUG:
#if FEATURE_ENABLE_LOG_DEBUG
        DebugUart_WriteString("[DEBUG] ");
#else
        return;
#endif
        break;
    case LOG_LEVEL_INFO:
        DebugUart_WriteString("[INFO] ");
        break;
    case LOG_LEVEL_WARN:
        DebugUart_WriteString("[WARN] ");
        break;
    case LOG_LEVEL_ERROR:
        DebugUart_WriteString("[ERROR] ");
        break;
    default:
        DebugUart_WriteString("[LOG] ");
        break;
    }

    DebugUart_WriteString(text);
    DebugUart_WriteString("\r\n");
#else
    (void) level;
    (void) text;
#endif
}

void Log_Debug(const char *text)
{
    Log_Write(LOG_LEVEL_DEBUG, text);
}

void Log_Info(const char *text)
{
    Log_Write(LOG_LEVEL_INFO, text);
}

void Log_Warn(const char *text)
{
    Log_Write(LOG_LEVEL_WARN, text);
}

void Log_Error(const char *text)
{
    Log_Write(LOG_LEVEL_ERROR, text);
}

} /* namespace services */