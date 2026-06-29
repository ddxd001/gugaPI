#include "services/log.h"

#include "config/feature_config.h"

namespace services {

void Log_Init(void)
{
}

void Log_WriteChar(char ch)
{
    (void) ch;
}

void Log_WriteString(const char *text)
{
#if FEATURE_ENABLE_LOG
    if (text == 0) {
        return;
    }

    while (*text != '\0') {
        Log_WriteChar(*text);
        text++;
    }
#else
    (void) text;
#endif
}

void Log_Info(const char *text)
{
    Log_WriteString(text);
}

void Log_Error(const char *text)
{
    Log_WriteString(text);
}

} /* namespace services */