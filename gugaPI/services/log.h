#ifndef SERVICES_LOG_H_
#define SERVICES_LOG_H_

#include "config/feature_config.h"

namespace services {

enum LogLevel {
    LOG_LEVEL_DEBUG = 0,
    LOG_LEVEL_INFO,
    LOG_LEVEL_WARN,
    LOG_LEVEL_ERROR
};

void Log_Init(void);
void Log_WriteChar(char ch);
void Log_WriteString(const char *text);
void Log_Write(LogLevel level, const char *text);
void Log_Debug(const char *text);
void Log_Info(const char *text);
void Log_Warn(const char *text);
void Log_Error(const char *text);

} /* namespace services */

#if FEATURE_ENABLE_LOG
#define LOG_INFO(text)  services::Log_Info(text)
#define LOG_WARN(text)  services::Log_Warn(text)
#define LOG_ERROR(text) services::Log_Error(text)
#else
#define LOG_INFO(text)  do { (void) sizeof(text); } while (0)
#define LOG_WARN(text)  do { (void) sizeof(text); } while (0)
#define LOG_ERROR(text) do { (void) sizeof(text); } while (0)
#endif

#if FEATURE_ENABLE_LOG && FEATURE_ENABLE_LOG_DEBUG
#define LOG_DEBUG(text) services::Log_Debug(text)
#else
#define LOG_DEBUG(text) do { (void) sizeof(text); } while (0)
#endif

#endif /* SERVICES_LOG_H_ */