#ifndef SERVICES_LOG_H_
#define SERVICES_LOG_H_

namespace services {

void Log_Init(void);
void Log_WriteChar(char ch);
void Log_WriteString(const char *text);
void Log_Info(const char *text);
void Log_Error(const char *text);

} /* namespace services */

#endif /* SERVICES_LOG_H_ */