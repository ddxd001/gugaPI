#ifndef SERVICES_SHELL_H_
#define SERVICES_SHELL_H_

#include <stdbool.h>
#include <stdint.h>

namespace services {

typedef void (*ShellCommandHandler)(int argc, const char * const argv[]);

void Shell_Init(void);
bool Shell_RegisterCommand(const char *name,
                           const char *help,
                           ShellCommandHandler handler);
void Shell_Process(void);
void Shell_WriteChar(char ch);
void Shell_WriteString(const char *text);
void Shell_WriteLine(const char *text);
void Shell_WriteUInt32(uint32_t value);
void Shell_PrintPrompt(void);

} /* namespace services */

#endif /* SERVICES_SHELL_H_ */