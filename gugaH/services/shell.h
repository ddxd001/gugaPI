#ifndef GUGAH_SERVICES_SHELL_H_
#define GUGAH_SERVICES_SHELL_H_

#include <stdbool.h>
#include <stdint.h>

namespace services {

typedef void (*ShellHandler)(int argc, const char * const argv[]);

void Shell_Init(void);
bool Shell_Register(const char *name,
                    const char *help,
                    ShellHandler handler);
void Shell_Process(void);
void Shell_Write(const char *text);
void Shell_WriteLine(const char *text);
void Shell_WriteInt(int32_t value);

} /* namespace services */

#endif /* GUGAH_SERVICES_SHELL_H_ */
