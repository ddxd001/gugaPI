#ifndef CONFIG_DEBUG_CONFIG_H_
#define CONFIG_DEBUG_CONFIG_H_

#include "services/fault.h"

#define DEBUG_ENABLE_ASSERT              (1U)
#define DEBUG_ENABLE_RUNTIME_CHECKS      (1U)

#define DEBUG_UART_RX_BUFFER_SIZE        (128U)
#define DEBUG_SHELL_LINE_BUFFER_SIZE     (96U)
#define DEBUG_SHELL_MAX_ARGS             (8U)
#define DEBUG_SHELL_MAX_COMMANDS         (12U)
#define DEBUG_SHELL_ECHO                 (1U)

#if DEBUG_ENABLE_ASSERT
#define APP_ASSERT(expr)                                                   \
    do {                                                                   \
        if (!(expr)) {                                                     \
            services::Fault_Panic(services::FAULT_ASSERT);                 \
        }                                                                  \
    } while (0)
#else
#define APP_ASSERT(expr) do { (void) sizeof(expr); } while (0)
#endif

#endif /* CONFIG_DEBUG_CONFIG_H_ */