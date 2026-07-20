#ifndef CONFIG_DEBUG_CONFIG_H_
#define CONFIG_DEBUG_CONFIG_H_

#include "config/feature_config.h"
#include "services/fault.h"

#define DEBUG_ENABLE_ASSERT              (1U)
#define DEBUG_ENABLE_RUNTIME_CHECKS      (1U)

#define DEBUG_UART_RX_BUFFER_SIZE        (128U)
/* Sized to hold the largest synchronous burst (the `help` listing is ~1.5 KB)
 * so non-blocking WriteChar does not drop boot/help output. SRAM is plentiful
 * (SRAM_BANK0 uses ~3 KB of 64 KB). */
#define DEBUG_UART_TX_BUFFER_SIZE        (4096U)
#define DEBUG_SHELL_LINE_BUFFER_SIZE     (96U)
#define DEBUG_SHELL_MAX_ARGS             (8U)
#define DEBUG_SHELL_MAX_COMMANDS         (24U)
#ifndef FEATURE_ENABLE_SHELL_ECHO
#define FEATURE_ENABLE_SHELL_ECHO        (1U)
#endif

#define DEBUG_SHELL_ECHO                 (FEATURE_ENABLE_SHELL_ECHO)

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
