#ifndef CONFIG_DEBUG_CONFIG_H_
#define CONFIG_DEBUG_CONFIG_H_

#include "services/fault.h"

#define DEBUG_ENABLE_ASSERT             (1U)
#define DEBUG_ENABLE_RUNTIME_CHECKS     (1U)

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