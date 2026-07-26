#ifndef CONFIG_FEATURE_CONFIG_H_
#define CONFIG_FEATURE_CONFIG_H_

/*
 * Select the build-time feature profile here.
 *
 * Competition profile keeps only peripherals used by the current robot path
 * and suppresses background debug chatter. Development profile keeps the
 * broader bring-up surface for bench diagnostics.
 */
#define FEATURE_PROFILE_COMPETITION (0U)

#if FEATURE_PROFILE_COMPETITION
#include "config/feature_competition_config.h"
#else
#include "config/feature_development_config.h"
#endif

#ifndef FEATURE_SHELL_USE_LORA_UART
#define FEATURE_SHELL_USE_LORA_UART (0U)
#endif

#if FEATURE_SHELL_USE_LORA_UART && !FEATURE_ENABLE_SHELL
#error "LoRa UART Shell transport requires FEATURE_ENABLE_SHELL"
#endif

#if FEATURE_SHELL_USE_LORA_UART && !FEATURE_ENABLE_LORA
#error "LoRa UART Shell transport requires FEATURE_ENABLE_LORA"
#endif

#if FEATURE_SHELL_USE_LORA_UART && !FEATURE_ENABLE_DEBUG_UART
#error "LoRa UART Shell transport requires FEATURE_ENABLE_DEBUG_UART"
#endif

#endif /* CONFIG_FEATURE_CONFIG_H_ */
