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

#endif /* CONFIG_FEATURE_CONFIG_H_ */
