#ifndef CONFIG_FEATURE_CONFIG_H_
#define CONFIG_FEATURE_CONFIG_H_

/*
 * Select the build-time feature profile here.
 *
 * Competition profile keeps only peripherals used by the current robot path
 * and suppresses background debug chatter. Development profile keeps the
 * broader bring-up surface for bench diagnostics.
 */
#ifndef FEATURE_PROFILE_COMPETITION
#define FEATURE_PROFILE_COMPETITION (0U)
#endif

#if FEATURE_PROFILE_COMPETITION
#include "config/feature_competition_config.h"
#else
#include "config/feature_development_config.h"
#endif

#if FEATURE_ENABLE_MOTOR_DRIVER && FEATURE_ENABLE_MOTOR
#error "Select either the external MotorDriver backend or the local motor backend"
#endif

#if FEATURE_ENABLE_MOTOR && !FEATURE_ENABLE_ENCODER
#error "Local closed-loop motor control requires wheel encoders"
#endif

#ifndef FEATURE_LOCAL_MOTOR_RIGHT_ONLY
#define FEATURE_LOCAL_MOTOR_RIGHT_ONLY (0U)
#endif

#define FEATURE_ENABLE_LOCAL_MOTOR (FEATURE_ENABLE_MOTOR && FEATURE_ENABLE_ENCODER)
#define FEATURE_ENABLE_CHASSIS     (FEATURE_ENABLE_MOTOR_DRIVER || FEATURE_ENABLE_LOCAL_MOTOR)

#if FEATURE_LOCAL_MOTOR_RIGHT_ONLY && !FEATURE_ENABLE_LOCAL_MOTOR
#error "Right-only motor mode requires the local motor and encoder backend"
#endif

#if FEATURE_ENABLE_LOCAL_MOTOR && !FEATURE_LOCAL_MOTOR_RIGHT_ONLY
#error "The current local motor backend implements only the right-side MR bench"
#endif

/* Heading, line following, road actions and competition sequences all assume
 * two independently driven wheels. Keep the chassis state/shell for the
 * explicit `chassis wheel 0 <right_rpm>` bench path, but do not expose those
 * differential-drive controllers in the right-only build. */
#define FEATURE_ENABLE_DIFFERENTIAL_CHASSIS \
    (FEATURE_ENABLE_CHASSIS && !FEATURE_LOCAL_MOTOR_RIGHT_ONLY)

#endif /* CONFIG_FEATURE_CONFIG_H_ */
