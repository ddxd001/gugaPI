#ifndef CONFIG_FEATURE_COMPETITION_CONFIG_H_
#define CONFIG_FEATURE_COMPETITION_CONFIG_H_

/*
 * Competition profile.
 *
 * Keep only peripherals used by the current robot:
 * - Buttons for competition start + OLED page selection.
 * - FRAM for persisted chassis/config parameters.
 * - INA219, GY931, grayscale sensor, and OLED.
 * - MotorDriver link for chassis control.
 * - ICM-45686 IMU for heading closed-loop (HOLD/TURN).
 * - Status LED + buzzer for competition state indication.
 * - Debug UART shell stays enabled for command/control, but startup prompt,
 *   echo, and debug logs are muted.
 */

#define FEATURE_ENABLE_LOG                 (1U)
#define FEATURE_ENABLE_LOG_DEBUG           (0U)
#define FEATURE_ENABLE_SHELL               (1U)
#define FEATURE_ENABLE_SHELL_BANNER        (0U)
#define FEATURE_ENABLE_SHELL_PROMPT        (0U)
#define FEATURE_ENABLE_SHELL_ECHO          (0U)
#define FEATURE_ENABLE_DEBUG_UART          (1U)
#define FEATURE_ENABLE_UART_COUNTER_TEST   (0U)

#define FEATURE_ENABLE_STATUS_LED          (1U)
#define FEATURE_ENABLE_LED_TEST            (0U)
#define FEATURE_ENABLE_BUZZER              (1U)
#define FEATURE_ENABLE_BUZZER_TEST         (0U)
#define FEATURE_ENABLE_BUTTONS             (1U)

#define FEATURE_ENABLE_FRAM                (1U)
#define FEATURE_ENABLE_INA219              (1U)
#define FEATURE_ENABLE_LORA                (0U)
#define FEATURE_ENABLE_MOTOR_DRIVER        (1U)
#define FEATURE_ENABLE_OLED                (1U)
#define FEATURE_ENABLE_IMU                 (1U)
#define FEATURE_ENABLE_MOTOR               (0U)
#define FEATURE_ENABLE_ENCODER             (0U)
#define FEATURE_ENABLE_GY931               (1U)
#define FEATURE_ENABLE_BUTTON_CHASSIS_TEST (0U)
#define FEATURE_ENABLE_GRAYSCALE           (1U)

#define FEATURE_ENABLE_BUTTON_OLED_LOG     (0U)
#define FEATURE_ENABLE_SCHEDULER_STATS     (1U)
#define FEATURE_ENABLE_SHELL_DIAGNOSTICS   (0U)

#endif /* CONFIG_FEATURE_COMPETITION_CONFIG_H_ */
