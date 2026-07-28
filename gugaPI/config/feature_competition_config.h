#ifndef CONFIG_FEATURE_COMPETITION_CONFIG_H_
#define CONFIG_FEATURE_COMPETITION_CONFIG_H_

/*
 * Competition profile.
 *
 * Keep only peripherals used by the current bench build:
 * - Buttons remain available for input diagnostics; competition start is
 *   rejected while differential drive is unavailable.
 * - FRAM stores the motor/chassis parameters used during commissioning.
 * - Grayscale remains available for sensor diagnostics. INA219, OLED and
 *   GY931 are temporarily disabled.
 * - Local DRV8876 MR motor drive and its hardware-QEI encoder. Differential
 *   chassis/competition motion remains disabled in this bench-only build.
 * - ICM-45686 remains available for IMU diagnostics; heading motion is
 *   disabled.
 * - Status LED remains available; buzzer is initialized but kept silent.
 * - CAN and JY-ME02 remain available for encoder integration diagnostics.
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
#define FEATURE_ENABLE_INA219              (0U)
#define FEATURE_ENABLE_LORA                (0U)
#define FEATURE_ENABLE_CAN                 (1U)
#define FEATURE_ENABLE_JYME02_CAN          (1U)
#define FEATURE_ENABLE_MOTOR_DRIVER        (0U)
#define FEATURE_ENABLE_OLED                (0U)
#define FEATURE_ENABLE_IMU                 (1U)
#define FEATURE_ENABLE_MOTOR               (1U)
#define FEATURE_ENABLE_ENCODER             (1U)
#define FEATURE_LOCAL_MOTOR_RIGHT_ONLY      (1U)
#define FEATURE_ENABLE_GY931               (0U)
#define FEATURE_ENABLE_BUTTON_CHASSIS_TEST (0U)
#define FEATURE_ENABLE_GRAYSCALE           (1U)

#define FEATURE_ENABLE_BUTTON_EVENT_LOG    (0U)
#define FEATURE_ENABLE_SCHEDULER_STATS     (1U)
#define FEATURE_ENABLE_SHELL_DIAGNOSTICS   (0U)

#endif /* CONFIG_FEATURE_COMPETITION_CONFIG_H_ */
