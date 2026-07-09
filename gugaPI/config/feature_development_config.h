#ifndef CONFIG_FEATURE_DEVELOPMENT_CONFIG_H_
#define CONFIG_FEATURE_DEVELOPMENT_CONFIG_H_

/* Full bring-up profile for bench diagnostics. */

#define FEATURE_ENABLE_LOG                 (1U)
#define FEATURE_ENABLE_LOG_DEBUG           (1U)
#define FEATURE_ENABLE_SHELL               (1U)
#define FEATURE_ENABLE_SHELL_BANNER        (1U)
#define FEATURE_ENABLE_SHELL_PROMPT        (1U)
#define FEATURE_ENABLE_SHELL_ECHO          (1U)
#define FEATURE_ENABLE_DEBUG_UART          (1U)
#define FEATURE_ENABLE_UART_COUNTER_TEST   (0U)

#define FEATURE_ENABLE_STATUS_LED          (1U)
#define FEATURE_ENABLE_LED_TEST            (0U)
#define FEATURE_ENABLE_BUZZER              (1U)
#define FEATURE_ENABLE_BUZZER_TEST         (0U)
#define FEATURE_ENABLE_BUTTONS             (1U)

#define FEATURE_ENABLE_FRAM                (1U)
#define FEATURE_ENABLE_INA219              (1U)
#define FEATURE_ENABLE_LORA                (1U)
#define FEATURE_ENABLE_MOTOR_DRIVER        (1U)
#define FEATURE_ENABLE_OLED                (1U)
#define FEATURE_ENABLE_IMU                 (1U)
#define FEATURE_ENABLE_MOTOR               (0U)
#define FEATURE_ENABLE_ENCODER             (0U)
#define FEATURE_ENABLE_GY931               (1U)
#define FEATURE_ENABLE_BUTTON_CHASSIS_TEST (0U)
#define FEATURE_ENABLE_GRAYSCALE           (1U)

#define FEATURE_ENABLE_BUTTON_OLED_LOG     (1U)
#define FEATURE_ENABLE_SCHEDULER_STATS     (1U)
#define FEATURE_ENABLE_SHELL_DIAGNOSTICS   (1U)

#endif /* CONFIG_FEATURE_DEVELOPMENT_CONFIG_H_ */
