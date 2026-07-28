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
#define FEATURE_ENABLE_INA219              (0U)
#define FEATURE_ENABLE_LORA                (0U)
#define FEATURE_ENABLE_MOTOR_DRIVER        (0U)
#define FEATURE_ENABLE_OLED                (0U)
#define FEATURE_ENABLE_IMU                 (1U)
#define FEATURE_ENABLE_MOTOR               (1U)
#define FEATURE_ENABLE_ENCODER             (1U)
/* Select the MR bridge plus its PC0/PC1 hardware-QEI encoder on the
 * unmodified U9 routing. This is a single-motor bench profile, not a
 * differential-drive chassis. */
#define FEATURE_LOCAL_MOTOR_RIGHT_ONLY      (1U)
#define FEATURE_ENABLE_GY931               (0U)
#define FEATURE_ENABLE_BUTTON_CHASSIS_TEST (0U)
#define FEATURE_ENABLE_GRAYSCALE           (1U)

#define FEATURE_ENABLE_BUTTON_EVENT_LOG    (1U)
#define FEATURE_ENABLE_SCHEDULER_STATS     (1U)
#define FEATURE_ENABLE_SHELL_DIAGNOSTICS   (1U)

#endif /* CONFIG_FEATURE_DEVELOPMENT_CONFIG_H_ */
