#ifndef CONFIG_FEATURE_CONFIG_H_
#define CONFIG_FEATURE_CONFIG_H_

/*
 * Feature switches for competition builds.
 *
 * Keep optional hardware and debug features behind these switches so the
 * project can be trimmed quickly during on-site testing.
 */

#define FEATURE_ENABLE_LOG              (0U)
#define FEATURE_ENABLE_SHELL            (0U)
#define FEATURE_ENABLE_DEBUG_UART       (1U)
#define FEATURE_ENABLE_UART_COUNTER_TEST (1U)
#define FEATURE_ENABLE_STATUS_LED       (1U)
#define FEATURE_ENABLE_LED_TEST         (0U)
#define FEATURE_ENABLE_BUZZER           (1U)
#define FEATURE_ENABLE_BUZZER_TEST      (0U)
#define FEATURE_ENABLE_OLED             (0U)
#define FEATURE_ENABLE_IMU              (0U)
#define FEATURE_ENABLE_MOTOR            (0U)
#define FEATURE_ENABLE_ENCODER          (0U)

#endif /* CONFIG_FEATURE_CONFIG_H_ */