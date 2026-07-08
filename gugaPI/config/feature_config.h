#ifndef CONFIG_FEATURE_CONFIG_H_
#define CONFIG_FEATURE_CONFIG_H_

/*
 * 比赛现场使用的功能开关。
 * 临时裁剪外设或调试功能时只改这里，避免分散修改各个驱动文件。
 * 1U 表示启用，0U 表示关闭。
 */

#define FEATURE_ENABLE_LOG               (1U) /* 启用日志输出接口。 */
#define FEATURE_ENABLE_LOG_DEBUG         (1U) /* 启用 DEBUG 等级日志。 */
#define FEATURE_ENABLE_SHELL             (1U) /* 启用调试命令行 shell。 */
#define FEATURE_ENABLE_DEBUG_UART        (1U) /* 启用 PC 调试串口。 */
#define FEATURE_ENABLE_UART_COUNTER_TEST (0U) /* 调试串口计数打印测试，默认关闭以免干扰 shell。 */
#define FEATURE_ENABLE_STATUS_LED        (1U) /* 启用状态指示灯驱动。 */
#define FEATURE_ENABLE_LED_TEST          (0U) /* 状态灯自动测试任务。 */
#define FEATURE_ENABLE_BUZZER            (1U) /* 启用蜂鸣器驱动。 */
#define FEATURE_ENABLE_BUZZER_TEST       (0U) /* 蜂鸣器自动测试任务。 */
#define FEATURE_ENABLE_BUTTONS           (1U) /* 启用按键驱动和消抖。 */
#define FEATURE_ENABLE_FRAM              (1U) /* 启用外部 FRAM 存储驱动。 */
#define FEATURE_ENABLE_INA219            (1U) /* 启用 INA219 电压电流采样驱动。 */
#define FEATURE_ENABLE_LORA              (1U) /* 启用 LoRa 串口透传模块驱动。 */
#define FEATURE_ENABLE_MOTOR_DRIVER      (1U) /* 启用 MotorDriver 串口连接测试模块。 */
#define FEATURE_ENABLE_OLED              (1U) /* 启用 OLED 显示功能。 */
#define FEATURE_ENABLE_IMU               (1U) /* 启用 ICM-45686 + LIS3MDLTR SPI 模块。 */
#define FEATURE_ENABLE_MOTOR             (0U) /* 预留电机控制功能。 */
#define FEATURE_ENABLE_ENCODER           (0U) /* 预留编码器采样功能。 */

#define FEATURE_ENABLE_GY931             (1U) /* Enable WIT GY931 I2C angle sensor. */
#define FEATURE_ENABLE_BUTTON_CHASSIS_TEST (1U) /* Button1 chassis wheel test. */

#endif /* CONFIG_FEATURE_CONFIG_H_ */
