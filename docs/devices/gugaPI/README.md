# gugaPI 外设手册索引

这些手册只描述主控 `gugaPI` 工程。

- [启动自检](STARTUP_HEALTH.md)
- [三路按键与事件](BUTTONS.md)
- [共享 I2C 控制器](I2C_CONTROLLER.md)
- [FM24CL64B FRAM](FM24CL64B_FRAM.md)
- [SSD1306 OLED](OLED_SSD1306.md)
- [INA219 与保护状态机](INA219.md)
- [ICM45686](ICM45686.md)
- [LIS3MDL](LIS3MDL.md)
- [八路灰度传感器](GRAYSCALE.md)
- [编码器距离闭环](ENCODER_DISTANCE.md)
- [LoRa UART 与帧协议](LORA.md)
- [ADC/PWM 命令边界](ADC_PWM_COMMANDS.md)

共同验证状态：本次版本已经使用 MSPM0 SDK 2.10.00.04、SysConfig 1.26.2 和
TI Arm Clang 5.1.1.LTS 完成生成、编译和链接。尚未完成整板烧录、逻辑分析仪和
故障注入，因此不能视为硬件验证完成。
