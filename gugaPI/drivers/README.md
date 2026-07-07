# 驱动目录说明

每个新硬件设备独立放在 `drivers/<device>/` 目录中，例如：

```text
drivers/oled/
  oled.h
  oled.cpp

drivers/encoder/
  encoder.h
  encoder.cpp
```

驱动只负责设备本身，不直接包含 `app/`。引脚、总线实例、片选线等板级信息放在 `board/`，由 `Config` 结构传给驱动。

新增驱动时可以参考 `drivers/template/` 的写法。
当前已接入的独立驱动包括：

```text
drivers/led/          PA27/PA26/PB27 低电平点亮 LED
drivers/buzzer/       PC16 高电平响有源蜂鸣器
drivers/fm24cl64b/    FM24CL64B-GTR I2C FRAM
drivers/oled/         HS91L02W2C01 128x32 I2C OLED
drivers/soft_i2c/     GPIO software I2C master for dedicated low-speed devices
drivers/gy931/        WIT GY931 angle sensor register driver
```
