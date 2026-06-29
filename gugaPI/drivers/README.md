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