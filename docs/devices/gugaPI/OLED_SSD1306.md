# SSD1306 OLED 手册

## 启动与参数

启用 `FEATURE_ENABLE_OLED` 后由 `Board_OledInit()` 初始化。设备位于共享 SENSOR_I2C，
7 位地址 `0x3C`，分辨率 128×32（4 页）。初始化包括地址探测、SSD1306 配置、清屏和
开显示；任一步失败都会清除 ready。

## API

```cpp
board::Board_OledClear();
board::Board_OledFill(pattern);
board::Board_OledTestPattern();
board::Board_OledWriteText(row, col, "text");
board::Board_OledSetDisplayOn(true);
board::Board_OledSetInvert(false);
```

文本使用 5×7 字体加 1 列间隔，`row=0..3`，`col=0..20`。不可打印字符显示为 `?`。
数据按 32 字节分块写入，共享 I2C 层负责 FIFO、超时和失败清理。

## Shell

```text
oled status
oled init
oled clear
oled fill 0xaa
oled test
oled text 0 0 hello
oled invert on
oled on
```

## 故障与验证

OLED 是可降级设备。失败不会阻止主控继续启动。验证地址 NACK、清屏、全亮、棋盘格、
四行文本、反色和与 FRAM/INA219 交替访问；确认超时后同一 I2C 总线仍可恢复。

