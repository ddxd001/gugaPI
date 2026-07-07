# gugaPI 调试 Shell 命令说明

本文档说明 gugaPI 固件当前支持的调试 shell 命令。调试串口为 `115200 8N1`。

数字参数支持十进制或 `0x` 开头的十六进制，例如 `16` 和 `0x10`。

## 通用命令

### `version`

显示板卡、MCU 和默认调试串口波特率。

```text
version
```

### `reset`

复位 MCU。

```text
reset
```

## LED

### `led status`

查看 3 个 LED 的初始化状态和当前逻辑状态。

```text
led status
```

### `led <1|2|3> on|off|toggle|status`

控制或查看指定 LED。

```text
led 1 on
led 2 toggle
led 3 status
```

### `led all on|off|toggle|status`

同时控制或查看全部 LED。

```text
led all on
led all off
led all status
```

## 蜂鸣器

### `buzzer on`

打开蜂鸣器。

```text
buzzer on
```

### `buzzer off`

关闭蜂鸣器。

```text
buzzer off
```

### `buzzer toggle`

翻转蜂鸣器状态。

```text
buzzer toggle
```

### `buzzer status`

查看蜂鸣器当前状态。

```text
buzzer status
```

## 按键

### `button`

读取三个按键状态。

```text
button
```

当前硬件连接：

| 按键 | MCU 引脚 | 说明 |
| --- | --- | --- |
| button1 | PC9 | 低电平按下 |
| button2 | PB20 | 低电平按下 |
| button3 | PB23 | 低电平按下 |

## FRAM

### `fram status`

查看 FRAM I2C 总线状态。

```text
fram status
```

输出里的 `scl`、`sda` 表示总线电平。

### `fram recover`

尝试恢复 I2C 总线。

```text
fram recover
```

### `fram test`

执行 FRAM 读写自测。

```text
fram test
```

### `fram read <addr> <len>`

从 FRAM 读取数据，`len` 范围是 `1..32`。

```text
fram read 0x0000 16
```

### `fram write <addr> <byte>`

向 FRAM 写入 1 字节，并执行读回校验。

```text
fram write 0x0000 0xAA
```

## INA219

### `ina219 status`

查看 INA219 总线、电平、当前地址和校准值。

```text
ina219 status
```

### `ina219 scan`

扫描 INA219 可能地址范围。

```text
ina219 scan
```

### `ina219 addr <0x40..0x4F>`

手动设置 INA219 地址。

```text
ina219 addr 0x40
```

### `ina219 recover`

尝试恢复 INA219 所在 I2C 总线。

```text
ina219 recover
```

### `ina219 config`

重新写入默认配置和校准值。

```text
ina219 config
```

### `ina219 reset`

复位 INA219。

```text
ina219 reset
```

### `ina219 read`

读取换算后的电压、电流、功率。

```text
ina219 read
```

典型输出：

```text
ina219 bus_mV=12312 shunt_uV=60 current_uA=12000 power_mW=152 cnvr=1 ovf=0
```

字段含义：

| 字段 | 含义 |
| --- | --- |
| `bus_mV` | 总线电压，单位 mV |
| `shunt_uV` | 采样电阻两端电压，单位 uV |
| `current_uA` | 电流，单位 uA |
| `power_mW` | 功率，单位 mW |
| `cnvr` | 转换完成标志 |
| `ovf` | 溢出标志 |

### `ina219 raw`

读取 INA219 原始寄存器。

```text
ina219 raw
```

### `ina219 reg <0..5> [value]`

读取或写入 INA219 寄存器。

读取：

```text
ina219 reg 0
```

写入：

```text
ina219 reg 0 0x399F
```

### `ina219 oled on [period_ms]`

把 INA219 的测量数据周期显示到 OLED。默认刷新周期为 500 ms，可设置范围为 `100..5000` ms。该命令使用调度器周期刷新，不会阻塞 shell。开启 INA219 OLED 显示时，会自动停止 GY931 OLED 实时显示，避免两个任务同时覆盖屏幕。
```text
ina219 oled on
ina219 oled on 500
```

OLED 4 行内容为：
```text
INA219 0x40 500ms
Bus: 12.345V
Cur: 123.456mA
P:152mW Sh:60uV
```

### `ina219 oled off`

停止 INA219 到 OLED 的周期刷新。
```text
ina219 oled off
```

### `ina219 oled status`

查看 OLED 实时显示任务是否开启、刷新周期和最近一次状态。
```text
ina219 oled status
```

### `ina219 oled once`

只刷新一次 OLED，用于先验证接线和显示格式。
```text
ina219 oled once
```

## GY931 角度传感器

GY931 通过 PA29/SCL、PA30/SDA 连接，固件使用 GPIO 模拟开漏 I2C。默认 7-bit 地址为 `0x50`，角度寄存器按维特标准协议读取 `Roll/Pitch/Yaw = 0x3D/0x3E/0x3F`，输出角度单位为度，保留三位小数。

### `gy931 status`

查看软件 I2C 总线电平、当前地址和探测结果。
```text
gy931 status
```

正常空闲时 `scl=H sda=H`，默认地址连接正确时 `probe=ok`。

### `gy931 scan [start end]`

扫描 GY931 软件 I2C 总线地址。默认扫描 `0x08..0x77`，也可以只扫默认地址。
```text
gy931 scan 0x50 0x50
```

### `gy931 addr [0x08..0x77]`

查看或临时切换固件使用的 GY931 地址。
```text
gy931 addr
gy931 addr 0x50
```

### `gy931 init`

重新初始化 GY931 驱动并探测当前地址。
```text
gy931 init
```

### `gy931 recover`

释放并恢复 PA29/PA30 软件 I2C 总线。若 SDA 被设备拉低，可先执行该命令。
```text
gy931 recover
```

### `gy931 angle`

读取 Roll、Pitch、Yaw 三轴角度。
```text
gy931 angle
```

典型输出：
```text
gy931 angle raw=123,-45,1000 deg=0.675,-0.247,5.493
```

### `gy931 sample`

一次读取加速度、角速度、磁场原始值和角度。
```text
gy931 sample
```

输出中的 `acc_g` 单位为 g，`gyro_dps` 单位为 deg/s，`angle_deg` 单位为 deg，`mag_raw` 为未换算的磁场原始寄存器值。

### `gy931 raw <reg> <words 1..16>`

读取连续 16-bit 小端寄存器，调试未知寄存器或核对协议时使用。
```text
gy931 raw 0x3D 3
gy931 raw 0x34 12
```

### `gy931 oled on [period_ms]`

把 GY931 的 Roll/Pitch/Yaw 周期显示到 OLED。默认刷新周期为 200 ms，可设置范围为 `50..5000` ms。该命令使用调度器周期刷新，不会阻塞 shell。
```text
gy931 oled on
gy931 oled on 100
```

OLED 4 行内容为：
```text
GY931 0x50 200ms
Roll: 12.345 deg
Pitch:-1.234 deg
Yaw:  90.000 deg
```

### `gy931 oled off`

停止 GY931 到 OLED 的周期刷新。
```text
gy931 oled off
```

### `gy931 oled status`

查看 OLED 实时显示任务是否开启、刷新周期和最近一次状态。
```text
gy931 oled status
```

### `gy931 oled once`

只刷新一次 OLED，用于先验证接线和显示格式。
```text
gy931 oled once
```

## OLED

OLED 模块为 Hansheng `HS91L02W2C01`，0.91 寸白色 128x32 IIC 屏，和 FRAM、INA219 共用 IIC3 / MCU I2C2 总线。数据手册标注 `ADD:0x78`，这是 8-bit 写地址；shell 和固件中使用 7-bit 地址 `0x3C`。

### `oled status`

查看 OLED 初始化状态、地址探测结果和 I2C 总线电平。

```text
oled status
```

### `oled init`

重新发送 OLED 初始化序列并清屏点亮。

```text
oled init
```

### `oled clear`

清空显示内容。

```text
oled clear
```

### `oled fill <0x00..0xFF>`

用固定字节填充整屏，用于确认显存写入方向和页组织。

```text
oled fill 0xFF
oled fill 0x00
oled fill 0xAA
```

### `oled test`

依次显示全亮、全灭和棋盘格测试图案。

```text
oled test
```

### `oled text <row 0..3> <col 0..20> <ascii...>`

在指定字符行和列显示 ASCII 字符串。当前使用 5x7 小字体，每个字符占 6 像素宽，128x32 屏幕可显示 4 行、每行 21 个字符。

```text
oled clear
oled text 0 0 MotorDriver
oled text 1 0 RPM: 120
oled text 2 0 I2C: OK
```

### `oled invert on|off`

开启或关闭反色显示。

```text
oled invert on
oled invert off
```

### `oled on|off`

打开或关闭 OLED 显示。

```text
oled off
oled on
```

## IMU / 磁力计 SPI

ICM-45686 和 LIS3MDLTR 共用 `IMU_SPI`，由 GPIO 手动控制片选：

| 器件 | 片选 | 中断 / 数据就绪 |
| --- | --- | --- |
| ICM-45686 | PC7，低电平选中 | INT1 = PC6，INT2/FSYNC = PA31 预留同步 |
| LIS3MDLTR | PC8，低电平选中 | DRDY = PA22 |

### `imu status`

查看 IMU SPI 片选线和中断线电平。
```text
imu status
```

`cs_icm_out` 和 `cs_lis_out` 表示 MCU 输出锁存和输出使能状态；正常空闲时应为 `H/OE`。
`cs_icm_in` 和 `cs_lis_in` 是 GPIO 输入缓冲读数，输出脚上不作为片选电压的唯一依据，必要时以万用表或示波器实测为准。

### `imu pins wiggle [loops]`

临时把 IMU SPI 的 PB18/SCLK、PB17/PICO 和 PB19/POCI 切成 GPIO 输出并翻转，用于示波器确认物理引脚和网络。命令结束后会恢复 SPI 复用。
```text
imu pins wiggle
imu pins wiggle 100000
```

### `imu spi burst [bytes] [byte]`

使用真正的 SPI0 外设选中 ICM-45686，并连续发送固定字节，便于示波器稳定触发 SCLK/PICO/POCI。
```text
imu spi burst
imu spi burst 100000 0xAA
```

### `imu spi mode <0..3>`

运行时切换 SPI0 的 4-wire Motorola 模式，用于确认 CPOL/CPHA。
```text
imu spi mode 0
imu spi mode 1
imu spi mode 2
imu spi mode 3
```

### `imu spi rx [count] [tx]`

选中 ICM-45686，连续发送固定字节并打印 SPI0 实际收到的字节，用于确认 MISO 输入路径。
```text
imu spi rx
imu spi rx 16 0x00
imu spi rx 16 0xFF
```

### `imu lis whoami`

读取 LIS3MDLTR `WHO_AM_I` 寄存器。
```text
imu lis whoami
```

正常应读到：
```text
imu lis whoami=0x3D expected=0x3D
```

### `imu lis reg <addr>`

读取 LIS3MDLTR 单个寄存器。
```text
imu lis reg 0x0F
```

### `imu icm reg <addr>`

按通用 SPI 读格式读取 ICM-45686 单个寄存器。
```text
imu icm reg 0x00
```

ICM-45686 的 `WHO_AM_I` 地址和值需要用最终数据手册确认后再固化为专用命令。

## 通用 I2C 诊断

当前注册的 I2C 总线：

| 名称 | 用途 |
| --- | --- |
| `motor` | MotorDriver 专用 IIC1 / MCU I2C1 总线 |
| `fram` | FRAM 所在的共享 IIC3 / MCU I2C2 总线 |
| `oled` | OLED 所在的共享 IIC3 / MCU I2C2 总线 |
| `ina219` | INA219 所在的共享 IIC3 / MCU I2C2 总线 |

### `i2c list`

列出可诊断的 I2C 总线。

```text
i2c list
```

### `i2c status <bus>`

查看指定 I2C 总线状态。

```text
i2c status ina219
```

### `i2c recover <bus>`

尝试恢复指定 I2C 总线。

```text
i2c recover ina219
```

### `i2c scan <bus> [start end]`

扫描指定地址范围。

```text
i2c scan ina219 0x40 0x4F
i2c scan fram 0x50 0x57
i2c scan oled 0x3C 0x3C
i2c scan motor 0x20 0x27
```

不建议长期全地址扫描未知设备，因为探测会发送 1 字节写操作，部分设备可能会改变内部寄存器指针。

### `i2c probe <bus> <addr>`

探测单个地址。

```text
i2c probe ina219 0x40
```

### `i2c read <bus> <addr> <reg8> <len>`

按 8 位寄存器地址读取数据，`len` 范围是 `1..32`。

```text
i2c read ina219 0x40 0x00 2
```

### `i2c write <bus> <addr> <reg8> <byte...>`

按 8 位寄存器地址写入数据。

```text
i2c write ina219 0x40 0x00 0x39 0x9F
```

### `i2c test <bus> [start end]`

执行总线状态检查、恢复和扫描。

```text
i2c test ina219 0x40 0x4F
```

## LoRa 串口透传

LoRa 使用 `115200 8N1` 串口透传。该命令只做原始串口收发，不理解 LoRa 模块协议。

### `lora status`

查看 LoRa 串口状态。

```text
lora status
```

字段含义：

| 字段 | 含义 |
| --- | --- |
| `baud` | 波特率 |
| `rx_buf` | 接收缓冲区未读字节数 |
| `dropped` | 接收缓冲区满后丢弃字节数 |
| `uart` | UART 状态寄存器 |
| `busy` | UART 是否忙 |
| `tx_empty` | TX FIFO 是否为空 |
| `tx_full` | TX FIFO 是否满 |

### `lora clear`

清空 LoRa 接收缓冲区。

```text
lora clear
```

### `lora test`

发送固定文本 `ping\r\n`。

```text
lora test
```

### `lora send <text...>`

发送文本，不自动追加换行。

```text
lora send hello
```

### `lora line <text...>`

发送文本，并自动追加 `\r\n`。

```text
lora line AT
```

### `lora hex <byte...>`

发送原始字节。

```text
lora hex 0x55 0xAA 0x01
```

### `lora read [len]`

读取 LoRa 接收缓冲区，默认最多 64 字节。

```text
lora read
lora read 16
```

## MotorDriver 控制

gugaPI 默认使用 `motor` I2C 总线控制 MotorDriver，默认 7-bit 地址 `0x20`。`PA8/UART1_TX`、`PA9/UART1_RX` 仍保留为 UART 调试链路，也可以通过 `motor bus uart` 手动切换到 UART 协议帧访问。

UART 硬件连接：

| gugaPI | MotorDriver | 说明 |
| --- | --- | --- |
| PA8 TX | RX | gugaPI 发送到 MotorDriver |
| PA9 RX | TX | gugaPI 接收 MotorDriver |
| GND | GND | 必须共地 |

串口参数：`115200 8N1`，3.3V TTL。

MotorDriver 协议帧：

```text
0xAA CMD REG LEN DATA... CRC8
```

CRC 为 `CRC-8 poly 0x07 init 0x00`，覆盖 `SOF` 到 `DATA`。

I2C 当前使用 `motor` 总线别名，走 gugaPI 的 MotorDriver 专用 IIC1 / MCU I2C1 总线，默认 7-bit 地址 `0x20`。I2C 不使用 UART 帧，协议是直接寄存器访问：写 1 字节寄存器地址后读 N 字节，或写 `[reg, data...]`。

### 安全建议

先执行：

```text
motor status
motor bus
motor ping
motor info
motor stop
```

确认通信正常后，再执行会让电机动作的 `motor m1 run ...` 或 `motor m2 run ...`。

### `motor status`

查看 MotorDriver 通信状态。

```text
motor status
```

常见字段：

| 字段 | 含义 |
| --- | --- |
| `bus` | 当前高层控制使用 `uart` 还是 `i2c` |
| `baud` | UART 波特率 |
| `rx_buf` | UART 接收缓冲区未读字节数 |
| `dropped` | UART 接收缓冲区满后丢弃字节数 |
| `uart` | UART 状态寄存器 |
| `i2c_addr` | MotorDriver I2C 目标地址 |
| `i2c_bus` | gugaPI 使用的 I2C 总线别名 |
| `scl` / `sda` | I2C 总线当前电平 |

### `motor bus [uart|i2c]`

查看或切换 MotorDriver 高层命令使用的通信方式。
开机默认值为 `i2c`。

```text
motor bus
motor bus uart
motor bus i2c
```

`motor send` / `motor hex` / `motor read` / `motor clear` 仍然是原始 UART 调试命令，不受 `motor bus` 影响。

### `motor i2caddr [addr]`

查看或设置 gugaPI 访问 MotorDriver 时使用的 I2C 地址。

```text
motor i2caddr
motor i2caddr 0x20
```

### `motor ping`

确认 MotorDriver 通信是否正常。

```text
motor ping
```

正常输出：

```text
motor ping: ok
```

UART 模式下该命令发送 heartbeat 帧，并会在存在活动命令时刷新 MotorDriver watchdog。I2C 模式下该命令只探测 `i2caddr` 地址是否应答，不刷新 watchdog。

### `motor info`

读取 MotorDriver 基础寄存器。

```text
motor info
```

输出字段：

| 字段 | 含义 |
| --- | --- |
| `id` | 设备 ID，正常应为 `0xA5` |
| `fw` | 固件版本 |
| `status` | 状态标志 |
| `fault` | 故障标志 |
| `ctrl` | 控制标志 |
| `i2c` | MotorDriver I2C 地址 |

### `motor reg <addr> <len>`

读取 MotorDriver 寄存器。

```text
motor reg 0x00 6
motor reg 0x10 4
motor reg 0x20 4
```

常用寄存器：

| 地址 | 名称 | 含义 |
| --- | --- | --- |
| `0x00` | DEVICE_ID | 设备 ID |
| `0x01` | FW_VERSION | 固件版本 |
| `0x02` | STATUS | 全局状态 |
| `0x03` | FAULT_FLAGS | 故障标志 |
| `0x04` | CONTROL_FLAGS | 控制标志 |
| `0x05` | I2C_ADDRESS | I2C 地址 |
| `0x10` | M1_MODE | 电机 1 模式 |
| `0x11` | M1_DUTY | 电机 1 占空比 |
| `0x12` | M1_DIRECTION | 电机 1 方向 |
| `0x13` | M1_STATUS | 电机 1 状态 |
| `0x20` | M2_MODE | 电机 2 模式 |
| `0x21` | M2_DUTY | 电机 2 占空比 |
| `0x22` | M2_DIRECTION | 电机 2 方向 |
| `0x23` | M2_STATUS | 电机 2 状态 |
| `0x30` | WATCHDOG_TIMEOUT | 看门狗超时 |

### `motor set <addr> <byte...>`

直接写 MotorDriver 寄存器。

```text
motor set 0x04 0x01
motor set 0x10 0x01 0x20 0x00
```

直接写寄存器可能让电机动作，测试前先确认寄存器含义。

### `motor stop`

停止两个电机。

```text
motor stop
```

该命令会把 M1/M2 写为 `coast`，并把 duty 写为 `0`。

### `motor m1 coast`

电机 1 滑行停止。

```text
motor m1 coast
```

### `motor m1 brake`

电机 1 刹车。

```text
motor m1 brake
```

### `motor m1 run <duty> [fwd|rev]`

运行电机 1。

```text
motor m1 run 10 fwd
motor m1 run 10 rev
```

`duty` 范围是 `0..100`。
开环 `run duty > 0` 期间 MotorDriver 会暂停对应电机的编码器 GPIO 中断，停止后约 3 秒再恢复，以保证高占空比测试和惯性转动期间通信仍能处理；此时该路编码器 count/RPM 不保证更新。需要编码器闭环时使用 `motor m1 speed` / `hold` / `pos` / `posrel`。

### `motor m2 coast`

电机 2 滑行停止。

```text
motor m2 coast
```

### `motor m2 brake`

电机 2 刹车。

```text
motor m2 brake
```

### `motor m2 run <duty> [fwd|rev]`

运行电机 2。

```text
motor m2 run 10 fwd
motor m2 run 10 rev
```

`duty` 范围是 `0..100`。
开环 `run duty > 0` 期间 MotorDriver 会暂停对应电机的编码器 GPIO 中断，停止后约 3 秒再恢复，以保证高占空比测试和惯性转动期间通信仍能处理；此时该路编码器 count/RPM 不保证更新。需要编码器闭环时使用 `motor m2 speed` / `hold` / `pos` / `posrel`。

### MotorDriver 原始串口调试命令

这些命令绕过 MotorDriver 协议，只做原始串口收发，适合排查线序、波特率和电平。

```text
motor clear
motor test
motor send <text...>
motor line <text...>
motor hex <byte...>
motor read [len]
```

自环测试时，短接 gugaPI 的 PA8 和 PA9：

```text
motor clear
motor test
motor read 16
```

如果能读到 `ping`，说明 gugaPI 侧 UART1 的 TX/RX 正常。

## ADC 和 PWM 占位命令

### `adc`

当前没有注册 ADC 驱动，只会提示占位信息。

```text
adc
```

### `pwm <0..100>`

当前没有注册 PWM 驱动，只会提示占位信息。

```text
pwm 50
```
