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

### `led on`

打开状态灯。

```text
led on
```

### `led off`

关闭状态灯。

```text
led off
```

### `led toggle`

翻转状态灯。

```text
led toggle
```

### `led status`

查看状态灯当前状态。

```text
led status
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
| `fram` | FRAM 总线 |
| `ina219` | INA219 总线 |

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

## MotorDriver 串口

gugaPI 使用 `PA8/UART1_TX`、`PA9/UART1_RX` 连接 MotorDriver。

硬件连接：

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

### 安全建议

先执行：

```text
motor status
motor ping
motor info
motor stop
```

确认通信正常后，再执行会让电机动作的 `motor m1 run ...` 或 `motor m2 run ...`。

### `motor status`

查看 MotorDriver UART 状态。

```text
motor status
```

### `motor ping`

发送 heartbeat 帧，确认 MotorDriver 协议通信是否正常。

```text
motor ping
```

正常输出：

```text
motor ping: ok
```

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
