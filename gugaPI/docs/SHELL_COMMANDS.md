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

### `sched`

查看调度器运行时统计（各任务周期、执行次数、最大/平均执行时间）。仅在开发配置（`FEATURE_ENABLE_SCHEDULER_STATS`）下可用。

```text
sched
```

### `txstat`

查看调试 UART（UART0）TX/RX 队列状态。仅在开发配置（`FEATURE_ENABLE_DEBUG_UART`）下可用。

```text
txstat
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

当前按键快捷功能：

| 按键 | 动作 |
| --- | --- |
| button1 | 等效 `ina219 oled on 100` |
| button2 | 等效 `gy931 oled on 100` |
| button3 | 等效 `gray oled on 100` |

### `button watch [duration_ms]`

实时监控按键 GPIO 电平变化，默认持续 5 秒，可设置 `100..30000` ms。按电平变化时打印时间戳和 GPIOB/GPIOC 输入寄存器快照。

```text
button watch
button watch 10000
```

### `button scan [duration_ms]`

扫描全部 GPIOA/GPIOB/GPIOC 引脚变化（比 `watch` 范围更广），默认持续 5 秒，可设置 `100..30000` ms。用于排查按键映射到哪个引脚。

```text
button scan
button scan 10000
```

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

### `gy931 algorithm [6axis|9axis]`

查看或临时切换姿态解算算法。`6axis` 使用加速度计和陀螺仪积分输出相对航向，`9axis` 使用磁场参与解算绝对航向。切换命令会解锁并读回校验 `AXIS6(0x24)`，但不会执行保存，传感器重新上电后恢复其已保存配置。

```text
gy931 algorithm
gy931 algorithm 6axis
gy931 algorithm 9axis
```

六轴模式不受磁场干扰，但 yaw 会随时间漂移，适合持续时间较短的直行保持和相对角度转弯。

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
| ICM-45686 | PC7，低电平选中 | INT1 = PC6，INT2 = PA31（开漏中断输入，FSYNC 不用） |
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

### `imu icm init`

对 ICM-45686 执行设备级初始化：软复位（写 `0x02` 到 `REG_MISC2@0x7F`）、轮询 `WHO_AM_I` 等待复位完成、校验 `WHO_AM_I=0xE9`、配置 INT1 推挽、设置加速度/陀螺仪量程与 ODR、`PWR_MGMT0=0x0F` 使能 LN 模式。开机时 `Board_ImuInit` 已自动执行一次；器件异常时可手动重试。
```text
imu icm init
```
正常返回 `imu icm init: ok`。失败常见原因：SPI 接线/CS、器件未供电、`WHO_AM_I` 读不到 0xE9。

### `imu icm whoami`

读取 ICM-45686 `WHO_AM_I` 寄存器（0x72）。
```text
imu icm whoami
```
正常：
```text
imu icm whoami=0xE9 expected=0xE9
```
读不到 0xE9 → 先检查 SPI 物理连接与 `imu status` 的 CS 电平。

### `imu icm sample`

单次突发读取 0x00~0x0D 共 14 字节，按大端解析加速度/陀螺仪/温度原始值（int16）。
```text
imu icm sample
```
输出（原始值）：
```text
imu icm acc=<ax>,<ay>,<az> gyr=<gx>,<gy>,<gz> t=<temp>
```
静置时加速度 Z 轴应约为 +1g（量程 ±4g 下 raw ≈ 8192），陀螺仪三轴接近 0，温度 raw 换算 `T=raw/128+25`。

### `imu icm reg <addr>`

读取 ICM-45686 单个寄存器（7 位地址，自动加读位）。
```text
imu icm reg 0x72
imu icm reg 0x10
```

### `imu icm wreg <addr> <val>`

写 ICM-45686 单个寄存器（用于手动调参/调试，谨慎操作）。
```text
imu icm wreg 0x1B 0x39
```

### `imu sample`

打印周期采样任务（10ms 一次，`app_imu`）缓存的最新数据，已按量程换算为 mg / mdps / centi℃，并减去 `ConfigStore` 里的 `imu_accel_bias_*` / `imu_gyro_bias_*` 偏置。
```text
imu sample
```
输出：
```text
imu acc=<mg>,<mg>,<mg> mg gyr=<mdps>,<mdps>,<mdps> mdps t=<cC> cC yaw=<deg> deg
```
若未就绪会提示 `imu sample: no data (run 'imu icm init')`。

### `imu oled on [period_ms]` / `off` / `status` / `once`

把 IMU 角度数据显示到 OLED（沿用 INA219 的 OLED 方案，与其他 OLED 数据源互斥）。周期采样任务（10ms）算出俯仰/横滚角并积分 Z 轴角速度得到相对 Yaw，OLED 任务按 `period_ms` 刷新。

```text
imu oled on
imu oled on 100
imu oled off
imu oled status
imu oled once
```

显示（4 行）：
```text
IMU 200ms
Pit: <ddd.ddd> deg
Rol: <ddd.ddd> deg
Yaw: <ddd.ddd> deg
```

- `Pit`/`Rol`：由加速度计算的俯仰/横滚角，0~360°（"360 度单位"）。
- `Yaw`：陀螺仪 Z 轴按真实采样间隔积分得到的相对角度，范围 -180°~180°。
- 无数据时显示 `IMU no data / run 'imu icm init'`。

## 灰度传感器（8 路 ADC）

8:1 多路复用灰度阵列：3 个选位引脚（PA16=bit2、PC20=bit1、PC21=bit0）选 1 路，PA15（ADC1 ADCIN0）读模拟值（0..4095）。10ms 周期任务缓存全 8 路。

### `gray status`

查看驱动就绪与周期数据有效性。
```text
gray status
```

### `gray read <0..7>`

按需读取单路（切换选位→~100µs 建立→ADC 转换）。
```text
gray read 0
gray read 7
```
输出：`gray ch0=1234`（0..4095）。遮挡/照射该路传感器可见数值变化。

### `gray all`

一次性读取全部 8 路（按需，非缓存）。
```text
gray all
```
输出：`gray: 1234 5678 ...`（8 个值，对应通道 0..7）。

### `gray data`

打印 10ms 周期任务缓存的最新 8 路数据（不触发新转换）。
```text
gray data
```
未就绪时提示 `gray: no data`。

### `gray oled on [period_ms]`

按 INA219 OLED 显示任务的同样风格，将灰度传感器 8 路 ADC 原始值持续显示到 OLED。开启后会关闭其它传感器的 OLED 周期显示任务。
```text
gray oled on
gray oled on 200
gray oled off
gray oled status
gray oled once
```

OLED 4 行显示格式：
```text
GRAY 200ms
0-2 1234 1234 1234
3-5 1234 1234 1234
6-7 1234 1234
```

`period_ms` 范围为 50..5000，默认 200ms。`gray oled once` 只刷新一次 OLED，不开启周期任务。

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

### `motor rpm`

读取两路速度环状态。MotorDriver 固件版本 2 及以上会额外显示内部控制遥测。

```text
motor rpm
```

字段含义：

| 字段 | 含义 |
| --- | --- |
| `target` | 主控请求的最终目标转速 |
| `control` | 经过加减速斜坡后的当前控制目标 |
| `actual` | 编码器实测转速 |
| `error` | 控制目标减去方向对齐后的实测转速 |
| `integral_q4` | 速度 PID 积分状态，Q4 格式 |
| `duty` | 速度控制器最终输出占空比 |

### `motor ramp [accel_rpm_s decel_rpm_s]`

查看或设置 MotorDriver 内部目标转速斜坡，单位为 RPM/s。参数为 `0` 时对应方向立即跟随请求目标，不执行斜坡限制。

```text
motor ramp
motor ramp 600 900
```

普通目标转速变化受斜坡限制；停车、故障和控制器禁用仍立即清除输出。

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

## 底盘控制

底盘控制层封装左右轮转速命令、线速度/角速度转换和编码器读取，是航向闭环、循迹和动作序列的基础。底层仍通过 `motor` 命令的 I2C/UART 通道访问 MotorDriver。

### `chassis status`

主动刷新并查看底盘完整状态（含两轮目标/实测 RPM、编码器 count/cps、底盘几何配置）。会触发一次 I2C 往返。

```text
chassis status
```

### `chassis stat`

查看缓存状态（单行摘要）。不触发 I2C 往返，数据来自 20 ms 周期反馈任务。用于快速确认 actual_rpm 是否在刷新。

```text
chassis stat
```

输出示例：

```text
chassis stat: L tgt=80 act=76 R tgt=80 act=78 last=ok
```

### `chassis stop`

停止底盘（两轮写入 coast + duty 0）。同时清除航向和循迹状态。

```text
chassis stop
```

### `chassis wheel <left_rpm> <right_rpm>`

直接设置左右轮目标转速，范围为 `-max_wheel_rpm..max_wheel_rpm`（默认 1000）。正值前进，负值后退。此命令会刷新 MotorDriver 看门狗。

```text
chassis wheel 80 80
chassis wheel -50 -50
chassis wheel 60 -60
```

### `chassis vel <linear_mm_s> <angular_mdeg_s>`

通过线速度和角速度设置底盘目标。线速度范围 `-5000..5000 mm/s`，角速度范围 `-720000..720000 mdeg/s`。内部根据 `wheel_radius_mm` 和 `wheel_track_mm` 换算为左右轮 RPM。

```text
chassis vel 200 0
chassis vel 0 90000
```

## 航向闭环

航向闭环使用 ICM-45686 陀螺仪 Z 轴 yaw 积分实现直行保持和相对角度转弯。航向源为 IMU yaw（毫度），不是 GY931。

50 ms 周期任务 `Heading_Update` 消费 IMU 数据并输出左右轮差速命令。安全机制：IMU 无效或数据过期（>200 ms）→ `FAULT_SENSOR_LOST` 停车；航向误差 > 90° → 异常停车；转弯超时 8 s → `FAULT_DRIVER_TIMEOUT` 停车。

### `heading status`

查看航向闭环当前状态。

```text
heading status
```

输出字段：

| 字段 | 含义 |
| --- | --- |
| `mode` | `idle` / `hold` / `turn` |
| `target` | 目标航向（度） |
| `error` | 当前航向误差（度，最短角度差） |
| `corr` | 当前差速修正 RPM（hold）或转弯速度（turn） |
| `at_target` | 转弯是否进入容差区间 |
| `last` | 上一次操作结果 |

### `heading hold <base_rpm>`

启动直行航向保持。锁定当前 IMU yaw 为目标航向，以 `base_rpm` 为基础速度直行，根据航向误差差速修正。范围为 `-max_wheel_rpm..max_wheel_rpm`。

```text
heading hold 80
heading hold -50
```

修正公式：`correction = error_mdeg * heading_kp / 1e6`，限幅到 `heading_max_correction_rpm`。`left = base - correction`，`right = base + correction`。

### `heading turn <deg>`

启动相对角度转弯。`deg` 范围 `-180..180`，正值左转，负值右转（取决于 `kYawSign`），走最短路径。接近目标后保持 `heading_settle_ms` 判定完成。

```text
heading turn 90
heading turn -180
heading turn 45
```

转弯速度：`speed = abs_err * heading_kp / 1e6`，限幅到 `[heading_turn_min_rpm, heading_turn_max_rpm]`。

### `heading stop`

停止航向闭环并停车。

```text
heading stop
```

## 循迹控制

8 路灰度循迹。需先标定（`lf cal`）再循迹（`lf start`）。50 ms 周期任务 `LF_Update` 消费灰度数据并输出左右轮差速命令。

安全机制：灰度数据无效 → `FAULT_SENSOR_LOST` 停车；丢线超过 `lost_timeout_ms` → 停车；故障 → 停车。

### `lf status`

查看循迹状态。

```text
lf status
```

输出字段：

| 字段 | 含义 |
| --- | --- |
| `mode` | `idle` / `cal` / `follow` |
| `cal` | 是否已完成标定 |
| `error` | 线路位置误差（`-3500..+3500`，0 = 居中） |
| `corr` | 当前差速修正 RPM |
| `lost` | 当前是否丢线 |
| `kp` | 循迹比例增益 |
| `maxcorr` | 最大修正 RPM |

### `lf cal`

启动标定（2 秒）。在 2 秒内手持传感器在黑线和白底之间来回扫动，记录各通道 min/max ADC 值。完成后自动计算 threshold 并设 `calibrated = true`。

```text
lf cal
```

### `lf start <rpm> <ms>`

启动循迹，以 `rpm` 为基础速度循迹 `ms` 毫秒。`rpm` 范围 `-max_wheel_rpm..max_wheel_rpm`，`ms` 范围 `0..30000`。必须先完成标定。

```text
lf start 80 10000
```

修正公式：`correction = error_mpos * kp / 1e6`，限幅到 `max_correction_rpm`。`left = base - correction`，`right = base + correction`。

### `lf stop`

停止循迹并停车。

```text
lf stop
```

### `lf kp <val>`

设置循迹比例增益（RAM，范围 `0..1000000`）。默认 10000（1 通道偏差约 10 RPM）。

```text
lf kp 15000
```

### `lf maxcorr <val>`

设置最大修正 RPM（RAM，范围 `0..500`）。默认 30。

```text
lf maxcorr 50
```

### `lf losttimeout <ms>`

设置丢线超时（RAM，范围 `100..10000` ms）。丢线后保留上次方向修正，超时则停车。默认 500。

```text
lf losttimeout 1000
```

## 动作序列

条件驱动的指令表解释器，通过 `run add` 逐条构建指令序列，`run start` 启动。50 ms 周期任务 `ActionRunner_Update` 执行当前指令，满足完成条件后跳转到 `on_success` / `on_timeout` 目标。

安全机制：故障 → 中止序列并停车；整序列超时 60 s → 中止；指令启动失败 → 走 `on_timeout` 路径；每条指令完成后调用 `StopAll` 清除运动状态。

### 指令格式

每条指令 6 个参数：

```text
run add <op> <param1> <param2_ms> <until> <onsuccess> <ontimeout>
```

| 参数 | 含义 |
| --- | --- |
| `op` | 操作码：`drive` / `turn` / `follow` / `wait` / `stop` / `branch` / `end` |
| `param1` | DRIVE/FOLLOW: 基础 RPM；TURN: 相对角度（度，`-180..180`）；其他: 0 |
| `param2` | 超时/持续时间（ms，`0..30000`），作为安全上限 |
| `until` | 完成条件：`timeout` / `heading_reached` / `line_detected` / `line_lost` / `button` / `immediate` |
| `onsuccess` | 成功跳转目标：`next`（下一条）或索引 `0..15` |
| `ontimeout` | 超时跳转目标：`abort`（中止序列）或索引 `0..15` |

操作码说明：

| 操作码 | 动作 | 典型条件 |
| --- | --- | --- |
| `drive` | 航向保持直行（`heading hold`） | `timeout` / `line_detected` / `line_lost` |
| `turn` | 相对角度转弯（`heading turn`） | `heading_reached` |
| `follow` | 循迹（`lf start`） | `timeout` / `line_lost` |
| `wait` | 等待（不产生运动） | `timeout` / `button` |
| `stop` | 立即停车 | `immediate` |
| `branch` | 条件跳转（不产生运动），成功走 onsuccess，失败走 ontimeout | 任意条件 |
| `end` | 序列完成（成功） | `immediate` |

### `run add <op> <p1> <p2_ms> <until> <onsuccess> <ontimeout>`

追加一条指令到序列末尾。最多 16 条。

```text
run add drive  80  5000  timeout          next abort
run add turn   90  8000  heading_reached  next abort
run add follow 80  30000 line_lost        next abort
run add wait   0   100   timeout          next abort
run add stop   0   0     immediate        next abort
run add end    0   0     immediate        next abort
```

### `run clear`

清空指令表（序列运行中时拒绝）。

```text
run clear
```

### `run start`

启动序列（从第 0 条指令开始）。有故障时拒绝启动。

```text
run start
```

### `run cancel`

中止正在运行的序列并停车。

```text
run cancel
```

### `run status`

查看序列执行状态。

```text
run status
```

输出示例：

```text
run 2/5 running=1 last=1 cur=turn
```

字段：`当前步/总步数`、`running`、`last`（上一步是否成功）、`cur`（当前操作码）。

### `run dump`

打印完整指令表。

```text
run dump
```

输出示例：

```text
seq 5
0 drive 80 5000 timeout 255 255
1 turn 90 8000 heading_reached 255 255
2 follow 80 30000 line_lost 255 255
3 stop 0 0 immediate 255 255
4 end 0 0 immediate 255 255
```

`255` = `ACT_NEXT`（onsuccess=下一条，ontimeout=中止）。

## 参数管理

参数持久化系统。所有底盘几何、速度环、位置环、IMU 偏置和航向闭环参数统一存储在 FRAM 中（地址 0x0000，magic "CFPG"，CRC32 校验）。当前版本 v3，兼容加载 v1/v2 历史布局。

### `param status`

查看参数存储状态。

```text
param status
```

输出字段：

| 字段 | 含义 |
| --- | --- |
| `loaded` | 是否从 FRAM 成功加载 |
| `dirty` | 是否有未保存的修改 |
| `len` | 存储的 payload 长度 |
| `crc` | 存储的 CRC32 |
| `load` | 上次加载结果 |
| `save` | 上次保存结果 |

### `param get [name]`

查看所有参数或单个参数。不带参数列出全部 32 项参数（含当前值和合法范围）。

```text
param get
param get heading_kp
```

输出示例：

```text
param heading_kp=1000 range=0..100000
```

参数列表：

| 参数名 | 范围 | 默认值 | 说明 |
| --- | --- | --- | --- |
| `left_counts_per_rev` | 1..100000000 | 364 | 左轮编码器 CPR |
| `right_counts_per_rev` | 1..100000000 | 364 | 右轮编码器 CPR |
| `wheel_radius_mm` | 1..1000 | 32 | 轮半径（mm） |
| `wheel_track_mm` | 1..2000 | 160 | 轮距（mm） |
| `max_wheel_rpm` | 1..1000 | 1000 | 最大轮速（RPM） |
| `motor_output_invert_flags` | 0..3 | 1 | 电机输出反向标志 |
| `motor_encoder_invert_flags` | 0..3 | 1 | 编码器反向标志 |
| `speed_kp` | 0..255 | 1 | 速度环 Kp（Q4.4） |
| `speed_ki` | 0..255 | 1 | 速度环 Ki（Q4.4） |
| `speed_kd` | 0..255 | 0 | 速度环 Kd（Q4.4） |
| `speed_max_duty` | 0..100 | 50 | 速度环最大占空比（%） |
| `speed_min_duty` | 0..100 | 6 | 速度环最小占空比（%） |
| `position_kp` | 0..255 | 15 | 位置环 Kp（Q4.4） |
| `position_ki` | 0..255 | 0 | 位置环 Ki（Q4.4） |
| `position_kd` | 0..255 | 0 | 位置环 Kd（Q4.4） |
| `position_max_rpm` | 0..1000 | 8 | 位置环最大转速（RPM） |
| `position_tolerance_counts` | 0..65535 | 50 | 位置环容差（counts） |
| `gy931_roll_zero_mdeg` | ±180000000 | 0 | GY931 Roll 零点（mdeg） |
| `gy931_pitch_zero_mdeg` | ±180000000 | 0 | GY931 Pitch 零点（mdeg） |
| `gy931_yaw_zero_mdeg` | ±180000000 | 0 | GY931 Yaw 零点（mdeg） |
| `imu_accel_bias_x_mg` | ±200000 | 0 | IMU 加速度 X 偏置（mg） |
| `imu_accel_bias_y_mg` | ±200000 | 0 | IMU 加速度 Y 偏置（mg） |
| `imu_accel_bias_z_mg` | ±200000 | 0 | IMU 加速度 Z 偏置（mg） |
| `imu_gyro_bias_x_mdps` | ±2000000 | 0 | IMU 陀螺仪 X 偏置（mdps） |
| `imu_gyro_bias_y_mdps` | ±2000000 | 0 | IMU 陀螺仪 Y 偏置（mdps） |
| `imu_gyro_bias_z_mdps` | ±2000000 | 0 | IMU 陀螺仪 Z 偏置（mdps） |
| `heading_kp` | 0..100000 | 1000 | 航向增益（1 RPM/deg） |
| `heading_max_correction_rpm` | 0..500 | 30 | 直行最大差速修正（RPM） |
| `heading_turn_max_rpm` | 0..1000 | 60 | 转弯最大轮速（RPM） |
| `heading_turn_min_rpm` | 0..500 | 20 | 转弯最小轮速（RPM） |
| `heading_tolerance_mdeg` | 0..90000 | 3000 | 转弯容差（mdeg，3000=3°） |
| `heading_settle_ms` | 0..5000 | 300 | 转弯到位保持时间（ms） |

### `param set <name> <value>`

修改单个参数（RAM，标记 dirty）。修改后需 `param save` 才会持久化到 FRAM。参数值超出合法范围或校验失败（如 `speed_min_duty > speed_max_duty`）时拒绝。

```text
param set heading_kp 1500
param set speed_max_duty 60
```

### `param save`

将当前参数持久化到 FRAM。清除 dirty 标志。

```text
param save
```

### `param load`

从 FRAM 重新加载参数。如果 FRAM 数据损坏或不兼容，回退到源码默认值。

```text
param load
```

### `param reset`

恢复源码默认值（RAM，不清除 dirty 标志，需 `param save` 持久化）。

```text
param reset
```

## 比赛模式

比赛模式状态机：`ARMED`（安全静止）→ `RUNNING`（序列执行中）→ `ARMED`。在比赛配置（`FEATURE_PROFILE_COMPETITION=1`）下上电自动进入 ARMED；开发配置下可用 `comp arm` 手动进入。

LED 指示：ARMED 慢闪（1Hz）、RUNNING 常亮、FAULT 快闪（5Hz）+ 蜂鸣器。

### `comp arm`

从开发模式进入比赛武装状态。停止底盘，禁用 chassis 任务。仅在 `dev-running` 模式下可用。

```text
comp arm
```

### `comp start`

启动比赛序列。需要预先用 `run add` 加载指令序列。可通过按键 1 替代。仅在 `armed` 模式下可用。

```text
comp start
```

### `comp stop`

取消比赛序列并返回武装状态。可通过按键 1 替代。仅在 `running` 模式下可用。

```text
comp stop
```

### `comp status`

查看当前比赛模式状态。

```text
comp status
```

输出示例：

```text
comp mode=armed
```

模式值：`armed`（安全静止）、`running`（序列执行中）、`fault`（故障锁定）、`dev-running`（开发模式）。

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
