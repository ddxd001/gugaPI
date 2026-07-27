# gugaPI 调试 Shell 命令说明

本文档说明 gugaPI 固件当前支持的调试 shell 命令。调试串口为 `115200 8N1`。

数字参数支持十进制或 `0x` 开头的十六进制，例如 `16` 和 `0x10`。

DEBUG UART 固定使用 UART6（PC11/TX、PC10/RX）。比赛配置仍保留
Shell 命令解析，但关闭 banner、提示符、输入回显和调试级日志。

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

查看调试 UART（UART6）TX/RX 队列和 TX DMA 状态。仅在开发配置（`FEATURE_ENABLE_DEBUG_UART`）下可用。

```text
txstat
```

输出字段中，`queued`/`dropped`表示待发送字节数和累计丢弃字节数；`dma_active`表示当前是否有TX块正在传输；`dma_blocks`是已完成的256字节以内DMA块数；`dma_errors`应始终为0。发送层采用4096字节环形缓冲和最大256字节的连续DMA块，DMA完成中断自动衔接下一块。

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

按键任务每 5 ms 扫描一次，使用 20 ms 软件消抖和 800 ms 长按门限。`button` 输出中的
`held_ms` 是当前或最近一次按压时长，`events` 四位依次表示 `D`(按下)、`U`(松开)、
`S`(短按)、`L`(长按)。命令会取走显示出的待处理事件。

开发配置还会在事件产生时自动打印，不消费上述待处理事件；比赛配置默认关闭自动打印：

```text
button event button1 types=pressed held_ms=0
button event button1 types=released,short held_ms=126
button event button1 types=long held_ms=800
```

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

## 3S1P 电量监测

电量监测复用 INA219 的 100 ms 后台采样，当前固定按 `3S1P`、`3000 mAh` 锂离子电池包计算。上电后先收集 20 个有效电压样本并根据整包电压估算初始 SOC，随后根据电流进行库仑积分。电流正值表示放电，负值表示充电。

所有电量状态只保存在 SRAM 中，不写入 FRAM；MCU 复位或断电后会重新执行电压估算。持续负载、电机启动压降、电芯老化和温度都会影响电压估算精度，因此 `soc` 适合用作运行状态参考，不代替 BMS，也不能判断三节串联电芯是否失衡。

### `battery status`

查看当前电池包电压、平均单节电压、电流、功率、SOC、累计消耗量、峰值电流、INA219 溢出和读取错误计数。

```text
battery status
```

关键字段：

| 字段 | 含义 |
| --- | --- |
| `source=estimating` | 尚在收集上电电压样本，`ready=0` |
| `source=voltage` | 初始 SOC 来自电压估算，之后使用电流积分 |
| `source=full` | 已通过 `battery full` 将本次运行的 SOC 校准为 100% |
| `remaining_uAh` / `consumed_uAh` | 本次运行期估算的剩余/消耗容量 |
| `low` / `critical` | 整包滤波电压低于 10.8 V / 9.9 V；仅作提示，不触发电机保护 |
| `overflow` | INA219 数学溢出次数；若持续增加，应检查分流器和电流量程 |

### `battery full`

确认电池包确实充满后，把本次运行期的剩余容量校准为 `3000 mAh`、SOC 设为 100%。该命令不写 FRAM。

```text
battery full
```

### `battery reset`

清除本次运行期的 SOC、积分量、峰值和错误计数，并重新收集 20 个电压样本。该命令不复位 INA219，也不写 FRAM。

```text
battery reset
```

### `battery log on [period_ms]` / `off` / `status`

周期输出实时电量采样，默认周期为 500 ms，可设置为 `100..5000` ms。任务由协作式调度器驱动，不保存历史数据；串口发送队列接近满时会主动丢弃当前一帧。

```text
battery log on
battery log on 1000
battery log status
battery log off
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

打印周期采样任务（5 ms 一次，200 Hz，`app_imu`）缓存的最新数据。IMU SPI 为 1 MHz、mode 3，每次突发读取 14 字节；数据按量程换算为 mg / mdps / centi℃，并减去 `ConfigStore` 里的 `imu_accel_bias_*` / `imu_gyro_bias_*` 偏置。
```text
imu sample
```
输出：
```text
imu acc=<mg>,<mg>,<mg> mg gyr=<mdps>,<mdps>,<mdps> mdps t=<cC> cC yaw=<deg> deg
```
若未就绪会提示 `imu sample: no data (run 'imu icm init')`。

### `imu bias ...`

在确认底盘静止后，用 2 秒窗口自动估计 ICM45686 Z 轴残余零偏。运行时零偏
只保存在 RAM；只有显式 `save` 才与固定零偏合并并写入 FRAM。

```text
imu bias status
imu bias calibrate
imu bias auto on
imu bias auto off
imu bias save
imu bias reset
```

- `status`：显示固定/运行时/总零偏、采集进度、标准差和拒绝原因；
- `calibrate`：请求一次静止校准，只更新 RAM；
- `auto on|off`：启停运行中自动静止学习；
- `save`：静止时合并零偏并保存到 FRAM；
- `reset`：只清除 RAM 运行时零偏，不擦除固定零偏。

完整启动、保存和排障流程见 `docs/IMU_BIAS_GUIDE.md`。

### `imu oled on [period_ms]` / `off` / `status` / `once`

把 IMU 角度数据显示到 OLED（沿用 INA219 的 OLED 方案，与其他 OLED 数据源互斥）。周期采样任务（5 ms）算出俯仰/横滚角并积分 Z 轴角速度得到相对 Yaw，OLED 任务按 `period_ms` 刷新。

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

8:1 多路复用灰度阵列：3 个选位引脚（PA16=bit2、PC20=bit1、PC21=bit0）选 1 路，PA15（ADC1 ADCIN0）读模拟值（0..4095）。1 ms 周期任务连续采集中间 1..6 路，每 7 ms 左右发布一个位置帧；最外侧 0、7 路在 14 ms 内轮流更新。每路使用 ADC 四次硬件平均。详见 `GRAYSCALE_GUIDE.md`。

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

打印周期任务缓存的最新完整 8 路帧（不触发新转换）。
```text
gray data
```
未就绪时提示 `gray: no data`。

### `gray process`

查看统一处理链输出，不触发新的 ADC 转换：

```text
gray process
```

输出包含归一化值、迟滞位图 `mask`、主循迹区 `track`、实际插值通道 `selected`、
异常位图、线位置、线强度、道路类型和迟滞阈值。`pos` 使用左正右负坐标，
`pos_valid` 表示位置是否可用于闭环，`confidence` 范围 0..1000，`track_state` 为
`valid/lost/multiple/wide/sensor_fault`，`weak_frames` 为有界弱模拟跟踪帧数，`invalid_frames` 为连续异常帧数。调试通道顺序、黑白极性、
岔路选择和丢线判断时应以此命令为准；循迹控制消费同一份处理结果。

### `gray calib ...`

推荐使用白、黑两阶段多帧平均标定：

```text
gray calib white
gray calib status
gray calib black
gray calib status
gray calib commit
param save
```

`white`/`black` 默认采集 64 个位置帧，可设为 1..128。两个阶段都完成后执行
`commit`；去掉两端各 1/8 样本后求均值，每个通道的黑白跨度必须至少为 400 ADC counts，且至少为采集噪声的 8 倍。`commit` 更新标定值和推荐处理参数，并
将 ConfigStore 标记为 dirty，断电保存还需执行 `param save`。

辅助命令：

```text
gray calib show
gray calib status
gray calib reload
gray calib sweep [ms]
gray calib cancel
```

`show` 查看当前白点、黑点和处理参数；`reload` 从 ConfigStore 重新装载；`sweep`
是在黑线和白底间扫动的兼容标定方式，默认 2000 ms，并假定白色 ADC 值高于黑色；
极性相反时必须使用显式 `white`/`black` 标定。`cancel` 终止尚未完成的采集。

### `gray oled on [period_ms]`

按 INA219 OLED 显示任务的同样风格，将灰度插值位置和底盘左右轮速度持续显示到 OLED。页面直接读取应用层已有快照，不会额外触发 ADC 采样或 MotorDriver 通信。开启后会关闭其它传感器的 OLED 周期显示任务。
```text
gray oled on
gray oled on 200
gray oled off
gray oled status
gray oled once
```

OLED 4 行显示格式：
```text
P:374 V:1 C:1000
S:1310 W:0 I:0
T L:56 R:64
A L:55 R:63
```

`P` 是归一化模拟量加权得到的插值位置，`V` 是位置有效标志，`C` 是置信度；
`S/W/I` 分别是线强度、弱跟踪窗口帧数和连续异常帧数。`T` 是左右目标 RPM，
`A` 是底盘反馈的左右实际 RPM。

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
motor ramp 1500 2000
```

普通目标转速变化受斜坡限制；停车、故障和控制器禁用仍立即清除输出。设置成功后还会同步更新
gugaPI ConfigStore 的 `speed_accel_rpm_s` / `speed_decel_rpm_s` 并标记dirty；执行
`param save` 后写入FRAM。下一次 `Chassis_Init()` 会自动重新下发到MotorDriver。

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

### 固定角度位置控制

```text
motor m1 pos <deg>
motor m1 posrel <deg>
motor m2 pos <deg>
motor m2 posrel <deg>
```

`pos` 设置相对于编码器零点的绝对角度，`posrel` 从当前编码器位置继续转动指定角度，正负号表示两个方向。默认位置参数为 `kp=15`、`ki=0`、`kd=0`、`max_rpm=40`、`tol_counts=3`；启动时还会设置 `min_duty=6`、`max_duty=10`、`exit_tol_counts=5`、`settle_ms=0`。

gugaPI 会通过已有的 100 ms `chassis` 服务自动刷新位置目标，避免超过 MotorDriver 的 1 秒看门狗期限。控制器第一次越过目标时不会立即结束；误差必须连续 5 个服务周期保持在到位窗口内，随后才切换到 coast。任一周期重新超出窗口都会清除稳定计数并继续纠偏。

查看和临时修改位置参数：

```text
motor pos
motor pospid
motor pospid 15 0 0 40 3
motor posctl
motor posctl 6 10 5 0
```

`motor pospid` / `motor posctl` 只修改 MotorDriver 当前运行值。需要让 `pospid` 参数跨 gugaPI 重启保存，应同时使用 `param set position_*` 和 `param save`；`posctl` 使用上述经过实机验证的启动默认值。

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

通过线速度和角速度设置底盘目标。线速度范围 `-5000..5000 mm/s`，角速度范围 `-720000..720000 mdeg/s`。内部根据高精度参数 `wheel_radius_um` 和 `wheel_track_mm` 换算为左右轮 RPM。

```text
chassis vel 200 0
chassis vel 0 90000
```

## 航向闭环

航向闭环使用 ICM-45686 陀螺仪 Z 轴 yaw 积分实现行走中的直行保持、相对角度转弯，以及带航向修正的编码器距离行驶。航向源为 IMU yaw（毫度），不是 GY931。

10 ms 周期任务 `Heading_Update` 消费最新的 200 Hz IMU 数据并输出左右轮差速命令。转弯模式根据陀螺仪 Z 轴角速度预测短时惯性转角并提前制动；只有在航向进入配置容差、角速度及两轮实际转速均低于配置阈值后才开始稳定计时。距离模式同时消费20 ms底盘编码器反馈。安全机制：IMU无效或数据过期（>200 ms）、编码器反馈超过100 ms未更新、航向误差超过90°、转弯或距离行为超时，均会停车并置故障。

### `heading status`

查看航向闭环当前状态。

```text
heading status
```

输出字段：

| 字段 | 含义 |
| --- | --- |
| `mode` | `idle` / `hold` / `turn` / `distance` |
| `target` | 目标航向（度） |
| `error` | 当前航向误差（度，最短角度差） |
| `corr` | 航向差速修正RPM（hold/distance）或转弯速度（turn） |
| `at_target` | 转弯或距离行为是否进入目标容差区间 |
| `last` | 上一次操作结果 |
| `profile` | 距离曲线阶段：`idle/legacy/accel/cruise/brake/creep/settle` |
| `profile_rpm` | gugaPI曲线当前输出的基础转速绝对值 |
| `brake_mm` | 根据实测转速计算的当前预计制动距离 |

### `heading hold <base_rpm>`

启动直行航向保持。锁定当前 IMU yaw 为目标航向，以 `base_rpm` 为基础速度直行，根据航向误差差速修正。范围为 `-max_wheel_rpm..max_wheel_rpm`。

这是“车辆行走时的航向保持”，没有位置完成条件，会持续运行直到执行 `heading stop`、被动作序列切换或发生故障；它不是独立的静止航向角保持。

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

### `heading turncfg [set|save]`

查看、修改和持久化预测制动参数：

```text
heading turncfg
heading turncfg set 60 500 1500 3
heading turncfg save
```

`set`的四个参数依次为：

| 参数 | 范围 | 含义 |
| --- | ---: | --- |
| `brake_ms` | 0..500 | 用当前Z轴角速度预测惯性转角的时间窗口 |
| `margin_mdeg` | 0..30000 | 在预测惯性角之外附加的固定提前量 |
| `settle_mdps` | 0..60000 | 允许开始稳定计时的最大Z轴角速度 |
| `settle_rpm` | 0..100 | 允许开始稳定计时的最大左右轮实测RPM |

`set`立即修改RAM参数；`save`写入FRAM。也可分别使用
`param get/set heading_turn_*`进行上位机自动调参。

### `heading lock` / `heading lockcfg`

静止锁向会捕获启动瞬间的ICM-45686相对Yaw。角度处于死区内时车轮保持
零转速；受到外力偏转超过唤醒角度后，使用独立PD参数原地回正，稳定后继续等待
下一次扰动。该模式只保持航向，不恢复车辆的平面位置。

启动前保持车辆静止至少2秒，并确认动态零偏已经有效：

```text
imu bias status
chassis stop
heading lock
heading status
heading stop
```

`imu bias status`中的`valid`必须为1，底盘目标与实测轮速必须为零附近，否则
`heading lock`分别返回`not-initialized`或`busy`。

锁向参数可逐项实时修改：

```text
heading lockcfg
heading lockcfg set kp 1500
heading lockcfg set kd 250
heading lockcfg set wake 2000
heading lockcfg set settle 800
heading lockcfg set minrpm 10
heading lockcfg set maxrpm 30
heading lockcfg set rate 1500
heading lockcfg set wheelrpm 3
heading lockcfg set settlems 250
heading lockcfg set timeout 3000
heading lockcfg save
```

`settle`必须小于`wake`，`minrpm`不得大于`maxrpm`，`maxrpm`不得超过
`max_wheel_rpm`。一次回正超过`timeout`后电机停止，`heading status`显示
`lock_phase=failed`和`lock_result=timeout`；物理阻挡不会单独触发全局故障。
IMU或MotorDriver通信故障仍使用现有全局安全停车路径。

### `heading distance <mm> <max_rpm> [timeout_ms]`

按编码器距离闭环行驶，并锁定启动瞬间的IMU航向。正距离前进，负距离倒退；范围为
`-10000..10000 mm`（不能为0）。`max_rpm` 是本次动作的巡航速度。具体加减速行为由
`heading profile` 选择，航向误差仍通过左右轮差速修正。

```text
heading distance 500 60
heading distance -200 40
heading distance 1000 80 15000
heading status
```

`timeout_ms` 可省略，固件根据距离、轮径和最大RPM生成有界超时。梯形模式在进入目标
容差或越过目标后立即停车，不会反向寻找；左右实测速度连续3次低于 `settle_rpm` 且
至少经过100 ms后回到 `idle`。该行为依赖正确的
`wheel_radius_um`、左右轮 `counts_per_rev` 和编码器方向配置。

65 mm 轮胎、13 PPR 霍尔编码器、28:1 减速比的理论参数为：

```text
param set left_counts_per_rev 1456
param set right_counts_per_rev 1456
param set wheel_radius_um 33050
param save
```

`wheel_radius_um=33050` 表示本车实测标定后的 `33.050 mm` 有效滚动半径。`wheel_radius_mm` 仍作为旧脚本兼容入口，但只能设置整数毫米；设置该旧参数会同时覆盖高精度值。

### `heading profile`

查看或设置定距动作的速度曲线。它只整形gugaPI发送给MotorDriver的RPM目标，**不会修改
MotorDriver当前100 ms速度环周期**，也不修改调度器。

```text
heading profile
heading profile mode trapezoid
heading profile accel 600
heading profile decel 900
heading profile creep 15
heading profile latency 360
heading profile margin 5
heading profile settle 3
heading profile tolerance 3
heading profile save
```

模式：

- `legacy`：保留旧的“剩余毫米数映射RPM”行为，越过目标后可能反向修正，仅用于回归对比。
- `trapezoid`：加速、巡航、预测制动、单方向低速逼近和停稳五阶段；默认模式。

参数：

| Shell项 | 持久化参数 | 范围 | 默认值 | 含义 |
| --- | --- | ---: | ---: | --- |
| `accel` | `distance_accel_rpm_s` | 1..5000 | 600 | 上层RPM命令加速度 |
| `decel` | `distance_decel_rpm_s` | 1..5000 | 900 | 上层RPM命令减速度及制动距离模型 |
| `creep` | `distance_creep_rpm` | 1..500 | 15 | 终点前最低逼近速度 |
| `latency` | `distance_stop_latency_ms` | 0..2000 | 360 | 固定时间延迟补偿；对应距离按实时轮速动态计算 |
| `margin` | `distance_brake_margin_mm` | 0..1000 | 5 | 额外提前制动距离 |
| `settle` | `distance_settle_rpm` | 0..100 | 3 | 判定车轮停稳的RPM阈值 |
| `tolerance` | `distance_tolerance_mm` | 1..100 | 3 | 终点容差 |

修改后立即作用于下一次定距动作，并将参数标为dirty；执行 `heading profile save` 或
`param save` 才会写入FRAM。旧V1～V7配置加载后使用上述默认曲线参数；旧V1～V8配置
加载后使用1500/2000 RPM/s的MotorDriver ramp默认值。兼容加载会标记dirty，保存后统一
升级为V9。定距动作正在运行时，`heading profile` 只允许查看，修改或保存返回`busy`；
先执行 `heading stop`。这里的600/900 RPM/s是gugaPI定距目标整形参数，与底层持久化的
MotorDriver ramp 1500/2000 RPM/s是两层不同的限速。

### `heading stop`

停止航向闭环并停车。

```text
heading stop
```

## 循迹控制

8 路灰度循迹。需先标定再循迹。灰度任务周期为 1 ms，中间六路位置帧约 7 ms；10 ms 周期任务 `LF_Update` 只在帧序号变化时消费结果。连续位置由 `track_mask=0x7E` 的中间六路插值，全八路迟滞位图独立识别道路类型，最外侧 0、7 路不拉动循迹质心。

安全机制：灰度数据无效或超过 200 ms、通道诊断异常 → 立即停车。强线之后允许短暂全白间隙，并可在相邻单段弱模拟信号重新出现时继续位置插值；全白与弱跟踪共享最多 8 个完整帧（约 56 ms）的恢复预算。每个全白帧同时计入独立的连续异常计数，相邻弱线恢复会将该计数清零；`lost/multiple/wide` 连续 6 个完整帧（约 42 ms）仍异常即停车。因此连续全白不会等待完整 56 ms，也不进行无限保持或盲目搜线。`confidence` 只作诊断，不再独立决定停车。正常跟踪期间始终使用命令指定的基础速度，不根据位置误差自动降速。

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
| `error` | 线路位置误差（默认约 `-1500..+1500`，0 = 居中） |
| `corr` | 当前差速修正 RPM |
| `lost` | 当前是否丢线 |
| `kp` | 循迹比例增益 |
| `kd` | 循迹微分增益 |
| `maxcorr` | 40 RPM 参考速度下的最大修正 RPM |
| `seq` | 最后消费的灰度完整帧序号 |
| `road` | 两帧确认后的道路类型 |
| `pos_valid` | 当前插值位置是否可用于闭环 |
| `selected` | 实际用于插值的连续通道位图 |
| `confidence` | 位置可信度，0..1000 |
| `source` | `core` / `left_edge` / `right_edge` / `held` / `none` |
| `track_state` | `valid` / `lost` / `multiple` / `wide` / `sensor_fault` |
| `weak_frames` | 有界弱模拟跟踪的连续帧数，0 表示当前使用正常强度证据，最大 8 |
| `invalid_frames` | 连续几何异常完整帧数 |
| `invalid_policy` | `confirm6` 表示连续 6 个几何异常完整帧（约 42 ms）后停车；硬件、过期和通道异常仍立即停车 |
| `ref_rpm` | 转向比例换算的参考速度，当前为 40 RPM |
| `max_ratio_permille` | 修正量相对基础速度的硬限幅，当前为 400‰ |
| `deadband` | 中心误差死区，单位为位置刻度 |

### `lf cal`

启动兼容扫动标定（2 秒）。在黑线和白底之间来回扫动，记录各通道 min/max；每通道跨度至少 400 才会更新统一灰度校准并设 `calibrated = true`。推荐精确校准使用 `gray calib white/black/commit`。

```text
lf cal
```

### `lf start <rpm> <ms>`

启动循迹，以 `rpm` 为基础速度循迹 `ms` 毫秒。`rpm` 范围 `-max_wheel_rpm..max_wheel_rpm`，`ms` 范围 `0..30000`。必须先完成标定。

```text
lf start 80 10000
```

修正先在 40 RPM 参考速度计算：`reference_correction = (error_mpos * kp + filtered_derivative * kd) / 1e6`，再按 `abs(base_rpm) / 40` 缩放并限制在基础速度的 40%。中心 `±50` 位置刻度使用死区，微分滤波时间常数为 40 ms。修正量变化率由 `lf_slew_permille_s` 限制，默认 25000；从最大左修正切换到最大右修正的理论斜率时间约 32 ms。`left = base - correction`，`right = base + correction`。

左右轮目标RPM占用MotorDriver连续寄存器，正常循迹更新使用一次4字节I²C块写入和一次4字节读回校验，避免两轮分开发送产生的时间差并减少总线事务。

### `lf stop`

停止循迹并停车。

```text
lf stop
```

### `lf kp <val>`

设置循迹比例增益（范围 `0..1000000`），立即生效并将配置标记为 dirty。当前实车在
100 RPM 调定的默认值为 3800。

```text
lf kp 3800
```

### `lf kd <val>`

设置滤波微分增益（`0..1000000`）。当前实车在 100 RPM 调定的默认值为 600。

```text
lf kd 600
```

### `lf maxcorr <val>`

设置 40 RPM 参考速度下的最大修正 RPM（范围 `0..500`）。实际修正按基础速度同比缩放，并额外受 40% 转向比例硬限幅。默认 30。

```text
lf maxcorr 50
```

### `lf slew <permille_per_s>`

设置循迹差速修正的最大变化率（范围 `1..65535`，默认 25000）。单位是“基础转速的千分之一每秒”；数值越大响应越快，数值过大则会增加转向冲击和灰度噪声敏感度。命令立即生效并将配置标记为 dirty，断电保存需要执行 `param save`。

```text
lf slew 25000
param save
```

### `lf losthold <ms>`

兼容旧配置的保留命令，必须不大于 `losttimeout`。当前策略为硬件异常立即停车、几何异常确认 6 帧，此参数不再产生丢线搜索运动。默认值仍为 150 ms。

```text
lf losthold 150
```

### `lf losttimeout <ms>`

兼容旧配置的保留命令（`0..10000` ms），必须不小于 `losthold`。当前策略为硬件异常立即停车、几何异常确认 6 帧，此参数不再控制停车延迟。默认值仍为 500 ms。

```text
lf losttimeout 1000
```

## 路口事件

路口检测使用全八路灰度掩码和连续帧状态机，类型包括`left_corner`、
`right_corner`、`left_branch`、`right_branch`、`t`和`cross`。事件只在进入路口时
生成一次；恢复连续6帧居中直线后才允许生成下一事件。

路口几何与动作策略分离。当前只有左右直角弯具有可选自动动作；左右分支和十字默认
保持直行，T字无前路默认停车。预留的动作序列策略尚未在本版本启用。

### `road status|event|clear`

```text
road status
road event
road clear
```

- `status`：显示控制模式、控制阶段、检测器阶段、当前类型、已观察路径和最后策略。
- `event`：显示最近一次事件的序号、类型、路径位、置信度以及进入/峰值/离开掩码。
- `clear`：清除Shell可见的最近事件，并将控制器消费位置同步到当前事件。

路径位为：左=`0x01`、前=`0x02`、右=`0x04`。

### `road mode detect|corner`

```text
road mode detect
road mode corner
```

上电默认`detect`，只检测和记录左右直角弯，不自动启动转向。`corner`允许循迹状态下的
`left_corner`和`right_corner`依次执行停车、可选前进对齐、相对90°航向转动、黑线
重捕获和恢复循迹。`road auto on|off`分别是`corner`和`detect`的简写；在自动弯道
正在执行时关闭自动模式会立即停止航向及循迹控制。

### `road turn show|set`

```text
road turn show
road turn set 90 -90 0 30 800
```

`set`依次设置左转角、右转角、转前对齐距离mm、对齐最大RPM和转后黑线重捕获超时ms。
范围分别为`1..180`、`-180..-1`、`0..300`、`1..300`、`1..5000`。参数立即生效但
本版本不写FRAM，复位后恢复`90/-90/0/30/800`。

## 动作序列

条件驱动的指令表解释器，通过 `run add` 逐条构建指令序列，`run start` 启动。50 ms 周期任务 `ActionRunner_Update` 执行当前指令，满足完成条件后跳转到 `on_success` / `on_timeout` 目标。

安全机制：故障 → 中止序列并停车；整序列超时 60 s → 中止；指令启动失败 → 走 `on_timeout` 路径；每条指令完成后调用 `StopAll` 清除运动状态。

### 指令格式

每条指令 6 个参数：

```text
run add <op> <param1> <param2> <until> <onsuccess> <ontimeout>
```

| 参数 | 含义 |
| --- | --- |
| `op` | 操作码：`drive` / `drive_mm` / `turn` / `follow` / `wait` / `stop` / `branch` / `end` |
| `param1` | DRIVE/FOLLOW: 基础RPM；TURN: 相对角度；DRIVE_MM: 有符号毫米 |
| `param2` | 一般为超时/持续时间ms；DRIVE_MM为最大RPM |
| `until` | 完成条件：`timeout` / `heading_reached` / `distance_reached` / `line_detected` / `line_lost` / `button` / `immediate` |
| `onsuccess` | 成功跳转目标：`next`（下一条）或索引 `0..15` |
| `ontimeout` | 超时跳转目标：`abort`（中止序列）或索引 `0..15` |

操作码说明：

| 操作码 | 动作 | 典型条件 |
| --- | --- | --- |
| `drive` | 航向保持直行（`heading hold`） | `timeout` / `line_detected` / `line_lost` |
| `drive_mm` | 编码器距离闭环并保持启动航向 | `distance_reached` |
| `turn` | 相对角度转弯（`heading turn`） | `heading_reached` |
| `follow` | 循迹（`lf start`） | `timeout` / `line_lost` |
| `wait` | 等待（不产生运动） | `timeout` / `button` |
| `stop` | 立即停车 | `immediate` |
| `branch` | 条件跳转（不产生运动），成功走 onsuccess，失败走 ontimeout | 任意条件 |
| `end` | 序列完成（成功） | `immediate` |

### `run add <op> <p1> <p2> <until> <onsuccess> <ontimeout>`

追加一条指令到序列末尾。最多 16 条。

```text
run add drive  80  5000  timeout          next abort
run add drive_mm 500 60 distance_reached  next abort
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

参数持久化系统。所有底盘几何、速度环、位置环、距离速度规划、IMU 偏置、航向闭环、电源保护和灰度循迹参数统一存储在 FRAM 中（地址 0x0000，magic "CFPG"，CRC32 校验）。当前版本 v13，payload 215 字节，兼容加载 v1-v12 历史布局。v12配置加载时保留全部旧值，并为新增静止锁向参数填充默认值；迁移后配置标记为dirty，保存后升级为v13。V9及更早配置自动使用新的循迹斜率默认值25000；v10及更早的旧版默认灰度掩码 `0x3C` 自动迁移为 `0x7E`，其它自定义掩码保持不变。

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

查看所有参数或单个参数。不带参数列出全部参数（含当前值和合法范围）。

```text
param get
param get heading_kp
```

输出示例：

```text
param heading_kp=1000 range=0..100000
```

### `param export [start [count]]`

分页批量读取 RAM 中的参数，供上位机快速刷新使用，不访问 FRAM。`start` 是参数表
索引，允许等于参数总数；`count` 范围为 `1..16`，默认16。每页先输出实际起点、数量
和总数，随后沿用 `param get` 的参数行格式：

```text
param export 0 16
param export start=0 count=16 total=79
param left_counts_per_rev=1456 range=1..100000000
...
```

新版上位机优先分页读取；连接不支持该命令的旧固件时，会自动退回逐项执行
`param get <name>`。现有 `param get` 接口保持不变。

参数列表：

| 参数名 | 范围 | 默认值 | 说明 |
| --- | --- | --- | --- |
| `left_counts_per_rev` | 1..100000000 | 1456 | 左轮编码器 CPR（含QEI四倍频） |
| `right_counts_per_rev` | 1..100000000 | 1456 | 右轮编码器 CPR（含QEI四倍频） |
| `wheel_radius_um` | 1000..1000000 | 33050 | 实测有效滚动半径（微米），内部换算使用此值 |
| `wheel_radius_mm` | 1..1000 | 32 | 兼容旧脚本的整数毫米入口；设置后会覆盖 `wheel_radius_um` |
| `wheel_track_mm` | 1..2000 | 160 | 轮距（mm） |
| `max_wheel_rpm` | 1..1000 | 1000 | 最大轮速（RPM） |
| `motor_output_invert_flags` | 0..3 | 3 | 电机输出反向标志 |
| `motor_encoder_invert_flags` | 0..3 | 1 | 编码器反向标志 |
| `speed_kp` | 0..255 | 2 | 速度环 Kp（Q4.4） |
| `speed_ki` | 0..255 | 2 | 速度环 Ki（Q4.4） |
| `speed_kd` | 0..255 | 0 | 速度环 Kd（Q4.4） |
| `speed_max_duty` | 0..100 | 60 | 速度环最大占空比（%） |
| `speed_min_duty` | 0..100 | 4 | 速度环最小占空比（%） |
| `speed_accel_rpm_s` | 0..65535 | 1500 | MotorDriver目标转速加速斜坡（RPM/s，0表示立即跟随） |
| `speed_decel_rpm_s` | 0..65535 | 2000 | MotorDriver目标转速减速斜坡（RPM/s，0表示立即跟随） |
| `position_kp` | 0..255 | 15 | 位置环 Kp（Q4.4） |
| `position_ki` | 0..255 | 0 | 位置环 Ki（Q4.4） |
| `position_kd` | 0..255 | 0 | 位置环 Kd（Q4.4） |
| `position_max_rpm` | 0..1000 | 40 | 位置环最大转速（RPM） |
| `position_tolerance_counts` | 0..65535 | 3 | 位置环容差（counts） |
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
| `lf_slew_permille_s` | 1..65535 | 25000 | 循迹差速修正变化率（基础RPM的千分之一/秒） |

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

比赛模式状态机：`ARMED`（安全静止并选择任务）→ `RUNNING`（序列执行中）→ `ARMED`。在比赛配置（`FEATURE_PROFILE_COMPETITION=1`）下上电自动进入 ARMED；开发配置下可用 `comp arm` 手动进入。

ARMED 下按键 1/3 在槽位 0..7 间向左/向右循环，按键 2 短按加载并启动当前槽位。RUNNING 下按键 2 短按取消任务并停车；按键 1 保留给 ActionRunner 的 `button` 条件。OLED 显示当前槽位、有效性、步数、运行进度和结束结果，FAULT 界面始终具有最高优先级。

LED 指示：ARMED 慢闪（1Hz）、RUNNING 常亮、FAULT 快闪（5Hz）；蜂鸣器保持关闭。

### `comp arm`

从开发模式进入比赛武装状态。取消现有 ActionRunner、停止底盘、禁用 chassis 任务、关闭传感器 OLED 周期页面，并选择最低编号的有效序列槽。仅在 `dev-running` 模式下可用。

```text
comp arm
```

### `comp select <0..7>`

在 ARMED 状态选择当前比赛序列槽，不加载或启动任务。EMPTY 槽也允许选择。

```text
comp select 3
```

### `comp start [0..7]`

校验并从 FRAM 加载当前槽，然后启动比赛序列。可选槽位参数会先更新当前选择。EMPTY、CRC 错误或 FRAM 读取失败时保持 ARMED 且不会产生电机动作。可通过按键 2 短按替代。仅在 `armed` 模式下可用。

```text
comp start
comp start 3
```

### `comp stop`

取消比赛序列、立即停车并返回武装状态。OLED 显示 `STOPPED` 2 秒。可通过按键 2 短按替代。仅在 `running` 模式下可用。

```text
comp stop
```

### `comp status`

查看比赛模式、当前槽位、槽位有效性、指令数、步骤、最近结果和状态码。

```text
comp status
```

输出示例：

```text
comp mode=armed slot=3 valid=1 any_valid=1 count=6 step=0 result=none last=ok
```

模式值：`armed`（安全静止）、`running`（序列执行中）、`fault`（故障锁定）、`dev-running`（开发模式）。结果值包括 `none`、`done`、`failed`、`stopped` 和 `load-error`。

## 遥测（FireWater / VOFA+）

FireWater 协议周期输出 CSV 数据，可被 VOFA+ 串口示波器直接接收实时画图。非阻塞，TX 缓冲接近满时自动丢帧。

通道定义：

| 通道 | 含义 |
| --- | --- |
| `t` | 系统运行时间（ms） |
| `mode` | App 模式（0=idle, 1=running, 2=fault, 3=armed, 4=comp-running） |
| `step` | ActionRunner 当前步（-1=未运行） |
| `L_tgt` | 左轮目标 RPM |
| `L_act` | 左轮实测 RPM |
| `R_tgt` | 右轮目标 RPM |
| `R_act` | 右轮实测 RPM |
| `yaw_tgt` | 航向目标（度） |
| `yaw` | 当前 yaw（度） |
| `head_err` | 航向误差（度） |
| `head_corr` | 航向修正量（RPM） |
| `gray_pos` | 灰度加权插值位置（mpos，左正右负） |
| `gray_strength` | 核心通道归一化线强度之和 |
| `gray_conf` | 插值位置置信度（0..1000） |
| `gray_valid` | 插值位置有效标志（0/1） |
| `gray_state` | 灰度轨迹状态枚举值 |
| `lf_err` | 循迹控制器当前位置误差（mpos） |
| `lf_corr` | 循迹左右差速修正量（RPM） |
| `lf_weak` | 弱线恢复窗口帧数 |
| `lf_invalid` | 连续无效灰度帧数 |
| `road_type` | 当前道路类型枚举 |
| `road_event_seq` | 最近路口事件序号，0表示尚无事件 |
| `road_event_type` | 最近路口事件类型枚举 |
| `road_paths` | 事件观察到的左/前/右路径位 |
| `road_phase` | 路口检测器阶段枚举 |
| `road_ctrl_phase` | 自动直角弯控制阶段枚举 |
| `head_turn_phase` | 航向转弯阶段：idle/drive/brake/settle枚举 |
| `head_turn_rate_mdps` | 当前转弯使用的Z轴角速度 |
| `head_turn_brake_mdeg` | 当前角速度计算出的动态制动角阈值 |
| `head_turn_brake_ms` | 配置的预测制动时间窗口 |
| `head_turn_margin_mdeg` | 配置的固定提前制动角 |
| `head_turn_settle_mdps` | 配置的稳定角速度阈值 |
| `head_turn_settle_rpm` | 配置的稳定轮速阈值 |

### `telem on [period_ms]`

开启遥测输出。默认周期 100ms（10Hz），范围 `50..5000`ms。开启时先发送通道名行（`#` 开头），然后周期输出数据行。

```text
telem on
telem on 200
```

输出示例：

```text
#t,...,road_ctrl_phase,head_turn_phase,head_turn_rate_mdps,head_turn_brake_mdeg,head_turn_brake_ms,head_turn_margin_mdeg,head_turn_settle_mdps,head_turn_settle_rpm
8435,...,0,1,125000,8000,60,500,1500,3
```

仓库中的上位机工具可以同时启用该 OLED 页面、执行一次有时间上限的巡线、保存 CSV
并生成 PNG 曲线（运行前必须确认场地安全并关闭占用串口的 VOFA+）：

```text
python host_tools/linefollow_capture.py --port COM14 --start-rpm 60 --run-ms 6000 --enable-oled
```

预测制动调试可执行一次有界相对转弯并生成33列CSV，以及目标角、实际角、轮速、动态
制动角、陀螺角速度和控制阶段曲线：

```text
python host_tools/heading_turn_capture.py --port COM14 --degrees 90
python host_tools/heading_turn_capture.py --port COM14 --degrees -90
```

该工具会产生真实底盘运动；运行前必须确认旋转范围安全。退出路径会发送
`heading stop`和`telem off`。

### `telem off`

关闭遥测输出。

```text
telem off
```

### `telem status`

查看遥测状态。

```text
telem status
```

## ADC 和 PWM 资源入口

没有硬件对象的通用 `adc`、`pwm` 占位命令已经删除。

- ADC1/PA15 属于八路灰度传感器，使用 `gray status|read|data|process`。
- 蜂鸣器使用 `buzzer on|off|toggle`，不是通用 PWM。
- 新增辅助 ADC/PWM 前必须先定义原理图网络、SysConfig 资源、参数范围和安全状态。

LoRa 帧协议、LIS3MDL、INA219 保护和灰度处理的新命令请参阅
`docs/devices/gugaPI/` 下对应设备手册。
