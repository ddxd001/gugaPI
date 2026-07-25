# MotorDriver

MotorDriver 是基于 MSPM0G3519 的双路 DRV8701 PH/EN 电机驱动固件。

当前阶段实现 UART 控制和 I2C 从机寄存器访问。固件支持：

- UART 寄存器读写和心跳。
- I2C 直接寄存器读写，默认 7-bit 地址 `0x20`。
- 双路开环 `coast` / `run` / `brake`。
- 增量式编码器计数、CPS、RPM 换算。
- 基于输出轴 RPM 的速度 PID。
- 基于编码器 count 的位置闭环，shell 侧支持角度命令。
- `speed 0` 位置保持：命令生效时记录当前编码器计数，并复用位置环参数保持该位置。

## 硬件配置

- MCU：MSPM0G3519，VQFN-48(RGZ)
- UART 控制口：UART4，PB17 TX，PB18 RX，115200 8N1
- I2C 从机：I2C1，PA17 SCL，PA18 SDA，默认地址 `0x20`
- 电机 1：PA5 PH，PA6 TIMG0_CCP1 EN PWM
- 电机 2：PB24 PH，PA23 TIMG0_CCP0 EN PWM（与 M1 共用 TIMG0 双通道）
- 电机 1 编码器：PB7 A，PB9 B
- 电机 2 编码器：PA22 A，PA21 B
- PWM 频率：20 kHz

## UART 帧格式

```text
0xAA CMD REG LEN DATA... CRC8
```

- CRC-8 多项式：0x07，初值 0x00，覆盖 SOF 到 DATA 的全部字节。
- CMD `0x01`：读寄存器，`LEN` 表示读取长度。
- CMD `0x02`：写寄存器，`LEN` 表示写入数据长度。
- CMD `0x03`：心跳，`LEN` 必须为 0。
- 响应 CMD 使用 `CMD | 0x80`。
- 状态响应数据字节：`0x00` OK，`0x01` ERROR。

CRC 错误时固件直接丢弃该帧，不返回响应。无效命令、无效寄存器、非法参数或超长帧会返回 ERROR。

## I2C 寄存器协议

I2C 不再套 UART 的 `0xAA CMD...CRC` 帧，而是直接访问同一张寄存器表：

```text
读：主控写 1 字节寄存器地址，然后 repeated START 读 N 字节
写：主控写 [寄存器地址, 数据...]
```

- 默认 7-bit 地址：`0x20`。
- 可通过 UART 写寄存器 `0x05` 修改地址（有效范围 `0x08`-`0x77`），修改后立即生效并掉电保存到 FLASH。
- 硬件文档中的 ADDR0/1/2 DIP 地址位后续再接入。
- I2C 读寄存器不会刷新 watchdog。
- I2C 写寄存器成功提交后会刷新 watchdog，和 UART 写命令一致。
- I2C 当前没有专用 heartbeat 寄存器；gugaPI 的 `motor ping` 在 I2C 模式下只做地址探测，不刷新 watchdog。

## 寄存器表

```text
0x00 DEVICE_ID          R       0xA5
0x01 FW_VERSION         R       0x02
0x02 STATUS             R
0x03 FAULT_FLAGS        R/W1C
0x04 CONTROL_FLAGS      R/W     默认 0x01，全局使能
0x05 I2C_ADDRESS        R/W     默认 0x20，可通过 UART 写入修改并掉电保存

0x10 M1_MODE            R/W     0 coast，1 run，2 brake，3 speed，4 position
0x11 M1_DUTY            R/W     0..100，speed/position 模式下为控制器输出
0x12 M1_DIRECTION       R/W     0 forward，1 reverse
0x13 M1_STATUS          R
0x14 M1_ENCODER_COUNT   R       int32 小端，寄存器 0x14..0x17
0x18 M1_ENCODER_CPS     R       int32 小端，寄存器 0x18..0x1B
0x1C M1_ENCODER_STATE   R       bit0=A，bit1=B

0x20 M2_MODE            R/W     0 coast，1 run，2 brake，3 speed，4 position
0x21 M2_DUTY            R/W     0..100，speed/position 模式下为控制器输出
0x22 M2_DIRECTION       R/W     0 forward，1 reverse
0x23 M2_STATUS          R
0x24 M2_ENCODER_COUNT   R       int32 小端，寄存器 0x24..0x27
0x28 M2_ENCODER_CPS     R       int32 小端，寄存器 0x28..0x2B
0x2C M2_ENCODER_STATE   R       bit0=A，bit1=B

0x30 WATCHDOG_TIMEOUT   R/W     单位 10 ms，默认 100，也就是 1 秒；写 0 关闭
0x31 ENCODER_CONTROL    W       bit0 reset M1，bit1 reset M2
0x32 M1_TARGET_RPM      R/W     uint16 小端，speed 目标 RPM；position 模式下为位置环输出
0x34 M2_TARGET_RPM      R/W     uint16 小端，speed 目标 RPM；position 模式下为位置环输出
0x36 M1_MEASURED_RPM    R       int16 小端，输出轴实测 RPM
0x38 M2_MEASURED_RPM    R       int16 小端，输出轴实测 RPM
0x3A SPEED_KP_Q4_4      R/W     默认 1
0x3B SPEED_KI_Q4_4      R/W     默认 1
0x3C SPEED_KD_Q4_4      R/W     默认 0
0x3D SPEED_MAX_DUTY     R/W     默认 50
0x3E SPEED_MIN_DUTY     R/W     默认 6

0x40 M1_COUNTS_PER_REV  R/W     uint32 小端，M1 输出轴每圈编码器计数，默认 364
0x44 M2_COUNTS_PER_REV  R/W     uint32 小端，M2 输出轴每圈编码器计数，默认 364
0x48 M1_HOLD_COUNT      R       int32 小端，M1 speed 0 保持目标
0x4C M2_HOLD_COUNT      R       int32 小端，M2 speed 0 保持目标

0x50 M1_TARGET_POSITION R/W     int32 小端，M1 position 目标 count
0x54 M2_TARGET_POSITION R/W     int32 小端，M2 position 目标 count
0x58 M1_POSITION_ERROR  R       int32 小端，目标 count - 当前 count
0x5C M2_POSITION_ERROR  R       int32 小端，目标 count - 当前 count
0x60 POSITION_KP_Q4_4   R/W     默认 20
0x61 POSITION_KI_Q4_4   R/W     默认 0
0x62 POSITION_KD_Q4_4   R/W     默认 0
0x63 POSITION_MAX_RPM   R/W     uint16 小端，默认 40，位置环内部 command RPM 上限
0x65 POSITION_TOLERANCE R/W     uint16 小端，默认 400 counts，进入目标区阈值
0x67 POSITION_STATUS    R       bit0 M1_AT_TARGET，bit1 M2_AT_TARGET
0x68 POSITION_MIN_DUTY  R/W     默认 5，位置修正最小占空比
0x69 POSITION_MAX_DUTY  R/W     默认 25，位置修正最大占空比
0x6A POSITION_EXIT_TOL  R/W     uint16 小端，默认 800 counts，离开目标区阈值
0x6C POSITION_SETTLE    R/W     10ms 单位，默认 10，也就是 100ms

0x6D M1_CONTROL_RPM     R       uint16 小端，速度斜坡后的控制目标
0x6F M2_CONTROL_RPM     R       uint16 小端，速度斜坡后的控制目标
0x71 M1_SPEED_ERROR     R       int16 小端，控制目标减去方向对齐后的实测 RPM
0x73 M2_SPEED_ERROR     R       int16 小端，控制目标减去方向对齐后的实测 RPM
0x75 M1_SPEED_INTEGRAL  R       int16 小端，速度环积分状态，Q4
0x77 M2_SPEED_INTEGRAL  R       int16 小端，速度环积分状态，Q4
0x79 M1_CONTROL_DUTY    R       速度控制器最终输出占空比
0x7A M2_CONTROL_DUTY    R       速度控制器最终输出占空比
0x7B SPEED_ACCEL        R/W     uint16 小端，RPM/s，默认 600；0 表示立即跟随
0x7D SPEED_DECEL        R/W     uint16 小端，RPM/s，默认 900；0 表示立即跟随
```

写寄存器是事务式的：整帧先校验，任意字段非法时整帧拒绝，不会部分修改电机输出。

## 状态位

`STATUS`：

```text
bit0 READY
bit1 ENABLED
bit2 ACTIVE
bit3 TIMEOUT
```

`FAULT_FLAGS`：

```text
bit0 WATCHDOG_TIMEOUT
```

`FAULT_FLAGS` 是 write-1-to-clear，写 1 清除对应故障位。

`M1_STATUS` / `M2_STATUS`：

```text
bit0 ACTIVE
bit1 REVERSE
bit2 AT_TARGET
```

## 控制语义

- 上电后两路电机默认 `coast`。
- `CONTROL_FLAGS` 默认使能。写 0 会立即清除两路运动命令、释放输出并解除 watchdog 活动状态。
- `motor stop` / `motor m1 coast` / `motor m2 coast` 表示释放输出，不保持位置。
- `motor m1 speed 0` / `motor m2 speed 0` 表示保持当前位置，不等价于 coast。
- `motor m1 hold` / `motor m2 hold` 进入正式 position 模式，并把当前 count 作为目标位置。
- `motor m1 pos <deg>` 使用绝对角度，角度零点来自当前编码器 count 零点。
- `motor m1 posrel <deg>` 使用相对角度，以执行命令时的当前位置为基准。
- `position` / `speed 0` 使用位置专用低速控制：位置误差 -> 内部 command RPM -> 位置专用 PWM，不再经过速度 PID 的 `SPEED_MIN_DUTY`。
- 进入目标区后输出为 coast；如果外力让误差超过 `POSITION_EXIT_TOL`，固件会重新给 PWM 拉回目标位置。这不是机械抱死。
- M1 使用 TIMG9 QEI 硬件计数，M2 使用 TIMG8 QEI 硬件计数；两个电机的 count、CPS、RPM 都来自 A/B 双相正交解码。

## Watchdog 规则

- `WATCHDOG_TIMEOUT` 默认 100，即 1 秒；写 0 关闭 watchdog。
- 活动命令包括：`run duty > 0`、`speed`、`speed 0`、`position`。
- 成功写入活动命令会刷新 watchdog。
- UART 心跳帧，也就是 UART 模式下的 gugaPI `motor ping`，会在存在活动命令时刷新 watchdog。
- I2C 模式下 `motor ping` 只探测地址，不刷新 watchdog；I2C 活动控制需要周期性写入命令或关闭 watchdog。
- 读寄存器不会刷新 watchdog。
- `stop/coast/brake/disable` 会解除活动命令，不产生超时故障。
- watchdog 超时会强制两路 `coast`，清零占空比和目标 RPM，并置位 `FAULT_FLAGS bit0`。

## 编码器和单位

当前默认参数按测试电机设置：

```text
当前实车标定值 = 364 counts/output-shaft revolution
```

更换电机时可以运行时覆盖输出轴每圈编码器计数：

```text
motor cfg <m1_counts_per_rev> <m2_counts_per_rev>
```

shell 的 `pos` / `posrel` 参数单位是输出轴角度 `deg`。固件寄存器仍使用 encoder count，换算公式为：

```text
count = deg * counts_per_rev / 360
```

默认速度 PID 参数等价于：

```text
motor pid 1 1 0 50 6
motor ramp 600 900
```

默认位置环参数等价于：

```text
motor pospid 20 0 0 40 400
motor posctl 5 25 800 100
```

位置参数的调试顺序建议：

- 先用 `motor posctl <min_duty> <max_duty> ...` 找到能让电机稳定拉回、但不过猛的占空比范围。
- 再用 `motor pospid <kp> 0 0 <max_rpm> <tol_counts>` 调位置误差到 command RPM 的比例。
- `tol_counts` 是进入目标区阈值，`exit_tol_counts` 是离开目标区阈值；后者应大于前者，用来避免左右摇摆。
- 第一版默认不使用位置环积分，`ki` 建议保持 0。

## gugaPI Shell 命令

基础通信：

```text
motor bus
motor bus uart
motor bus i2c
motor i2caddr
motor i2caddr 0x20
motor ping
motor info
motor status
```

寄存器读写：

```text
motor reg <addr> <len 1..32>
motor set <addr> <byte...>
```

编码器：

```text
motor enc
motor enc reset
```

速度、位置和配置：

```text
motor cfg
motor cfg <m1_counts_per_rev> <m2_counts_per_rev>
motor rpm
motor ramp
motor ramp <accel_rpm_s> <decel_rpm_s>
motor pid
motor pid <kp_q4.4> <ki_q4.4> <kd_q4.4> [max_duty [min_duty]]
motor pos
motor pospid
motor pospid <kp_q4.4> <ki_q4.4> <kd_q4.4> [max_rpm [tol_counts]]
motor posctl
motor posctl <min_duty> <max_duty> [exit_tol_counts [settle_ms]]
motor m1 speed <rpm 0..1000> [fwd|rev]
motor m2 speed <rpm 0..1000> [fwd|rev]
motor m1 hold
motor m2 hold
motor m1 pos <deg>
motor m2 pos <deg>
motor m1 posrel <deg>
motor m2 posrel <deg>
```

开环控制：

```text
motor m1 run <duty 0..100> [fwd|rev]
motor m2 run <duty 0..100> [fwd|rev]
motor m1 coast
motor m2 coast
motor stop
```

开环 `run` 只用于驱动输出测试。M1 使用 TIMG9 QEI 硬件计数，M2 使用 TIMG8 QEI 硬件计数；两路都不依赖 GPIO 编码器中断。

串口调试：

```text
motor hex <byte...>
motor read [len 1..64]
motor clear
```

## 测试建议

烧录：

```text
MotorDriver/Debug/MotorDriver.out
gugaPI/Debug/gugaPI.out
```

基础检查：

```text
motor bus
motor ping
motor info
motor cfg
motor pid
motor pospid
motor posctl
motor enc
```

I2C 链路检查：

```text
i2c status motor
i2c scan motor 0x20 0x27
motor bus
motor ping
motor info
motor reg 0x00 6
motor reg 0x10 4
```

预期：`i2c scan motor 0x20 0x27` 能看到 `0x20`，`motor info` 的 `i2c` 字段为 `0x20`。

位置保持测试：

```text
motor set 0x30 0x00
motor enc reset
motor m1 hold
motor pos
motor enc
motor stop
```

预期：`motor m1 hold` 后，M1 进入 position 模式；轻微拧动输出轴时电机会尝试回到进入 hold 时的位置。`motor stop` 后释放。

小角度相对运动测试：

```text
motor set 0x30 0x00
motor enc reset
motor m1 posrel 10
motor pos
motor rpm
motor m1 posrel -10
motor pos
motor stop
```

绝对角度测试：

```text
motor enc reset
motor m1 pos 90
motor pos
motor m1 pos 0
motor pos
motor stop
```

watchdog 测试：

```text
motor set 0x03 0x01
motor set 0x30 0x64
motor m1 posrel 10
motor ping
```

如果 1 秒内不继续执行 `motor ping` 或新的活动写命令，预期电机超时 coast，`motor info` 中 `status` 出现 TIMEOUT，`fault` 出现 bit0。清故障：

```text
motor set 0x03 0x01
```

## 构建

在 `MotorDriver/Debug` 或 `gugaPI/Debug` 目录执行：

```text
C:/ti/ccs2100/ccs/utils/bin/gmake.exe clean all
```
