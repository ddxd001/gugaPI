# MotorDriver

MotorDriver 是基于 MSPM0G3519 的双路 DRV8701 PH/EN 电机驱动固件。

当前版本支持 UART 控制、增量式编码器原始反馈，以及基于输出轴 RPM 的速度 PID 闭环控制。I2C 从机支持和角度闭环控制暂时不在当前阶段实现。

## 硬件配置

- MCU：MSPM0G3519，封装 VQFN-48(RGZ)
- UART 控制口：UART4，PB17 TX，PB18 RX，115200 8N1
- 电机 1：PA5 PH，PA6 TIMG0_CCP1 EN PWM
- 电机 2：PA24 PH，PA25 TIMG12_CCP1 EN PWM
- 电机 1 编码器：PB7 A，PB9 B
- 电机 2 编码器：PA22 A，PA21 B
- PWM 频率：20 kHz

## UART 帧格式

```text
0xAA CMD REG LEN DATA... CRC8
```

- CRC-8 多项式：0x07，初值 0x00，覆盖 SOF 到 DATA 的全部字节
- CMD 0x01：读寄存器，LEN 表示读取长度
- CMD 0x02：写寄存器，LEN 表示数据长度
- CMD 0x03：心跳
- 响应 CMD 使用 `CMD | 0x80`
- 响应状态字节：0x00 OK，0x01 ERROR

CRC 错误时固件直接丢弃该帧，不返回响应。无效命令、无效寄存器、非法参数或超长帧会返回 ERROR。

## 寄存器表

```text
0x00 DEVICE_ID          R       0xA5
0x01 FW_VERSION         R       0x01
0x02 STATUS             R
0x03 FAULT_FLAGS        R/W1C
0x04 CONTROL_FLAGS      R/W     默认 0x01，全局使能
0x05 I2C_ADDRESS        R       0x00，本版本禁用 I2C

0x10 M1_MODE            R/W     0 coast，1 run，2 brake，3 speed PID
0x11 M1_DUTY            R/W     0..100；speed 模式下为 PID 输出
0x12 M1_DIRECTION       R/W     0 forward，1 reverse
0x13 M1_STATUS          R
0x14 M1_ENCODER_COUNT   R       int32 小端，寄存器 0x14..0x17
0x18 M1_ENCODER_CPS     R       int32 小端，寄存器 0x18..0x1B
0x1C M1_ENCODER_STATE   R       bit0=A，bit1=B

0x20 M2_MODE            R/W     0 coast，1 run，2 brake，3 speed PID
0x21 M2_DUTY            R/W     0..100；speed 模式下为 PID 输出
0x22 M2_DIRECTION       R/W     0 forward，1 reverse
0x23 M2_STATUS          R
0x24 M2_ENCODER_COUNT   R       int32 小端，寄存器 0x24..0x27
0x28 M2_ENCODER_CPS     R       int32 小端，寄存器 0x28..0x2B
0x2C M2_ENCODER_STATE   R       bit0=A，bit1=B

0x30 WATCHDOG_TIMEOUT   R/W     单位 10 ms，默认 100，也就是 1 秒；写 0 关闭
0x31 ENCODER_CONTROL    W       bit0 reset M1，bit1 reset M2
0x32 M1_TARGET_RPM      R/W     uint16 小端，输出轴目标 RPM
0x34 M2_TARGET_RPM      R/W     uint16 小端，输出轴目标 RPM
0x36 M1_MEASURED_RPM    R       int16 小端，输出轴实测 RPM
0x38 M2_MEASURED_RPM    R       int16 小端，输出轴实测 RPM
0x3A SPEED_KP_Q4_4      R/W     默认 1
0x3B SPEED_KI_Q4_4      R/W     默认 1
0x3C SPEED_KD_Q4_4      R/W     默认 1
0x3D SPEED_MAX_DUTY     R/W     默认 30
0x3E SPEED_MIN_DUTY     R/W     默认 0
0x40 M1_COUNTS_PER_REV  R/W     uint32 小端，M1 输出轴每圈编码器计数，默认 22400
0x44 M2_COUNTS_PER_REV  R/W     uint32 小端，M2 输出轴每圈编码器计数，默认 22400
```

写寄存器是事务式的：整帧会先校验，任意字段非法时整帧拒绝，不会部分修改电机输出。

## 状态和故障位

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
```

## 安全规则

- 上电后两个电机默认 coast。
- `CONTROL_FLAGS` 默认全局使能。
- 当前版本 brake 的输出行为和 coast 相同。
- 只有成功的写帧会刷新看门狗。
- 心跳帧和读帧不会刷新看门狗。
- 看门狗超时会强制两个电机 coast，并置位 `FAULT_FLAGS bit0`。
- 长时间调试速度闭环时，可以写 `0x30 = 0x00` 关闭看门狗。

## 编码器和速度换算

当前固件默认按测试电机参数进行 RPM 换算：

```text
448 CPR * 50 减速比 = 22400 counts/output-shaft revolution
```

如果换成其他电机，可以运行时覆盖输出轴每圈编码器计数：

```text
counts_per_rev = 编码器每电机轴圈计数 * 减速比
```

例如重新设置为当前测试电机参数：

```text
motor cfg 22400 22400
```

## gugaPI Shell 命令

基础通信：

```text
motor ping
motor info
motor status
```

读写原始寄存器：

```text
motor reg <addr> <len 1..32>
motor set <addr> <byte...>
```

编码器：

```text
motor enc
motor enc reset
```

速度闭环：

```text
motor cfg
motor cfg <m1_counts_per_rev> <m2_counts_per_rev>
motor rpm
motor pid
motor pid <kp_q4.4> <ki_q4.4> <kd_q4.4> [max_duty [min_duty]]
motor m1 speed <rpm 0..1000> [fwd|rev]
motor m2 speed <rpm 0..1000> [fwd|rev]
```

开环控制：

```text
motor m1 run <duty 0..100> [fwd|rev]
motor m2 run <duty 0..100> [fwd|rev]
motor m1 coast
motor m2 coast
motor stop
```

串口调试：

```text
motor hex <byte...>
motor read [len 1..64]
motor clear
```

## 编码器测试

烧录：

```text
MotorDriver/Debug/MotorDriver.out
gugaPI/Debug/gugaPI.out
```

然后在 gugaPI debug shell 执行：

```text
motor ping
motor enc
motor enc reset
motor enc
motor m1 run 10 fwd
motor enc
motor m1 run 10 rev
motor enc
motor stop
```

预期现象：

- `count` 会随电机轴转动变化。
- `cps` 是 counts per second，每 100 ms 更新一次。
- `state` 是当前 AB 相状态，范围是 `0x00..0x03`。
- 如果某个电机方向相反，可以交换该电机 A/B，或在固件中反向该通道的方向约定。

## 速度闭环测试

先关闭看门狗，查看默认 PID 参数和当前 RPM：

```text
motor set 0x30 0x00
motor cfg
motor pid
motor rpm
```

M1 低速测试：

```text
motor m1 speed 10 fwd
motor rpm
motor reg 0x10 4
motor stop
```

M2 低速测试：

```text
motor m2 speed 10 fwd
motor rpm
motor reg 0x20 4
motor stop
```

`motor pid` 输出的是原始 Q4.4 参数。默认参数等价于执行 `motor pid 1 1 1 30 0`。默认 `SPEED_MAX_DUTY=30`，第一次闭环测试时最大占空比限制为 30%。

如果速度响应太慢，可以先提高 `kp` 或 `max_duty`。如果速度明显振荡，先降低 `kp`，再考虑调整 `ki`。

## 构建

在 `MotorDriver/Debug` 目录下执行：

```text
C:/ti/ccs2100/ccs/utils/bin/gmake.exe clean all
```
