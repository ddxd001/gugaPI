# 达妙 DM-G6220 独立台架测试

测试时拆下或可靠固定摆杆，使用限流电源，并确保可以立即断电。所有命令只在
系统 `READY` 状态接受；位置命令硬限制在 `-1000..+1000 mrad`。

调试串口为 PB0/TX、PB1/RX，115200 8N1。

## 1. 只读检查

```text
dm status
```

正常 CAN 接线应显示 `valid=1`、`can=1/busoff=0`，并且多次读取时 `rx` 持续
增加。`state=0` 表示失能，`state=1` 表示 MIT 模式已使能。

## 2. 清错和使能

```text
dm disable
dm clear
dm enable
dm status
```

每条命令后等待至少 100 ms。使能过程会发送三次使能帧；最终必须显示：

```text
valid=1 ready=1 bench=1 enabling=0 state=1
```

`bench=1` 表示台架命令已经取得达妙控制权，READY 状态的水平保持不会覆盖
测试目标。使能确认后固件会立即保持反馈当前位置并按 100 Hz 刷新，避免触发
电机通信看门狗。

若 `ready=0`，不得继续位置测试。检查 CANH/CANL、120 Ω 终端、TCAN3413 STB、
电机 ID=1、CAN 波特率和电机电源。

`state=8..14` 是电机故障，其中十进制 `13`（十六进制 `D`）表示通信丢失。
固件检测到这些状态或反馈超过 120 ms 后会停止 MIT 控制并排队重发三次失能帧。

## 3. 当前点保持

```text
dm hold
dm status
```

电机应产生保持力但不应发生明显跳变。`target/ref` 应从当前 `pos` 附近平滑
建立；若方向突变或电流异常，立即执行 `dm disable` 或断电。

## 4. 小步位置测试

首先记录当前位置，然后只做 100 mrad 小步：

```text
dm position 100
dm status
dm position 0
dm status
dm position -100
dm status
dm position 0
```

参考位置按 `2000 mrad/s` 限速，且始终限制在实际反馈 ±250 mrad 内。正常现象
是 `ref` 连续接近 `target`，`pos` 同方向跟随，`age` 保持小于 100 ms，CAN
`tx/rx` 持续增加且 `busoff=0`。

完成后失能：

```text
dm disable
dm status
```

最终应为 `ready=0 state=0`，电机失去保持力。需要恢复比赛控制时，重新使能后
执行 `dm auto`；直接启动比赛任务也会自动收回台架控制权。
