# ICM45686 陀螺仪零偏自动补偿手册

## 1. 功能范围

gugaPI 以 200 Hz 读取 ICM45686，并在现有 5 ms `imu` 任务中估计 Z 轴
静止残余零偏。该功能不增加调度任务、不增加 SPI/I2C 访问，也不会自动写
FRAM。

航向使用的角速度为：

```text
校正角速度 = 传感器换算值 - FRAM固定零偏 - RAM运行时零偏
```

FRAM 固定零偏断电保存；RAM 运行时零偏用于跟踪温度和本次上电环境，复位后
重新估计。它只能降低相对航向漂移，不能恢复已经产生的绝对方向误差。

## 2. 启动方法

上电后把车辆放稳至少 2 秒，不要搬动、旋转或触碰车体。自动学习默认开启，
满足以下条件后开始采集约 400 个样本：

- 左右目标转速为 0；
- 左右实际转速不超过 3 RPM；
- 底盘反馈有效且没有超时；
- 航向、定距、转向、循迹和动作序列均未运行；
- 加速度模长接近 1 g；
- 陀螺仪角速度及采样噪声低于限制。

任一条件不满足都会丢弃当前窗口并重新等待。校准期间航向积分冻结，但采样
时间戳继续更新，因此恢复运动时不会补算整段校准时间。

## 3. Shell 命令

### 查看状态

```text
imu bias status
```

关键字段：

| 字段 | 含义 |
| --- | --- |
| `state` | `waiting`、`collecting`、`cooldown` 或 `disabled` |
| `reject` | 当前不能学习的原因 |
| `fixed_z_mdps` | ConfigStore/FRAM 固定 Z 轴零偏 |
| `runtime_z_mdps` | 本次上电自动估计的运行时零偏 |
| `total_z_mdps` | 两种零偏之和 |
| `corrected_z_mdps` | 已应用两种补偿的当前 Z 轴角速度 |
| `samples` | 当前静止窗口样本数，目标为 400 |
| `mean_mdps` / `stddev_mdps` | 最近完成窗口的均值和标准差 |
| `accepted` / `rejected` | 接受和拒绝的窗口计数 |

常见 `reject`：

- `feedback_stale`：底盘反馈超过 100 ms 或读取失败；
- `target_active` / `wheels_moving`：存在运动命令或轮速；
- `control_active`：航向、循迹、动作序列或比赛动作运行中；
- `acceleration`：车体振动、倾倒或被搬动；
- `gyro_motion` / `noise`：车体正在旋转或采样噪声过大；
- `cooldown`：一次有效更新后等待 5 秒再学习。

### 手动请求一次校准

```text
imu bias calibrate
```

命令只发出请求。车辆满足静止条件并完成 2 秒采集后，结果写入 RAM；不会写
FRAM。可反复执行 `imu bias status` 查看进度。

### 开关自动学习

```text
imu bias auto on
imu bias auto off
```

关闭后保留当前运行时补偿，但不再自动采集。手动 `calibrate` 在自动学习关闭
时仍然有效。

### 保存到 FRAM

```text
imu bias save
```

车辆必须静止，并且至少已有一次有效估计。保存过程执行：

```text
新固定零偏 = 旧固定零偏 + 当前运行时零偏
运行时零偏 = 0（仅在FRAM保存成功后）
```

保存失败时恢复旧的 RAM 固定值，运行时补偿保持不变，避免补偿丢失或重复。

### 清除运行时补偿

```text
imu bias reset
```

只清除 RAM 运行时零偏并重新等待静止，不修改 FRAM。如需清除固定零偏，必须
显式执行：

```text
param set imu_gyro_bias_z_mdps 0
param save
```

## 4. 推荐调试流程

```text
imu bias status
imu bias calibrate
imu bias status
```

等待 `valid=1` 后保持车辆静止 60 秒，间隔观察 `yaw`：

```text
imu sample
imu bias status
```

确认漂移明显降低且校准期间没有移动后，再执行一次：

```text
imu bias save
param status
```

复位后再次用 `imu bias status` 核对 `fixed_z_mdps`。不建议每次自动学习后都
保存；温度引起的短期变化应保留在运行时零偏中。

## 5. 使用限制

纯陀螺仪无法区分“非常缓慢且匀速的人工旋转”和“传感器固定零偏”。校准时
仍必须保证车辆不被搬动。需要长期绝对方向恢复时，应引入经过硬软铁标定的
磁力计或场地外部方向参考。
