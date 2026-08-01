# MotorDriver 5 ms 速度闭环优化

> 2026-08-01 更新：当前固件版本为 `0x04`，编码器每 5 ms 发布一次
> 速度估计，使用 6 个样本保持约 30 ms 滑动窗口；速度 PID 每 5 ms
> 执行一次。下文的 10 ms 数据保留为上一版 `0x03` 的台架基线。

## 1. 基线

- 官方基线：`ddxd001/gugaPI` 的 `feature/chassis-control`，提交 `72fa3180bd27d860fe5fb0730ef94659847b9c4a`。
- 本轮开始点：本地 `clock` 分支提交 `8f9b1a898de1ef0d3a62e966a21d7246d3fc68ee`。
- 开始点相对官方 MotorDriver 的有效差异只有：默认输出轴 CPR 从 364 修正为 1456、I2C 模拟毛刺滤波为 50 ns，以及相应文档更新。
- 本轮保留上述差异，不修改 gugaPI 调度器，不改变 I2C/UART 寄存器地址和现有主控调用方式。

## 2. 目标和边界

目标是缩短循迹指令到电机实际输出修正之间的底板延迟，同时保持协议兼容：

1. CPU 固定使用 80 MHz。
2. 编码器增量采样从 50 ms 缩短到 10 ms，并用约 30 ms 滑动窗口抑制量化噪声。
3. 速度 PID 从 100 ms 缩短到 10 ms。
4. 保持旧 PID 参数的 100 ms 时间语义，避免周期缩短后积分增大 10 倍或微分缩小 10 倍。
5. 闭环内部占空比不再局限于整数百分比。
6. 位置环、位置保持、watchdog、故障处理和通信协议保持原行为。

本轮不把控制计算移入中断。SysTick 只提供 1 ms 时间基准，TIMG8/TIMG9 负责硬件 QEI 计数；主循环执行定时门控的测速和控制计算。这样 I2C 中断保持短小，也避免控制 ISR 与寄存器写入并发修改状态。

## 3. 时钟和 PWM

SysConfig 生成后的实际时钟为：

| 项目 | 配置 |
| --- | ---: |
| CPUCLK | 80 MHz |
| BUSCLK / TIMG0 输入 | 40 MHz |
| TIMG0 周期计数 | 2000 |
| PWM 频率 | 20 kHz |
| 两路 PWM | TIMG0 CCP0 / CCP1 |

计算关系：`40 MHz / 2000 = 20 kHz`。TIMG0 不支持该器件的 Capture/Compare 影子更新，因此不启用零点影子模式；保持现有引脚和定时器资源比迁移 PWM 外设风险更低。

驱动内部提供 Q8.8 百分比占空比接口，PID 当前可形成 1/16% 的控制步进；2000 计数 PWM 的硬件步进是 0.05%。外部 `REG_Mx_DUTY` 和遥测仍四舍五入为 0–100，旧主控无需修改。

## 4. 数据与控制流水线

```text
TIMG8/TIMG9 硬件 QEI 连续计数
          ↓ 每 10 ms 取 count 增量
3 样本滚动累计（约 30 ms，使用实际 elapsed_ms）
          ↓ CPS / RPM
10 ms 速度目标斜坡 + 时间归一化 PID + 抗积分饱和
          ↓ Q8.8 duty
DRV8701 PH/EN 状态化输出
          ↓
20 kHz PWM
```

速度 PID 的兼容处理：

- 比例项不随周期变化。
- 积分增量乘实际 `elapsed_ms / 100 ms`。
- 微分误差换算到原 100 ms 参考周期。
- 异常延迟的 PID 时间步最大按 100 ms 计算，避免主循环短暂阻塞后一次性产生过大的积分或微分动作。
- 输出同向饱和时暂停继续积分，减少解除饱和后的拖尾。

DRV8701 输出层会缓存当前方向和 EN 输出模式。同方向调占空比时只更新 TIMG0 compare，不再每 10 ms 重复把 EN 拉低并重新做 GPIO/PWM 复用；只有方向或输出模式真正变化时才执行安全切换。

## 5. 兼容性

- `REG_Mx_DUTY`、`REG_Mx_TARGET_RPM`、PID、斜坡、位置和 watchdog 寄存器布局不变。
- UART 和 I2C 帧格式不变；gugaPI 的 `motor ...` shell 命令不变。
- 默认 CPR 保持 1456，即 `13 PPR × 4 倍频 × 28:1`。
- 位置环和 `speed 0` 保持周期仍是 100 ms，原位置参数不需要随本轮重调。
- 固件版本从 `0x02` 更新为 `0x03`，用于识别新的 10 ms 速度链路；主控对 `>= 0x02` 的遥测兼容判断仍成立。
- 整车落地方向测试确认最终组合为输出反相 `0x03`、编码器反相 `0x01`；该组合使 `chassis wheel <正> <正>` 向前直行，并保持两路闭环反馈为正。
- 实测速度 PID 默认值更新为 `kp=2, ki=2, kd=0, max_duty=60, min_duty=4`。

## 6. 编译验证

进入 `MotorDriver/Debug` 后执行：

```text
E:/TI/ccs/utils/bin/gmake.exe -k -j 32 all -r -O
```

必须分别确认：SysConfig 生成成功、TI Clang 编译零警告、链接成功并生成 `MotorDriver.out`。SysConfig 关于 32 MHz 以上 Flash 状态清除的提示已经由 `BoardNvm_SaveI2cAddress()` 中的 `DL_FlashCTL_executeClearStatus()` 覆盖。

## 7. 烧录后台架验收

先悬空车轮，先烧录 MotorDriver，再烧录协议兼容的 gugaPI：

```text
motor bus i2c
motor ping
motor info
motor cfg
motor pid
motor ramp
motor enc
```

预期 `motor info` 显示固件版本 3、I2C 地址正确，`motor cfg` 两路均为 1456。

逐级测试单轮，命令之间观察 2–3 秒，不连续刷 shell：

```text
motor m1 speed 60 fwd
motor rpm
motor m1 speed 120 fwd
motor rpm
motor m1 speed 200 fwd
motor rpm
motor stop
```

M2 重复同样测试，再执行双轮底盘低速和比赛目标速度测试。验收重点：

- 轮子连续转动，无旧版整数占空比造成的周期性顿挫。
- 改变目标后约一个测速窗口内出现明确修正，且没有持续放大的高频抖动。
- 正反向切换先经过零输出，不出现方向脚与 PWM 同时突变。
- `motor status` 无 watchdog / I2C 故障，编码器方向和 RPM 符号正确。
- 在 60、120、200 RPM 以及比赛目标速度分别记录稳定段平均 RPM、峰峰值和阶跃超调；参数只在这些数据基础上微调。

最后落地测试循迹，先低速再提高。若循迹仍呈固定相位的慢摆，优先调 gugaPI 循迹参数；若单轮 RPM 本身出现明显 10–30 ms 高频振荡，再调整速度 PID。不要同时修改两层参数，否则无法区分误差来源。

## 8. 首次台架结果

80 MHz、20 kHz PWM、10 ms 速度环固件烧录后，悬空双轮测试结果：

| 工况 | 结果 |
| --- | --- |
| 原编码器反相 `0x01`，60 RPM | M1 可收敛；M2 测得负转速，控制输出错误封顶 |
| 编码器反相 `0x03`，60 RPM | 两轮稳定 59–60 RPM，约 17% duty |
| PID `1/1/0`，120→80/160 RPM | 约 1.2 s 接近目标，差速响应偏慢 |
| PID `2/2/0`，120→80/160 RPM | 约 0.8 s 接近目标，无持续振荡 |
| `max_duty=40`，目标 200 RPM | duty 封顶，只能达到约 148–151 RPM |
| `max_duty=60`，目标 200 RPM | 两轮达到约 199–201 RPM，约 53% duty |
| PID `2/2/0`，60→200 RPM | 约 0.85 s 达到 195 RPM，峰值约 202 RPM |
| `output=0x01, encoder=0x03`，30 RPM | 车辆原地旋转 |
| `output=0x00, encoder=0x02`，30 RPM | 车辆向后直行 |
| `output=0x03, encoder=0x01`，30 RPM | 车辆向前直行，确定为最终配置 |

全部测试结束后目标、占空比和积分均回到 0，MotorDriver `fault=0x00`，INA219 未产生保护锁存或通信错误。
