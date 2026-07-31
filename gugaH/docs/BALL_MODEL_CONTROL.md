# 杆球模型控制

> 当前台架对比固件暂时整体关闭五点静态保持角补偿。参数表仍被保留，`ball status` 中的 `hold_mdeg` 固定为 0，恢复时无需重新设计配置格式。

## 控制结构

控制器采用串级结构：DM-G6220 自身负责快速位置环，gugaH 负责球的位置外环。外环的积分仅作为受限的慢速静差补偿：

```text
50 Hz 视觉位置
  -> 500 Hz 物理模型预测 + 视觉位置残差校正
  -> 位置 P + 慢速静差 I + 观测速度 D
  -> 五点静态保持角 + 车体加速度/俯仰前馈
  -> 五点机构映射
  -> DM 位置环
```

钢球近似实心球，沿杆方向的相对加速度为：

```text
a_ball = k_roll * (g * theta_effective - a_chassis)
theta_effective = theta_actual + pitch - theta_hold(x)
```

默认 `k_roll=0.714`。固件每 2 ms 使用 DM 实际反馈角和上式预测位置、速度；新视觉帧到达时，只用位置残差校正两个状态。五点视觉差分速度仅用于诊断和确认钢球是否已克服静摩擦，不直接注入观测速度或电机命令。

`theta_hold(x)` 是五点分段线性表，可表示两端略高和局部不平。默认表为：

| 位置（0.1 mm） | -1000 | -500 | 0 | 500 | 1000 |
| ---: | ---: | ---: | ---: | ---: | ---: |
| 保持角（mdeg） | -918 | -518 | -118 | 282 | 682 |

这些角度是初值，实车应分别把球放在五个位置，手动寻找“球不会持续滚动”的角度后写入。角度不要求单调，因此也能补偿弯曲或安装误差。

## 主要参数

| Shell 参数 | 默认值 | 含义 |
| --- | ---: | --- |
| `model_roll` | 714 | 有效滚动系数，千分数 |
| `observer_alpha` | 500 | 视觉位置残差对位置状态的校正，千分数 |
| `observer_beta` | 80 | 位置残差对速度状态的校正，千分数；不是视觉差分速度权重 |
| `ball_pid_kp` | 40 | 位置外环 P，mdeg/mm |
| `ball_pid_ki` | 20 | 低速静差补偿 I，mdeg/(mm·s) |
| `ball_pid_kd` | 20 | 速度阻尼 D，mdeg/(mm/s) |
| `ball_pid_ilim` | 1500 | 静差补偿最大绝对角，mdeg |
| `model_breakaway` | 2300 | 静止超过 400 ms 后的最大启滚补偿角，mdeg |
| `model_rolling_friction` | 0 | 运动时的方向性摩擦前馈，mdeg |
| `pitch_gain` | 0 | 车体俯仰补偿比例；完成 IMU 零偏标定前保持 0 |
| `max_angle` | 6000 | 最大摆杆角，mdeg |

旧的 `model_tau`、`model_plan_accel`、`model_vmax`、`model_curvature` 和 `model_curve_origin` 字段为 FRAM 兼容保留，不再参与外环命令计算。

## 五点表修改

```text
config set hold_x0 -1000
config set hold_a0 -918
config set hold_x1 -500
config set hold_a1 -518
config set hold_x2 0
config set hold_a2 -118
config set hold_x3 500
config set hold_a3 282
config set hold_x4 1000
config set hold_a4 682
config save
```

`hold_x0..4` 必须严格递增且位于 ±115 mm，`hold_a0..4` 位于 ±6°。只允许在非运行状态保存。

## 调试顺序

1. 确认水平机械零位约对应 DM `-570 mrad`。
2. 先在 0、±50 mm、±100 mm 标定五个静态保持角。
3. 先用 `ball_pid_ki=0` 调好 Kp/Kd，再恢复 `ball_pid_ki=20` 消除静差；若低频往返，每次降低 Ki 5。
4. 若观测速度追随位置噪声，减小 `observer_beta`；若真实运动明显滞后，再小步增大。
5. ±10 mm 内不会施加大启滚脉冲，但 P/D 会持续微调；允许球和杆持续小幅动作。
6. 静态稳定后再测 H3 的 `0 -> +50 mm -> -50 mm`，最后测试底盘加减速前馈。

球位置达到 ±80 mm 时应准备人工停止；固件的 ±115 mm、视觉、DM 和 IMU 超时保护仍然有效。
