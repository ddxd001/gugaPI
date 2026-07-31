# 杆球外环 PID 调参

当前控制器不是普通 PID。物理模型负责 500 Hz 状态预测，P 把球拉向目标，D 按观测速度制动，受限的慢速 I 只学习长期静差：

```text
feedback_mdeg = Kp * position_error_mm + static_trim - Kd * velocity_mm_s
beam_mdeg = hold_angle(position) + feedback + chassis/pitch feedforward
```

默认参数：

| 参数 | 默认值 | 单位 |
| --- | ---: | --- |
| `ball_pid_kp` | 40 | mdeg/mm |
| `ball_pid_ki` | 20 | mdeg/(mm·s) |
| `ball_pid_kd` | 20 | mdeg/(mm/s) |
| `ball_pid_ilim` | 1500 | mdeg |
| `observer_alpha` | 500 | 千分数 |
| `observer_beta` | 80 | 千分数 |

建议每次只改一个参数：

1. 保持 Kd=20、Ki=20，从 Kp=40 开始。回位太慢时每次增加 5；持续大幅往返时降低。
2. 球越过目标且速度较大时，每次增加 Kd 2～5；动作迟钝或很早反向时降低。
3. 静差收敛太慢时，每次增加 Ki 5；目标附近低频往返时降低 Ki。I 仅在误差不超过 30 mm、球速不超过 5 mm/s、视觉正常时累积，并在 1 mm 内停止追逐噪声。
4. `ball status` 中 `pid_mdeg=总量/P/I/D` 的 I 是已学习的静态补偿角；正常应缓慢变化且不超过 `ball_pid_ilim`。`hold_mdeg` 是当前插值得到的静态保持角。
5. 调试阶段先不要保存；连续多次稳定且不越过 ±80 mm 后再执行 `config save`。

示例：

```text
config set ball_pid_kp 45
config set ball_pid_ki 20
config set ball_pid_kd 25
config set ball_pid_ilim 1500
ball hold 0
ball status
```

若球持续向端部运动，立即执行 `ball stop`，随后用 `dm position -570` 回到机械水平位。
