# 杆球外环 PD 调参

当前控制器不是纯 PID，也不使用积分。物理模型负责 500 Hz 状态预测，五点表负责静态保持角，外环 PD 负责把球拉向目标并按观测速度制动：

```text
feedback_mdeg = Kp * position_error_mm - Kd * velocity_mm_s
beam_mdeg = hold_angle(position) + feedback + chassis/pitch feedforward
```

默认参数：

| 参数 | 默认值 | 单位 |
| --- | ---: | --- |
| `ball_pid_kp` | 40 | mdeg/mm |
| `ball_pid_ki` | 0 | 不使用 |
| `ball_pid_kd` | 20 | mdeg/(mm/s) |
| `observer_alpha` | 500 | 千分数 |
| `observer_beta` | 80 | 千分数 |

建议每次只改一个参数：

1. 保持 Kd=20、Ki=0，从 Kp=40 开始。回位太慢时每次增加 5；持续大幅往返时降低。
2. 球越过目标且速度较大时，每次增加 Kd 2～5；动作迟钝或很早反向时降低。
3. 不要加入 Ki。长期位置偏差应修改对应位置的 `hold_a0..4`，而不是积累积分。
4. `ball status` 中 `pid_mdeg=总量/P/I/D` 的 I 应始终为 0，`hold_mdeg` 是当前插值得到的静态保持角。
5. 调试阶段先不要保存；连续多次稳定且不越过 ±80 mm 后再执行 `config save`。

示例：

```text
config set ball_pid_kp 45
config set ball_pid_ki 0
config set ball_pid_kd 25
ball hold 0
ball status
```

若球持续向端部运动，立即执行 `ball stop`，随后用 `dm position -570` 回到机械水平位。
