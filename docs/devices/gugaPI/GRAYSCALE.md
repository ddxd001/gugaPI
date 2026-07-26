# 八路灰度传感器与循迹手册

## 硬件与采样

主控使用 MSPM0G3519 ADC1 的 PA15 输入。PA16、PC20、PC21 组成三位模拟
多路选择地址。ADC 为 12 位，采样范围 `0..4095`，选通后等待约 200 us，再使用
125 us ADC 采样时间。

任务接口和调度器实现保持不变：`App_GrayscaleUpdate()` 每 2 ms 读取一路，八次后
原子发布完整帧。典型完整帧周期约 16 ms（约 62.5 Hz）。读取失败会丢弃半帧并从
通道 0 重新开始。

## 唯一数据处理链

`app_grayscale` 是灰度处理的唯一数据源。`linefollow` 不再从原始 ADC 重复校准或
归一化，只消费完整帧的 `processed_valid`、`line_position`、`line_strength`、
`position_valid`、`position_confidence`、`active_mask`、`road_type` 和 `sequence`。

每通道归一化为 0（白）到 1000（黑）：

```text
normalized = clamp((raw - white) * 1000 / (black - white), 0, 1000)
```

公式同时支持“黑色 ADC 更低”和“黑色 ADC 更高”。

## 迟滞、位置和路口

数字位图使用双阈值迟滞：

```text
关闭 → normalized >= threshold_on  → 开启
开启 → normalized <  threshold_off → 关闭
```

默认 `gray_threshold=500`、`gray_hysteresis=300`，所以 `threshold_on=650`、
`threshold_off=350`。阈值中间区域保持上一帧状态，避免边界抖动。

实板逐路遮挡已确认：从车体左侧到右侧依次为 ch0..ch7。位置坐标规定左正右负：

```text
+3000, +2000, +1500, +1000, -1000, -1500, -2000, -3000
```

`gray_track_mask=0x3C` 定义中间主循迹区，但位置计算允许八个通道参与。线段成员门限
取 `gray_position_floor` 与“本帧最小归一化值 + 50”中的较大值，用来抵消安装高度、
环境光造成的全通道共同背景偏置；线段强度仍按 `gray_position_floor` 扣除，配置参数
含义保持不变。随后把相邻有效通道组成连续线段：无历史位置时选最强线段，有历史
位置时选最接近上一位置的线段。这样线从中间移到边缘时位置连续，左右岔路出现两个
分离黑块时也不会把两条线直接平均成错误的中心位置。

输出字段：

- `selected`：本帧实际用于位置插值的连续线段位图；
- `source`：`core`、`left_edge`、`right_edge`、`held` 或 `none`；
- `confidence`：位置可信度 `0..1000`，按强度、线宽、多线段和位置突变扣分；
- `pos_valid`：可信度至少 300 且线段不超过 4 路时为 1；
- `pos`：左侧为正、右侧为负，0 为中心。

超过 4 路的连续黑区通常是 T 路或十字路口。此时仍保留 `line=1` 供行为条件使用，
但输出 `source=held`、`pos_valid=0`，避免巡线把整片黑区误当成可靠位置。

路口分类需要连续两帧确认。左侧外通道掩码为 `0x03`，右侧为 `0xC0`，输出：`unknown`、`lost`、`straight`、
`left_branch`、`right_branch`、`t`、`cross`。路口类型和线位置是两个独立字段，
不会使用特殊位置数值表示路口。

运行时还会检测连续 8 帧 ADC 饱和，以及某通道 ADC 数值逐位完全不变而其它通道
持续变化 125 帧的疑似卡死。异常位写入 `anomaly`；诊断异常不会删除原始处理结果，
但巡线会强制半速。标定错误、采集失败或数据过期仍会禁止闭环。

## 校准

推荐使用白、黑两阶段多帧平均：

```text
gray calib white 16
gray calib status
gray calib black 16
gray calib status
gray calib commit
param save
```

每个阶段默认平均 16 个完整帧，最多 128 帧。`commit` 要求每个通道黑白跨度至少
200 ADC counts，否则返回 `invalid_arg`，`gray calib status` 的 `fault` 位图指出失败
通道。提交只更新 RAM 并标记配置为 dirty；`param save` 才写入 FRAM。

兼容命令 `lf cal` 和 `gray calib sweep [ms]` 会在传感器扫过黑线/白底时记录 min/max。
扫动校准默认假设白色 ADC 高于黑色 ADC；极性相反时必须使用显式 white/black
校准。

## 参数

```text
gray_white_0..7       0..4095
gray_black_0..7       0..4095
gray_threshold        1..999
gray_hysteresis       0..998
gray_position_floor   0..999
gray_min_strength     1..8000
gray_track_mask       1..255
lf_kp                 0..1000000
lf_kd                 0..1000000
lf_maxcorr            0..500
lf_lost_hold_ms       0..10000
lf_lost_stop_ms       1..10000
```

约束：迟滞上下阈值必须保持在 `1..999`，`lf_lost_stop_ms` 不得小于
`lf_lost_hold_ms`。参数通过 `param set` 修改后，灰度参数执行 `gray calib reload`，
循迹参数重启后加载；`lf kp/kd/maxcorr/losthold/losttimeout` 可立即修改并同步标记
ConfigStore dirty。

## 循迹控制

```text
lf cal
lf status
lf start 40 10000
lf stop
```

20 ms 控制任务只在 `sequence` 变化时消费新完整帧，避免对同一帧重复计算微分：

```text
correction = (kp * error + kd * filtered_derivative) / 1000000
left_rpm   = base_rpm - correction
right_rpm  = base_rpm + correction
```

`kd` 默认 0，首次实车应保持 P 控制，确认方向正确后再从小值增加。可信度至少 700
时使用设定基础速度，300..699 时自动降到一半基础速度；位置无效或可信度低于 300
按丢线处理。丢线后先以一半基础速度保持最后转向；超过 `lost_hold_ms` 后以零基础
速度按最后方向搜索；达到
`lost_stop_ms` 后停车。数据无效或超过 200 ms 会立即停车并设置传感器丢失故障。

## 诊断和验证

```text
gray status
gray data
gray process
gray calib show
lf status
```

本车 2026-07-26 的现场标定样本（仅用于追溯，每块板仍应重新标定）：

```text
white: 3178 3122 3134 3182 3083 3147 2876 3110
black:   488  379  741  735  523  593  334  603
span:   2690 2743 2393 2447 2560 2554 2542 2507
```

逐路遮挡确认了 ch0/ch1 为车体左外侧、ch2/ch3 为中间偏左、ch6/ch7 为车体右外侧，
ch4/ch5 按单调布线为中间偏右。白底归一化最大约 29，黑底约 982..1000，完整帧约
16 ms。软件单元样例已覆盖左偏、右边缘、分离双线和全黑宽线；TI Clang 5.1.1
Debug 配置编译无警告。

烧录后只需做一次综合验收，不再反复逐通道采样：先在白底、中心黑线、左右偏线各
执行一次 `gray process`，确认左右符号和 `pos_valid`；然后以低速执行
`lf start 20 3000`，最后遮离赛道确认系统能按丢线策略停车。通过后再逐级提高速度。
