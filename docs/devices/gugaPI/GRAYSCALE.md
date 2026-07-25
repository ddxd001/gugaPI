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
`active_mask`、`road_type` 和 `sequence`。

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

连续位置默认只使用 `gray_track_mask=0x3C`，即中间 2..5 通道；全八路位图仍用于
路口识别。位置表暂按感为模拟板参考布局缩放为：

```text
-3000, -2000, -1500, -1000, 1000, 1500, 2000, 3000
```

计算位置前先扣除 `gray_position_floor`，再按剩余黑度加权。强度低于
`gray_min_strength` 时判定丢线。

路口分类需要连续两帧确认，输出：`unknown`、`lost`、`straight`、
`left_branch`、`right_branch`、`t`、`cross`。路口类型和线位置是两个独立字段，
不会使用特殊位置数值表示路口。

通道左右顺序和物理距离尚需在实板上逐路遮挡确认；若方向相反，应调整通道映射，
不能仅靠提高 PD 参数补偿。

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

`kd` 默认 0，首次实车应保持 P 控制，确认方向正确后再从小值增加。丢线后先以一半
基础速度保持最后转向；超过 `lost_hold_ms` 后以零基础速度按最后方向搜索；达到
`lost_stop_ms` 后停车。数据无效或超过 200 ms 会立即停车并设置传感器丢失故障。

## 诊断和验证

```text
gray status
gray data
gray process
gray calib show
lf status
```

实板验收顺序：逐路遮挡确认通道顺序与左右符号；记录纯白/纯黑原始值；确认迟滞区
不抖动；低速测试居中、左右偏移和弯道；最后测试丢线、T 路和十字路口。ADC 的
低/中/高输入、选通建立时间和完整帧周期需要示波器或日志验证。目前代码仅完成编译
验证，不能视为硬件验收完成。
