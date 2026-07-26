# 定距分段速度曲线使用与验收手册

## 1. 范围

本功能用于 `heading distance` 和动作序列中的 `drive_mm`。它在gugaPI上整形目标RPM，
不修改MotorDriver的100 ms速度环，不修改调度器实现，也不改变原有函数调用接口。

## 2. 工作流程

默认 `trapezoid` 模式包含以下阶段：

1. `accel`：按 `distance_accel_rpm_s` 提高基础RPM。
2. `cruise`：保持命令指定的 `max_rpm`。
3. `brake`：根据实测车速预测停止距离并降低RPM。
4. `creep`：以 `distance_creep_rpm` 单方向接近目标。
5. `settle`：停车并等待左右轮实际RPM连续3次低于阈值。

越过目标后只停车并记录最终误差，不反向修正。`legacy` 模式保留旧行为，仅用于对比。

## 3. 启动

```text
heading profile
heading profile mode trapezoid
heading profile save
heading distance 500 60
```

动作运行期间可用以下命令观察阶段和剩余距离：

```text
heading status
chassis stat
motor rpm
```

需要立即终止时执行：

```text
heading stop
```

## 4. 参数调整顺序

当前小车已通过1 m地面测试将有效轮径标定为 `33050 um`，CPR保持 `1456`；
500 mm / 120 RPM地面测试将停止延迟补偿最终标定为 `360 ms`。`latency` 是固定的
系统响应时间估计，但固件使用实时轮速将其换算成距离，因此不是按速度挡位查表。
后续按以下顺序整定：

1. 先设置巡航速度，不修改曲线参数。
2. 若起步过猛，降低 `accel`；若达到巡航速度过慢，提高 `accel`。
3. 若高速终点仍快，提高 `latency` 或降低 `decel`；若过早进入低速逼近，降低
   `latency` 或 `margin`。
4. 若终点前无法克服静摩擦，提高 `creep`；若低速冲过终点，降低 `creep`。
5. 最后才调整 `tolerance`，不要用放大容差掩盖制动参数问题。

每次只改变一个参数。验证完成后执行 `heading profile save`。

当前已记录的单次实车结果：1 m / 60 RPM实测约1003 mm；500 mm / 90 RPM在
`latency=300 ms`时编码器终点505 mm、卷尺实测约506 mm；500 mm / 120 RPM在
300 ms时实测513 mm、320 ms时实测510 mm、最终360 ms时编码器终点507 mm。
这些结果只证明当前负载和地面条件下的单次精度，批量验收仍应检查重复性。

## 5. 推荐测试矩阵

先架空确认方向和速度连续，再放到不少于2 m的直线场地：

| 距离 | 最大转速 | 次数 | 目标 |
| ---: | ---: | ---: | --- |
| 500 mm | 60 RPM | 3 | 低速基准 |
| 500 mm | 90 RPM | 3 | 中速短距离 |
| 500 mm | 120 RPM | 3 | 高速短距离 |
| 1000 mm | 120 RPM | 3 | 比赛主验收 |
| 1000 mm | 150 RPM | 3 | 通过120 RPM后再测试 |

120 RPM对应约0.408 m/s，150 RPM对应约0.511 m/s。每次记录卷尺实测距离、最终
左右编码器、最大航向偏差、终点航向和电池电压。

验收建议：500 mm/120 RPM误差不超过±12 mm；1000 mm/120 RPM误差不超过±15 mm；
同条件3次极差不超过10 mm；最终航向误差不超过3°；无反向寻找、无故障、无任务
超时。150 RPM未通过时将比赛速度上限固定为120 RPM。

## 6. 烧录后检查

旧配置首次启动后：

```text
param status
heading profile
param save
param status
```

预期旧V7配置能够正常加载并显示dirty；保存后升级为当前V9布局、长度更新为181且dirty清零。随后执行：

```text
motor info
motor status
sched
heading status
```

确认MotorDriver在线、故障位为0、所有任务timeout为0，再进行运动测试。
