# 编码器距离闭环行为手册

## 功能

`HEADING_DISTANCE` 使用左右轮编码器测量行驶距离，同时使用 ICM-45686 的Yaw保持
启动瞬间的航向。它是应用层外环：MotorDriver继续完成每个车轮的RPM内环，gugaPI
根据剩余毫米和航向误差周期性更新左右轮目标RPM。

当前 `Heading_HoldStart(base_rpm)` 是行走中的航向保持：锁定启动Yaw并持续以
`base_rpm` 行驶，没有距离完成条件。距离模式在此基础上增加编码器目标和自动停车。

## 启动条件

- IMU数据有效且不超过200 ms。
- `chassis_fb` 编码器反馈有效且不超过100 ms。
- `wheel_radius_mm`、`left_counts_per_rev`、`right_counts_per_rev`正确。
- `motor_encoder_invert_flags`已保证车辆前进时两轮编码器增量为正。
- 系统无故障，INA219未禁止运动。

## Shell使用

```text
heading distance 500 60
heading status
heading stop
```

参数：

- `mm`：`-10000..10000`，不能为0；正数前进，负数倒退。
- `max_rpm`：`1..max_wheel_rpm`。
- `timeout_ms`：可选，范围500..60000；省略时按距离、轮径和速度自动估算，增加
  3倍余量和2秒启动余量，并限制在3..60秒。

控制过程：

```text
剩余距离 = 目标毫米 - 两轮平均行驶毫米
基础RPM  = clamp(abs(剩余距离), 15, max_rpm) × 剩余方向
航向修正 = clamp(航向误差 × heading_kp / 1e6)
左轮RPM  = 基础RPM - 航向修正
右轮RPM  = 基础RPM + 航向修正
```

左右轮分别进入目标 `±3 mm` 后停车，持续100 ms仍在范围内即完成并回到
`HEADING_IDLE`。

## 动作序列

`drive_mm` 的 `param1` 是有符号毫米，`param2` 是最大RPM，完成条件必须是
`distance_reached`：

```text
run clear
run add drive_mm 500 60 distance_reached next abort
run add turn 90 8000 heading_reached next abort
run add end 0 0 immediate next abort
run start
```

距离动作内部使用自动超时；整个ActionRunner仍有60秒总超时。

## 安全与验收

以下情况立即停车：IMU或编码器反馈过期、航向误差超过90°、I2C/电机命令失败、
距离超时、全局故障或INA219运动禁止。

实板验收建议从架空低速开始：

1. 执行 `chassis status`，手推前进确认两轮编码器均递增，后退均递减。
2. 核对一圈实际计数和 `counts_per_rev`，测量有效轮径并更新 `wheel_radius_mm`。
3. 依次测试 `100 mm / 30 rpm`、`500 mm / 60 rpm`、`-200 mm / 40 rpm`。
4. 记录实际距离误差、左右轮终点计数、`heading status`和`sched`最大耗时。
5. 在不同电量和地面重复至少20次，确认无反向 runaway、超时或通信故障。

当前验证等级仅为编译和静态检查；烧录后的编码器方向、轮径标定、停止误差和航向
修正方向仍需实板确认。
