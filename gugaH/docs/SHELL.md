# gugaH Shell

调试串口为 UART0，PB1/RX、PB0/TX，115200 8N1。命令只在主循环解析。

| 命令 | 说明 |
| --- | --- |
| `status` | 当前题号、细分失败原因、锁存时间、最大球误差和分设备错误计数 |
| `task 2..8` | READY 状态选择题目；7 为 CAL_ZERO，8 为 CAL_H2_LOOP |
| `start` / `stop` | 启动或立即中止 |
| `fault clear` | 非运行状态清故障并回 READY |
| `gray raw` | 八路原始 ADC |
| `gray white` / `gray black` | 用当前帧更新白/黑标定 |
| `gray status` | 质心、有效掩码和赛道状态 |
| `chassis status` | 轮速、编码器和反馈有效性 |
| `chassis test L R` | READY 下限幅 ±60 RPM 低速测试 |
| `chassis stop` | 结束底盘测试 |
| `vision status` / `vision stats` | 位置、帧龄、有效帧和 CRC 统计 |
| `vision inject P [C D]` | READY 下安全注入位置(0.1mm)、置信度、延迟 |
| `vision clear` | 清除注入/最近视觉帧 |
| `ball status` | 球估计、视觉/IMU/DM 有效性、帧龄、错误计数和 DM 反馈 |
| `ball hold mm` / `ball move mm` | READY 下 ±100 mm 安全测试 |
| `ball stop` | 摆杆限速回水平 |
| `dm status` | 独立显示使能状态、台架控制权、位置/速度/力矩/温度、目标/参考和 CAN 统计 |
| `dm enable` / `dm disable` | READY 下三次使能确认或三次失能 |
| `dm hold` | READY 下保持当前实际位置 |
| `dm position MRAD` | READY 下绝对位置小步测试，硬限制在 ±1000 mrad |
| `dm level` | 杆静止且水平时，把当前达妙反馈位置标定为五点映射中心 |
| `dm clear` | 失能状态下连续三次清除达妙故障 |
| `dm auto` | 退出台架独占模式，恢复 READY 状态的水平保持 |

达妙独立上电、使能和小步位置测试见 [`DM_BENCH_TEST.md`](DM_BENCH_TEST.md)。
| `config show` | 显示主要参数和五点 DM 映射 |
| `config set NAME VALUE` | 非运行状态修改并立即校验 |
| `config save` | 写 FRAM 后读回逐字节校验 |
| `config defaults` | 恢复保守编译期默认值（未自动保存） |
| `telem on` / `telem off` | 开关 50 ms 固定 CSV 遥测 |

可修改参数名：`cruise`、`approach`、`h4_speed`、`h4_launch_ramp`、
`h4_stop_ramp`、`h4_brake_distance`、`h4_heading_kp`、
`h4_heading_max_corr`、`imu_gyro_bias_z`、`ball_chassis_ff`、`h5_speed`、
`h5_curve_speed`、
`h5_launch_ramp`、`h5_stop_ramp`、`h5_brake_distance`、
`h4_b_distance`、`h4_stop_distance`、
`h6_speed`、`h6_approach`、`finish_gate`、
`approach_start`、
`h6_finish_gate`、`h6_approach_start`、`h2_loop_offset`、`finish_offset`、`line_kp`、
`line_kd`、`gray_threshold`、`gray_hysteresis`、`gray_position_floor`、
`gray_min_strength`、`gray_track_mask`、`speed_kp`、`speed_ki`、`speed_kd`、
`speed_max_duty`、`speed_min_duty`、`speed_accel_rpm_s`、
`speed_decel_rpm_s`、`position_kp`、`position_ki`、`position_kd`、
`position_max_rpm`、`position_tolerance_counts`、
`motor_output_invert_flags`、`motor_encoder_invert_flags`、`ball_kp`、
`ball_kd`、`ball_ki`、`accel_ff`、`max_angle`、
`vision_invert`、`ball_zero`（0.1 mm）、`beam0..beam4`、`dm0..dm4`。

`finish_gate` 仅为旧配置兼容保留。H2 的停车目标由
`lap_distance + h2_loop_offset` 和左右轮平均编码器里程决定；`finish_offset`
是 `h2_loop_offset` 的兼容别名。偏移范围为 ±200 mm，默认 -100 mm；最后
200 mm继续巡线并从55 RPM降到15 RPM，达到目标后直接停车，H2没有总超时。

面板选择 H6 后的 `CAL ZERO` 可不使用串口完成零位标定：长按 B1 并松手
启动，B2/B3 每次向负/正半轴移动零位 1 mm，按下 B1 退出。退出后固件会
等待 OLED 总线空闲，自动将新零位写入 FRAM 并读回校验。

`CAL ZERO` 后的 `CAL H2 LOOP` 用于无串口微调 H2 停车点：长按 B1 并松手
启动，B2 每次减少10 mm，B3每次增加10 mm，长按可连续调整；OLED 同时显示
偏移和最终停车里程。短按 B1 退出后自动写入 FRAM 并读回校验。

MotorDriver 参数在上电或执行 `fault clear` 重新初始化底盘时下发；修改后应
先 `config save`，再复位或执行 `fault clear`，并从低速悬空测试重新确认。

CSV 字段依次为时间、任务状态、灰度位置/掩码、左右轮速、里程、
球位置/速度/误差、摆杆目标角、IMU 实测杆角、DM 位置和累计通信错误。

`imu status` 的 `beam_mdeg` 仅保留用于诊断，不再修正 DM 杆角命令。
