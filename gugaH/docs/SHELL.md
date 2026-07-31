# gugaH Shell

调试串口为 UART0，PB1/RX、PB0/TX，115200 8N1。命令只在主循环解析。

| 命令 | 说明 |
| --- | --- |
| `status` | 当前题号、细分失败原因、锁存时间、最大球误差和分设备错误计数 |
| `task 2..6` | READY 状态选择题目 |
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

可修改参数名：`cruise`、`approach`、`h4_speed`、`h5_speed`、
`h5_approach`、`h6_speed`、`h6_approach`、`finish_gate`、
`approach_start`、`h5_finish_gate`、`h5_approach_start`、
`h6_finish_gate`、`h6_approach_start`、`finish_offset`、`line_kp`、
`line_kd`、`gray_threshold`、`gray_hysteresis`、`gray_position_floor`、
`gray_min_strength`、`gray_track_mask`、`speed_kp`、`speed_ki`、`speed_kd`、
`speed_max_duty`、`speed_min_duty`、`speed_accel_rpm_s`、
`speed_decel_rpm_s`、`position_kp`、`position_ki`、`position_kd`、
`position_max_rpm`、`position_tolerance_counts`、
`motor_output_invert_flags`、`motor_encoder_invert_flags`、`ball_kp`、
`ball_kd`、`ball_ki`、`pitch_gain`、`accel_ff`、`max_angle`、
`vision_invert`、`beam0..beam4`、`dm0..dm4`。

MotorDriver 参数在上电或执行 `fault clear` 重新初始化底盘时下发；修改后应
先 `config save`，再复位或执行 `fault clear`，并从低速悬空测试重新确认。

CSV 字段依次为时间、任务状态、灰度位置/掩码、左右轮速、里程、
球位置/速度/误差、摆杆角度、pitch、DM 位置和累计通信错误。
