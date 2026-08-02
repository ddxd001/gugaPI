# gugaH Shell

调试串口为 UART0，PB1/RX、PB0/TX，115200 8N1。命令只在主循环解析。

| 命令 | 说明 |
| --- | --- |
| `status` | 当前题号、细分失败原因、锁存时间、最大球误差和分设备错误计数 |
| `task 2..10` | READY 状态选择题目；7为H7，8为CAL_ZERO，9为CAL_H2_LOOP，10为CAL_GRAY |
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
`course_line_kp`、`course_line_kd`、`course_line_max_corr`、`course_line_slew`、
`h4_b_distance`、`h4_stop_distance`、
`h6_speed`、`h6_approach`、`h2_loop_offset`、`finish_offset`、`line_kp`、
`line_kd`、`line_max_corr`、`line_slew`、`gray_threshold`、`gray_hysteresis`、`gray_position_floor`、
`gray_min_strength`、`gray_track_mask`、`speed_kp`、`speed_ki`、`speed_kd`、
`speed_max_duty`、`speed_min_duty`、`speed_accel_rpm_s`、
`speed_decel_rpm_s`、`position_kp`、`position_ki`、`position_kd`、
`position_max_rpm`、`position_tolerance_counts`、
`motor_output_invert_flags`、`motor_encoder_invert_flags`、`ball_kp`、
`ball_kd`、`ball_ki`、`accel_ff`、`max_angle`、
`vision_invert`、`ball_zero`（0.1 mm）、`beam0..beam4`、`dm0..dm4`。

H2 的停车目标由 `lap_distance + h2_loop_offset` 和左右轮平均编码器里程决定；`finish_offset`
是 `h2_loop_offset` 的兼容别名。偏移范围为 ±200 mm，默认 -100 mm；最后
200 mm继续巡线并从55 RPM降到15 RPM，达到目标后直接停车，H2没有总超时。

H5/H6共用的直道默认参数为 `course_line_kp=18`、`course_line_kd=12`、
`course_line_max_corr=25 RPM`、`course_line_slew=400 RPM/s`。旧的
`h5_line_*` 名称仍可作为兼容别名。弯道继续使用全局
`line_kp/line_kd`、`line_max_corr`和`line_slew`；
进入和离开物理弯道的250 mm范围内平滑插值。入弯速度转换固定使用
30 RPM/s减速度并预留180 mm稳球距离；`h5_stop_ramp`仍只控制最终停车，
`h5_launch_ramp`继续控制起步和出弯加速。直道D项还使用0.2系数低通，
随弯道插值逐渐恢复到原始响应。

H6底盘完全复用H5的 `h5_speed`、`h5_curve_speed`、`h5_launch_ramp`、
`h5_stop_ramp`和`h5_brake_distance`，并同样在 `lap_distance + 50 mm` 冻结成绩后
缓停，不再识别A线。`h6_speed`和`h6_approach`仅作为修改共用H5速度的兼容别名；
原 `h6_finish_gate`、`h6_approach_start` 已停用。H5保持球在0 mm，H6保持B2/B3
设定位置。

H7底盘完全复用H4的直行策略和参数：`h4_speed`、`h4_launch_ramp`、
`h4_stop_ramp`、`h4_brake_distance`、`h4_b_distance`、`h4_stop_distance`
以及IMU航向控制均与H4相同，不使用灰度循线。区别仅是H4固定保持0 mm，
H7在READY下用B2/B3以1 mm步进设置±100 mm钢球目标，并在发车前提前保持
到位。H7同样在8秒内通过B点、通过后缓停，PASS后继续保持设定目标。

面板选择 H7 后的 `CAL ZERO` 可不使用串口完成零位标定：长按 B1 并松手
启动，B2/B3 每次向负/正半轴移动零位 1 mm，按下 B1 退出。退出后固件会
等待 OLED 总线空闲，自动将新零位写入 FRAM 并读回校验。

`CAL ZERO` 后的 `CAL H2 LOOP` 用于无串口微调 H2 停车点：长按 B1 并松手
启动，B2 每次减少10 mm，B3每次增加10 mm，长按可连续调整；OLED 同时显示
偏移和最终停车里程。短按 B1 退出后自动写入 FRAM 并读回校验。

`CAL H2 LOOP` 后的 `CAL GRAY` 用于无串口完成灰度白黑标定：长按 B1 并
松手启动，将全部探头置于白色底面后短按 B2，再置于黑线或黑色标定面后
短按 B3。OLED 分别显示 `WHITE OK`、`BLACK OK`；无效采样显示
`CAPTURE ERR`，并保留上一组有效标定值。短按 B1 退出后自动写入 FRAM
并读回校验。校准过程中底盘始终停止，也不要求旧灰度标定有效。

MotorDriver 参数在上电或执行 `fault clear` 重新初始化底盘时下发；修改后应
先 `config save`，再复位或执行 `fault clear`，并从低速悬空测试重新确认。

CSV 原有字段依次为时间、任务状态、灰度位置/掩码、左右轮速、里程、
球位置/速度/误差、摆杆目标角、IMU 实测杆角、DM 位置和累计通信错误；末尾
新增H5/H6巡线弯道系数（0–1000）、轮速修正、滤波D项、左右轮目标RPM。

`imu status` 的 `beam_mdeg` 仅保留用于诊断，不再修正 DM 杆角命令。
