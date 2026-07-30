# gugaPI 八路灰度传感器使用手册

## 1. 硬件与采样

传感器通过 8:1 模拟多路复用器连接到 gugaPI：PA16、PC20、PC21 分别为地址
bit2、bit1、bit0，PA15 为 ADC1 ADCIN0。ADC 为 12 位，结果范围 0..4095。

灰度任务仍以 1 ms 周期调用，调度器接口没有改变。采样流水线连续采集中间六路
`1..6`，每个位置帧前交替插入一路最外侧通道：

```text
0,1,2,3,4,5,6, 7,1,2,3,4,5,6
```

因此中间六路通常每 7 ms 形成一帧，最外侧 0、7 路在 14 ms 内各更新一次。ADC 对每路
执行 4 次硬件累加并除以 4，降低单次转换噪声。ADC 中断只负责保存结果和预选下一路，
数据处理仍在任务上下文中执行；未使用 DMA，因为 DMA 不能独立完成跨两个 GPIO 端口的
MUX 地址切换。

## 2. 数据处理

每路使用自己的白点和黑点进行整数归一化：白色为 0，黑色为 1000，同时兼容 ADC
正、反两种极性。

- `active_mask`：八路数字迟滞结果，只用于道路类型、路口和诊断。
- `track_mask`：连续位置计算范围，默认 `0x7E`，即通道 1..6。
- `selected_mask`：本帧实际参与位置加权的核心通道。
- `line_position`：左正右负；中间六路坐标为 `+2000,+1500,+1000,-1000,-1500,-2000`。
- `track_state`：`valid`、`lost`、`multiple`、`wide`、`sensor_fault`。
- `weak_frames`：在探头物理间隙内连续使用弱模拟信号插值的帧数，正常强信号为 0。
- `confidence`：0..1000 的诊断量，不再单独决定是否停车。

冷启动既接受越过开启阈值的强数字证据，也接受窄模拟证据：`selected_mask` 必须只有
一个连续段且最多包含两路，总强度至少为
`max(min_line_strength/2, max(threshold-position_floor, 1))`。因此黑线位于单个探头
边缘或相邻探头之间时，可以用两路较弱黑度的总能量取得连续加权位置；数字
`active_mask` 仍独立负责道路类型和路口判定。分离弱峰、三路及以上弱响应和总强度
不足均不能从冷状态获取线路。

已经跟踪到有效线后，如果当前仅剩一个与上一线段相邻的短暂全白间隙后重新出现的
连续弱模拟信号，算法仍使用其加权位置；全白间隙与弱跟踪合计最多占用 8 个位置帧
（约 56 ms），强度门槛为正常最小线强度的 1/4。该机制不允许弱信号跨越到不相邻
通道。分离的两个强峰判为 `multiple`，循迹区内四路及以上同时有效判为 `wide`。
最外侧 0、7 路不会拉动循迹位置，也不会单独造成宽线误判。

循迹启动时必须已经是强 `valid`。运行中硬件故障、数据超时或通道异常立即停车。
启动后只要处理结果仍为 `line_detected=1、position_valid=1、track_state=valid`，包括
处于探头间隙的弱模拟位置，都继续参与正常 PID。几何真正变为
`lost/multiple/wide` 时，巡线层锁存丢失前最后一组左右轮目标并持续重发，不按
`losttimeout` 减速或停车；连续 3 个有效位置帧重新出现后清除微分历史并恢复 PID。
道路分类器的 `observing` 窗口仍优先保持基础速度直行，以便事件发布和路口控制接管。
无限保持只适用于新鲜且无通道异常的 ADC8 几何丢失；人工停止、动作时限、底盘通信
失败、原始采样过期和全局故障仍立即结束运动。

## 3. 首次启动与标定

保持车辆静止，先放在白底，再放在黑线上：

```text
gray status
gray live
gray calib begin
gray calib white
gray calib status
gray calib preview
gray calib black
gray calib status
gray calib preview
gray calib commit
param save
```

`white` 和 `black` 默认各采 64 个位置帧。算法按通道排序，去掉两端各 1/8 样本后
取均值；每路黑白跨度必须至少为 400 ADC counts，并且必须达到采集噪声的 8 倍。
任一路失败时 `commit` 返回错误，`fault_mask` 指出失败通道。`begin` 和每次采集前
都要求最近 200 ms 内收到有效完整帧；采集中 500 ms 没有新完整帧或超过总期限会
自动结束为 `timeout`，不会永久占用标定状态。

成功 `commit` 只更新运行中的 ConfigStore RAM 参数并标记 dirty，同时设置推荐处理参数：

| 参数 | 默认值 | 含义 |
| --- | ---: | --- |
| `gray_threshold` | 500 | 数字迟滞中心 |
| `gray_hysteresis` | 300 | 开/关间隔，对应 on=650、off=350 |
| `gray_position_floor` | 100 | 位置计算噪声底 |
| `gray_min_strength` | 600 | 相邻弱响应的最小总强度 |
| `gray_track_mask` | 126 (`0x7E`) | 中间六路循迹区 |

当前源码同时保存了本车 FRAM 读取出的八路白/黑标定值，作为 `param reset` 或 FRAM
无有效配置时的恢复值。循迹参数默认采用本车在 100 RPM 下调定的
`kp=3800、kd=600、maxcorr=30、slew=25000`；40 RPM 仅为控制器增益归一化参考，
不是默认运行速度。

全新FRAM布局不读取旧配置。首次启动或格式化后使用上述源码默认值并标记
dirty；检查参数无误后执行一次`param save`写入ConfigStore双副本。未执行
`param save` 时重启会恢复旧值，这是有意保留的 RAM/FRAM 安全边界。

## 4. 诊断与循迹

```text
gray live
gray data
gray process
gray calib show
param status
lf status
lf start 40 5000
lf stop
```

`gray process` 应重点检查：

- 白底：核心归一化值接近 0，`track_state=lost`。
- 线在中心：`track_state=valid`、`pos_valid=1`，位置接近 0。
- 单独遮住最外侧 0 或 7：`active_mask` 可变化，但核心位置不应跳变。
- 两个分离核心黑点：`track_state=multiple`。
- 循迹区任意连续四路全黑：`track_state=wide`。
- 线经过探头间隙：`track_state=valid` 且 `weak_frames` 在 1..8 内短暂递增，重新获得
  强信号后回到 0。
- `frame_ms`：正常约 7 ms；持续明显增大时检查任务超期和 ADC 超时。

首次动态验证使用 40 RPM、5 秒直线，再逐步提高速度。每次测试同时记录
`gray process`、`lf status` 和 `sched`；不要在故障未清除时直接发送运动命令。

## 5. 路口事件与自动直角弯

道路检测使用全八路迟滞位图，但不使用单帧结果直接触发动作。分类器依次经过
`normal -> observing -> latched`，收集连续侧向证据并在离开路口后重新允许触发。
单帧边缘噪声不会生成事件；同一个物理路口在重新获得连续6帧居中直线前只生成一次
事件。整个`observing`阶段都不会抢先执行6帧无效停车；左右直角弯由
连续转弯控制接管，T字仍按策略停车。分类器离开`observing`后恢复普通无效帧安全规则。

事件类型：

| 类型 | 路径含义 | 当前默认行为 |
| --- | --- | --- |
| `left_corner` | 左路，无前路 | `corner`模式自动左转 |
| `right_corner` | 右路，无前路 | `corner`模式自动右转 |
| `left_branch` | 左路+前路 | 保持直行 |
| `right_branch` | 前路+右路 | 保持直行 |
| `t` | 左路+右路，无前路 | 安全停车 |
| `cross` | 左路+前路+右路 | 保持直行 |

路口几何与动作策略分离。事件包含独立序号、路径位、置信度和进入/峰值/离开掩码；
后续可以把任意类型映射到动作序列，而不用修改灰度分类器。本版本尚未绑定自定义
动作序列。

上电默认是只检测模式，不会因左右弯事件自动转向：

```text
road status
road event
road mode detect
```

手推车辆经过不同路口，先用`road event`确认类型。需要实车自动处理左右直角弯时，
车轮方向、IMU航向和急停均已验证后执行：

```text
road turn show
road mode corner
lf start 40 10000
```

默认左转`+90°`、右转`-90°`、对齐距离0 mm、路口连续处理速度30 RPM、移动捕线超时
800 ms。运行期可修改：

```text
road align set 20 40
param save
road turn set 90 -90 20 40 800
```

`road_align_distance_mm`和`road_align_rpm`由`road align set`在线修改，执行`param save`
后保存到FRAM；转角和重捕获超时仍是运行期参数，复位后恢复默认值。路口控制按当前循迹
速度和配置上限中的较小值连续接管：编码器恒速走完对齐距离，保持前进速度进行IMU圆弧
转向。进入目标航向前最后20°时就开始累计新线路证据；若到达目标航向时已经获得
连续2帧有效黑线，会在同一轮控制更新中无停车恢复循迹及进入路口前的循迹速度。
证据不足时才继续按路口基础速度移动捕线，满足2帧后立即交接。
MotorDriver速度斜坡仍负责限制交接后的实际加速度，因此不会产生一步跳变的轮速命令。
`align_mm=0`会从循迹直接进入圆弧转向。`align_mm`应在测量灰度阵列到轮轴的实际安装距离
并完成低速验证后再设置。任何自动弯道阶段可执行`road auto off`；若正在对齐、转弯或
捕线，该命令会安全停车。IMU、编码器、底盘通信故障和捕线超时也始终停车。

## 6. 相关代码

| 层级 | 文件 |
| --- | --- |
| 采样驱动 | `drivers/grayscale/grayscale.cpp/.h` |
| 板级封装 | `board/board_grayscale.cpp/.h` |
| 标定与帧发布 | `app/app_grayscale.cpp/.h` |
| 归一化和插值 | `drivers/grayscale/grayscale_processing.cpp/.h` |
| 道路分类 | `app/grayscale_road.cpp/.h` |
| 路口行为控制 | `app/road_event_controller.cpp/.h` |
| 循迹控制 | `app/linefollow.cpp/.h` |
| 硬件配置 | `empty_cpp.syscfg` |
