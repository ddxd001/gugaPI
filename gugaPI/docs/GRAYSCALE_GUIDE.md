# gugaPI 八路灰度传感器使用手册

> 当前右侧 MR 单电机配置可进行灰度采样、标定和道路类型识别，但
> `FEATURE_ENABLE_DIFFERENTIAL_CHASSIS=0`，不得执行 `lf start`、自动
> 路口转向或任何需要底盘运动的验收步骤。本文中的运动步骤保留给双轮硬件
> 恢复后使用。

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

至少一路核心通道越过开启阈值，或者相邻两路同时越过关闭阈值且总强度足够，位置才
能从冷启动状态获得有效线。已经跟踪到有效线后，如果当前仅剩一个与上一线段相邻的
短暂全白间隙后重新出现的连续弱模拟信号，算法仍使用其加权位置；全白间隙与弱跟踪
合计最多占用 8 个位置帧（约 56 ms），强度门槛为正常最小线强度的 1/4。该机制用于
跨过窄线经过相邻探头之间时的物理间隙，不允许从弱噪声冷启动，也不允许弱信号跨越到
不相邻通道。分离的两个强峰判为 `multiple`，循迹区内四路及以上同时有效判为
`wide`。最外侧 0、7 路不会拉动循迹位置，也不会单独造成宽线误判。

循迹启动时必须已经是 `valid`。运行中硬件故障、数据超时或通道异常立即停车；有界
每个全白帧也会计入连续异常计数，相邻弱线恢复后该计数清零；`lost/multiple/wide`
连续 6 个完整位置帧（约 42 ms）仍异常即停车，因此连续全白不会等待完整 56 ms。
弱跟踪耗尽后，弱响应同样按无效帧处理，
不执行无限保持或盲目搜线。

## 3. 首次启动与标定

保持车辆静止，先放在白底，再放在黑线上：

```text
gray status
gray calib white
gray calib status
gray calib black
gray calib status
gray calib commit
param save
```

`white` 和 `black` 默认各采 64 个位置帧。算法按通道排序，去掉两端各 1/8 样本后
取均值；每路黑白跨度必须至少为 400 ADC counts，并且必须达到采集噪声的 8 倍。
任一路失败时 `commit` 返回错误，`fault_mask` 指出失败通道。

成功提交会同时设置并持久化推荐处理参数：

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

旧配置中的默认掩码 `0x3C` 会迁移为 `0x7E`；旧实车曾使用的精确参数组合
`200/40/20/50/0x3C` 也会迁移到上述安全值。迁移后配置标记为 dirty，执行一次
`param save` 才会写入 FRAM。用户主动配置的其它掩码保持不变。

## 4. 诊断与循迹

```text
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
事件。

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

默认左转`+90°`、右转`-90°`、对齐距离0 mm、对齐速度30 RPM、转后静止等待黑线
800 ms。运行期可修改：

```text
road turn set 90 -90 0 30 800
```

这组参数尚未写入ConfigStore，复位后恢复默认值。`align_mm`应在测量灰度阵列到轮轴的
实际安装距离并完成低速验证后再设置。任何自动弯道阶段可执行`road auto off`；若正在
转弯，该命令会停止航向和循迹控制。

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
