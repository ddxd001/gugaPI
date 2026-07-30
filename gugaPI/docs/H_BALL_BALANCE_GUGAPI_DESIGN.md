# H题车载平衡滚球：gugaPI新增设计

## 1. 目标与职责边界

本文描述 gugaPI 为“车载平衡滚球运动控制系统（H题）”需要新增的固件和上位机能力。

系统分工如下：

- MaixCAM Pro安装在摆杆正上方，只负责识别钢球并输出钢球相对中心点O的物理位置。
- gugaPI负责视觉数据接收、球状态估计、滚球闭环、DM-G6220控制、小车巡线、比赛计时和序列执行。
- MaixCAM不直接生成摆杆角度或达妙电机命令，防止视觉算法与执行机构耦合。
- 第一阶段完成题目第2～5项；完整无线图传录像和第6项任意指定位置入口后续补充。

## 2. 当前代码基础

当前 `master` 已具备：

- 8路ADC灰度和三路UART红外两种线路传感器来源；
- 巡线、IMU航向、编码器定距、路口复合控制；
- MotorDriver底盘控制和通信故障停车；
- DM-G6220 MIT协议、定位、定速、保持、失能和全局故障处理；
- ActionRunner、8个FRAM序列槽、序列编辑器和比赛模式；
- 按钮、OLED、Shell和主机测试框架。

当前主要调度周期：

| 模块 | 周期 |
| --- | ---: |
| IMU采样 | 5 ms |
| 巡线任务 | 2 ms |
| DM-G6220/CAN任务 | 10 ms |
| 底盘反馈 | 20 ms |
| ActionRunner | 50 ms |

序列启动时由 `SeqStore_Load()` 把FRAM指令读取、校验并反序列化到
`ActionRunnerState::instrs[54]`。运行期间不逐条读取FRAM，因此序列槽不会降低滚球或巡线控制频率。

## 3. 新增控制架构

新增控制链：

```text
MaixCAM位置帧
    -> BallVision协议解析与新鲜度判断
    -> 球位置/速度估计
    -> BallBalance位置控制、俯仰补偿、加速度前馈
    -> 双连杆标定映射
    -> DM-G6220连续位置参考
```

序列只启动、停止和轮询这些控制器：

```text
ActionRunner 20 Hz
    -> BallBalance 100 Hz
    -> TrackCourse / LineFollow 100~500 Hz调度
    -> DM-G6220 100 Hz
```

### 3.1 BallVision

新增 `FEATURE_ENABLE_BALL_VISION`，按现有分层建立：

- `drivers/ball_vision_uart`：UART收发和环形缓冲；
- `board/board_ball_vision`：UART4资源绑定，PB22作为RX连接MaixCAM TX；
- `app/ball_vision`：固定帧解析、CRC、统计、新鲜度和有效性。

建议使用UART RX中断和256字节环形缓冲。数据量较小，不占用当前已经用于调试串口、OLED和IR3的DMA资源。ISR只搬运字节和记录错误，协议解析在1～2 ms前台任务中完成。

调试接口：

```text
vision status
vision stats
vision clear
vision inject <position_0p1mm> <confidence>
```

`vision inject`仅在 `dev-running` 中可用，使MaixCAM尚未完成时也能开发和测试滚球控制器。

### 3.2 球状态估计

BallVision发布的每个有效样本包含接收时间和视觉处理延迟。100 Hz估计器使用整数α-β滤波：

- 新样本到达时校正位置和速度；
- 两帧之间按当前速度预测；
- 拒绝越界、时间异常和低置信度样本；
- 视觉延迟用于回推实际采样时刻。

默认状态策略：

- 有效球位置年龄不超过60 ms：正常控制；
- 60～120 ms：进入降级，冻结积分并限制最大梁角；
- 超过120 ms：当前滚球任务失败；
- 球位置绝对值超过115 mm：认为接近摆杆端部，立即终止车辆运动。

通信持续但没有识别到球与UART完全断线必须分别统计和显示。

### 3.3 BallBalance

新增100 Hz非阻塞控制器：

```cpp
BallBalance_StartHold(target_0p1mm);
BallBalance_StartMove(target_0p1mm, timeout_ms);
BallBalance_Stop(disable);
BallBalance_Update();
BallBalance_GetState();
```

状态至少包括：

- `IDLE`
- `ENABLING`
- `HOLD`
- `MOVE`
- `LEVELING`
- `SUCCESS`
- `FAILED`

控制结构：

1. 球位置和球速度PD/PID生成绝对梁角目标；
2. 使用IMU pitch补偿车体俯仰；
3. 使用左右轮平均速度变化率估计纵向加速度并提供前馈；
4. 对梁角、梁角变化率和积分进行限幅；
5. 通过五点单调标定表把梁角转换为DM电机位置；
6. 位置、速度同时满足阈值并连续稳定200 ms后才判定完成。

H3内部使用平滑目标轨迹执行 `0 -> +50 mm -> -50 mm`，避免参考值瞬间反向造成严重超调。

### 3.4 DM-G6220连续控制

现有 `dm_position` 是带轨迹、稳定判定和超时的离散动作，不应被滚球控制器每10 ms重复启动。

DM控制器需要增加连续外部位置参考：

```cpp
DmG6220Controller_ExternalAcquire(owner);
DmG6220Controller_ExternalSetPosition(target_mrad);
DmG6220Controller_ExternalRelease(disable);
```

控制所有权至少区分：

- Shell手动调试；
- 普通DM序列动作；
- BallBalance。

BallBalance持有控制权时，Shell和普通DM动作返回 `busy`。DM反馈超过100 ms、CAN bus-off和电机状态码8～14继续进入现有全局故障并失能。

## 4. 序列槽设计

正式比赛继续使用FRAM序列槽，默认映射：

| 槽位 | 题目 | 默认流程 |
| ---: | --- | --- |
| 0 | H2 | 开始 → 一圈A点精停 → 结束 |
| 1 | H3 | 开始 → 滚球到+50 mm → 滚球到-50 mm → 结束 |
| 2 | H4 | 开始 → 保持0 mm → 通过B后停车 → 结束 |
| 3 | H5 | 开始 → 保持0 mm → 一圈通过A后停车 → 结束 |

计划新增操作码：

- `ACT_OP_BALL_HOLD = 23`：启动后台保持并立即完成；
- `ACT_OP_BALL_MOVE = 24`：移动到目标并等待稳定；
- `ACT_OP_BALL_DISABLE = 25`：停止滚球闭环并失能；
- `ACT_OP_TRACK_COURSE = 26`：执行到B、一圈精停或一圈通过A。

`ball_hold`完成后，BallBalance仍在后续巡线节点期间运行。H题比赛管理器持续监视后台滚球状态；即使当前ActionRunner正在执行巡线，视觉或滚球失败也必须取消序列、停车并收平摆杆。

普通序列结束继续默认失能DM。H3/H4/H5由H题比赛管理器使用“成功后保留滚球保持”策略启动，便于评委观察最终位置；B2停止、故障、切换模式或复位时统一失能。

## 5. 巡线扰动抑制

小车纵向加速时钢球会相对车体向后运动，刹车时会向前运动。小角度下可近似为：

```text
ball_acceleration ~= 5/7 * (g * beam_angle - chassis_acceleration)
```

gugaPI侧采用以下措施：

- H4/H5使用尽量恒定的基础速度；
- 起步、停车和速度切换使用平滑斜坡；
- 限制巡线差速修正变化率，避免左右高频摆动；
- 根据编码器里程在直线、圆弧和终点前使用不同速度；
- 使用IMU pitch补偿摆杆相对重力方向的变化；
- 使用轮速变化率形成纵向加速度前馈；
- 记录球位置、梁角、pitch、轮速和巡线修正，依据实测数据调节前馈增益。

第一轮控制先完成静止PD和pitch补偿，确认稳定后再加入加速度前馈，不同时调整全部参数。

## 6. 比赛界面和持久化

competition profile当前关闭OLED，H题版本必须重新开启。OLED至少显示：

- H题编号；
- 当前序列阶段；
- 0.1 s计时；
- 球位置；
- 完成时间、最大球误差和失败原因。

结果保持到下一次选择或启动，不在2秒后自动清除。

当前使用全新FRAM布局：ConfigStore A/B各1 KiB，滚球、红外、达妙和原有
参数统一保存；8个SeqStore槽每槽最多54条。滚球参数和五点双连杆映射均可
由上位机调整，启动 `hold/move` 时复制为运行快照。

上位机提供独立“滚球系统”联调栏目。该页面使用专用
`telem on ball 20` 以50 Hz显示滚球、视觉、DM和车体pitch，不扩展旧版全字段
遥测。页面支持最近60秒曲线、最多100000帧记录和CSV导出；运动命令受
`dev-running`、无故障、序列空闲和每次连接的机构安全确认共同约束。
滚球参数先提交RAM并在下一次 `hold/move` 采用，五点映射通过单条
`ball map` 原子提交，二者均只有显式 `param save` 才写入FRAM。

最新代码默认线路传感器为IR3。若比赛实车使用8路ADC板，必须明确执行并保存：

```text
param set line_sensor_source 0
param save
```

## 7. 实施顺序

1. 冻结MaixCAM固定12字节协议和测试向量；
2. 完成BallVision解析器、主机测试和 `vision inject`；
3. 用PC串口脚本模拟MaixCAM连续发送；
4. 完成DM连续位置参考和所有权；
5. 完成静止中心保持；
6. 完成H3的 `+50 mm -> -50 mm`；
7. 加入IMU pitch补偿；
8. 在直线起步/刹车中调加速度前馈；
9. 增加序列操作码23～26和槽0～3模板；
10. 完成H4/H5滚球保持与巡线并行；
11. 完成A线检测、B点里程和H2精停；
12. 完成FRAM新布局、上位机参数面板和比赛OLED。

### 7.1 当前分支实施状态

`codex/h-ball-balance` 已完成第一阶段控制链：

- UART4/PB22非阻塞接收、固定12字节协议、CRC、重同步和新鲜度统计；
- `vision status/stats/clear/inject` 调试命令；
- BallBalance 100 Hz位置/速度估计、PD/PID、俯仰补偿接口、限幅、斜率限制和五点映射；
- DM-G6220外部连续位置控制所有权；
- `ball status/params/hold/move/stop` 调试命令；
- 序列操作码23～25，以及上位机编译、反编译、模拟设备和帮助中心支持；
- 专用50 Hz滚球遥测，以及上位机曲线记录、手动控制、参数快调、视觉注入和
  五点双连杆采样页面；
- ConfigStore A/B双副本、滚球参数持久化和全新8槽×54条FRAM布局；
- 视觉、滚球、DM所有权、ActionRunner和序列编辑器主机测试。

当前仍属于台架联调版本。加速度前馈、`ACT_OP_TRACK_COURSE=26`、H2～H5
默认槽位模板和比赛专用OLED页面尚未实施。
默认双连杆映射仅用于验证软件链路，连接实物前必须按机构重新标定。

## 8. 验收目标

- MaixCAM位置流50 Hz，连续通信无CRC错误；
- gugaPI滚球控制和DM发送保持100 Hz且无明显任务超期；
- H3总时间不超过5 s，±50 mm处最大误差绝对值不超过10 mm；
- H4的AB时间不超过8 s，球位置误差绝对值不超过10 mm；
- H5整圈不超过30 s，球位置误差绝对值不超过10 mm；
- 视觉断线、球丢失、CAN断线、DM故障、巡线丢失和estop均能安全停车。
