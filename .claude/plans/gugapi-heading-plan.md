# gugaPI IMU 航向闭环实现计划

## 目标

用 ICM-45686 的陀螺仪 yaw(已验证可用)实现底盘航向闭环,替代 CHASSIS_CONTROL_PLAN 阶段二原计划的 GY931。提供两个能力:

1. **航向保持(HOLD)**:锁定当前 yaw,直线行驶时用差速修正抵消偏移。
2. **相对转弯(TURN)**:转到相对当前朝向 ±N 度的目标,接近目标时分段减速,稳定后停车。

约束:保守、安全优先(上电不自动动;IMU 失效/误差过大/超时立即停车);风格对齐现有 `app/` 模块(namespace、DriverStatus、ConfigStore 参数)。

## 数据源

`app_imu` 的 `g_data.yaw_mdeg`(陀螺 Z 积分,-180000..180000 毫度,相对量,会漂移)。已验证静态稳定。漂移靠 `imu_gyro_bias_z`(ConfigStore)标定抑制;比赛短时运行可接受。**不做 GY931 融合**(用户明确用 IMU)。

## 模块设计:`app/heading.{h,cpp}`

### 状态
```cpp
enum HeadingMode { HEADING_IDLE, HEADING_HOLD, HEADING_TURN };

struct HeadingState {
    HeadingMode mode;
    int32_t target_yaw_mdeg;   // HOLD:锁定目标;TURN:wrap(当前+delta)
    int32_t base_rpm;           // HOLD 前进基准;TURN=0
    int32_t correction_rpm;     // 最近一次差速修正
    int32_t error_mdeg;         // 最近一次误差(最短角差)
    bool at_target;             // TURN 到达目标
    uint32_t at_target_since_ms;// TURN 进入容差的时间
    uint32_t last_imu_update_ms;// IMU 新鲜度
    drivers::DriverStatus last_status;
};
```

### 接口
- `Heading_Init()` -- 初始化状态(从 ConfigStore 读参数)。
- `Heading_HoldStart(int32_t base_rpm)` -- 锁当前 yaw 为目标,mode=HOLD,base_rpm。
- `Heading_TurnStart(int32_t delta_deg)` -- target=wrap(当前+delta);**|delta|>180 拒绝**;mode=TURN,base=0。
- `Heading_Stop()` -- mode=IDLE,`Chassis_Stop()`。
- `Heading_Update()` -- 周期任务(50ms):
  - IDLE 直接返回。
  - 查 `Fault_HasFault()` / `App_ImuGetData()->valid` / IMU 新鲜度(>200ms 视为失效) -> 任一不满足:`Heading_Stop()` + `Fault_Set(FAULT_SENSOR_LOST)`,返回。
  - 读 `yaw_mdeg`,`error = ShortestAngleDiff(target, yaw)`(wrap 到 ±180000)。
  - **HOLD**:`correction = clamp(error_mdeg * heading_kp / 1_000_000, ±heading_max_correction_rpm)`;`left = base - correction`,`right = base + correction`;`Chassis_SetWheelRpm(left, right)`。|error|>90000(90°)视为异常 -> 停车。
  - **TURN**:`speed = clamp(|error| * heading_kp / 1_000_000, heading_turn_min_rpm, heading_turn_max_rpm) * sign(error)`;`left = -speed`,`right = +speed`;`Chassis_SetWheelRpm`。|error|<tolerance -> 标记 at_target + 计时;连续 settle_ms 内保持在容差 -> 完成,`Heading_Stop()`。TURN 超时(如 8s) -> 停车。
- `Heading_GetState()` -- shell/遥测用。

`ShortestAngleDiff`:`d = target - cur; while(d>180000) d-=360000; while(d<-180000) d+=360000;` 处理回绕。TURN 限 ±180°,最短路径转向。

### 控制器选择
**纯整数 P**,不用 float PID(M0+ 无 FPU;航向环 20Hz 整数足够;v1 不上积分,简化+防 windup)。`correction = error_mdeg * heading_kp / 1_000_000`。若台架测出稳态误差,后续再加 I(带抗 windup)。

## ConfigStore 参数(新增 6 个,版本 2->3)

`ConfigStoreParams` 新增字段(`config_store.h`):
```cpp
int32_t heading_kp;                 // 增益,1°误差->correction_rpm = kp/1e6 RPM
int32_t heading_max_correction_rpm; // HOLD 差速上限
int32_t heading_turn_max_rpm;       // TURN 最大轮速
int32_t heading_turn_min_rpm;       // TURN 最小轮速(克服摩擦)
int32_t heading_tolerance_mdeg;     // TURN 到达容差(如 3000=3°)
uint16_t heading_settle_ms;         // TURN 稳定时间
```

`config_store.cpp`:`kParamDescriptors[]` 加 6 条(PARAM_I32 / U16,带 min/max);`kVersion` 2->3;`kPayloadLength` 68->68+26=94(6 个 I32=24 + 1 个 U16=2,对齐填充);`ResetDefaults` 设保守默认:
- `heading_kp = 200000`(1° -> 0.2 RPM,弱增益起步)
- `heading_max_correction_rpm = 30`
- `heading_turn_max_rpm = 60`,`heading_turn_min_rpm = 20`
- `heading_tolerance_mdeg = 3000`,`heading_settle_ms = 300`

FRAM 迁移:Load 时版本不匹配 -> 加载默认(现有逻辑已支持)。旧 v2 数据被忽略,用户重新调参+save。

## 调度

`app_main.cpp`:`App_Init` 里 `Heading_Init()` + `Scheduler_AddTask("heading", App_HeadingTask, 50ms, ...)`(50ms/20Hz)。任务函数包 `Heading_Update()`。`FEATURE_ENABLE_IMU && FEATURE_ENABLE_MOTOR_DRIVER` 门控。

## Shell 命令(`app_shell.cpp`)

```
heading status                       -- mode/target/error/correction/at_target/last
heading hold <base_rpm>              -- 锁当前航向,以 base_rpm 前进并保持
heading turn <deg>                   -- 相对转弯(±180)
heading stop                         -- 停
heading param <name> [val]           -- 查看/设置 heading_* 参数(走 ConfigStore_Set)
```
安全:`heading hold/turn` 前提示先 `motor ping`/`chassis stat` 确认通信。所有命令不直接写电机,只设置 heading 模式,由周期任务驱动。

## 安全审计

- 周期任务每次都查 fault / IMU valid / IMU 新鲜度 / 误差过大 -> 任一异常立即 `Chassis_Stop()` + 置 fault。
- TURN 超时(8s)停车,防卡死。
- `Heading_Stop()` 总是 `Chassis_Stop()`(双轮 coast)。
- 上电默认 IDLE,不自动动。
- 模块不直接写 MotorDriver 寄存器,只调 `Chassis_SetWheelRpm`(走现有 lease/watchdog 路径)。

## 实施顺序(每步可独立构建+回归)

1. ConfigStore:加 6 参数 + 版本/长度 bump + 默认值。构建零警告。`param list`/`param get heading_*` 验证。
2. `app/heading.{h,cpp}`:HOLD + TURN + 安全 + ShortestAngleDiff。构建。
3. `app_main`:Heading_Init + 注册 50ms 任务。构建。
4. `app_shell`:heading 命令。构建 + 烧录。
5. 台架回归(架空先):
   - `heading status` -> IDLE。
   - `heading hold 0` -> 锁航向,不动(base=0)。
   - `heading hold 30` -> 直行,手扶偏转后应回正。
   - `heading turn 90` / `turn -90` / `turn 180` -> 转到目标停。
   - `heading stop` -> coast。
   - 故障注入:拔 IMU(或停 IMU 任务) -> 应自动停车+fault。
6. 调参:`heading param heading_kp ...`,确认稳态后 `param save`。

## 风险与回退

- IMU 漂移:短时可接受;长时需 bias 标定或回退 GY931。已在代码注释说明。
- 整数 P 稳态误差:若台架测出明显偏差,加 I(复用刚修好的抗 windup 逻辑或整数条件积分)。
- TURN ±180° 边界:最短路径方向可能不一致,默认按 delta 符号;若需指定方向,加 `heading turn <deg> <cw|ccw>`。
- 单步提交,可 `git revert` 单阶段。

## 不做(v1 范围外)

- 多圈连续航向累计(unwrapping)-- TURN 限 ±180°,最短路径。
- 动作序列 ActionRunner(CHASSIS_CONTROL_PLAN 阶段四)-- heading 模块接口已为它预留(`HoldStart/TurnStart/Stop/Update`)。
- GY931 融合。
- 比赛模式/遥测 CSV(阶段六)。
