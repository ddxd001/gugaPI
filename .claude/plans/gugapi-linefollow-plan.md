# gugaPI 循迹(Line-Following)实现计划 (阶段五 v1)

## 目标

用 8 路灰度传感器跟踪黑线,作为 ActionRunner 的一个动作(`LINE_FOLLOW`),底盘按线行驶。对应 CHASSIS_CONTROL_PLAN §8.1/8.2。v1 不做赛道事件识别(§8.3,v2)。

## 数据源

`App_GrayscaleGetData()->raw[8]`(0..4095,10ms 周期刷新)。8 路排成一条线,通道 0..7 对应物理位置(左右顺序需台架确认,用 `kLineSign` 翻)。

## 模块:`app/linefollow.{h,cpp}`

### 标定(自动,RAM)
`LF_CalibrateStart()` 启动 2 秒标定;`LF_Update` 在标定态记录每通道 min/max(用户把传感器扫过线+地面);2 秒后计算 `threshold[i] = (min[i]+max[i])/2`,标定完成。标定数据存 RAM(v1,掉电丢失;后续可入 ConfigStore/FRAM)。
- 极性:`kLinePolarity`(默认线=低 ADC,即黑面反射弱):`blackness = (max - value)/(max-min)`。台架确认后翻。
- 未标定时拒绝 `LF_Start`(返回 NOT_INITIALIZED)。

### 线路位置(加权平均,整数)
```
total=0; weighted=0;
for i in 0..7:
    b = clamp(blackness_i, 0, 1) * 1000   // 0..1000
    weighted += i * b;  total += b
if total < kMinLineWeight:  // 丢线
    lost = true
else:
    pos = weighted * 1000 / total    // 0..7000 (毫通道)
    error_mpos = (pos - 3500) * kLineSign   // -3500..+3500, 0=居中
```

### 控制(P,整数)
`correction = clamp(error_mpos * lf_kp / 1_000_000, ±lf_max_correction_rpm)`;`left = base - corr`,`right = base + corr`;`Chassis_SetWheelRpm(left, right)`。与 heading 一致的差速框架。

### 丢线
丢线时:保留上一次 `correction` 方向,搜索 `lf_lost_timeout_ms`(默认 500ms);超时仍丢线 -> `LF_Stop` + 停车(安全)。

### 接口
```cpp
enum LFMode { LF_IDLE, LF_CAL, LF_FOLLOW };
struct LFState { LFMode mode; int32_t error_mpos; int32_t correction_rpm; bool lost; uint32_t follow_start_ms; uint32_t follow_duration_ms; int32_t base_rpm; bool calibrated; ...cal data... };

LF_Init(); LF_CalibrateStart(); LF_Start(rpm, ms); LF_Stop(); LF_Update(); LF_GetState();
```
参数(`lf_kp`/`lf_max_correction_rpm`/`lf_base_rpm`/`lf_lost_timeout_ms`)存 LF 自身 RAM,shell 可调(`lf kp <val>` 等)。**v1 不动 ConfigStore**(避免 v3->v4 迁移),稳定后再持久化。

### 安全
- 每次 Update 查 `Fault_HasFault` / 灰度 `valid` / 标定完成 / 丢线超时 -> 任一不满足 `LF_Stop` + 停车。
- `LF_Stop` 总 `Chassis_Stop`。
- 上电 IDLE,不自动跑。
- 不直接写电机,走 `Chassis_SetWheelRpm`(lease/watchdog)。

## ActionRunner 集成

新增 `ACTION_LINE_FOLLOW`:
- StartAction:`LF_Start(param1=rpm, param2=duration_ms)`。
- UpdateAction:`if (LF_GetState()->mode == LF_IDLE) { Fault? FAILED : DONE }`(LF 跟到时或丢线超时都回 IDLE)。
- shell:`run follow <rpm> <ms>` 追加该动作。

## 调度

`app_main.cpp`:`LF_Init()` + `Scheduler_AddTask("linefollow", App_LFTask, 50ms, ...)`,门控 `FEATURE_ENABLE_GRAYSCALE && FEATURE_ENABLE_MOTOR_DRIVER`。新 .cpp -> 加进 `Debug/app/subdir_vars.mk` + makefile(CLI 手工,CCS Clean Build 重生)。

## Shell 命令(`app_shell.cpp`)

```
lf status              -- mode/error/correction/lost/calibrated/base_rpm
lf cal                  -- 启动 2s 标定(扫传感器过线+地面)
lf start <rpm> <ms>     -- 跟线行驶 N ms
lf stop
lf kp <val>             -- 调 lf_kp
lf maxcorr <val>        -- 调 lf_max_correction_rpm
lf baserpm <val>        -- 调 lf_base_rpm
run follow <rpm> <ms>  -- ActionRunner 里追加跟线动作
```

## 实施顺序(每步可独立构建+回归)

1. `app/linefollow.{h,cpp}`:标定 + 位置 + P 控制 + 丢线 + 安全。构建零警告。
2. `app_main`:Init + 注册 50ms 任务(+ makefile 加 linefollow.cpp)。构建。
3. `app_shell`:`lf` + `run follow` 命令。构建 + 烧录。
4. 台架回归(需一条测试黑线):
   - `lf status` -> idle, not calibrated。
   - `lf cal` -> 扫传感器 -> calibrated。
   - `lf kp` / `lf baserpm` 调参。
   - `lf start 30 3000` -> 跟线 3s,看 error/correction。
   - `run clear; run follow 30 3000; run start` -> 序列跟线。
   - 故障注入(挡住传感器/拔灰度) -> 丢线超时停车。
5. 调 `lf_kp`,`kLinePolarity`/`kLineSign`(若方向反)。

## 风险与回退

- 极性/通道顺序未确认:`kLinePolarity`/`kLineSign` 在 linefollow.cpp,台架翻。
- 标定 RAM:掉电丢失,每次开机重标(v1 可接受;后续入 FRAM)。
- 加权平均对单线好,对十字/宽线可能误判 -> v2 加事件识别。
- 丢线搜索仅保留方向,不主动找线 -> v2 可加左右搜索。
- 单步提交,可 revert。

## 不做(v1 范围外)

- 赛道事件识别(十字/直角/起终点)-- v2。
- 主动丢线搜索(左右扫)-- v2,仅保留方向。
- 标定/参数 FRAM 持久化 -- 稳定后入 ConfigStore(v4)。
- 弯道自适应降速 -- v2(可先手动调 base_rpm)。
