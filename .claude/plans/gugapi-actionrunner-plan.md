# gugaPI ActionRunner 实现计划

## 目标

把已验证的航向环原语(直行保持、按角度转弯)包装成统一的"动作",并提供一个顺序执行器(ActionRunner),让一条动作表能组合成比赛流程。对应 CHASSIS_CONTROL_PLAN 阶段三/四。

## 设计:tagged union + 顺序执行器(无堆,匹配项目风格)

### 动作类型
```cpp
enum ActionType { ACTION_NONE, ACTION_DRIVE, ACTION_TURN, ACTION_WAIT, ACTION_STOP };
enum ActionStatus { ACTION_PENDING, ACTION_RUNNING, ACTION_DONE, ACTION_FAILED };

struct Action {
    ActionType type;
    int32_t param1;       // DRIVE: base_rpm; TURN: delta_deg
    uint32_t param2;      // DRIVE/WAIT: duration_ms
    ActionStatus status;
    uint32_t start_ms;
};
```
固定数组 `Action actions[16]`(384 B),无动态分配。

### 执行器状态
```cpp
struct ActionRunnerState {
    Action actions[16];
    uint8_t count;
    uint8_t current;
    bool running;
    ActionStatus overall;     // DONE/FAILED
    uint32_t seq_start_ms;
};
```

### 接口(`app/action.{h,cpp}`)
- `ActionRunner_Init()`
- `ActionRunner_Clear()` -- 清空序列
- `ActionRunner_AddDrive(rpm, ms)` / `AddTurn(deg)` / `AddWait(ms)` / `AddStop()` -- 追加动作(满则返回 FULL)
- `ActionRunner_Start()` -- 从第 0 个开始执行
- `ActionRunner_Cancel()` -- 取消 + `Chassis_Stop` + `Heading_Stop`
- `ActionRunner_Update()` -- 周期(50ms):执行当前动作
- `ActionRunner_GetState()` -- shell/遥测

### Update 逻辑
```
if (!running) return;
if (Fault_HasFault()) -> Cancel, overall=FAILED, return;
if (current >= count) -> running=false, overall=DONE, Heading_Stop+Chassis_Stop, return;
a = &actions[current];
if (a->status==PENDING) -> StartAction(a), status=RUNNING;
r = UpdateAction(a);
if (r==DONE) -> a->status=DONE, current++; (若到末尾 -> 完成,停车)
if (r==FAILED) -> running=false, overall=FAILED, 停车
```

### StartAction / UpdateAction
- **DRIVE**: Start=`Heading_HoldStart(rpm)`, start_ms=now. Update= `if (now-start_ms > duration) DONE else RUNNING`(航向保持由 heading 任务做)。超时 kActionDriveTimeoutMs(15s)->FAILED。
- **TURN**: Start=`Heading_TurnStart(deg)`. Update= `if (Heading_GetState()->mode==IDLE) { if Fault -> FAILED else DONE }`(转弯完成时 heading 内部已 Heading_Stop)。超时 kActionTurnTimeoutMs(12s)->FAILED(航向环自身 8s 超时是兜底)。
- **WAIT**: Start=start_ms=now. Update= `if (now-start_ms > duration) DONE else RUNNING`。
- **STOP**: Start=`Heading_Stop()`+`Chassis_Stop()` -> 立即 DONE。

### 安全
- 每次 Update 查 `Fault_HasFault()` -> 取消 + FAILED。
- 序列总超时 kSequenceTimeoutMs(30s) -> 取消。
- 完成或取消都 `Chassis_Stop` + `Heading_Stop`。
- 上电不自动跑(`running=false`)。
- 不直接写电机,只调 heading/chassis(走 lease/watchdog)。

## 调度

`app_main.cpp`:`App_Init` 里 `ActionRunner_Init()` + `Scheduler_AddTask("action", App_ActionTask, 50ms, ...)`。门控 `FEATURE_ENABLE_IMU && FEATURE_ENABLE_MOTOR_DRIVER`。新 .cpp -> 需要把 action.cpp 加进 `Debug/app/subdir_vars.mk` + makefile link 行(CLI 手工加,CCS Clean Build 会重生)。

## Shell 命令(`app_shell.cpp`)

```
run clear                       -- 清空序列
run drive <rpm> <ms>            -- 追加定时直行(带航向保持)
run turn <deg -180..180>        -- 追加转弯
run wait <ms>                   -- 追加等待
run start                       -- 开始执行序列
run cancel                      -- 取消执行(停车)
run status                      -- running / current / count / 当前动作类型+状态 / overall
```
示例:`run clear; run drive 30 1500; run turn 90; run drive 30 1000; run turn -90; run wait 500; run start`。

## 实施顺序(每步可独立构建+回归)

1. `app/action.{h,cpp}`:动作结构 + 执行器 + StartAction/UpdateAction + 安全。构建零警告。
2. `app_main`:Init + 注册 50ms 任务(+ makefile 加 action.cpp)。构建。
3. `app_shell`:`run` 命令。构建 + 烧录。
4. 台架回归(架空先,再地面):
   - `run status` -> not running, 0/0。
   - `run clear; run drive 0 1000; run start` -> hold 0 for 1s,完成。
   - `run drive 30 1500; run turn 90; run drive 30 1000; run start` -> 跑序列。
   - `run cancel` 中途取消 -> 停车。
   - 故障注入(拔 IMU) -> 序列 FAILED,停车。

## 风险与回退

- TURN 完成判定靠 `Heading_GetState()->mode==IDLE`:转弯成功/超时/fault 都会让 mode=IDLE,需用 `Fault_HasFault()` 区分成功 vs 失败。
- 动作间无间隔:一个 DONE 立即开始下一个(下个周期)。若需间隔,加 `run wait`。
- 序列存在 RAM(掉电丢失)-- 比赛流程后续可考虑固化到 FRAM(阶段六)。
- 单步提交,可 revert。

## 不做(v1 范围外)

- 动作持久化(FRAM)-- 阶段六。
- 条件分支/循环(只顺序执行)。
- 循迹动作(阶段五,但接口已预留:`AddLineFollow` 可后加)。
- 比赛模式自动启动(阶段六)。
