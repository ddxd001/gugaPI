# gugaPI 修复计划

## 目标与约束

修复上一轮评价中列出的系统性问题。每阶段都必须**独立可构建、可烧录、可实机回归**,遵循项目既有纪律:TI Arm Clang `-O2 -Wall` 零警告 → 烧录 → COM 串口回归 → 提交到 `feature/chassis-control` 并附串口日志与硬件版本(参考 `FOC/PROJECT_HANDOFF.md` §13)。

贯穿原则:
- **非阻塞优先**:串口 TX 采用 FOC 已验证的"环形缓冲 + 主循环协作排空"方案,**不**用 TX 完成中断(FOC 实测在该硬件上产生乱码,见 `FOC/PROJECT_HANDOFF.md` 末尾)。不需要改 `empty_cpp.syscfg`。
- **保持上电不转**:任何阶段都不能让底盘上电自动运动。
- **保守增量**:先修安全/实时/潜伏 bug,后做架构搬迁;架构搬迁(Phase 6)触碰关键控制路径,放在最后并设检查点。
- **风格对齐**:C++ 命名空间、`DriverStatus` 返回、Config 结构、`XXX_Init/Update` 约定。

## 阶段总览

| Phase | 主题 | 严重度 | 可独立交付 | 触碰关键控制路径 |
| --- | --- | --- | --- | --- |
| 0 | 非阻塞串口 TX(根因) | 🔴 高 | 是 | 否 |
| 1 | 潜伏正确性 bug(ICM 字节序 / PID 抗饱和 / chassis 整数) | 🔴 高 | 是 | 否 |
| 2 | Fault 实化 + 初始化失败冒泡 + init 顺序 | 🔴 高 | 是 | 否(但改 init 顺序) |
| 3 | 底盘状态反馈周期任务 | 🟠 中 | 是 | 否 |
| 4 | I2C2 互斥 + INA219 原子快照 | 🟠 中 | 是 | 否 |
| 5 | MotorDriver 事务原子性 + UART 帧总超时 | 🟠 中 | 是 | 是(协议层) |
| 6 | 架构搬迁:协议下沉到 drivers/ + .h/.cpp 拆分 + 循环 include | 🟡 中低 | 是 | 是(大改) |
| 7 | 杂项清理 | 🟡 低 | 是 | 否 |

**推荐执行节奏**:Phase 0→3 作为第一个里程碑(安全 + 实时 + 潜伏 bug,全部本地化、高 ROI、互相解耦),完成后**停下来回归并检查点**,再决定是否继续 4→7。Phase 6 是风险最高的控制路径搬迁,单独评审。

---

## Phase 0 — 非阻塞串口 TX(根因)

**问题**:`DebugUart_WriteChar` 逐字节阻塞 `DL_UART_Main_transmitDataBlocking` 且丢弃返回值(`services/debug_uart.cpp:72`);`LOG_*`、`Shell_Write*` 全部走它,50 字符阻塞约 13ms,与协作调度器无截止期丢弃(`scheduler.cpp:158`)叠加,任何快任务里打日志都会爆截止期。`Time_DelayMs` 是裸 busy-wait(`time.cpp:51-57`),被 shell 用到 30s。

**改动**:
- `services/debug_uart.h`:新增 TX 环形接口
  - `void DebugUart_TxPump(void);`(主循环调用,每次有界地往 TX FIFO 推 N 字节)
  - `uint32_t DebugUart_GetTxDroppedCount(void);`
  - `uint16_t DebugUart_GetTxPending(void);`
  - 保留 `DebugUart_WriteChar`;新增内部 `DebugUart_WriteCharBlocking(char)`(仅 boot/panic 用)
- `services/debug_uart.cpp`:
  - 新增 `g_txHead/g_txTail/g_txBuffer[DEBUG_UART_TX_BUFFER_SIZE]`(建议 512 B,可配置于 `debug_config.h`)、`g_txDroppedCount`(volatile)。
  - `DebugUart_WriteChar`:`g_debugUartReady` 为假时走 blocking(保证 boot banner 出现);为真时压入环形,满则 `g_txDroppedCount++` 丢弃,**立即返回**。
  - `DebugUart_TxPump`:当 TX FIFO 不满且环形非空时,取字节 `DL_UART_Main_transmitDataBlocking`;**检查其返回值**,超时/错误不再吞掉(至少计数)。每调用最多推 FIFO 深度个字节,保证 pump 有界。
  - TX 环形同 RX 一样用 SPSC(主循环单写、pump 单读),无需临界区。
- `config/debug_config.h`:新增 `DEBUG_UART_TX_BUFFER_SIZE`。
- `app/app_main.cpp` 或 `empty_cpp.cpp`:`main` while 循环里在 `Scheduler_Run/App_Run/Shell_Process` 旁加 `DebugUart_TxPump()`(`empty_cpp.cpp:234-238`)。
- `services/log.cpp`:`Log_Write` 改为只调 `DebugUart_WriteString`(已非阻塞)。可选新增运行时等级阈值 `Log_SetLevel(LogLevel)`(默认 INFO),受 `FEATURE_ENABLE_LOG` 门控;`Log_Init` 从 feature 配置读默认等级。
- `services/time.cpp`:`Time_DelayMs` 加注释标注"仅限 shell/非实时上下文",并新增 `Time_DelayMsNonBlocking` 的说明;**不在本阶段删 API**(shell 仍用),但确保控制路径任务不调用它(审计 `app/` 内 `Time_DelayMs` 调用点,把任务内的改用调度器周期)。
- 新增 shell 命令 `txstat`:打印 tx pending/dropped、rx available/dropped(对齐 FOC 的 `u`)。

**验收**:串口跑 `help`(24 命令长列表)、`motor rpm`、电机活动时 `f` 长状态行;前后 `txstat` 显示 `dropped=0`;IMU 10ms 任务与 chassis 100ms 任务在持续打印时仍按期执行(用 `FEATURE_ENABLE_SCHEDULER_STATS` 的 `max_late_ms` 验证不增长)。

---

## Phase 1 — 潜伏正确性 bug

### 1a. ICM-45686 字节序(🔴 高,潜伏)
- `drivers/icm45686/icm45686.cpp`:`CombineLe`(`:112-117`)改为大端组合 `(lo << 8) | hi`,重命名 `CombineBe`;`ReadSensors`(`:247-253`)调用同步更新。依据 `HARDWARE_INTERFACE.md:373`(ICM-45686 大端)。
- 验收:`imu sample` 静置时 `acc` Z 轴 ≈ +1g(±4g 量程 raw≈8192 或换算后 ≈1000 mg),陀螺三轴 ≈0,温度合理。

### 1b. PID 抗积分饱和(🔴 高)
- `algorithm/pid.h`:`PIDController` 可不动字段;`PID_Update` 实现条件积分——先算 `raw = kp*err + ki*integral + kd*deriv`,再 `out = Clamp(raw, min, max)`;仅当 `out == raw`(未饱和)或误差方向指向解除饱和时才 `integral += err*dt`(标准条件积分)。或采用 back-calculation:`integral += (out - raw) * back_gain`。选条件积分(实现简单、足够)。
- 验证:加一个 shell 临时命令 `pidtest` 跑阶跃(目标 100、测量 0、dt 0.01s、output_max 50),打印 200 步 integral/output,确认饱和期 integral 不无限增长、回退后能收敛。(命令可置于 `FEATURE_ENABLE_*` 开关后,默认关。)

### 1c. chassis 整数安全(🟡 低,顺手)
- `app/chassis.cpp`:`AbsInt32`(`:37-40`)对 `INT32_MIN` 是 UB;改为安全 abs 或调用前 clamp。`SetOneWheelTargetRpm`(`:263-270`)的 `uint16_t` 截断:已有 `max_wheel_rpm`(uint16)夹紧,补一条 `<= 65535` 显式断言注释。
- `app/chassis.cpp`:`kPiMicro` 用 `3141593U` 在 `WheelMmPerSecondToRpm` 里分子分母都乘了 `counts_per_rev`(`:77,81`)可约掉——确认数学正确(实际正确,只是冗余,保留即可,加注释)。

**验收**:`imu sample` 数值正确;`pidtest` 输出符合预期;chassis 速度命令不受影响(回归 `motor m1 speed`/`stop`)。

---

## Phase 2 — Fault 实化 + 初始化失败冒泡 + init 顺序

### 2a. init 顺序(让初始化失败可被记录)
- `empty_cpp.cpp:219-239`:把 `DebugUart_Init()` + `Log_Init()` 移到 `Board_Init()` **之前**。新顺序:`SYSCFG_DL_init → Fault_Init → Scheduler_Init → Time_Init → DebugUart_Init → Log_Init → Shell_Init → AppShell_RegisterCommands → Board_Init → App_Init → Board_LateInit`。(DebugUart 只依赖 SYSCFG_DL_init,已就绪。)
- 同步更新 `gugaPI/docs/PROJECT_GUIDE.md` §二启动流程,与代码一致(当前文档漏了 DebugUart/Shell/AppShell/Board_LateInit/Shell_Process)。

### 2b. Board_Init 失败冒泡
- `board/board.h`:`Board_Init` 改返回 `drivers::DriverStatus`;新增 `Board_GetInitReport()` 返回各子驱动 init 结果(结构体 + 名称)。
- `board/board.cpp:21-70`:去掉所有 `(void)`;收集每个 `Board_*Init` 结果;返回第一个非 OK;在 `Log_Init` 已就绪后用 `LOG_ERROR` 逐条打印失败项。
- `board/board_imu.cpp:140-144`:停止吞 `Icm45686_Init`;返回其状态(让 `Board_Init` 报告)。shell `imu init` 重试路径保留。

### 2c. Fault 实化
- `services/fault.h/.cpp`:
  - `Fault_Set` 改为**锁存第一个非 NONE 故障**(已存在则不覆盖),新增 `Fault_HasFault()`、`Fault_GetCount()`。
  - `Fault_Panic(code)`:置 fault → `LOG_ERROR("PANIC <code>")` → LED 闪烁循环(用 `board_led`)→ 可选复位。至少要有可观测信号,不再静默死循环。
  - RX 满丢字节路径(`debug_uart.cpp:29-31`)触发 `Fault_Set(FAULT_UART_OVERFLOW)`(限速:首次置位即可,避免风暴)。
- `app/app_main.cpp`:`App_Init` 捕获 chassis 任务的 `SchedulerTaskId`(从 `Scheduler_AddTask` 的 `out_id`),存全局。`App_Run` 检测 `Fault_Get() != NONE` 时:置 `APP_MODE_FAULT`、**一次性**调用 `Chassis_Stop()`、`Scheduler_EnableTask(chassis_id, false)` 抑制运动类任务(保留急停/通信)。这样 fault 真正会停车。

**验收**:拔掉/模拟一个 I2C 设备缺失,串口 boot 打印该 init 失败;`fault` 命令显示锁存故障;制造 RX 溢出(短时狂发)后 `FAULT_UART_OVERFLOW` 置位;fault 后底盘不再接受速度命令直到 `Fault_Clear`。

---

## Phase 3 — 底盘状态反馈周期任务

**问题**:`Chassis_Update`(读实测 RPM/编码器进 `ChassisState`)只在 `app_shell.cpp:4194` 的 `motor rpm` 按需调用,**无周期任务**,反馈控制无从谈起。

**改动**:
- `app/app_main.cpp`:`App_Init` 里新增 `Chassis_Update` 周期任务(建议 20ms),`FEATURE_ENABLE_MOTOR_DRIVER` 门控;捕获 `SchedulerTaskId`。
- 新增 shell 命令 `chassis stat`:打印 target/actual rpm、encoder count、last_status、lease active、task max_late_ms。
- 保留 `motor rpm` 的 on-demand 调用(不冲突)。

**验收**:电机运行时 `chassis stat` 的 `actual_rpm` 持续刷新且与 `motor rpm` 一致;静止时为 0;任务 `max_late_ms` 不增长。

---

## Phase 4 — I2C2 互斥 + INA219 原子快照

### 4a. I2C2 协作锁
- `board/board_i2c_bus.h/.cpp` 或 `drivers/i2c_diag`:新增 per-controller 协作锁 `Board_I2cBus_Lock(inst, owner)`/`Unlock`,内部 `volatile bool busy` + owner 名 + 重入 assert。
- FRAM/INA219/OLED 入口加锁/解锁。当前因协作调度+无 ISR 触碰 I2C2 而安全,锁作为**tripwire**;在 `PROJECT_GUIDE` 与代码注释中明确**契约:I2C2 不得在中断上下文使用**(若未来要加 ISR 消费者,需改 `NVIC_DisableIRQ` 包裹——记为后续工作)。

### 4b. INA219 原子快照
- `drivers/ina219/ina219.cpp:419-461`:`Ina219_ReadRawRegisters` 改为**单次自增突发读**(从 0x00 一次读 6+ 寄存器),消除 N/N+1 转换混合。

**验收**:`i2c scan fram/ina219/oled` 正常;`ina219 read` 在电机运行(I2C2 与其它任务并发)时数值不跳变;`fram test` 通过。

---

## Phase 5 — MotorDriver 事务原子性 + UART 帧总超时

### 5a. 事务原子性
- `app/motor_driver_client.h`(或 Phase 6 后的 driver):
  - `SetSpeed`/`SetPositionCounts`(`:1247-1282, 1309-1339`):两事务(写目标 + 写模式)中若第二事务失败,对**该轮**执行回滚(恢复前一次 mode/target)或直接 `SetCoast` 该轮;返回明确失败让 `chassis.cpp` 的 `StopMotorsForSafety` 兜底(已存在,确认路径覆盖部分写)。
  - `Stop`(`:1436-1450`):两轮都尝试 coast,任一失败重试一次;返回合并状态(保留 `StopResult` 双轮明细,chassis 侧记录)。

### 5b. UART 帧总超时
- `ReceiveFrame`(`:393-437`):把按字节 `kResponseWaitIterations=5e6`(`:23`)改为**整帧墙钟超时**——用 `Time_Millis` 设 deadline(如 50ms),循环里检查超时;`ReadByteWait` 改为短轮询 + 总 deadline。消除"从机持续吐非帧头字节导致主控长时间挂起"的 hang 风险。(默认 I2C 模式不受影响,此路径仅 `motor bus uart` 触发。)
- 给 `kResponseWaitIterations`、`kI2cWriteVerifyDelayCycles`、`kI2cErrataDelayCycles`、`BOARD_GY931_I2C_*_CYCLES` 补时间推导注释,绑定 `BOARD_CPU_MHZ`。

**验收**:`motor bus uart` 模式下故意不接 MotorDriver,`motor ping` 在 ≤50ms 内返回 TIMEOUT 而非挂起;I2C 模式回归 `motor m1/m2 speed`/`hold`/`posrel`/`stop` 全通过,看门狗不超时。

---

## Phase 6 — 架构搬迁:协议下沉到 drivers/ + 拆分(检查点后执行)

**风险最高,触碰关键控制路径,单独立项评审。**

- 新建 `drivers/motor_driver/motor_driver_protocol.{h,cpp}`:把 `app/motor_driver_client.h` 里的 CRC8 组帧、寄存器表、字节序、写后校验、lease、`SetSpeed/SetCoast/SetBrake/SetPosition/Stop/Read*` 全部搬入 driver 层。`app/motor_driver_client` 降为薄封装(或直接由 `chassis.cpp` 调用 driver + ConfigStore)。
- 严格 `.h`(接口)/`.cpp`(实现)拆分,消除 1450 行 inline 头。**需要 CCS Clean Build 重新生成 makefile**(`PROJECT_GUIDE` §八)。
- 破除 `config/debug_config.h:5` ↔ `services/fault.h` 的循环 include:把 `APP_ASSERT` 移到 `services/assert.h` 或改为仅前向声明 `Fault_Panic` 不 include fault.h。
- 保留所有寄存器地址/常量与 MotorDriver README 一致;回归全量寄存器读写。

**验收**:Clean Build 零警告;全量 `motor` shell 命令回归(`info/status/reg/set/rpm/ramp/pid/pospid/posctl/enc/m1 m2 speed/hold/pos/posrel/stop`);`motor ping` I2C/UART 双模式;看门狗与写后校验行为不变。

---

## Phase 7 — 杂项清理

- `drivers/i2c_diag/i2c_diag.cpp:215-216`:删重复的 `DriveLineLow`。
- `drivers/lora/lora.cpp:209-229`:ISR `default` 分支清 overrun/framing/parity(对齐 `motor_driver_uart.cpp` 的 `RxClearMask`)。
- `board/board.cpp` init 顺序:INA219(配 I2C2 上拉)提到 FRAM 之前,或在 `Board_Init` 开头统一配置 I2C2 上拉一次,避免 FRAM 在无上拉时静默失败。
- `board/board_gy931.cpp:138-148` 等:移除 read 路径里的自动 init;要求显式 `gy931 init`;`SetAddress` 探测失败时回滚地址。
- `services/shell.cpp:209-234`:注册命令加重名保护(拒绝或替换);`:74` arg 截断到 8 时置溢出标志,让 handler 可感知。
- **不做(记为后续)**:`float` PID→定点 Q(仅在 gugaPI 侧出现实际控制环时再做;MotorDriver 速度 PID 在从机,不在本工程范围)。

**验收**:相关 shell 命令回归;I2C/OLED/LoRa/GY931 行为不变。

---

## 跨阶段验收基线(每阶段结束都跑)

1. 构建:`gmake -k -j 32 all -r -O` 零警告。
2. 烧录 `Debug/gugaPI.out` 到 MSPM0G3519。
3. 串口(115200 8N1)回归:`version`、`motor bus/ping/info/status`、`motor m1/m2 speed`+`stop`、`imu sample`、`gy931 angle`、`i2c scan motor/fram/oled`、`fram test`、OLED、按键、`txstat`/`chassis stat`/`fault`(按阶段新增)。
4. 电机命令后确认无看门狗超时、无 fault 锁存、`txstat dropped=0`。
5. 提交 `feature/chassis-control`,附串口日志与硬件版本。

## 风险与回退

- 每阶段一个提交,出问题可直接 `git revert` 单阶段。
- Phase 0 改 TX 路径:若实机出现乱码(FOC 第一版 TX 中断的教训),立即回退到 blocking,不改用 TX 中断,改排查 pump 节奏/FIFO 深度。
- Phase 2 改 init 顺序:若 boot 无串口输出,确认 `DebugUart_Init` 仍依赖且仅依赖 `SYSCFG_DL_init`。
- Phase 6 需要 Clean Build:若 makefile 未收录新 .cpp,确认执行了 Clean Build 而非增量。
