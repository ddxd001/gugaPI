# gugaPI 电赛小车固件

[![Host Tests](https://github.com/ddxd001/gugaPI/actions/workflows/host-tests.yml/badge.svg)](https://github.com/ddxd001/gugaPI/actions/workflows/host-tests.yml)

本仓库是一个面向全国大学生电子设计竞赛智能小车的 TI Code Composer
Studio（CCS）Theia 工作区，目标芯片为 MSPM0 Arm Cortex-M0+。

仓库不仅包含小车上位控制器 `gugaPI`，还包含独立的双路有刷电机驱动器
`MotorDriver`、无刷 FOC 实验固件 `FOC`，以及用于临时验证的 `TEST` 工程。

## 系统组成

```text
┌──────────────────────────────────────────────────────────────┐
│ PC 上位机                                                   │
│ 串口终端 / 比赛序列编辑器 / 参数树 / FRAM 参数管理          │
└─────────────────────────────┬────────────────────────────────┘
                              │ 115200 8N1
                              ▼
┌──────────────────────────────────────────────────────────────┐
│ gugaPI · MSPM0G3519                                         │
│ 比赛状态机、任务序列、底盘控制、循迹、IMU 航向、CAN、LoRa   │
└───────────────┬───────────────────────────────┬──────────────┘
                │ I2C / UART                    │ CAN
                ▼                               ▼
┌──────────────────────────────┐  ┌────────────────────────────┐
│ MotorDriver · MSPM0G3519     │  │ TCAN3413 / JY-ME02        │
│ 双路有刷电机闭环与看门狗     │  │ CAN 总线与编码器设备      │
└───────────────┬──────────────┘  └────────────────────────────┘
                │
                ▼
        左右轮电机与 QEI 编码器

FOC · MSPM0G3507 是独立的无刷电机实验工程，不属于上述底盘运行链路。
```

## 子工程

| 工程 | MCU | 语言 | 作用 |
| --- | --- | --- | --- |
| `gugaPI` | MSPM0G3519 | C++ | 小车主控制器、比赛逻辑、传感器与上位机接口 |
| `MotorDriver` | MSPM0G3519 | C++ | 双路 DRV8701 有刷电机驱动，从属于 gugaPI |
| `FOC` | MSPM0G3507 | C | DRV8323RS + AS5048B 的编码器闭环 FOC 实验固件 |
| `TEST` | MSPM0G3519 | C | 最小工程和临时验证代码，不属于正式部署系统 |

四个工程彼此独立，没有顶层统一构建系统。每个工程均由 CCS managed make
和各自的 `.syscfg`、`.cproject`、`.ccsproject` 管理。

## 主要功能

### gugaPI

- 比赛模式、调试模式和安全停止状态管理。
- 通过按键选择比赛子任务，通过序列存储执行比赛动作。
- 灰度循迹、路口识别、连续转向和循迹交接。
- ICM-45686 IMU、航向保持、定角转向和零偏估计。
- 速度、距离、位置及左右轮控制接口。
- FRAM 配置和比赛序列持久化。
- 调试串口 Shell，支持状态查询、参数修改和设备诊断。
- 上位机串口终端、序列编辑器和 QGroundControl 风格参数树。
- TCAN3413 CAN 驱动及 JY-ME02 CAN 编码器支持。
- LoRa 通信代码和比赛节点协议，由功能配置决定是否启用。
- LED、蜂鸣器、按钮、OLED、INA219、GY931 等模块化驱动；部分设备可在
  功能配置中关闭。

### MotorDriver

- 双路 DRV8701 PH/EN 有刷电机驱动。
- 速度、位置和占空比控制。
- 硬件 QEI 编码器计数。
- UART/I2C 寄存器协议。
- 通信看门狗：失联时强制双轮滑行停止。

### FOC

- AS5048B 磁编码器反馈。
- 100 Hz 位置环和速度 PI。
- 20 kHz d/q 电流环、反 Park 变换和 SVPWM。
- 三相电流采样、转子对齐和电角度标定。
- UART、CAN 和 I2C 非阻塞调试路径。
- 上电默认停止，不会自动启动电机。

## 分支与当前主线配置

默认分支为 `master`，稳定功能通过 PR 合入。

`master` 当前使用外置 `MotorDriver` 作为双轮底盘后端：

- 调试 Shell：UART6，PC11 TX、PC10 RX，115200 8N1。
- MotorDriver：默认通过 PA11/PA10 的专用 I2C1 总线控制，也支持
  PA8/PA9 UART 调试。
- LoRa：UART3，PA14 TX、PA13 RX，115200 8N1。
- CAN：PC26 TX、PC27 RX、PC25 STB，经典 CAN 250 kbit/s。
- CAN/JY-ME02、IMU、灰度、FRAM、按键和状态指示功能启用。
- INA219、OLED、GY931 和板载电机后端暂时关闭。

板载 DRV8876 + MR 电机迁移在
`feature/onboard-mr-motor-control` 分支继续开发。该分支尚未合入
`master`，不能把它的引脚、功能开关和台架限制当作主线配置。

引脚和功能开关可能随硬件阶段变化。以
[`gugaPI/HARDWARE_INTERFACE.md`](gugaPI/HARDWARE_INTERFACE.md)、
[`gugaPI/empty_cpp.syscfg`](gugaPI/empty_cpp.syscfg) 和
[`gugaPI/config/feature_config.h`](gugaPI/config/feature_config.h) 为准。

## 目录结构

```text
.
├── FOC/                    # 独立无刷 FOC 工程
├── MotorDriver/            # 双路有刷电机驱动工程
├── gugaPI/
│   ├── algorithm/          # 与硬件无关的 PID、滤波和数学算法
│   ├── app/                # 比赛状态机、底盘、循迹、序列和应用逻辑
│   ├── board/              # 引脚、外设资源所有权和板级初始化
│   ├── config/             # 开发和比赛功能配置
│   ├── drivers/            # 设备驱动
│   ├── services/           # 时间、调度、日志、Shell、故障服务
│   ├── tools/seq_editor/   # 上位机图形界面
│   ├── docs/               # gugaPI 调试和验收文档
│   └── empty_cpp.syscfg    # gugaPI SysConfig 源文件
├── host_tests/             # 可在 PC 上运行的算法和目录回归测试
├── docs/                   # 外设资料和历史设计文档
└── .github/workflows/      # GitHub Actions
```

## 开发环境

- TI Code Composer Studio Theia 21.0
- TI Arm Clang `TICLANG_5.1.1.LTS`
- MSPM0 SDK 2.10
- SysConfig 1.26
- XDS110 调试器
- 主机测试：支持 C++17 的 `g++` 和 Node.js 22

代码使用 `-O2 -Wall`，所有正式构建必须达到：

```text
0 errors, 0 warnings
```

## 构建

推荐在 CCS Theia 中执行 Clean Build。命令行构建示例：

```powershell
cd gugaPI\Debug
C:\ti\ccs2100\ccs\utils\bin\gmake.exe -k -j 32 all -r -O
```

其他工程将目录替换为 `FOC\Debug` 或 `MotorDriver\Debug`。

新增 `.cpp` 或 `.c` 文件后必须执行 Clean Build，使 CCS 重新生成 managed
make 文件。不要直接编辑 `Debug/` 中的 makefile 或生成代码。

## 主机回归测试

在 Linux、WSL 或 Git Bash 中执行：

```bash
bash host_tests/run_tests.sh
```

测试包括配置迁移、灰度处理、航向数学、IMU 零偏、循迹交接、路口控制和
上位机 Shell 命令目录。

主机测试不能替代硬件台架验证。涉及外设、电机、控制参数或时序的改动仍需
完成对应工程文档规定的烧录和串口回归。

## 烧录与串口

使用各工程 `targetConfigs/*.ccxml` 和 XDS110 烧录。

| 工程 | 调试串口 | 协议 |
| --- | --- | --- |
| `gugaPI` 主线 | UART6 PC11/PC10 | 115200 8N1，行式 Shell |
| `MotorDriver` | UART4 PB17/PB18 | 115200 8N1，二进制寄存器帧 |
| `FOC` | UART1 PA8/PA9 | 115200 8N1，单字符命令 |

详细命令参见
[`gugaPI/docs/SHELL_COMMANDS.md`](gugaPI/docs/SHELL_COMMANDS.md)、
[`MotorDriver/README.md`](MotorDriver/README.md) 和
[`FOC/README.md`](FOC/README.md)。

## gugaPI 架构规则

依赖方向固定为：

```text
app → services / algorithm / drivers → board → SysConfig / DriverLib
```

- `app/` 只处理业务、策略和状态机，不直接操作寄存器。
- `board/` 拥有引脚和外设资源，并通过 `Config` 结构传给驱动。
- `drivers/` 不硬编码板级引脚；初始化必须返回 `DriverStatus`。
- `services/` 提供公共运行机制，应用层不得绕过服务重复实现。
- `algorithm/` 必须保持硬件无关，不包含 `ti_msp_dl_config.h`。
- 严重驱动故障通过 `services::Fault_Set()` 上报。

新增设备的一般顺序：

```text
.syscfg
  → board/board_pins.h
  → drivers/<device>/
  → board 初始化
  → app 调用
  → scheduler 周期任务（如需要）
```

## 调度与实时性规则

gugaPI 使用 1 ms SysTick 和协作式调度器，不使用 RTOS。

- 周期任务不得阻塞或忙等外设。
- 慢速总线必须设置超时。
- UART 发送必须非阻塞。
- ISR 只清中断、搬运 FIFO、写入环形缓冲区或设置标志。
- ISR 中禁止格式化日志、长延时和等待 I2C/SPI/UART 完成。
- Shell 解析只能在主循环中执行。
- 电机通信失败时必须进入安全停止。

## SysConfig 和生成文件

外设和引脚变更必须修改对应 `.syscfg`，再由 SysConfig 重新生成代码。

禁止手动修改或提交以下生成内容：

```text
Debug/ti_msp_dl_config.c
Debug/ti_msp_dl_config.h
Debug/device.opt
Debug/device_linker.cmd
Debug/ 和 Release/ 下的 managed make 文件
```

工作区 `.theia/launch.json` 含机器相关绝对路径，不应提交本机修改。

## Git 与 PR 规则

`master` 是受保护分支：

- 新功能从最新 `master` 创建 `feature/<name>`。
- 修复建议使用 `fix/<name>`。
- 禁止直接推送、强推或删除 `master`。
- 所有主线改动通过 PR。
- PR 必须通过 `host-tests`，且合并前同步最新 `master`。
- PR 讨论必须全部解决。
- 当前单人维护模式不强制人工审批。
- `master` 和所有 `feature/**` 推送都会运行 CI。

建议每个提交只包含一个可说明、可验证的开发阶段：

```text
feat(gugapi): add ...
fix(motordriver): handle ...
test: cover ...
docs: update ...
```

每个硬件阶段应记录：

- 使用的 PCB/模块版本。
- 固件提交号。
- 构建结果。
- 烧录方式。
- 关键串口日志。
- 台架连接和限流条件。
- 通过项、失败项和未验证项。

## 电机调试安全

- 首次运行必须架空车轮并使用限流电源。
- 上电、复位和烧录后保持电机停止。
- 从低占空比、低转速和保守电流限制开始。
- 调试前确认左右轮映射、编码器方向和急停命令。
- 出现振荡、过流、角度发散或通信异常时立即停止。
- 先保存完整串口日志，再修改控制参数。
- 不得为了消除报错而绕过电机通信看门狗或安全停止逻辑。

## 重要设计约束

- FOC 上电永不自动启动；每次启动先执行转子对齐。
- FOC 相电流极性是 `ADC_count - offset`，不得按其他 EVM 经验反转。
- 不得改变 FOC 的 TIMG0 同步 ADC 触发链。
- MotorDriver 左轮为 M2、右轮为 M1，输出轴均为 364 counts/rev。
- MotorDriver 看门狗默认 1 秒，读寄存器和 I2C `motor ping` 不刷新看门狗。
- GY931 使用 PA29/PA30 软件 I2C，修改前必须检查总线冲突。
- 板载 MR 电机功能仍在独立功能分支开发，不属于当前 `master` 部署路径。

完整约束以 [`AGENTS.md`](AGENTS.md) 为准。

## 文档索引

| 内容 | 文档 |
| --- | --- |
| gugaPI 架构 | [`gugaPI/docs/PROJECT_GUIDE.md`](gugaPI/docs/PROJECT_GUIDE.md) |
| gugaPI 硬件接口 | [`gugaPI/HARDWARE_INTERFACE.md`](gugaPI/HARDWARE_INTERFACE.md) |
| Shell 命令 | [`gugaPI/docs/SHELL_COMMANDS.md`](gugaPI/docs/SHELL_COMMANDS.md) |
| 底盘控制计划 | [`gugaPI/docs/CHASSIS_CONTROL_PLAN.md`](gugaPI/docs/CHASSIS_CONTROL_PLAN.md) |
| CAN 编码器测试 | [`gugaPI/docs/JYME02_CAN_TEST.md`](gugaPI/docs/JYME02_CAN_TEST.md) |
| MotorDriver | [`MotorDriver/README.md`](MotorDriver/README.md) |
| FOC | [`FOC/README.md`](FOC/README.md) |
| 协作和安全规则 | [`AGENTS.md`](AGENTS.md) |
