# MSPM0G3507 + DRV8323RS + AS5048B + TCAN3413 电机控制工程交接文档（最终版）

## 1. 文档信息

| 项目 | 内容 |
| --- | --- |
| 文档版本 | Final v1.0 |
| 交接日期 | 2026-07-16 |
| 工程目录 | `D:\M0\empty_LP_MSPM0G3507_nortos_ticlang` |
| MCU | MSPM0G3507，VQFN-32（RHB） |
| 栅极驱动器 | DRV8323RS |
| 编码器 | AS5048B-HTSP-500，14 位绝对式磁编码器 |
| CAN 收发器 | TCAN3413DDFR，3.3 V CAN FD 收发器 |
| 验证状态 | 电流环、速度环、单圈位置环已完成空载验证；500 kbit/s CAN 已完成实机双向收发验证 |
| 当前固件 | [Debug/empty_LP_MSPM0G3507_nortos_ticlang.out](./Debug/empty_LP_MSPM0G3507_nortos_ticlang.out) |
| 版本管理 | 当前目录尚未初始化 Git 仓库，接手后应优先建立版本基线 |

本文档描述的是当前可运行调试版本。电机额定参数、母线条件和负载条件尚未完整记录，因此当前电流限制和 PI 参数属于保守调试参数，不应直接视为量产参数。

## 2. 当前实现概览

控制链如下：

```text
AS5048B机械角度
       │
       ▼
100 Hz单圈位置环（POSITION模式）
       │ 目标速度，限幅±50 RPM
       ▼
100 Hz速度PI
       │ Iq参考，限幅约±250 mA
       ▼
20 kHz d/q电流PI
       │ d/q电压
       ▼
反Park变换 + SVPWM
       │
DRV8323RS三路20 kHz中心对齐PWM
```

当前功能：

- DRV8323RS SPI 初始化、配置回读和故障清除。
- 3x PWM 模式，三路中心对齐 20 kHz PWM。
- 三相低侧分流电流同步采样，Clarke/Park 变换和 d/q 电流闭环。
- AS5048B I2C 角度、磁场强度、AGC 和诊断状态读取。
- 上电后不自动转动；启动时执行 500 ms 转子对齐并计算电角度偏置。
- 速度闭环，支持正转、反转、主动制动和 25 RPM 步进。
- 单圈绝对位置闭环，支持最短路径和 ±10° 步进。
- nFAULT、编码器丢失和软件过流保护。
- UART 115200 8N1 调试控制台。
- PA26/PA27 经典 CAN 500 kbit/s 调试口，支持测试帧发送、标准帧接收和总线状态诊断。

## 3. 工程文件说明

| 文件 | 作用 |
| --- | --- |
| [empty.c](./empty.c) | 主程序、UART 命令、CAN 调试收发、AS5048B I2C、ADC 校准/统计、故障报告和中断入口 |
| [motor_open_loop.c](./motor_open_loop.c) | FOC、电流环、速度环、位置环、SVPWM 和保护逻辑 |
| [motor_open_loop.h](./motor_open_loop.h) | 电机控制接口、状态结构和命令步进参数 |
| [drv8323rs.c](./drv8323rs.c) | DRV8323RS SPI 寄存器配置、状态读取和 ENABLE/CAL 控制 |
| [drv8323rs.h](./drv8323rs.h) | DRV8323RS 公共接口 |
| [empty.syscfg](./empty.syscfg) | MCU、GPIO、SPI、I2C、UART、ADC、TIMG0、CANFD0 和 CANCLK 配置源文件 |
| [README.md](./README.md) | 工程基础说明和命令速查 |
| [targetConfigs/MSPM0G3507.ccxml](./targetConfigs/MSPM0G3507.ccxml) | CCS 调试目标配置 |
| `Debug/` | CCS Debug 构建产物和生成文件，不应作为主要源代码修改位置 |

注意：`motor_open_loop.c/.h` 是历史文件名。文件内部已经实现编码器 FOC，不再是开环六步换相。保留文件名是为了避免破坏 CCS managed build 的文件枚举。

## 4. 开发环境与构建

已验证环境：

| 工具 | 当前版本或路径 |
| --- | --- |
| Code Composer Studio | `F:\CCS\ccs` |
| TI Arm Clang | `ti-cgt-armllvm_5.1.1.LTS` |
| MSPM0 SDK | `C:\TI\mspm0_sdk_2_10_00_04` |
| SysConfig | 1.26.2+4477 |
| CPU 时钟 | 32 MHz |
| CANCLK | 80 MHz，由内部 32 MHz SYSOSC 经 SYSPLLCLK1 产生；不改变 CPU/MCLK |
| 构建配置 | Debug |

PowerShell 命令行构建：

```powershell
Set-Location D:\M0\empty_LP_MSPM0G3507_nortos_ticlang\Debug
& 'F:\CCS\ccs\utils\bin\gmake.exe' -k -j 32 all -r -O
```

成功后输出：

```text
D:\M0\empty_LP_MSPM0G3507_nortos_ticlang\Debug\empty_LP_MSPM0G3507_nortos_ticlang.out
```

最后一次交接前构建已经通过 `-Wall` 编译和链接检查，无错误、无警告；加入 CAN 和 PA23 STB 控制后输出文件大小为 478692 字节。链接映射显示 Flash 使用约 40.9 KiB、SRAM 静态使用约 832 B。

烧录可在 CCS 中使用 [targetConfigs/MSPM0G3507.ccxml](./targetConfigs/MSPM0G3507.ccxml) 连接目标板，然后加载上述 `.out` 文件。

## 5. 硬件连接

### 5.1 DRV8323RS 控制与通信

| MCU 引脚 | DRV8323RS 信号 | 当前用途 |
| --- | --- | --- |
| PA22 | INHA | TIMA0 C1，A 相高侧控制 PWM |
| PA3 | INHB | TIMA0 C2，B 相高侧控制 PWM |
| PA12 | INHC | TIMA0 C3，C 相高侧控制 PWM |
| PA4 | INLA | A 相半桥使能 |
| PA11 | INLB | B 相半桥使能 |
| PA18 | INLC | C 相半桥使能 |
| PA10 | ENABLE | DRV8323RS 使能 |
| PA21 | CAL | 电流采样放大器零点校准 |
| PA5 | nFAULT | 低有效故障输入，下降沿中断，内部上拉作为后备 |
| PA17 | nSCS | SPI 片选，GPIO 控制 |
| PA6 | SCLK | SPI0 时钟 |
| PA14 | SDI | SPI0 MOSI/PICO |
| PA13 | SDO | SPI0 MISO/POCI |

SPI 参数：1 MHz、16 位、CPOL=0、CPHA=1，片选由 PA17 GPIO 手动控制。SysConfig 中的 `frameFormat=MOTO3` 是三线数据/时钟格式名称，不应误解为常说的 SPI Mode 3。

### 5.2 电流采样

| MCU 引脚 | ADC | DRV8323RS 信号 | 相位 |
| --- | --- | --- | --- |
| PA24 | ADC0 CH3 | SOA | A 相 |
| PA25 | ADC0 CH2 | SOB | B 相 |
| PA15 | ADC1 CH0 | SOC | C 相 |

电流硬件参数：

- 三相采样电阻：20 mΩ。
- DRV8323RS CSA 增益：20 V/V。
- 双向采样零点：VREF/2。
- ADC 参考：VDDA，代码按标称 3.3 V 换算。
- 1 ADC count 约等于 2.015 mA。
- 本板已验证极性：`phase_current = ADC - calibrated_offset`。
- ADC0/ADC1 使用 4 次硬件平均。
- TIMG0 通过事件通道 14 以 20 kHz 同步触发两个 ADC，并由固件与 TIMA0 PWM 安静区相位对齐。

不要随意把 ADC 触发源改成 TIMA0 直接发布。此前直接修改后曾出现校准样本数为 0；当前 TIMG0 → 事件通道 14 → ADC 的路径已验证可靠。

### 5.3 AS5048B 编码器

| MCU 引脚 | 编码器信号 | 说明 |
| --- | --- | --- |
| PA0 | SDA | I2C0 SDA，外接约 4.7 kΩ 上拉到 3.3 V |
| PA1 | SCL | I2C0 SCL，外接约 4.7 kΩ 上拉到 3.3 V |

编码器参数：

- I2C 速率：400 kHz。
- 7 位地址：`0x40`，对应 A1=A2=GND。
- 角度分辨率：14 位，0～16383 对应 0～360°。
- 完整诊断读取寄存器：`0xFA～0xFF`。
- 运行期间快速角度读取寄存器：`0xFE/0xFF`。
- 当前电机极对数在代码中固定为 14。

### 5.4 UART 与调试

| MCU 引脚 | 功能 |
| --- | --- |
| PA8 | UART1 TX |
| PA9 | UART1 RX |
| PA20 | SWCLK |
| PA19 | SWDIO |

串口参数：115200 baud、8 数据位、无校验、1 停止位。

### 5.5 TCAN3413 CAN 调试口

| MCU/收发器引脚 | 连接 | 说明 |
| --- | --- | --- |
| PA26 / CAN_TX | TCAN3413 pin 1 TXD | MCU 发送到收发器 |
| PA27 / CAN_RX | TCAN3413 pin 4 RXD | 收发器发送到 MCU |
| TCAN3413 pin 2 GND | 数字地/功率地公共参考 | 必须与 MCU 和 CAN 分析仪共地 |
| TCAN3413 pin 3 VCC | 3.3 V | 收发器总线侧电源，建议就近 100 nF 去耦 |
| TCAN3413 pin 5 VIO | MCU 3.3 V | I/O 电平电源，建议就近去耦 |
| TCAN3413 pin 6 CANL | CANL | 接双绞线低线 |
| TCAN3413 pin 7 CANH | CANH | 接双绞线高线 |
| PA23 | TCAN3413 pin 8 STB | 固件上电输出低电平进入 Normal 模式；高电平为 Standby |

CAN 当前参数：

- CANFD0，经典 CAN，11 位标准帧，500 kbit/s，不启用 CAN FD/BRS。
- CANCLK 80 MHz，标称采样点 80%。
- 接收过滤器接受全部 `0x000～0x7FF` 标准数据帧，放入 8 项 FIFO0；扩展帧和远程帧被拒绝。
- 一个专用发送缓冲区；调试发送 ID 固定为 `0x321`、DLC=8。
- CANFD0 中断优先级为 2，低于 ADC 电流环优先级 1；中断中只记录事件，FIFO 读取和串口打印均在主循环执行。
- PA23 在 GPIO 初始化和 CAN 软件初始化中均被明确拉低；`n` 命令会显示 `PA23=STB(NORMAL)`。
- 总线两端各需要一只 120 Ω 终端电阻。只有本板位于物理总线端点时才在本板装 120 Ω；断电测 CANH-CANL，双端终端并联时应约为 60 Ω。

`t` 命令发送的数据定义：

| 字节 | 内容 |
| --- | --- |
| 0 | 固定帧头 `0xA5` |
| 1 | 发送序号，0～255 循环 |
| 2 | FOC 状态：0=STOP、1=ALIGN、2=RUN、3=FAULT |
| 3 | 控制模式：0=SPEED、1=POSITION |
| 4～5 | 目标 RPM，`int16` 小端 |
| 6～7 | 实测 RPM，`int16` 小端 |

## 6. DRV8323RS 当前配置

当前 [drv8323rs.c](./drv8323rs.c) 中的初始化值：

| 寄存器 | 数值 | 作用 |
| --- | --- | --- |
| DRIVER_CONTROL | `0x0020` | 3x PWM 模式 |
| GATE_DRIVE_HS | `0x0322` | 约 60 mA source / 120 mA sink，1 µs TDRIVE |
| GATE_DRIVE_LS | `0x0122` | 低侧栅极驱动设置 |
| OCP_CONTROL | `0x0119` | 锁存 OCP、附加 100 ns dead time、4 µs deglitch、0.75 V VDS 阈值 |
| CSA_CONTROL | `0x0283` | VREF/2 双向采样、20 V/V、1 V sense-OCP 阈值 |

初始化会写入以上寄存器、清除故障并回读校验。`nFAULT=1` 且回读一致时，串口显示：

```text
Driver ready; motor is stopped.
FAULT1=0x0000 FAULT2=0x0000 nFAULT=1
```

## 7. 控制参数

### 7.1 PWM 和电流环

| 参数 | 当前值 |
| --- | --- |
| PWM 方式 | 三相中心对齐 SVPWM |
| PWM/电流环频率 | 20 kHz |
| TIMA0 计数周期 | 1600 counts，LOAD=800 |
| 最大相电压调制增量 | ±200 PWM counts |
| 电流 PI Kp | 384/256 = 1.5 PWM count / ADC count |
| 电流 PI Ki | 1/256 每采样周期 |
| d 轴电流参考 | 0 |
| q 轴参考限制 | ±124 ADC counts，约 ±250 mA |
| 软件相电流硬保护 | ±298 ADC counts，约 ±600 mA |
| 过流确认 | 连续 3 个 20 kHz 样本 |

### 7.2 速度环

| 参数 | 当前值 |
| --- | --- |
| 更新频率 | 100 Hz |
| 默认目标 | +25 RPM |
| 串口步进 | 25 RPM |
| 目标范围 | ±400 RPM |
| 比例项 | `speed_error`，输出为电流 ADC counts |
| 积分格式 | Q6 |
| 抗积分饱和 | 已实现 |
| 降速/反转 | 会立即释放方向错误的旧积分并输出制动电流 |

### 7.3 位置环

| 参数 | 当前值 |
| --- | --- |
| 更新频率 | 100 Hz |
| 类型 | 单圈绝对位置 P 外环 |
| 路径 | ±180° 范围内最短路径 |
| 串口步进 | ±10° |
| 位置误差到速度 | 约每 10 encoder counts 产生 1 RPM |
| 最大目标速度 | ±50 RPM |
| 目标近区 | 128 counts，约 2.81° |
| 近区控制 | 位置误差 P + 速度阻尼，q 轴参考限幅约 ±64 mA |

远离目标时位置环生成速度目标，之后经过速度 PI 和 d/q 电流 PI；进入目标近区后改用限流位置 PD 电流参考，避免低速积分往返。当前位置保持需要的转矩仍受 q 轴限幅约束。

### 7.4 对齐和保护

| 项目 | 当前值或行为 |
| --- | --- |
| 转子对齐 | 固定 α 轴小电压矢量，持续 500 ms |
| 电角度偏置 | 每次启动后根据 AS5048B 绝对机械角度自动计算 |
| 编码器采样 | 1 kHz 协作式非阻塞 I2C 状态机，20 kHz 电流环在样本间预测电角度 |
| 编码器超时 | 超过 5 ms 无有效更新，三相桥进入 Hi-Z；单次预测最多外推 2 ms |
| nFAULT | GPIO 下降沿中断立即关闭三相桥 |
| 软件停止 | `x` 后三相 INL 拉低，电机滑行 |

UART TX 已改为 1 KiB 非阻塞队列，AS5048B 快速角度读取不再等待 I2C 完成。
COM4 实测 1 kHz 采样在 25 RPM 运行时零错误、零超时，最大间隔 2.2 ms。

## 8. 串口命令

| 命令 | SPEED 模式 | POSITION 模式或通用行为 |
| --- | --- | --- |
| `s` | 对齐并启动速度 FOC | 已运行时忽略 |
| `p` | 停止时会自动对齐启动，然后捕获当前位置 | 重新捕获当前位置作为保持目标 |
| `v` | 选择速度模式，目标设为 0 RPM | 退出位置模式并进入 0 RPM 速度模式 |
| `+` | 速度幅值增加 25 RPM | 目标位置增加 10° |
| `-` | 速度幅值减少 25 RPM | 目标位置减少 10° |
| `r` | 反转速度目标；0 RPM 时设置为 -25 RPM | 不执行，提示使用位置步进 |
| `x` | 停止并滑行 | 停止并滑行 |
| `f` | 输出控制状态和 DRV8323RS 故障 | 同左，并增加位置、目标和误差 |
| `i` | 输出三相 ADC、电流平均值和峰值 | 同左 |
| `a` | 输出 AS5048B 角度、幅值、AGC 和诊断 | 同左 |
| `z` | 仅停止状态下重新校准三相电流零点 | 同左 |
| `c` | 停止、清除驱动器故障和软件停止原因 | 同左 |
| `t` | 发送一帧 ID `0x321` 的 CAN 电机状态测试帧 | 同左 |
| `n` | 输出 CAN 控制器状态、错误计数和最后收到的一帧 | 同左 |
| `h` / `?` | 打印帮助 | 同左 |

位置状态示例：

```text
FOC state=RUN mode=POSITION pos=237.3deg target_pos=237.4deg err=+0.0deg
target_rpm=+0 speed_rpm=+0 Iq_ref=-6mA Id/Iq=-4/-2mA
PWM(A/B/C)=50.0/49.5/50.5% enc=10804 stop=none
```

## 9. 已完成验证

### 9.1 电流采样

- 上电 CAL 自动零点校准成功。
- 停止状态三相零点约位于 2060～2073 ADC counts，具体值随通道和温度变化。
- 开环阶段已经观察到随相位切换的正负电流。
- FOC 运行时 d/q 电流方向已经验证为 `ADC-offset`。

### 9.2 速度闭环

空载测试已经完成：

- +25 RPM：稳定约 +25 RPM。
- +50 RPM：稳定约 +51 RPM。
- +75 RPM：稳定约 +74 RPM。
- +100 RPM：稳定约 +99 RPM。
- 从 +100 降到 +75 RPM 时，`Iq_ref` 能立即变为负值进行主动制动。
- 从 +75 反转到 -75 RPM 时，立即得到负 q 轴电流参考并成功反转。
- 速度 PI 已修复旧版本减速时正积分滞留的问题。
- 整数 IIR 已修复静止时锁在 `+5/-5 RPM` 的余差问题，电角速度预测也能回零。
- 最新空载回归中，+75 RPM 稳态为 74～76 RPM，反转后稳定为 -73～-76 RPM。
- 速度比例项提高到 1 current-count/RPM 后，25 RPM 瞬时纹波由约 3～47 RPM
  收敛到约 20～31 RPM，末值 25 RPM。

已短时运行到约 200 RPM，但修复抗积分饱和后完整的稳态验证主要完成到 100 RPM。高转速、带载和热态参数仍需重新测试。

### 9.3 位置闭环

最新空载单圈位置阶跃测试：

1. `p` 对齐后捕获当前位置。
2. `+` 增加 10° 后，稳态误差约 0.4～0.7°。
3. `-` 返回原目标后，稳态误差约 0.5～0.7°。
4. 静止速度反馈可回到 0 RPM，仅偶发 ±1 RPM；全程 `stop=none`。

这是空载静态结果，不代表所有负载、温度和母线条件下的精度保证。

### 9.4 CAN 构建与实机验证

- SysConfig 已确认 PA26=`CANFD0_CANTX`、PA27=`CANFD0_CANRX`。
- PA23 已配置为 TCAN3413 STB 输出，上电和运行期间保持低电平 Normal 模式。
- CPU/MCLK 仍为 32 MHz；SYSPLLCLK1/CANCLK 为 80 MHz。
- 生成位时序为 500 kbit/s，经典 CAN、采样点约 80%。
- CAN 收发、FIFO、中断和串口诊断代码已经通过 TI Arm Clang 5.1.1 LTS 的 `-Wall` 编译与链接。

PCAN 端实测配置：

| 项目 | 实测配置 |
| --- | --- |
| 控制器模式 | CAN (SJA1000) |
| 适配器时钟 | 8 MHz |
| 波特率 | 500 kbit/s |
| 位时序 | BTR0=`0x40`、BTR1=`0x2B` |
| 采样点 | 约 81%，接近 MCU 端约 80% |
| 帧类型 | Classic CAN、11 位标准帧 |
| 接收过滤 | `0x000～0x7FF` |

PCAN → MCU 接收验证：

```text
发送：STD ID=0x123 DLC=8 data=11 22 33 44 55 66 77 88
结果：rx/lost=1/0 TEC/REC=0/0 LEC=0 irq=0x00000001
CAN RX last: STD ID=0x00000123 DLC=8
data=0x11 0x22 0x33 0x44 0x55 0x66 0x77 0x88
```

MCU → PCAN 发送验证：

```text
tCAN TX requested: STD ID=0x321 DLC=8 data=A5 0x00 state/mode/rpm.
tx_req/done=1/1 pending=0 TEC/REC=0/0 LEC=0
BO/EP/EW=0/0/0 irq=0x00000200
```

其中 `irq=0x00000001` 对应 FIFO0 新消息，`irq=0x00000200` 对应发送完成。实测已证明 PA26 TX、PA27 RX、PA23 STB、TCAN3413 物理接口、500 kbit/s 位时序、标准帧过滤、ACK 和中断处理均正常。长线、高总线负载、温度、电压和 EMC 条件仍需产品阶段验证。

## 10. 推荐复测流程

### 10.1 上电检查

1. 电机保持空载，母线电源设置保守限流。
2. 打开 115200 8N1 串口后复位。
3. 确认：

```text
Driver ready; motor is stopped.
Current ADC ready.
FAULT1=0x0000 FAULT2=0x0000 nFAULT=1
ENC ... OCF=1 COF=0 strong=0 weak=0 valid=1
```

如果 `Current ADC ready` 未出现，不要启动电机。

### 10.2 速度模式复测

1. 发送 `s`，观察 500 ms 对齐；每次启动都会恢复默认 +25 RPM。
2. 在 25 RPM 等待稳定后发送 `f`。
3. 依次发送 `+`，测试 50、75、100 RPM；命令间留出处理时间，每级稳定后发送 `f`。
4. 100 RPM 时发送 `-`，确认目标 75 RPM 且 `Iq_ref` 立即为负。
5. 稳定到 75 RPM 后发送 `r`，确认目标 -75 RPM、`Iq_ref` 立即为负且最终速度为 -75 RPM。
6. 发送 `x` 停止。

### 10.3 位置模式复测

1. 停止状态发送 `p`。
2. 电机会先对齐，然后进入当前位置保持。
3. 确认 `mode=POSITION`、`pos≈target_pos`、`stop=none`。
4. 发送一次 `+`，应正向移动约 10°并停止。
5. 等待 2～3 秒后发送 `f`，建议误差保持在 ±0.3° 内。
6. 发送一次 `-`，应返回初始保持位置。
7. 轻微偏转转轴几度后松开，应返回目标位置。
8. 发送 `x` 停止。

### 10.4 CAN 复测流程

1. 复测时先保持电机停止，只给 MCU、TCAN3413 和 CAN 分析仪上电。
2. 确认 TCAN3413 `VCC=3.3 V`、`VIO=3.3 V`，并测量 PA23/TCAN3413 STB 应为 0 V；确认所有设备共地。
3. CANH/CANL 使用双绞线；确认总线两端终端正确，断电测 CANH-CANL 约 60 Ω。
4. CAN 分析仪配置为 Classic CAN、500 kbit/s、11-bit ID；PCAN/SJA1000 可使用 BTR0=`0x40`、BTR1=`0x2B`、约 81% 采样点；不要启用 CAN FD 或 Silent/Listen-only。
5. 复位 MCU，串口应多出：

```text
Boot: CAN initialized (500 kbps classic, PA26 TX, PA27 RX, PA23 STB=LOW).
```

6. 串口发送 `n`。空闲健康状态应包含 `op=NORMAL`、`BO/EP/EW=0/0/0`、`pending=0`。
7. 串口发送 `t`，CAN 分析仪应收到标准帧 `ID=0x321 DLC=8`，首字节为 `A5`。
8. 再发送 `n`，应看到 `tx_req/done=1/1`、`TEC/REC=0/0`。若 `done` 不增加且 `pending=1`，说明没有节点 ACK，优先检查分析仪是否处于只听/静默模式、波特率、STB、CANH/CANL、共地和终端。
9. 从 CAN 分析仪发送任意 11 位标准数据帧，例如 `ID=0x123 DLC=8 data=11 22 33 44 55 66 77 88`。
10. 串口发送 `n`，应看到 `rx` 增加，并显示 `CAN RX last: STD ID=0x00000123 ...`。
11. 收发均正常后再启动 FOC，分别在 STOP、SPEED 和 POSITION 状态发送 `t`，核对帧中状态、模式、目标转速和实测转速。

初次带载测试不要直接堵转。出现持续摆动、敲击、啸叫、过流或角度误差发散时立即发送 `x` 并保存完整串口日志。

## 11. 常见问题排查

### 11.1 上电串口没有初始化信息

- 检查 PA8/PA9、115200 8N1 和串口工具端口。
- 检查代码是否停在外设初始化或驱动器 SPI 等待。
- 当前正确启动顺序应至少打印三条 Boot 信息，然后打印控制台帮助。

### 11.2 Current ADC calibration failed / samples=0

- 检查 TIMG0 是否以 50 µs 周期运行。
- 检查 SysConfig 事件发布通道和两个 ADC 的订阅通道均为 14。
- 检查 ADC0 MEM1 中断是否启用。
- 保留当前 `MOTOR_FOC_enableAdcTrigger()` 中的 TIMG0 与 TIMA0 相位同步逻辑。
- 不要在电机运行时发送 `z`。

### 11.3 编码器读取失败或 valid=0

- 检查 PA0/PA1 上拉、电源和共地。
- 检查 A1/A2 地址选择是否对应 `0x40`。
- 正常诊断应为 `OCF=1 COF=0 strong=0 weak=0 valid=1`。
- `strong=1` 或 `weak=1` 时检查磁铁距离、偏心、磁铁规格和安装方向。

### 11.4 启动立即 overcurrent

- 确认三相电流极性仍为 `ADC-offset`，不要改回 `offset-ADC`。
- 核对 SOA/SOB/SOC 和 A/B/C 相序。
- 核对编码器正方向、14 极对数和电角度偏置。
- 检查启动前电流零点是否有效。

### 11.5 电机抖动但不能连续转动

- 核对 PA22/PA3/PA12 是否分别输出三路不同相位的 20 kHz PWM。
- 核对 PA4/PA11/PA18 在运行时为高、停止时为低。
- 核对 DRV8323RS 的 PWM_MODE 回读值是否为 `0x0020`。
- 检查电机相序和编码器方向是否匹配。
- 确认 `MOTOR_POLE_PAIRS=14` 与实际电机一致。

### 11.6 位置模式来回摆动或发出敲击声

- 立即发送 `x`。
- 先检查速度模式 25～100 RPM 是否仍稳定。
- 检查对齐时转子是否可靠吸到固定位置。
- 增大 `POSITION_COUNTS_PER_RPM` 可降低位置环增益。
- 增大 `POSITION_DEADBAND_COUNTS` 可降低静止抖动。
- 不要先提高电流限制掩盖机械、相序或角度问题。

### 11.7 CAN 发送一直 pending 或进入 Bus-Off

- CAN 发送必须由另一个处于正常模式的节点 ACK；只有本板一台设备、或 CAN 分析仪处于 Silent/Listen-only 时，发送会不断重试。
- 发送 `n` 查看 `tx_req/done`、`pending`、`TEC/REC`、`LEC` 和 `BO/EP/EW`。
- 检查 PA23/TCAN3413 pin 8 STB 是否确实为低、VCC/VIO 是否为 3.3 V。
- 检查 PA26→TXD、PA27←RXD 是否接反，检查 CANH/CANL、共地和 120 Ω 终端。
- 确认所有节点都是 500 kbit/s Classic CAN；当前固件不接收 CAN FD 帧。
- Bus-Off 后最简单的恢复方法是先修复总线，再复位 MCU。

### 11.8 CAN 能发送但收不到

- 当前仅接收 11 位标准数据帧；扩展 ID 和远程帧会被硬件过滤。
- 发送 `n` 时检查 `rx/lost`；`lost` 增加表示主循环没有及时清空 8 项 FIFO。
- 电机运行期间不要让 CAN 分析仪连续满总线发送调试帧，以免主循环的编码器读取被大量 CAN 服务占用。

## 12. 已知限制与风险

- 当前是单圈位置控制；没有多圈计数和掉电位置圈数保存。
- 位置命令目前只有相对 ±10°，没有串口数字角度解析。
- 没有速度/位置梯形或 S 曲线加减速规划，位置目标改变后直接由位置 P 环限速到 50 RPM。
- 每次 FOC 启动都必须对齐，转子会被吸到一个电角度固定位置；位置模式捕获的是对齐后的角度，不是发送 `p` 前的原始机械角度。
- 电角度偏置不会写入 Flash。
- 没有电机参数辨识、前馈解耦、弱磁、母线电压补偿和温度补偿。
- 当前电流环、速度环和位置环参数主要根据空载台架调通，尚未覆盖额定负载、堵转、低母线、高母线和热态。
- DRV8323RS 栅极驱动和 VDS OCP 参数是保守 bring-up 配置，需要依据实际 MOSFET Qg、母线电压和 PCB 波形复核。
- UART TX 使用非阻塞队列；`u` 查看排队和丢弃计数。AS5048B 快速读取使用
  1 kHz 状态机；`e` 查看采样统计，`j` 查看 ADC ISR 执行时间。
- 当前 CAN 仅用于手动调试，未实现固定周期遥测、命令解析、节点 ID、应用层协议或 CANopen。
- 当前 CAN 时钟由内部 SYSOSC 经 PLL 生成；进入长线、高噪声或严格量产容差场景前，应复核整个温度/电压范围的时钟容差和总线误码率。
- TCAN3413 STB 已连接 PA23，但当前固件固定输出低电平，仅使用 Normal 模式，尚未提供待机切换命令和唤醒策略。
- 当前工程没有 Git 历史，无法通过 commit 追溯修改。

## 13. 接手后优先事项

1. 初始化 Git 仓库并提交当前可运行基线，同时保存当前 `.out`、串口日志和硬件版本号。
2. 补录电机型号、极对数确认方法、相电阻、相电感、Kv、额定/峰值电流和允许温升。
3. 补录母线标称/最大电压、电源限流值、MOSFET 型号、采样电阻精度和 PCB 版本。
4. 在 25、50、100、200 RPM 下完成空载与分级负载的 `f`/`i` 数据表。
5. 做位置阶跃响应测试，记录超调、稳定时间、静态误差和负载扰动恢复。
6. 根据实测电机参数重新设计电流 PI、速度 PI 和位置 P，而不是仅凭听感调参。
7. 将 UART 改为非阻塞或 DMA；确定 CAN 应用层协议，并把遥测改为固定周期、可解析且带版本号的帧格式。
8. 根据产品需求增加任意角度命令、多圈位置、轨迹规划、限位和掉电策略。
9. 完成过流、编码器断线、nFAULT、母线异常和堵转保护测试。
10. 完成 CAN 温度/电压/线长/负载率测试，并根据系统架构决定是否增加 STB 待机控制命令、CAN FD、节点 ID 和 Bus-Off 自动恢复。

## 14. 尚待交接双方确认的信息

以下信息在当前工程文件和调试记录中没有完整记录：

- 电机准确型号、额定电压、额定电流、Kv、相电阻和相电感。
- 实际母线电压、启动和测试时的电源限流值。
- 功率 MOSFET 型号、栅极电阻、母线电容和 PCB 版本。
- 编码器磁铁型号、直径、气隙、同轴度和机械安装公差。
- 最终负载惯量、摩擦、传动比、运动范围和允许定位误差。
- 是否需要多圈、机械限位、原点开关、断电保持或安全制动。
- 最终 EMC、热设计和保护等级要求。

这些信息会直接影响 PI 参数、电流限制、栅极驱动强度、OCP 阈值和位置控制性能，进入产品化阶段前必须补齐。

## 15. 交接结论

当前版本已经完成从硬件驱动、三相电流采样、AS5048B 角度读取到编码器 FOC、速度闭环和单圈位置闭环的完整链路，空载调试结果正常；基于 TCAN3413 的 PA26/PA27 经典 CAN 调试接口也已完成 500 kbit/s 实机双向收发、ACK、过滤和中断验证。接手人应先固化当前版本并复现第 10 节测试，再开展带载整定、CAN 长线与高负载验证、轨迹规划、多圈位置和产品级保护工作。
