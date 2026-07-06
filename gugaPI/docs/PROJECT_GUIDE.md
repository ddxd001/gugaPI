# gugaPI 项目框架与驱动规范

本文档用于约束这块自研 MSPM0G3519 开发板的比赛代码结构。目标是：新硬件能快速接入，比赛题目逻辑不和底层驱动混在一起，现场调试时能快速定位问题。

## 一、总体分层

```text
gugaPI/
  app/          比赛题目相关逻辑
  board/        自研开发板的硬件定义和板级初始化
  drivers/      独立设备驱动
  services/     时间、调度、日志、错误处理等系统服务
  algorithm/    PID、滤波、控制算法等纯逻辑
  config/       功能开关和调试配置
  docs/         项目文档
```

分层依赖方向固定为：

```text
app -> services / algorithm / drivers -> board -> SysConfig / DriverLib
```

禁止反向依赖。尤其是 `drivers/` 不应该包含 `app/` 的头文件。

## 二、启动流程

入口在 `empty_cpp.cpp`：

```text
SYSCFG_DL_init()
Fault_Init()
Scheduler_Init()
Time_Init()
Board_Init()
Log_Init()
App_Init()
while (1) {
    Scheduler_Run()
    App_Run()
}
```

`SYSCFG_DL_init()` 由 SysConfig 生成，负责时钟、电源、引脚复用和外设基础初始化。不要手改 `Debug/ti_msp_dl_config.c` 或 `Debug/ti_msp_dl_config.h`。

## 三、目录职责

### app/

只放比赛题目逻辑，例如状态机、任务组合、策略控制、模式切换。`app` 可以调用驱动和算法，但不要直接写寄存器。

### board/

描述“这块板子有什么”：

- 引脚归属
- 外设实例归属
- 板级初始化
- 电源使能、默认电平、硬件版本差异

驱动不能私自写死引脚。引脚和总线信息应从 `board/` 通过配置结构传入驱动。

### drivers/

每个硬件设备一个独立目录。推荐结构：

```text
drivers/oled/
  oled.h
  oled.cpp
```

驱动只解决设备本身的问题，例如 OLED 显示、编码器计数、IMU 读取、电机 PWM 输出。

### services/

系统通用能力：

- `time`：1 ms 系统时间戳和延时
- `scheduler`：无 RTOS 协作式任务调度
- `debug_uart`：UART3 调试串口底层收发，内部维护 RX 中断环形缓冲区
- `log`：日志等级接口，提供 `LOG_INFO`、`LOG_WARN`、`LOG_ERROR`、`LOG_DEBUG`
- `shell`：调试命令行，主循环中调用 `Shell_Process()` 解析命令
- `fault`：统一错误码和严重错误处理

### algorithm/

放不依赖硬件的算法，例如 PID、滤波、路径规划、姿态解算。算法层不应该包含 `ti_msp_dl_config.h`。

### config/

放功能开关和调试开关。比赛现场需要快速裁剪功能时，优先改这里。

## 四、任务调度规范

当前使用 1 ms SysTick 和协作式调度器，不引入 RTOS。

推荐任务周期：

```text
1 ms    按键扫描、编码器采样
5 ms    电机闭环、快速控制
10 ms   传感器读取、姿态更新
50 ms   屏幕刷新
100 ms  串口遥测、状态上报
```

任务函数要求：

- 不要长时间阻塞。
- 不要在任务里死等外设。
- 慢外设必须有超时。
- 任务之间通过状态结构或环形缓冲区通信。
- 中断里只做清标志、读写缓冲区、置位 flag。

## 五、驱动代码要求

### 1. 文件结构

每个驱动至少包含：

```text
xxx.h       公开接口、配置结构、状态结构
xxx.cpp     私有实现、寄存器操作、协议细节
```

复杂驱动可以继续拆分：

```text
xxx_protocol.cpp
xxx_registers.h
xxx_port.cpp
```

### 2. 接口命名

推荐统一使用：

```text
XXX_Init()
XXX_Deinit()
XXX_Update()
XXX_Read()
XXX_Write()
XXX_IsReady()
XXX_GetStatus()
```

初始化必须返回 `DriverStatus`，不要只返回 `void`。

### 3. 配置方式

驱动不得写死板级资源。推荐：

```cpp
struct XxxConfig {
    /* GPIO port, pin, timer instance, I2C instance, timeout, etc. */
};

struct XxxContext {
    const XxxConfig *config;
    bool initialized;
};
```

由 `board/` 或 `app/` 创建配置，再传给 `XXX_Init()`。

### 4. 错误和超时

所有访问外设的驱动必须考虑：

- 参数为空
- 未初始化
- 总线忙
- 设备无响应
- 超时
- 数据校验失败

统一使用 `drivers/common/driver_status.h` 中的 `DriverStatus`。严重错误再上报到 `services::Fault_Set()`。

### 5. 中断要求

中断函数必须短：

- 清中断标志
- 读/写硬件 FIFO
- 放入 ring buffer
- 设置状态标志

禁止在中断里做复杂算法、格式化打印、长延时、等待 I2C/SPI/UART 完成。

### 6. 阻塞要求

驱动允许提供阻塞式接口，但必须有超时参数或内部超时。推荐同时提供非阻塞 `Update()` 风格接口，方便接入调度器。

### 7. 日志要求

驱动内部不要大量打印。调试信息通过 `services/log` 输出，并受 `FEATURE_ENABLE_LOG` 控制。
日志宏只接受已经整理好的字符串，不在中断中格式化输出：

```cpp
LOG_INFO("system ready");
LOG_WARN("battery low");
LOG_ERROR("sensor timeout");
LOG_DEBUG("control loop entered");
```

### 8. 调试串口和 Shell 要求

调试串口分三层维护：

```text
app/app_shell.cpp       注册板级和业务命令
services/shell.cpp      命令行输入、分词、命令分发
services/log.cpp        日志等级格式
services/debug_uart.cpp UART3 TX/RX、RX 中断环形缓冲
```

要求：
- UART 引脚、时钟、波特率、RX 中断由 `empty_cpp.syscfg` 配置。
- 应用层不直接调用 DriverLib，只调用 `services/debug_uart.h`、`services/log.h`、`services/shell.h`。
- UART RX 必须走中断，中断函数只读取 RX FIFO、写入环形缓冲区、清除中断标志。
- 命令解析必须在主循环的 `Shell_Process()` 中完成，不能放进中断。
- 新增 Shell 命令优先放到 `app/app_shell.cpp` 注册，命令函数里再调用板级接口或业务接口。

当前内置命令：
```text
help
version
reset
led on
led off
buzzer on
buzzer off
adc
pwm 50
```

`adc` 和 `pwm` 当前是占位命令，等 ADC/PWM 驱动接入后再替换为真实读写逻辑。

### 9. SysConfig 要求

新增 UART、I2C、SPI、PWM、ADC、GPIO 等外设时，优先在 `empty_cpp.syscfg` 中配置，让 SysConfig 生成初始化代码。自己的驱动只调用生成的宏和 DriverLib API。

不要手改这些生成文件：

```text
Debug/ti_msp_dl_config.c
Debug/ti_msp_dl_config.h
Debug/device_linker.cmd
Debug/device.opt
```

## 七、新增一个硬件设备的流程

1. 在 `empty_cpp.syscfg` 中添加外设和引脚。
2. 在 `board/board_pins.h` 或新的板级配置文件中登记资源归属。
3. 在 `drivers/<device>/` 中新增驱动。
4. 用 `Config` 结构把板级资源传入驱动。
5. 在 `app/App_Init()` 或专门的设备初始化函数中调用驱动初始化。
6. 如果需要周期运行，把 `XXX_Update()` 注册进 `Scheduler_AddTask()`。
7. 失败时返回 `DriverStatus`，必要时调用 `Fault_Set()`。

## 八、构建注意事项

CCS Theia 会根据工程源码重新生成 `Debug/` 下的 makefile。新增 `.cpp` 文件后，建议执行一次 Clean Build。

命令行环境中 `make` 当前不在 PATH，但本机存在：

```text
C:\ti\ccs2100\ccs\utils\bin\gmake.exe
```

不要为了让命令行构建通过而手工维护 `Debug/*.mk`，这些文件属于 CCS 生成产物。
## 九、当前板级资源

### 状态 LED

- 引脚：PB6
- 电平：低电平点亮，高电平熄灭
- SysConfig 分组：`GPIO_LEDS.STATUS_LED`
- 板级接口：`board/board_led.h`
- 驱动实现：`drivers/led/`
- 当前测试程序：`app` 中注册 `status_led_test` 状态机任务，500 ms 亮，500 ms 灭

PB6 的初始输出配置为 `SET`，所以上电初始化后 LED 默认熄灭。
### 有源蜂鸣器

- 引脚：PA12
- 电平：高电平响，低电平停
- SysConfig 分组：`GPIO_BUZZER.BUZZER`
- 板级接口：`board/board_buzzer.h`
- 驱动实现：`drivers/buzzer/`
- 当前测试程序：`app` 中注册 `buzzer_test` 状态机任务，响 200 ms，停 800 ms

PA12 的初始输出配置为 `CLEARED`，所以上电初始化后蜂鸣器默认不响。

### 调试串口 UART3

- 引脚：RX PA13，TX PA14
- 电平：TTL 串口电平
- 波特率：115200
- SysConfig 实例：`DEBUG_UART`
- 生成宏：`DEBUG_UART_INST`
- 底层接口：`services/debug_uart.h`
- 日志接口：`services/log.h`
- Shell 接口：`services/shell.h`
- 命令注册：`app/app_shell.cpp`
- 当前交互：主循环调用 `Shell_Process()`，支持 `help`、`version`、`reset`、`led on/off`、`buzzer on/off`、`adc`、`pwm 50`

调试串口由 `FEATURE_ENABLE_DEBUG_UART` 控制，日志由 `FEATURE_ENABLE_LOG` 控制，Shell 由 `FEATURE_ENABLE_SHELL` 控制。计数打印测试仍保留在 `FEATURE_ENABLE_UART_COUNTER_TEST` 后面，默认关闭，避免干扰 Shell 交互。

### FM24CL64B-GTR FRAM

- 器件：FM24CL64B-GTR，64 Kbit / 8 KiB I2C FRAM
- I2C 地址：默认 `0x50`，如果 A2/A1/A0 硬件电平不同，需要修改 `BOARD_FRAM_I2C_ADDRESS`
- 单片机连接：LQFP-100(PZ) 封装下，65 号脚为 SCL，对应 `PC2 / I2C2_SCL`；66 号脚为 SDA，对应 `PC3 / I2C2_SDA`
- 板级总线：IIC3，共享给 FRAM、INA219 和 OLED；当前 MCU 外设为 `I2C2`
- SysConfig 实例：`SENSOR_I2C`
- 生成宏：`SENSOR_I2C_INST`，FRAM 板级别名仍是 `BOARD_FRAM_I2C_INST`
- 板级接口：`board/board_fram.h`
- 驱动实现：`drivers/fm24cl64b/`
- 功能开关：`FEATURE_ENABLE_FRAM`

FRAM 驱动使用 DriverLib 操作共享的 IIC3 / MCU I2C2，应用层不要直接调用 DriverLib。驱动提供阻塞式 `Read/Write/ReadByte/WriteByte/SelfTest` 接口，每次总线等待都有超时，写入按 I2C FIFO 大小拆成小块。FM24CL64B 是 FRAM，不需要 EEPROM 那种写入后等待内部擦写完成的延时。

Shell 测试命令：

```text
fram test
fram read 0x0000 16
fram write 0x0000 0x5A
```

`fram status` 用于查看 I2C 控制器状态和 SCL/SDA 电平；正常空闲时 SCL/SDA 应为高电平。`fram test` 会在 `BOARD_FRAM_SELF_TEST_ADDRESS` 指定地址备份 8 字节原始数据，写入测试图样并读回校验，最后恢复原始数据。为了避免误操作，Shell 的 `fram read` 单次最多读取 32 字节。

### HS91L02W2C01 OLED

- 器件：Hansheng HS91L02W2C01，0.91 寸白色 OLED，128 x 32 dots
- I2C 地址：数据手册标注 `ADD:0x78`，这是 8-bit 写地址；固件使用 7-bit 地址 `0x3C`
- 单片机连接：和 FRAM、INA219 共用 IIC3，`PC2 / I2C2_SCL` 为 SCL，`PC3 / I2C2_SDA` 为 SDA
- SysConfig 实例：`SENSOR_I2C`
- 生成宏：`SENSOR_I2C_INST`，OLED 板级别名为 `BOARD_OLED_I2C_INST`
- 板级接口：`board/board_oled.h`
- 驱动实现：`drivers/oled/`
- 功能开关：`FEATURE_ENABLE_OLED`

OLED 驱动按 SSD1306 风格 I2C 控制协议实现。命令使用控制字节 `0x00`，显存数据使用控制字节 `0x40`；初始化序列使用 SSD1306 128x32 兼容配置，并启用 charge pump，适配 3.3V IIC 模块。当前第一版不维护全局帧缓存，测试命令直接写整屏页数据，也支持 5x7 ASCII 字符串按页显示。

Shell 测试命令：

```text
i2c scan oled 0x3C 0x3C
oled status
oled init
oled test
oled clear
```
