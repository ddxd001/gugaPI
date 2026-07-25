# gugaPI 共享 I2C 控制器手册

## 用途与启动

`drivers/i2c_controller/` 统一主控侧 I2C 事务。总线硬件仍由 SysConfig 在
`SYSCFG_DL_init()` 中初始化；设备通过 `Board_I2cBusFind()` 取得配置，无需另建任务。

## 配置参数

`I2cControllerConfig` 包含总线名、`I2C_Regs*`、超时轮询次数以及 SCL/SDA 的 GPIO、
IOMUX 和复用功能。地址必须是未左移的 7 位地址 `0x08..0x77`；单次控制器长度上限
为 `0x0FFF` 字节，TX FIFO 为 8 字节。

## API

- `I2cController_Probe()`：区分地址 NACK 与其他错误。
- `I2cController_Write()`：支持前缀加数据，发送过程中持续填充 FIFO。
- `I2cController_Read()`：纯读。
- `I2cController_WriteRead()`：写寄存器地址后 repeated START 读取。
- `I2cController_GetBusStatus()`：读取控制器、SCL、SDA 状态。
- `I2cController_RecoverBus()`：切到 GPIO，最多输出 9 个 SCL 脉冲并生成 STOP。

原 `I2cDiagBusConfig` 和 `I2cDiag_*` 是兼容别名/适配函数，现有 Shell 调用不用改名。

## Shell

```text
i2c list
i2c status fram
i2c scan fram 0x08 0x77
i2c probe oled 0x3c
i2c recover ina219
```

FRAM、OLED、INA219 共用 SENSOR_I2C。当前只允许主循环上下文访问；ISR 不得调用
这些同步 API。所有等待有界，但一个事务执行期间仍会占用单线程主循环。

## 错误语义

超时返回 `DRIVER_ERROR_TIMEOUT`，地址/数据拒绝返回 `DRIVER_ERROR_NACK`，仲裁丢失
返回 `DRIVER_ERROR_BUSY`。失败路径发送 STOP（若需要）、清 FIFO 并复位传输状态。

## 验证

执行正常扫描、无设备地址探测、SDA 拉低恢复、长块写和 repeated START 读；用逻辑
分析仪确认地址、ACK/NACK、STOP 和 SCL 频率。

