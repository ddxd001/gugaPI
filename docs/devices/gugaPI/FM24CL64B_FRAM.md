# FM24CL64B FRAM 手册

## 启动

`Board_FramInit()` 先执行无损字节读来确认 0x50 应答。若首次读失败，会通过共享
I²C 控制器发送恢复时钟并恢复引脚复用，然后自动重试一次初始化；这用于处理上电时
`SCL=H、SDA=L` 的总线占用状态，不会擦除或改写 FRAM 内容。

启用 `FEATURE_ENABLE_FRAM` 后，`Board_Init()` 调用 `Board_FramInit()`。初始化会在
地址 `0x50` 做一次不改写数据的单字节读取；未收到响应时不会标记 ready。

## 参数

- 容量：8192 字节。
- 7 位地址：`0x50`。
- 超时：100000 次轮询。
- 自检保留地址：`0x1FF0`，长度 8 字节。
- 驱动连续读写分块：256 字节。底层控制器会在事务进行时补充 8 字节 FIFO，FIFO
  深度不是事务长度上限；当前 195 字节 ConfigStore 镜像可在一个事务内完成。

FM24CL64B 数据手册说明内部地址会在每个字节后自动递增、没有页缓冲，单次连续读写
没有器件侧字节数限制；驱动保留256字节上限，是为了限制同步轮询占用时间并避免跨越
`0x1FFF` 后地址回卷。

## API

```cpp
board::Board_FramRead(address, data, length);
board::Board_FramWrite(address, data, length);
board::Board_FramReadByte(address, &value);
board::Board_FramWriteByte(address, value);
board::Board_FramSelfTest();
board::Board_FramRecoverBus();
```

读写范围必须完全位于 `0..8191`。长度为 0 的调用不传输数据。自检会保存原 8 字节、
写入模式、读回比较并恢复；不要把 `0x1FF0..0x1FF7` 用作应用数据。

## ConfigStore

配置从 FRAM `0x0000` 开始，格式为 magic、版本、负载和 CRC32。当前 v11 负载为
183 字节；兼容读取 v1(66)、v2(68)、v3(90)、v4(103)、v5(137)、v6(158)、
v7(162)、v8(177)、v9(181)和v10(183)字节负载。旧版本缺失的字段采用默认值并标记
dirty，用户执行 `param save` 后迁移到v11。v10新增循迹差速修正斜率参数，默认25000
permille/s；v11保持相同负载布局，把旧版默认循迹掩码`0x3C`迁移为`0x7E`，其它
用户自定义掩码保持不变。

## 故障与验证

连续写也不是原子操作。掉电可能留下部分新数据，ConfigStore 依靠版本、长度和 CRC
拒绝损坏镜像。验证应覆盖首尾地址、跨 256 字节边界、自检恢复、断线超时和总线恢复。

SENSOR_I2C 当前工作在 400 kHz。FRAM 连续读写由同步轮询完成；DMA 不会提高总线
时钟，因此参数调试优先采用 RAM 镜像、批量 Shell 读取和一次性 `param save`。只有在
运行状态需要持续传输大块数据并确认同步等待导致任务超期后，才考虑异步 I²C/DMA。
