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
- 共享层每块最多写 6 字节（2 字节地址 + 6 字节数据填满 FIFO），读分块 32 字节。

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

配置从 FRAM `0x0000` 开始，格式为 magic、版本、负载和 CRC32。当前 v6 负载为
158 字节；可读取 v1(66)、v2(68)、v3(90)、v4(103)、v5(137) 字节负载。旧版本
的新字段采用默认值，并在后续保存时迁移到 v6。

## 故障与验证

多块写不是原子事务。掉电可能留下部分新数据，ConfigStore 依靠版本、长度和 CRC
拒绝损坏镜像。验证应覆盖首尾地址、跨 6/32 字节边界、自检恢复、断线超时和总线恢复。
