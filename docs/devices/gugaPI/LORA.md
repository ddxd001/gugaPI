# LoRa UART 与帧协议手册

## 启动与兼容模式

板级 UART 参数为 115200 baud，RX 环形缓冲 256 字节，TX 等待上限 100000 次轮询。
`Board_Init()` 初始化 UART；`App_LoraProtocolInit()` 初始化协议状态但默认关闭。

协议关闭时，原有 `lora send|line|hex|read|clear|test` 原始字节流行为不变。执行
`lora proto on` 后，`App_Run()` 每轮最多消费 64 个接收字节并处理 ACK 超时；此时使用
`lora proto recv` 读取帧，不再使用原始 `lora read`。

## 帧格式

```text
A5 5A | version | type | flags | sequence | length_le16 | payload | crc_le16
```

- version：1。
- 最大 payload：128 字节。
- type `0x01` 为普通数据，`0x02` 保留为 ACK；应用发送 type 不能为 0 或 ACK。
- flags bit0 表示要求 ACK，其他位必须为 0。
- CRC：CRC-16/CCITT，多项式 `0x1021`，初值 `0xFFFF`，覆盖 version 到 payload。
- 接收队列深度：4 帧。

## ACK 和重试

默认 ACK 超时 500 ms，最多重试 3 次。同一时刻只允许一个等待 ACK 的发送；否则返回
`DRIVER_ERROR_BUSY`。重复的已确认数据帧不会再次入队，但会再次回复 ACK。

## Shell

```text
lora proto on
lora proto status
lora proto send 1 ack hello world
lora proto recv
lora proto reset
lora proto off
```

`status` 给出 CRC、格式、长度、重复、队列丢弃、重试、发送错误和意外 ACK 统计。

## API 与验证

应用可调用 `App_LoraProtocolSend()`、`App_LoraProtocolReadFrame()`；纯协议层可用
`LoraProtocol_ProcessByte()` 做桌面单元测试。验证正常帧、CRC 错误、拆包、粘包、重复、
队列满、ACK 丢失、序号回绕和原始/协议模式切换。

