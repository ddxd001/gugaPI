# gugaPI 启动自检手册

## 启动方法

`empty_cpp.cpp` 先调用 `board::Board_Init()`，再初始化时间、故障、调度和应用层。
无需改变原入口。`Board_Init()` 仍返回首个失败状态，同时把每一项结果保存到
`BoardInitReport`。

## 状态级别

- `BOARD_INIT_OPTIONAL`：可选功能失败，不改变核心能力。
- `BOARD_INIT_DEGRADED`：系统可继续运行，但相关显示、存储或辅助传感器不可用。
- `BOARD_INIT_MOTION_INHIBIT`：运动所需设备不可信，启动代码置故障并禁止正常运动。

当前 ICM45686、INA219、灰度和主控侧运动通信入口属于禁止运动级；LED、蜂鸣器、
按键、FRAM、OLED、GY931、LIS3MDL、LoRa 属于可降级级。LIS3MDL 与 ICM45686
分别记录，磁力计失败不会伪装成主惯导失败。

## 使用方法

```cpp
const board::BoardInitReport *report = board::Board_GetInitReport();
for (uint8_t i = 0; i < report->count; ++i) {
    const board::BoardInitEntry &entry = report->entries[i];
    // entry.name, entry.status, entry.severity
}
```

启动日志会列出失败设备及错误码。应用层另外检查 ConfigStore 和底盘初始化结果。

## 兼容性

- `Board_Init()` 签名不变。
- `App_Init()` 仍为 `void`。
- 不修改任务注册、周期或调度器实现。
- 初始化失败不会被强制转换为成功。

## 验证

分别断开 OLED、FRAM、LIS3MDL、ICM45686、INA219，检查报告名称、错误码和级别；
确认可降级设备不阻止诊断，禁止运动设备不会进入可运动状态。

