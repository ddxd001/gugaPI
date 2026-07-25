# 三路按键与事件手册

## 硬件与启动

主控有三个低电平有效按键：Button1=`PC9`、Button2=`PB20`、Button3=`PB23`，
均由 SysConfig 配置内部上拉。`Board_ButtonsInit()` 在板级初始化阶段建立三个独立状态机，
应用中的 `buttons` 任务每 5 ms 调用一次 `Board_ButtonsUpdate()`。

本实现继续使用周期扫描，不依赖 GPIO 外部中断。这样长按计时和消抖都在任务上下文完成，
不会增加共享 `GROUP1_IRQHandler` 的处理复杂度，也不修改调度器。

## 参数与事件语义

- 扫描周期：5 ms，位于 `app/app_main.cpp`。
- 软件消抖：20 ms，`BOARD_BUTTON_DEBOUNCE_MS`。
- 长按门限：800 ms，`BOARD_BUTTON_LONG_PRESS_MS`。
- `PRESSED`：消抖后的按下沿，只产生一次。
- `RELEASED`：消抖后的松开沿，只产生一次。
- `SHORT_PRESSED`：未达到长按门限时，在松开沿产生。
- `LONG_PRESSED`：持续按下达到 800 ms 时立即产生一次；之后松开只产生
  `RELEASED`，不再产生短按。

四类事件分别锁存在位图中。读取某一类事件只清除该位，不会清除其他类型；如果同一类
事件在消费前连续发生多次，位图会合并为一次，所以应用应以不慢于正常任务周期及时消费。
驱动还保存最近一次 `Button_Update()` 新产生的事件快照，供调试日志读取；该快照不会
清除锁存位，也不会和比赛模式或动作序列争抢事件。

## API

旧接口继续可用：

```cpp
board::Board_ButtonIsPressed(id);
board::Board_ButtonWasPressed(id);
board::Board_ButtonWasReleased(id);
```

新增接口：

```cpp
board::Board_ButtonWasShortPressed(id);
board::Board_ButtonWasLongPressed(id);
board::Board_ButtonGetPressDurationMs(id);

uint32_t events = board::Board_ButtonTakeEvents(
    id,
    drivers::BUTTON_EVENT_ALL);
uint32_t pending = board::Board_ButtonPeekEvents(id);
uint32_t generated = board::Board_ButtonGetGeneratedEvents(id);
```

`TakeEvents()` 返回并清除掩码内的事件；`PeekEvents()` 只查看、不清除。多个模块需要消费
同一种事件时，应由一个应用级入口统一消费后再分发，不能让多个任务分别调用
`WasShortPressed()` 抢同一个锁存位。

开发配置启用 `FEATURE_ENABLE_BUTTON_EVENT_LOG`，每次事件产生时会异步输出到调试
Shell；比赛配置默认关闭，避免比赛运行时产生额外串口流量：

```text
button event button1 types=pressed held_ms=0
button event button1 types=released,short held_ms=126
button event button2 types=long held_ms=800
button event button2 types=released held_ms=936
```

## Shell 与验证

执行 `button` 可查看原始电平、消抖状态、按压时长和事件：

```text
button1 raw=released debounced=released held_ms=126 events=DUS-
```

`events` 四个字符依次为 `D`(按下)、`U`(松开)、`S`(短按)、`L`(长按)，`-` 表示
该事件没有待处理。该命令会取走并清除显示出的事件。

实物验证应覆盖：20 ms 附近的抖动不产生假事件；短按得到 `D/U/S`；按住 800 ms
得到 `D/L`，松开再得到 `U`；跨越 `uint32_t` 毫秒计数回卷后时长判定仍正常。
