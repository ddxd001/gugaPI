# gugaPI 中文图形化序列编辑器

在本目录运行 `start.bat`，或执行 `server.ps1` 后打开 `http://localhost:8080/`。推荐使用 Chromium 系浏览器，以便通过 Web Serial 连接 gugaPI Shell。

## 基本流程

1. 从左侧动作库拖入动作块，在右侧设置带单位和范围的中文参数。
2. 从动作块右侧端口拖到目标动作。绿色为完成路径，红色为失败/超时路径；红色路径连到“中止”时编译为 `ontimeout=abort`。
3. 先用“试运行当前画布”下发 RAM 表。编辑器依次执行 `run clear/add/validate/dump/start`，回读不一致时不会启动，也不会写 FRAM。
4. 确认流程后用“保存到 FRAM”。覆盖前会确认，保存后通过 `seq dump` 逐项比对。
5. 发生异常时使用页面顶栏始终可见的“紧急停止”。它会依次取消序列、比赛、航向、循迹和底盘控制。

运动测试前必须架空车轮并清理机械运动范围。真实设备验收建议临时使用槽位 7，并在结束后恢复其原内容。

## 工程 JSON v1

工程文件使用 `format: "gugapi-sequence-project"`、`version: 1`，保存工程名、绑定槽位、语义参数、节点坐标及端口连线。浏览器会自动保存到 `localStorage`，也可导入或导出 JSON。

图形坐标不会写入 MCU。编译器从“开始”节点按稳定广度优先顺序生成最多 64 条指令，并把所有连线转换成明确索引；FRAM 仍使用 SeqStore v1 的 10 字节指令和 8 个槽位。

## 本阶段动作

支持固件现有 14 种 `run add` 动作：`drive`、`drive_mm`、`turn`、`follow`、`wait`、`stop`、`branch`、`end`、三种 LED 动作和三种蜂鸣器动作。任意 Shell 命令不会作为动作块执行；后续功能应通过新增固件 `ActionOp` 扩展，以保持脱机 FRAM 运行能力。
