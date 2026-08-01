# gugaH：H题专用固件

`gugaH` 是面向 2026 电赛 H 题“车载平衡滚球运动控制系统”的独立
MSPM0G3519 工程。它不链接 `gugaPI`，只保留 H2–H6 所需的固定状态机、
控制算法和设备协议。

## 工程约束

- MSPM0 SDK `2.10.00.04`
- SysConfig `1.26.2`
- TI Arm Clang `5.1.1.LTS`
- `-O2 -Wall`，编译和链接零警告
- 1 ms SysTick、固定周期协作调度，无 RTOS、无动态任务注册
- 上电默认两轮停止；设备初始化完成前不向 DM 下发位置目标

分层依赖固定为：

```text
app -> control -> drivers -> board -> SysConfig / DriverLib
```

核心公开接口包括 `HApp_Start/Abort/Update`、
`Course_Start/Update`、`Ball_StartHold/Move/Stop/Update`、
`Chassis_SetWheelRpm/Stop/GetFeedback` 和
`HConfig_Load/Save/Defaults`。

## 硬件资源

| 设备 | 外设与引脚 |
| --- | --- |
| 8 路灰度 | ADC1 PA15；选通 PA16、PC20、PC21 |
| MotorDriver | I2C1 PA10/PA11，7 位地址 `0x20` |
| DM-G6220 | CANFD1 PC26/PC27；TCAN3413 STB PC25 |
| MaixCAM | UART6 RX PC10，115200 8N1 |
| ICM-45686 | SPI PB17/PB18/PB19；CS PC7 |
| OLED/FRAM | I2C2 PC2/PC3 |
| 按键 | B1 PC9、B2 PB20、B3 PB23 |
| 调试串口 | UART0 RX PB1、TX PB0，115200 8N1 |

MotorDriver 物理映射保持为左轮=M2、右轮=M1。循迹采用完整灰度帧驱动：
每 1 ms 扫描一路，8 路扫描完成后依次执行一次灰度处理、一次巡线 PD 和
一次左右轮速度下发，完整控制周期约 8 ms（约 125 Hz）。不会对同一帧
重复计算或重复下发速度；非循迹状态下仍按独立周期执行停车和安全监控。
灰度板当前左右反向安装：电气通道 0 在车体右侧、通道 7 在左侧；
代码已在位置映射层反转通道方向，各通道白黑标定值保持不变。
H5/H6按编码器里程使用相同的分段巡线：直道采用低增益、低修正限幅和强D项
滤波，弯道保留现有灵敏参数；物理弯道前后150 mm内连续插值，避免参数
切换造成轮速阶跃。两题也共用H5的缓启动、直弯速度切换、最终缓减速以及
`lap_distance + 50 mm`里程完成判据；H6只把滚球目标换成B2/B3设定位置。
H2仍使用全局 `line_*` 参数，不受分段参数影响。

保守默认配置已吸收 2026-07-30 原 `gugaPI` 参数导出中可直接兼容的实车值：
八路灰度黑白标定和判定参数、`33.050 mm` 有效轮径、左右轮 `1456 CPR`，
以及 MotorDriver 的反向标志、速度 PID/斜坡和位置 PID。旧循迹增益因控制
公式不同未直接复制，航向、红外、INA219 和通用动作参数不属于本工程。

## 构建与验证

首次加入源码后在 CCS Theia 对 `gugaH` 执行一次 **Clean Build**，
让托管构建自动生成 `Debug/` 规则。不要手工修改 `Debug/` 中的文件。

不依赖 `Debug/` 的完整目标验证：

```powershell
powershell -ExecutionPolicy Bypass -File gugaH/tools/verify_target_build.ps1
```

主机算法/状态机测试：

```bash
bash host_tests/run_gugah_tests.sh
```

## 操作

- B1 短按：循环 H2–H6、CAL ZERO、CAL H2 LOOP、CAL GRAY；长按达到判定时间后，松开按键才启动。
- H6 READY 下，B2/B3 每次调整 1 mm，长按连续调整，范围 ±100 mm；
  READY 杆球控制会立即跟随设定值，发车前便把球移动并保持在目标位置。
- CAL ZERO 启动后持续保持球在当前零位；B2 向负半轴、B3 向正半轴
  每次移动零位 1 mm，长按连续移动，范围 ±30 mm。按下 B1 退出并自动写入 FRAM。
- CAL H2 LOOP 启动后不驱动车轮；B2 减少、B3 增加 H2 停车距离，每次
  10 mm，长按连续调整，偏移范围 ±200 mm。短按 B1 退出并自动写入 FRAM。
- CAL GRAY 启动后不驱动车轮；将全部探头放在白色底面短按 B2，再放在
  黑色标定面短按 B3。短按 B1 退出并自动写入 FRAM；无效采样不会覆盖
  上一组有效标定值。
- H2–H6 RUNNING 时任意按键立即中止并停车。
- 结果保持显示；B1 短按确认返回 READY。
- H2、H5、H6 在灰度仍正常出帧但丢失轨迹时，保持当前平均速度沿右前弧线
  搜索；重新找到线后自动恢复 PD 循迹。灰度停止出帧、标定故障或传感器
  硬故障仍会安全停车。
- H2 不再识别 A 宽线；停车目标为标称一圈里程加 `h2_loop_offset_mm`，默认
  `6142-100=6042 mm`，按左右轮平均编码器里程判断。前段保持巡航速度，剩余200 mm时继续巡线并将
  速度从55 RPM逐渐降至15 RPM；达到目标立即停车并PASS，且H2不设置总超时。

调试串口输入 `help` 查看精简命令，详见
[`docs/SHELL.md`](docs/SHELL.md)，台架顺序见
[`docs/BENCH_TEST.md`](docs/BENCH_TEST.md)。

MaixCAM 端不属于本工程。`gugaH` 仅在 UART6/PC10 预留接收通道，
接收 115200 8N1、50 Hz、固定 12 字节、CRC16-Modbus 的球位置帧。
