# LIS3MDL 磁力计手册

## 启动

LIS3MDL 与 ICM45686 共用 `IMU_SPI`，使用独立片选。`Board_ImuInit()` 会独立调用
`Lis3mdl_Init()`；失败作为可降级故障记录，ICM45686 仍可工作。可用
`Board_Lis3mdlInit()` 单独重试。

初始化步骤：软复位、WHO_AM_I=`0x3D`、BDU、little-endian、四线 SPI、性能模式、
量程、ODR 和连续模式，并对配置位读回校验。

## 默认与枚举参数

- 量程 `0..3`：±4/±8/±12/±16 gauss。
- ODR `0..7`：0.625/1.25/2.5/5/10/20/40/80 Hz；默认 20 Hz（5）。
- 性能 `0..3`：低功耗/中/高/超高；默认 XY/Z 均为超高。
- 模式 `0..2`：连续/单次/掉电；默认连续。
- SPI 超时：100000 次轮询。

## API 与 Shell

```cpp
board::Board_Lis3mdlReadRaw(&raw);
drivers::Lis3mdl_RawToMilliGauss(raw.x,
                                 board::Board_Lis3mdlGetFullScale());
```

```text
imu lis status
imu lis init
imu lis whoami
imu lis sample
imu lis scale 0
imu lis odr 5
imu lis mode 0
imu lis reg 0x0f
```

改变参数只修改运行期寄存器，不写 ConfigStore，复位后恢复默认值。

## 总线约束与验证

访问前板级层会让两个片选均为高并把 SPI 恢复为 mode 3，避免 Shell 调试改变模式后破坏
传感器访问。验证 WHO_AM_I、三轴方向、量程比例、ODR、数据就绪、软复位和两个片选
不会同时为低；使用已知磁场或转台完成标定前，不应用它做绝对航向。

