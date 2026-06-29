# gugaPI

这是用于自研 MSPM0G3519 单片机开发板的全国大学生电子设计大赛代码工程。

工程基于 TI CCS Theia、MSPM0 SDK、SysConfig、DriverLib 和 TI Clang。底层初始化仍由 SysConfig 生成，业务代码按比赛扩展需求划分为 `app`、`board`、`drivers`、`services`、`algorithm` 和 `config`。

## 当前目标

- 芯片：MSPM0G3519
- 工程类型：NoRTOS C++
- 调试器：XDS110
- SysConfig 文件：`empty_cpp.syscfg`
- 程序入口：`empty_cpp.cpp`

## 文档

项目框架、目录职责、任务调度方式和驱动代码要求见：

```text
docs/PROJECT_GUIDE.md
```

## 重要约定

不要手改 `Debug/ti_msp_dl_config.c` 和 `Debug/ti_msp_dl_config.h`。新增外设时先改 `empty_cpp.syscfg`，再在 `drivers/` 中写设备驱动。