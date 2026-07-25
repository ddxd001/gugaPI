# ADC/PWM Shell 命令边界

旧的通用 `adc`、`pwm` 命令没有绑定明确硬件，只打印“未注册”或“请求值”，容易让人
误以为已经控制外设，因此已删除。

现有 ADC1/PA15 属于八路灰度采样，请使用：

```text
gray status
gray read 0
gray data
gray process
```

蜂鸣器使用明确的板级开关接口，不是通用 PWM。主控工程目前没有可由 Shell 任意分配的
PWM 输出。若将来新增 PWM，必须先确定原理图网络、SysConfig 资源、频率/占空比范围、
上电安全状态、板级 API 和实物验证，再注册命令。

本清理没有删除灰度 ADC、蜂鸣器或任何已有设备驱动，也没有修改调度器。
