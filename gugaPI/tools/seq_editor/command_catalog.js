'use strict';

/*
 * Audited against app/app_shell.cpp and the development feature profile.
 * Keep one entry for every top-level command registered by the firmware plus
 * the built-in help command. Source-present but disabled groups remain visible
 * when the user enables "显示未启用命令".
 */
var SHELL_CATALOG_META={
  sourceRevision:'2026-07-30',
  activeProfile:'development',
  activeProfileLabel:'开发配置',
  riskLabels:{R:'只读',W:'会改变状态',M:'可能运动'}
};

function ShellForm(syntax,description,risk){
  return{syntax:syntax,description:description,risk:risk||'R'};
}

function ShellCommand(name,title,category,summary,risk,profiles,forms,note){
  return{name:name,title:title,category:category,summary:summary,risk:risk,
    profiles:profiles,forms:forms,note:note||''};
}

var BOTH=['development','competition'];
var DEV=['development'];
var OFF=[];

var SHELL_COMMAND_LIBRARY=[
  ShellCommand('help','列出当前固件命令','系统',
    '读取固件运行时实际注册的顶层命令，是连接新固件后核对命令可用性的第一步。','R',BOTH,[
      ShellForm('help','打印当前固件实际注册的命令和英文简述。')
    ],'help 是 Shell 服务内建命令，不在 AppShell_RegisterCommands() 中重复注册。'),

  ShellCommand('version','查看固件与板级信息','系统',
    '显示板卡名称、MCU型号和默认调试串口波特率。','R',BOTH,[
      ShellForm('version','读取固件和板级基本信息。')
    ]),

  ShellCommand('reset','复位主控','系统',
    '触发 MSPM0G3519 软件复位，所有运动输出和运行期状态会被重新初始化。','W',BOTH,[
      ShellForm('reset','立即复位 MCU。','W')
    ],'执行前先停止运动并保存仍需保留的参数；未执行 param save 的 RAM 修改会丢失。'),

  ShellCommand('sched','调度器运行统计','系统',
    '查看各周期任务的运行次数、耗时、延迟和超时统计，或清零统计值。','W',BOTH,[
      ShellForm('sched','输出所有已注册任务的运行统计。'),
      ShellForm('sched reset','清零调度统计，不改变任务配置。','W')
    ]),

  ShellCommand('txstat','调试串口队列统计','系统',
    '查看 UART RX/TX 队列、DMA块、丢弃字节和错误计数。','R',BOTH,[
      ShellForm('txstat','输出调试 UART 队列和 DMA 统计。')
    ]),

  ShellCommand('led','LED状态与控制','板级IO',
    '读取或改变三个板载状态LED的逻辑状态。','W',BOTH,[
      ShellForm('led status','一次输出全部LED状态。'),
      ShellForm('led <1|2|3> on|off|toggle|status','控制或查询指定LED。','W'),
      ShellForm('led all on|off|toggle|status','同时控制或查询全部LED。','W')
    ],'status 返回驱动保存的逻辑状态，不替代对实际发光情况的观察。'),

  ShellCommand('buzzer','蜂鸣器控制','板级IO',
    '打开或关闭板载蜂鸣器。当前代码只支持 on/off。','W',BOTH,[
      ShellForm('buzzer on|off','打开或关闭蜂鸣器。','W')
    ]),

  ShellCommand('button','按键状态与事件观察','板级IO',
    '查看三个按键的原始电平、消抖状态和事件，或在限定时间内观察变化。','R',BOTH,[
      ShellForm('button','输出当前按键状态和累计事件。'),
      ShellForm('button watch [duration_ms 100..30000]','在指定时间内观察已消抖按键事件。'),
      ShellForm('button scan [duration_ms 100..30000]','在指定时间内扫描并输出按键变化。')
    ]),

  ShellCommand('fram','FRAM存储诊断','存储',
    '诊断 FM24CL64B 的共享I²C总线、执行保留区自检以及原始字节读写。','W',BOTH,[
      ShellForm('fram status','查看FRAM就绪状态和I²C线状态。'),
      ShellForm('fram recover','发送恢复时钟并重新恢复I²C控制器。','W'),
      ShellForm('fram test','在保留测试区执行备份、写入、读回和恢复。','W'),
      ShellForm('fram read <addr> <len 1..32>','从指定地址读取1到32字节。'),
      ShellForm('fram write <addr> <byte>','写入单字节并读回校验。','W')
    ],'fram write/test 会改变非易失存储；测试期间必须保持供电稳定。'),

  ShellCommand('param','运行参数与持久化','存储',
    '读取RAM参数镜像、修改参数，并显式加载或保存到FRAM。','W',BOTH,[
      ShellForm('param status','查看加载来源、dirty状态、版本长度和CRC。'),
      ShellForm('param get [name]','查看全部参数或指定参数及合法范围。'),
      ShellForm('param export [start [count 1..16]]','分页批量读取RAM参数，供上位机快速刷新。'),
      ShellForm('param set <name> <value>','修改RAM中的单个参数并标记dirty。','W'),
      ShellForm('param save','把当前RAM参数镜像写入FRAM。','W'),
      ShellForm('param load','丢弃未保存修改并从FRAM重新加载。','W'),
      ShellForm('param reset','把RAM参数恢复为源码默认值，仍需save才持久化。','W')
    ],'调参时先set并验证，确认正确后只执行一次save。'),

  ShellCommand('ina219','电压电流与保护诊断','传感器',
    'INA219测量、寄存器、保护状态及OLED页面。当前开发配置关闭FEATURE_ENABLE_INA219。','W',OFF,[
      ShellForm('ina219 status','查看初始化、地址和最近通信状态。'),
      ShellForm('ina219 scan','扫描可能的INA219地址。'),
      ShellForm('ina219 addr <0x40..0x4F>','修改运行期目标地址。','W'),
      ShellForm('ina219 recover','恢复共享I²C总线。','W'),
      ShellForm('ina219 config','重新写入测量配置。','W'),
      ShellForm('ina219 reset','复位INA219寄存器。','W'),
      ShellForm('ina219 read','读取换算后的电压、电流和功率。'),
      ShellForm('ina219 raw','读取原始测量寄存器。'),
      ShellForm('ina219 reg <0..5> [value]','读取寄存器；提供value时写寄存器。','W'),
      ShellForm('ina219 protect status|clear','查看或清除保护锁存。','W'),
      ShellForm('ina219 oled on [period_ms 100..5000]|off|status|once','控制INA219 OLED页面。','W')
    ]),

  ShellCommand('battery','电池SOC估算','传感器',
    '基于INA219的3S1P/3000mAh运行期SOC估算。当前随INA219功能关闭。','W',OFF,[
      ShellForm('battery status','查看SOC、电压、电流和估算来源。'),
      ShellForm('battery full','把本次运行期容量校准为满电。','W'),
      ShellForm('battery reset','清除运行期SOC与积分状态。','W'),
      ShellForm('battery log on [period_ms 100..5000]','启动周期电量输出。','W'),
      ShellForm('battery log off|status','停止日志或查看日志状态。','W')
    ]),

  ShellCommand('oled','OLED显示诊断','显示',
    '初始化、清屏、填充、文本、反色和开关SSD1306 OLED。','W',BOTH,[
      ShellForm('oled status','查看OLED初始化和最近操作状态。'),
      ShellForm('oled init','重新初始化OLED控制器。','W'),
      ShellForm('oled clear','清空显示缓存和屏幕。','W'),
      ShellForm('oled fill <0x00..0xFF>','用指定字节填充全屏。','W'),
      ShellForm('oled test','显示测试图案。','W'),
      ShellForm('oled text <row 0..3> <col 0..20> <ascii...>','在指定字符位置显示ASCII文本。','W'),
      ShellForm('oled invert on|off','打开或关闭反色显示。','W'),
      ShellForm('oled on|off','打开或关闭面板显示。','W')
    ]),

  ShellCommand('timer','OLED大字计时器','显示',
    '以M:SS.t格式占满128×32 OLED显示正向计时，采用异步DMA局部窗口刷新。','W',BOTH,[
      ShellForm('timer start|stop|resume|reset|hide|status','从零开始、停止冻结、继续、清零、释放屏幕或查看计时状态。','W')
    ],'显示范围为0:00.0到9:59.9；达到上限后自动停止。计时器不会修改参数或FRAM。'),

  ShellCommand('gy931','GY931姿态模块诊断','传感器',
    '软件I²C通信、角度读取、算法选择和OLED页面。当前配置关闭FEATURE_ENABLE_GY931。','W',OFF,[
      ShellForm('gy931 status','查看地址、就绪和通信状态。'),
      ShellForm('gy931 init','重新初始化模块。','W'),
      ShellForm('gy931 recover','恢复软件I²C总线。','W'),
      ShellForm('gy931 scan [start end]','扫描指定I²C地址范围。'),
      ShellForm('gy931 addr [0x08..0x77]','读取或修改模块地址。','W'),
      ShellForm('gy931 angle','读取欧拉角。'),
      ShellForm('gy931 algorithm [6axis|9axis]','读取或切换姿态算法。','W'),
      ShellForm('gy931 sample','读取完整传感器样本。'),
      ShellForm('gy931 raw <reg> <words 1..16>','读取原始寄存器字。'),
      ShellForm('gy931 oled on [period_ms 50..5000]|off|status|once','控制GY931 OLED页面。','W')
    ]),

  ShellCommand('imu','ICM45686与LIS3MDL诊断','传感器',
    '查看组合IMU数据，并提供SPI片选、原始传输、ICM45686和LIS3MDL诊断入口。','W',BOTH,[
      ShellForm('imu status','查看IMU初始化、采样和错误状态。'),
      ShellForm('imu cs idle|icm|lis|float','强制控制SPI片选状态，仅用于硬件诊断。','W'),
      ShellForm('imu pins wiggle [loops]','循环翻转SPI相关引脚供示波器观察。','W'),
      ShellForm('imu spi mode <0..3>','临时切换SPI模式。','W'),
      ShellForm('imu spi burst [bytes] [byte]','发送指定长度的重复字节并统计吞吐。','W'),
      ShellForm('imu spi rx [count 1..16] [tx]','执行短原始SPI收发。','W'),
      ShellForm('imu oled on [period_ms]|off|status|once','控制IMU OLED页面。','W'),
      ShellForm('imu bias status','查看固定零偏、运行时零偏、静止采集进度和拒绝原因。'),
      ShellForm('imu bias calibrate','等待车辆静止并执行一次2秒运行时零偏校准，只更新RAM。','W'),
      ShellForm('imu bias auto on|off','开启或关闭运行中静止零偏自动学习。','W'),
      ShellForm('imu bias save','把固定零偏与运行时零偏合并后写入FRAM。','W'),
      ShellForm('imu bias reset','仅清除RAM中的运行时零偏，不擦除FRAM固定零偏。','W'),
      ShellForm('imu lis whoami','读取LIS3MDL WHO_AM_I。'),
      ShellForm('imu lis reg <addr>','读取LIS3MDL寄存器。'),
      ShellForm('imu lis status|init|sample','查看状态、重新初始化或读取样本。','W'),
      ShellForm('imu lis scale <0..3>|odr <0..7>|mode <0..2>','修改LIS3MDL量程、输出速率或工作模式。','W'),
      ShellForm('imu sample','读取组合IMU姿态和传感器样本。'),
      ShellForm('imu icm init','重新初始化ICM45686。','W'),
      ShellForm('imu icm whoami','读取ICM45686 WHO_AM_I。'),
      ShellForm('imu icm sample','读取ICM45686加速度与角速度。'),
      ShellForm('imu icm reg <addr>','读取ICM45686寄存器。'),
      ShellForm('imu icm wreg <addr> <val>','写入ICM45686原始寄存器。','W')
    ],'自动零偏只在底盘和控制器确认静止时学习；原始SPI、片选和寄存器写入可能破坏当前传感器配置，完成后应重新初始化。'),

  ShellCommand('gray','八路灰度传感器','传感器',
    '查看原始ADC、归一化、插值位置、线路状态并完成黑白标定。','W',BOTH,[
      ShellForm('gray status','查看灰度任务、DMA/ADC和处理状态。'),
      ShellForm('gray read <0..7>','读取指定通道。'),
      ShellForm('gray all','立即读取全部八路原始值。'),
      ShellForm('gray data','读取周期任务缓存的八路原始值。'),
      ShellForm('gray process','输出归一化、位置、强度和线路判定。'),
      ShellForm('gray calib show|status|reload|sweep [ms]','查看标定、重载参数或执行位置扫描。','W'),
      ShellForm('gray calib white [frames]|black [frames]|commit|cancel','采集白/黑标定并提交或取消。','W'),
      ShellForm('gray oled on [period_ms 50..5000]|off|status|once','控制灰度OLED页面。','W')
    ],'calib commit 会更新ConfigStore RAM参数；需要param save才会写入FRAM。'),

  ShellCommand('linesensor','选择真实线路传感器','传感器',
    '在八路 ADC 灰度传感器与三路串口红外传感器之间切换统一循迹数据来源。','W',BOTH,[
      ShellForm('linesensor status','查看当前来源、RAM配置来源、有效性、标定状态和道路能力。'),
      ShellForm('linesensor source adc8|ir3','切换真实线路传感器来源，只修改RAM并标记参数未保存。','W')
    ],'运行中禁止切换。ir3 只支持基础循迹和全黑检测，不支持自动路口；只有 param save 才写入FRAM。'),

  ShellCommand('irsensor','三路串口红外传感器','传感器',
    '诊断 PA1/UART0 RX 上的真实三路红外模块，并执行五步低延迟循迹标定。','W',BOTH,[
      ShellForm('irsensor status|raw|stats|diag|clear','读取状态、裸数据、底层诊断或统计，或清零通信统计。','W'),
      ShellForm('irsensor calib begin|status|commit|cancel','开始、查看、提交或取消五步标定。','W'),
      ShellForm('irsensor calib capture <white|black|center|left|right>','采集指定标定位置的64个正确帧。','W'),
      ShellForm('irsensor status','查看帧新鲜度、线路、全黑、超时和标定状态。'),
      ShellForm('irsensor raw','读取模块偏差、标准化位置、全黑标志与三路ADC裸值。'),
      ShellForm('irsensor stats','查看有效率、CRC、语义错误、通信健康状态、循环DMA积压/覆盖和控制延迟。'),
      ShellForm('irsensor diag','查看UART上电/使能、PA1/RX电平、128字节循环DMA位置、积压和全局DMA故障。'),
      ShellForm('irsensor clear','只清零解析、UART、DMA和延迟统计，不中断接收且不修改标定或FRAM。','W'),
      ShellForm('irsensor calib begin','开始新的五步标定会话。','W'),
      ShellForm('irsensor calib capture white|black|center|left|right','采集指定位置的64个CRC正确帧。','W'),
      ShellForm('irsensor calib status','查看每一步进度、稳健平均值和最近状态。'),
      ShellForm('irsensor calib commit|cancel','校验并提交到RAM，或取消本次标定。','W')
    ],'传感器是真实硬件；上位机“模拟连接”只用于离线界面测试。commit 后仍需 param save 才会写入FRAM。'),

  ShellCommand('lora','LoRa串口与帧协议','通信',
    '诊断LoRa串口透传和带CRC/ACK的帧协议，仅开发配置启用。','W',DEV,[
      ShellForm('lora status','查看串口和协议统计。'),
      ShellForm('lora send <text...>','原样发送文本，不自动追加换行。','W'),
      ShellForm('lora line <text...>','发送文本并追加行结束符。','W'),
      ShellForm('lora hex <byte...>','发送十六进制字节序列。','W'),
      ShellForm('lora read [len 1..64]','读取接收缓冲中的字节。'),
      ShellForm('lora clear','清空接收缓冲和相关计数。','W'),
      ShellForm('lora test','发送固定测试内容。','W'),
      ShellForm('lora proto on|off|status|reset|recv','控制帧协议、查看状态或取接收帧。','W'),
      ShellForm('lora proto send <type> <ack|noack> [text...]','发送带类型和ACK策略的协议帧。','W')
    ]),

  ShellCommand('motor','MotorDriver底层控制','运动',
    '选择通信总线、读取反馈、配置控制参数并直接控制单个电机。','M',BOTH,[
      ShellForm('motor status','查看客户端总线、错误和目标状态。'),
      ShellForm('motor bus [uart|i2c]','读取或切换MotorDriver传输总线。','W'),
      ShellForm('motor i2caddr [0x08..0x77]','读取或修改MotorDriver I²C地址。','W'),
      ShellForm('motor ping','探测当前MotorDriver链路。'),
      ShellForm('motor info','读取MotorDriver版本和能力信息。'),
      ShellForm('motor enc','读取双编码器计数。'),
      ShellForm('motor enc reset','清零双编码器计数。','W'),
      ShellForm('motor rpm','读取双电机目标与实际转速。'),
      ShellForm('motor ramp [accel_rpm_s decel_rpm_s]','读取或设置MotorDriver速度斜坡。','W'),
      ShellForm('motor cfg','读取编码器每圈计数配置。'),
      ShellForm('motor cfg <m1_counts_per_rev> <m2_counts_per_rev>','写入编码器每圈计数。','W'),
      ShellForm('motor invert','读取电机输出和编码器方向配置。'),
      ShellForm('motor invert m1|m2 on|off','设置指定电机输出方向。','W'),
      ShellForm('motor invert enc m1|m2 on|off','设置指定编码器反馈方向。','W'),
      ShellForm('motor pid','读取速度环PID和占空比限制。'),
      ShellForm('motor pid <kp_q4.4> <ki_q4.4> <kd_q4.4> [max_duty [min_duty]]','写入速度环PID参数。','W'),
      ShellForm('motor pos','读取位置控制状态。'),
      ShellForm('motor pospid','读取位置环PID。'),
      ShellForm('motor pospid <kp_q4.4> <ki_q4.4> <kd_q4.4> [max_rpm [tol_counts]]','写入位置环PID。','W'),
      ShellForm('motor posctl','读取位置控制占空比和稳定判定参数。'),
      ShellForm('motor posctl <min_duty> <max_duty> [exit_tol_counts [settle_ms]]','写入位置控制参数。','W'),
      ShellForm('motor reg <addr> <len 1..32>','读取MotorDriver原始寄存器。'),
      ShellForm('motor set <addr> <byte...>','写入MotorDriver原始寄存器。','W'),
      ShellForm('motor stop','停止两个电机。','M'),
      ShellForm('motor m1|m2 coast|brake','设置指定电机滑行/制动状态。','M'),
      ShellForm('motor m1|m2 run <duty 0..100> [fwd|rev]','按占空比直接运行指定电机。','M'),
      ShellForm('motor m1|m2 speed <rpm 0..1000> [fwd|rev]','按速度闭环运行指定电机。','M'),
      ShellForm('motor m1|m2 hold','保持指定电机当前位置。','M'),
      ShellForm('motor m1|m2 pos <deg>','让指定电机到达绝对编码器角度。','M'),
      ShellForm('motor m1|m2 posrel <deg>','让指定电机相对转动指定角度。','M'),
      ShellForm('motor send <text...>','通过Motor UART发送原始文本。','W'),
      ShellForm('motor line <text...>','通过Motor UART发送一行文本。','W'),
      ShellForm('motor hex <byte...>','通过Motor UART发送原始字节。','W'),
      ShellForm('motor read [len 1..64]','读取Motor UART接收缓冲。'),
      ShellForm('motor clear','清空Motor UART接收缓冲。','W'),
      ShellForm('motor test','发送Motor UART固定测试帧。','W')
    ],'任何run/speed/hold/pos/posrel命令都可能使车轮运动；先架空车轮并准备motor stop。'),

  ShellCommand('chassis','双轮底盘控制','运动',
    '读取底盘状态，直接设置左右轮转速或线速度/角速度。','M',BOTH,[
      ShellForm('chassis status','读取底盘配置、目标和反馈状态。'),
      ShellForm('chassis stat','读取底盘通信与块传输统计。'),
      ShellForm('chassis stop','停止底盘运动。','M'),
      ShellForm('chassis wheel <left_rpm> <right_rpm>','直接设置左右轮目标RPM。','M'),
      ShellForm('chassis vel <linear_mm_s> <angular_mdeg_s>','按线速度和角速度换算左右轮RPM。','M')
    ],'运动前确认底盘周围安全，异常时优先使用chassis stop。'),

  ShellCommand('heading','航向与定距闭环','运动',
    '使用IMU航向和编码器反馈执行航向保持、闭环转向和指定距离行驶。','M',BOTH,[
      ShellForm('heading status','查看航向闭环模式、目标、误差和修正量。'),
      ShellForm('heading hold <base_rpm>','以指定基础RPM直行并保持启动时航向。','M'),
      ShellForm('heading lock','车辆静止时捕获当前航向，受外力偏转后自动原地回正。','M'),
      ShellForm('heading lockcfg','查看静止锁向PD、死区、轮速和超时参数。'),
      ShellForm('heading lockcfg set <key> <value>','修改锁向参数；key支持kp、kd、wake、settle、minrpm、maxrpm、rate、wheelrpm、settlems和timeout。','W'),
      ShellForm('heading lockcfg save','把当前静止锁向参数保存到FRAM。','W'),
      ShellForm('heading turn <deg -180..180>','相对当前航向闭环转动指定角度。','M'),
      ShellForm('heading turncfg','查看转向预测制动、提前量和停止判定参数。'),
      ShellForm('heading turncfg set <brake_ms> <margin_mdeg> <settle_mdps> <settle_rpm>','同时修改四项闭环转向停止参数。','W'),
      ShellForm('heading turncfg save','把当前转向停止参数保存到FRAM。','W'),
      ShellForm('heading distance <mm -10000..10000> <max_rpm> [timeout_ms]','按编码器距离闭环行驶。','M'),
      ShellForm('heading profile','查看定距速度曲线参数。'),
      ShellForm('heading profile mode <legacy|trapezoid>','选择恒速或梯形速度规划。','W'),
      ShellForm('heading profile <accel|decel|creep|latency|margin|settle|tolerance> <value>','修改定距曲线参数。','W'),
      ShellForm('heading profile save','把当前曲线参数保存到FRAM。','W'),
      ShellForm('heading stop','停止航向或定距动作。','M')
    ]),

  ShellCommand('run','RAM动作序列','流程',
    '在 RAM 中构建、检查和执行最多 64 步的动作状态机。','M',BOTH,[
      ShellForm('run add <op> <p1> <p2> <until> <onsuccess> <ontimeout>','追加传统运动、等待、结束及 LED/蜂鸣器动作。','W'),
      ShellForm('run add condition <source> <cmp> <value> <instant|wait>','后续填写超时、稳定时间和真假跳转，追加通用条件判断。','W'),
      ShellForm('run add drive_if|follow_if <rpm> <source> <cmp> <value>','后续填写超时、稳定时间和跳转，运动期间持续判断通用条件。','W'),
      ShellForm('run add loop <count> 0 immediate <body_index> <done_index>','追加计数循环；循环体返回本节点，完成出口连接后续动作。','W'),
      ShellForm('run add road_nav <route> <rpm> <timeout_ms> <onsuccess> <onfailure>','循迹通过下一个路口；route 支持左/直/右及左右圆弧或原地掉头。','M'),
      ShellForm('run clear|validate|start|cancel|status|dump','清空、校验、启动、取消、查看状态或导出 RAM 动作序列；另支持 validate competition。','M')
    ],'start可能产生运动；建议先run dump核对每一步和跳转目标。'),

  ShellCommand('lf','线路循迹控制','运动',
    '使用当前真实线路传感器启动/停止循迹；八路ADC与三路红外分别保存控制增益。','M',BOTH,[
      ShellForm('lf status','查看循迹误差、修正量和丢线计数。'),
      ShellForm('lf cal','执行兼容标定入口。','W'),
      ShellForm('lf start <rpm> <ms>','以指定基础RPM运行限定时间的循迹。','M'),
      ShellForm('lf stop','停止循迹和底盘。','M'),
      ShellForm('lf kp <val>','修改循迹比例增益。','W'),
      ShellForm('lf kd <val>','修改循迹微分增益。','W'),
      ShellForm('lf maxcorr <val>','修改最大差速修正RPM。','W'),
      ShellForm('lf slew <permille_per_s 1..65535>','修改修正量变化率。','W'),
      ShellForm('lf losthold <ms> (compatibility only)','兼容入口：修改短时丢线保持时间。','W'),
      ShellForm('lf losttimeout <ms> (compatibility only)','兼容入口：修改持续丢线停车时间。','W')
    ]),

  ShellCommand('road','路口与弯道事件','运动',
    '查看路口检测事件、切换检测/弯道状态，并配置闭环转弯和重新捕线参数。','W',BOTH,[
      ShellForm('road status|event|clear','查看状态、读取最近事件或清除事件。','W'),
      ShellForm('road mode detect|corner','切换普通检测或弯道恢复模式。','W'),
      ShellForm('road auto on|off','自动弯道模式开关的兼容入口。','W'),
      ShellForm('road align show','查看持久化的路口对齐距离和连续处理速度上限。'),
      ShellForm('road align set <distance_mm 0..300> <rpm 1..300>','设置实际转弯位置及对齐/圆弧/未确认线路时的捕线速度；转弯末段提前确认新线路，param save后写入FRAM。','W'),
      ShellForm('road turn show','查看左右转角、距离对齐和重新捕线参数。'),
      ShellForm('road turn set <left_deg> <right_deg> <align_mm> <rpm> <reacquire_ms>','设置路口闭环转弯参数。','W')
    ]),

  ShellCommand('comp','比赛状态机','流程',
    '选择FRAM序列槽、进入待命状态并启动或停止比赛任务。','M',BOTH,[
      ShellForm('comp arm','进入ARMED待命状态。','W'),
      ShellForm('comp select <0..7>','选择比赛序列槽。','W'),
      ShellForm('comp start [seq 0..7]','启动当前或指定槽位的比赛序列。','M'),
      ShellForm('comp stop','停止比赛任务和底盘。','M'),
      ShellForm('comp status','查看模式、槽位、步骤和结果。')
    ]),

  ShellCommand('estop','软件全停','运动',
    '取消所有应用层运动控制器并强制停止底盘；不会清除锁存故障，也不能替代硬件断电。','M',BOTH,[
      ShellForm('estop','立即取消序列、航向、循迹和道路控制，并停止两个车轮。','M')
    ],'该命令用于调试软件紧急停止。涉及人身或设备安全时仍应切断电机电源。'),

  ShellCommand('telem','周期遥测输出','通信',
    '按指定周期输出FireWater/VOFA+兼容的CSV遥测行，支持全量兼容模式和仪表盘按组模式。','W',BOTH,[
      ShellForm('telem on [period_ms 50..5000]','启动旧版全字段遥测，可指定周期。','W'),
      ShellForm('telem on <profile> [period_ms 50..5000]','只输出实时仪表盘当前图表需要的字段；使用 telem 命令帮助查看完整 profile。','W'),
      ShellForm('telem off','停止周期遥测。','W'),
      ShellForm('telem status','查看遥测开关、数据组和周期。')
    ]),

  ShellCommand('seq','FRAM动作序列槽','流程',
    '列出、读写、删除或直接运行0到7号FRAM持久化动作序列。','M',BOTH,[
      ShellForm('seq list','列出全部槽位有效性和指令数。'),
      ShellForm('seq dump <0..7>','导出指定槽的动作序列。'),
      ShellForm('seq save <0..7>','把当前RAM动作序列保存到指定槽。','W'),
      ShellForm('seq load <0..7>','把指定槽加载到RAM。','W'),
      ShellForm('seq del <0..7>','删除指定FRAM序列槽。','W'),
      ShellForm('seq run <0..7>','加载并立即执行指定序列。','M')
    ],'seq run可能立即产生运动；来源不明的序列应先dump检查。'),

  ShellCommand('i2c','通用I²C诊断','通信',
    '列出逻辑总线、查看线状态、恢复、扫描、探测和执行原始寄存器访问，仅开发配置启用。','W',DEV,[
      ShellForm('i2c list','列出可用逻辑I²C总线名。'),
      ShellForm('i2c status <bus>','查看控制器状态和SCL/SDA电平。'),
      ShellForm('i2c recover <bus>','发送恢复时钟并恢复控制器。','W'),
      ShellForm('i2c scan <bus> [start end]','扫描指定地址范围。'),
      ShellForm('i2c probe <bus> <addr>','探测单个7位地址。'),
      ShellForm('i2c read <bus> <addr> <reg8> <len 1..32>','从8位寄存器地址连续读取。'),
      ShellForm('i2c write <bus> <addr> <reg8> <byte...>','向目标寄存器写入原始字节。','W'),
      ShellForm('i2c test <bus> [start end]','对地址范围执行诊断测试。','W')
    ],'fram、oled、ina219逻辑名共享SENSOR_I2C；原始写入前必须确认器件和寄存器含义。'),

  ShellCommand('can','经典CAN总线诊断','通信',
    '诊断TCAN3413收发器与CAN1控制器，读取总线状态、收发经典CAN帧并处理错误恢复。','W',BOTH,[
      ShellForm('can status','查看250 kbit/s总线模式、收发队列、错误计数和bus-off状态。'),
      ShellForm('can mode normal|standby','切换TCAN3413正常或待机模式。','W'),
      ShellForm('can send std|ext <hex_id> [hex_byte ...]','异步发送标准帧或扩展帧；ID和数据字节均使用十六进制。','W'),
      ShellForm('can read [count 1..32]','从上位机诊断队列读取最多32帧。'),
      ShellForm('can watch on|off','打开或关闭周期接收监视；监视会消费诊断队列中的帧。','W'),
      ShellForm('can clear|cancel|recover','清空统计、取消待发送帧或尝试恢复CAN控制器。','W')
    ],'recover只表示控制器重新进入正常模式；仍需确认终端电阻、CANH/CANL和TEC/REC。'),

  ShellCommand('jyme02','JY-ME02 CAN编码器','传感器',
    '查看JY-ME02角度、角速度、圈数、温度和新鲜度，或读取寄存器并调整固件解析参数。','W',BOTH,[
      ShellForm('jyme02 status','查看解析器地址、采样时间、数据新鲜度、计数和最新测量。'),
      ShellForm('jyme02 readreg <hex_reg>','通过CAN请求读取指定8位寄存器。','W'),
      ShellForm('jyme02 regs','查看最近一次寄存器响应中的三个16位值。'),
      ShellForm('jyme02 address <hex_id>       (parser only)','只修改固件解析器接受的11位CAN标识，不写传感器。','W'),
      ShellForm('jyme02 sampletime <100us>     (parser only)','只修改固件解析器换算角速度所用的采样时间，范围1..65535，单位100 us。','W'),
      ShellForm('jyme02 clear','清空解析数据、原始诊断队列和统计。','W')
    ],'address和sampletime都不会写入JY-ME02；修改传感器配置需按其协议另行执行并重新验证总线。')
];
