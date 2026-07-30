(function(root,factory){
  if(typeof module==='object'&&module.exports)module.exports=factory();
  else root.HelpCatalog=factory();
}(typeof self!=='undefined'?self:this,function(){
'use strict';

var CATEGORIES=[
  {id:'quick',label:'快速开始',description:'连接设备并完成第一次安全试运行'},
  {id:'safety',label:'模式与安全',description:'比赛、调试模式和紧急停止'},
  {id:'sequence',label:'序列编辑器',description:'工程、连线、校验、RAM 与 FRAM'},
  {id:'modules',label:'模块说明',description:'全部动作模块的参数与接线案例'},
  {id:'dashboard',label:'实时仪表盘',description:'遥测、记录、曲线和数据导出'},
  {id:'parameters',label:'参数管理',description:'RAM 参数、默认值和持久化'},
  {id:'terminal',label:'终端与命令',description:'Shell、命令库和风险标记'},
  {id:'troubleshooting',label:'故障排查',description:'常见报错、原因与处理步骤'}
];

function Article(id,category,title,summary,keywords,sections,risk){
  return{id:id,category:category,title:title,summary:summary,
    keywords:keywords||[],sections:sections||[],risk:risk||'safe'};
}

var ARTICLES=[
  Article('quick-start','quick','第一次连接与试运行',
    '从浏览器连接串口，先用模拟设备熟悉界面，再以架空车轮的方式试运行序列。',
    ['连接串口','模拟设备','第一次','Chrome','Edge','试运行'],[
      {title:'推荐顺序',steps:[
        '使用支持 Web Serial 的 Chrome 或 Edge 打开上位机。',
        '先点击“模拟设备”，确认序列编辑器、仪表盘和参数页面可以正常切换。',
        '连接 gugaPI 调试串口：UART6，PC10/PC11，115200 8N1。',
        '实车首次测试前架空车轮并限制电源电流，保证“紧急停止”按钮始终可点击。',
        '先使用“试运行当前画布”写入 RAM；确认动作正确后再保存到 FRAM。'
      ]},
      {title:'重要区别',body:[
        '试运行只更新 MCU RAM，复位后丢失；保存到 FRAM 才会覆盖绑定槽位。',
        '模拟设备只验证上位机流程，不代表传感器、MotorDriver 或机械机构已经通过验收。'
      ]}
    ],'warning'),
  Article('serial-connect','quick','串口连接与模拟设备',
    '解释浏览器串口权限、端口选择、断开重连以及模拟模式的边界。',
    ['串口','Web Serial','COM','权限','115200','模拟'],[
      {title:'连接要求',body:[
        '选择 gugaPI 的调试串口，而不是 MotorDriver 或 FOC 的独立调试口。',
        '浏览器会弹出端口授权窗口；没有看到设备时检查 USB 线、驱动和端口是否被 CCS 占用。'
      ]},
      {title:'模拟模式',body:[
        '模拟模式提供槽位、参数和遥测假数据，适合学习界面和检查序列编译。',
        '模拟模式不会验证固件版本、真实灰度传感器、IMU、底盘通信或电机方向。'
      ]}
    ]),
  Article('mode-safety','safety','比赛模式、调试模式与急停',
    '上电默认比赛待命；B1+B3 长按切换运行态，任何运动测试都以安全停车为最高优先级。',
    ['competition','armed','dev-running','B1','B3','B2','急停','模式'],[
      {title:'模式行为',body:[
        '主控上电默认进入 competition-armed，选择最低编号的有效 FRAM 槽位并保持停车。',
        'B1+B3 同时保持 1 秒可在比赛待命与 dev-running 调试运行态之间切换；两键必须释放后才能再次触发。',
        '比赛待命中 B1/B3 选择前后槽位，B2 启动；比赛运行中 B2 停止。'
      ]},
      {title:'上位机修改序列',body:[
        '比赛待命状态可以通过上位机修改未运行的 RAM 表和 FRAM 槽位。',
        '运行中的序列应先停止再覆盖；上位机的运行态切换不会恢复被 competition profile 编译关闭的功能。'
      ]},
      {title:'安全底线',body:[
        '紧急停止会取消序列、道路控制、航向、循迹和底盘命令，但软件按钮不能替代物理断电。',
        '出现振荡、过流、方向错误、传感器跳变或通信丢失时立即停车并保存完整日志。'
      ]}
    ],'danger'),
  Article('sequence-basics','sequence','画布、端口与执行规则',
    '理解开始、成功、失败/超时、循环回边、结束以及隐式终止的含义。',
    ['画布','绿色端口','红色端口','失败','超时','隐式终止','连线'],[
      {title:'端口规则',body:[
        '绿色端口表示动作正常完成，除“结束”外都必须连接后续动作。',
        '红色端口表示失败、条件不成立或超时；不连接时会终止当前序列并安全停车，连接后按用户指定路径执行。',
        '循环的“循环完成”虽复用内部失败字段，但它是绿色正常出口，必须显式连接。'
      ]},
      {title:'规模与超时',body:[
        '每条序列最多 64 个实际动作，整条 ActionRunner 序列受 300 秒运行保护。',
        '单动作和条件等待的超时上限仍为 30 秒；含循环序列不做静态展开，但运行时仍受 300 秒保护。'
      ]}
    ]),
  Article('sequence-storage','sequence','试运行、保存、回读与 JSON',
    '说明 RAM 临时表、FRAM 槽位、回读比对和工程 JSON 各自保存什么。',
    ['RAM','FRAM','槽位','JSON','回读','CRC','保存'],[
      {title:'RAM 与 FRAM',body:[
        '“试运行当前画布”依次执行 clear、add、validate、dump 和 start，只修改 RAM。',
        '“保存到 FRAM”会覆盖绑定槽位，并在写入后通过 seq dump 逐项比对。'
      ]},
      {title:'工程文件',body:[
        'JSON v2 保存节点位置、参数和连线，适合备份与交换；FRAM 保存编译后的 14 字节动作指令。',
        '旧固件不认识循环 op18 或循迹通过路口 op19 时会拒绝校验，不会启动未知动作。'
      ]}
    ],'warning'),
  Article('dashboard-use','dashboard','实时仪表盘与 CSV 记录',
    '按需选择遥测图表、调整采样周期和观察窗口，并把采样记录导出为 CSV。',
    ['遥测','曲线','CSV','采样周期','记录','telem'],[
      {title:'基本使用',steps:[
        '从左侧选择一张图表，上位机会为该图表请求对应的 telemetry profile。',
        '根据调试目标选择 50–1000 ms 采样周期和 10–60 秒观察窗口。',
        '需要留证时点击“开始记录”，完成后停止并导出 CSV。'
      ]},
      {title:'页面切换',body:[
        '离开仪表盘时，上位机会停止当前遥测流以减少串口占用；返回后会恢复已选择的图表。',
        '显示“数据超时”时先检查串口、固件 telemetry 支持和传感器数据新鲜度。'
      ]}
    ]),
  Article('parameter-use','parameters','参数的读取、修改与持久化',
    '区分当前 RAM 值、源码默认值、dirty 状态、FRAM 保存以及需要重启生效的参数。',
    ['参数','dirty','默认值','param save','FRAM','重启'],[
      {title:'状态含义',body:[
        '修改参数首先改变 RAM 镜像并标记 dirty；只有执行“保存到 FRAM”后才会跨复位保留。',
        '“从 FRAM 重载”会丢弃未保存的 RAM 修改；“恢复默认”也只恢复 RAM，仍需保存才持久化。'
      ]},
      {title:'修改原则',body:[
        '始终遵守页面显示的单位、范围和参数间约束，不要批量导入来源不明的配置。',
        '运动控制参数每次只改一类，以低速、架空轮胎方式验证并记录修改前后的串口日志。'
      ]}
    ],'warning'),
  Article('terminal-command','terminal','串口终端与 Shell 命令库',
    '终端用于直接收发 gugaPI Shell 命令；命令库提供格式、中文说明和风险标记。',
    ['Shell','终端','命令库','只读','写入','运动风险','Tab 补全'],[
      {title:'终端操作',body:[
        'Enter 发送，方向键浏览历史，Tab 补全顶层命令，Ctrl+L 清屏。',
        '终端与其他栏目共享同一个串口路由；实时遥测默认不会混入普通命令响应。'
      ]},
      {title:'风险标记',body:[
        '只读命令仅查询状态；写入命令可能修改 RAM、FRAM 或控制器状态；运动命令可能立即驱动车轮。',
        '命令库展示的是上位机审核目录，连接新固件后仍应通过 help 核对实际注册命令。'
      ]}
    ],'warning'),
  Article('faq-sequence','troubleshooting','序列校验失败怎么办',
    '针对缺少成功出口、循环无返回路径、不可达节点、超时合计和旧固件操作码给出处理方法。',
    ['校验失败','循环体入口不存在','不可达','缺少出口','操作码18','操作码19'],[
      {title:'连线问题',body:[
        '“成功端口未连接”：为该节点连接后续动作；最终流程必须到达“结束”模块。',
        '“循环体入口不存在返回路径”：循环体的正常完成路径必须连回同一个循环节点，且不能先进入循环完成出口。',
        '红色失败口可以留空；循环完成口不能留空。'
      ]},
      {title:'结构与版本',body:[
        '不可达节点需要从开始节点建立路径，或从画布删除。',
        '无循环序列静态最长时间超过 300 秒时应缩短等待/超时；含循环序列运行中仍会在 300 秒终止。',
        '旧固件拒绝 op18-op22 时升级固件，不能通过修改 JSON 绕过固件校验。'
      ]}
    ]),
  Article('faq-device','troubleshooting','车辆不动、突然停车或传感器失效',
    '按故障、通信、传感器新鲜度、模式和动作参数的顺序定位问题。',
    ['车辆不动','停车','MotorDriver','失联','传感器过期','fault','灰度','IMU'],[
      {title:'排查顺序',steps:[
        '确认当前模式、序列状态和是否存在 fault；FAULT 下组合键不会绕过故障锁定。',
        '查看 chassis、motor 和反馈新鲜度，确认 gugaPI 与 MotorDriver 通信正常。',
        '转向和 road_nav 检查 IMU；循迹和 road_nav 检查灰度有效性、标定与数据年龄。',
        '核对动作速度、距离、方向和超时参数，低速重新测试。'
      ]},
      {title:'安全行为',body:[
        'MotorDriver 失联、故障、急停、取消或关键传感器过期都会触发停车，这是预期保护而不是继续重试的理由。',
        '不要在故障原因未查清时提高速度、放宽超时或屏蔽传感器检查。'
      ]}
    ],'danger'),
  Article('faq-browser','troubleshooting','页面无串口、无数据或设置未保存',
    '处理浏览器兼容、端口占用、遥测停止和 RAM 修改未写入 FRAM 等常见问题。',
    ['浏览器','无串口','无数据','未保存','端口占用','遥测停止'],[
      {title:'浏览器与串口',body:[
        'Firefox 和 Safari 不提供本工具需要的 Web Serial；请使用桌面版 Chrome 或 Edge。',
        '端口列表为空时关闭占用串口的 CCS 终端或其他串口工具，再拔插 USB 并重新授权。'
      ]},
      {title:'数据与保存',body:[
        '仪表盘无数据时先选择图表并确认串口已连接；离开仪表盘后遥测暂停属于正常行为。',
        '参数或序列复位后消失，通常表示只修改了 RAM 而没有保存到 FRAM。'
      ]}
    ])
];

function DemoNode(type,summary,label){
  return{type:type,summary:summary||'',label:label||''};
}

function DemoFlow(label,nodes,ports,note){
  return{label:label,nodes:nodes,ports:ports||[],note:note||''};
}

var ACTION_GUIDES={
  drive:{
    purpose:'按指定有符号转速直行，并用当前航向作为保持目标。',
    parameters:['目标转速：正数前进、负数后退，绝对值不能超过 max_wheel_rpm。',
      '完成方式：可运行指定时间，也可在运动中等待通用条件成立。',
      '安全超时：条件模式下限制本动作最长运行时间。'],
    success:'时间到或通用条件连续成立后走“完成”出口。',
    failure:'启动失败、故障、传感器/通信异常或条件等待超时走红色出口；留空则终止并停车。',
    example:{title:'以 80 RPM 直行 2 秒后停车',paths:[
      DemoFlow('正常路径',[
        DemoNode('system_start'),DemoNode('drive','80 RPM · 2000 ms'),
        DemoNode('stop'),DemoNode('end')
      ],['success','success','success'])
    ]},
    tips:['首次实车测试使用低转速并架空车轮。','直行不是定距控制；需要精确距离时使用“定距行驶”。'],
    risk:'motion'
  },
  drive_mm:{
    purpose:'根据双轮编码器反馈行驶指定距离，并使用速度曲线减速停稳。',
    parameters:['行驶距离：正数前进、负数后退，不能为 0。',
      '最大转速：限制定距动作速度，范围 1..max_wheel_rpm。'],
    success:'达到目标距离并满足停稳条件后走“完成”出口。',
    failure:'编码器、IMU、底盘反馈失效或控制超时走红色出口；留空则终止。',
    example:{title:'前进 500 mm 后结束',paths:[
      DemoFlow('正常路径',[
        DemoNode('system_start'),DemoNode('drive_mm','500 mm · ≤80 RPM'),
        DemoNode('stop'),DemoNode('end')
      ],['success','success','success'])
    ]},
    tips:['距离精度依赖轮径、编码器计数和机械打滑。','负距离表示倒车，不需要把 RPM 设为负数。'],
    risk:'motion'
  },
  turn:{
    purpose:'以当前航向为基准执行闭环相对转向。',
    parameters:['相对转角：正数左转、负数右转，范围 -180°..180° 且不能为 0。',
      '安全超时：限制转向动作最长执行时间。'],
    success:'到达目标角度并满足停稳条件后走“完成”出口。',
    failure:'IMU 无效、底盘失联、启动失败或超时走红色出口。',
    example:{title:'左转 90° 后继续前进',paths:[
      DemoFlow('正常路径',[
        DemoNode('system_start'),DemoNode('turn','左 90° · 5000 ms'),
        DemoNode('drive_mm','300 mm · ≤80 RPM'),DemoNode('stop'),
        DemoNode('end')
      ],['success','success','success','success'])
    ]},
    tips:['先核对 IMU yaw 正方向。','超调时优先调转向控制参数，不要只缩短超时。'],
    risk:'motion'
  },
  follow:{
    purpose:'使用八路灰度传感器沿黑线循迹，可按时间或通用条件结束。',
    parameters:['基础转速：正数为正向循迹，也可用负数执行已验证的反向策略。',
      '完成方式：运行指定时间，或持续比较灰度、路口等通用数据。',
      '连续成立：过滤单帧抖动；安全超时限制最长循迹时间。'],
    success:'时间到或所选条件稳定成立后走“完成”出口。',
    failure:'灰度数据过期、丢线保护、通信故障或等待超时走红色出口。',
    example:{title:'循迹直到稳定丢线',paths:[
      DemoFlow('正常路径',[
        DemoNode('system_start'),DemoNode('follow','80 RPM · 丢线 100 ms'),
        DemoNode('stop'),DemoNode('end')
      ],['success','success','success'])
    ]},
    tips:['先完成黑白标定并确认 gray_valid。','通过明确路口方向时优先使用“循迹通过路口”。'],
    risk:'motion'
  },
  wait:{
    purpose:'先停车，再等待指定时间，用于动作间隔、消抖或声光提示节拍。',
    parameters:['等待时间：0..30000 ms；0 表示立即通过。'],
    success:'等待时间到后走“完成”出口。',
    failure:'等待动作通常不会业务失败；故障或序列级保护仍会终止运行。',
    example:{title:'蜂鸣 200 ms，等待后继续',paths:[
      DemoFlow('正常路径',[
        DemoNode('system_start'),DemoNode('buzzer_on','200 ms'),
        DemoNode('wait','500 ms'),DemoNode('led_on','LED2+LED3 · 500 ms'),
        DemoNode('end')
      ],['success','success','success','success'])
    ]},
    tips:['LED/蜂鸣器自动关闭不会暂停序列，需要节拍时显式加入等待。'],
    risk:'safe'
  },
  condition:{
    purpose:'立即检查一个数据条件，或停车等待条件持续成立。',
    parameters:['数据源：按键、灰度/道路、IMU、底盘、编码器距离或应用状态。',
      '比较方式和值：支持 ==、!=、<、<=、>、>= 和位包含。',
      '执行方式：立即判断或停车等待；等待模式还可设置超时与连续成立时间。'],
    success:'条件成立且满足稳定时间后走“条件成立”出口。',
    failure:'立即模式不成立/数据无效，或等待模式超时/数据无效时走红色出口；留空则终止。',
    example:{title:'检测到十字路口后提示，否则默认终止',paths:[
      DemoFlow('条件成立',[
        DemoNode('system_start'),
        DemoNode('condition','等待 · road_type == 十字'),
        DemoNode('led_on','LED2+LED3 · 500 ms'),DemoNode('end')
      ],['success','success','success']),
      DemoFlow('不成立 / 超时',[
        DemoNode('condition','等待 · road_type == 十字')
      ],[],'红色端口保持未连接，运行时默认终止序列并安全停车。')
    ]},
    tips:['Button2 在比赛中固定用于停止，引用 Button2 的条件只能用于 RAM 调试。'],
    risk:'safe'
  },
  loop:{
    purpose:'按固定次数重复执行一段动作，支持多个循环和嵌套循环。',
    parameters:['循环次数：1..1000，表示循环体实际执行次数；1 执行一次，0 不是无限循环。'],
    success:'“执行循环体”进入循环内容；循环体正常路径必须返回当前循环节点。',
    failure:'“循环完成”是正常绿色出口，完成全部次数后执行，必须显式连接实际动作。',
    example:{title:'闪灯三次后结束',paths:[
      DemoFlow('执行循环体',[
        DemoNode('system_start'),DemoNode('loop','循环 3 次'),
        DemoNode('led_toggle','LED2+LED3 · 0 ms'),DemoNode('wait','200 ms'),
        DemoNode('loop','返回循环')
      ],['success','success','success','success']),
      DemoFlow('循环完成',[
        DemoNode('loop','循环 3 次'),DemoNode('led_off','LED2+LED3'),
        DemoNode('end')
      ],['failure','success'])
    ]},
    tips:['异常路径可以留空终止或连接恢复动作，但正常循环体必须形成返回路径。',
      '循环仍受整条序列 300 秒运行上限保护。'],
    risk:'safe'
  },
  road_nav:{
    purpose:'从当前黑线循迹到下一个路口，按指定方向通过并重新捕获黑线。',
    parameters:['路口方向：左、直、右、左掉头或右掉头。',
      '掉头方式：滚动圆弧保持连续性；停车原地先停车再旋转。',
      '循迹速度：1..max_wheel_rpm，仅支持正向；整体超时按 50 ms 步进设置。'],
    success:'完成转向、连续两帧重捕线并恢复循迹后走“通过并重捕线”出口。',
    failure:'目标出口不存在、重捕线失败、传感器/通信异常或整体超时走红色出口。',
    example:{title:'下个路口左转，失败则停车',paths:[
      DemoFlow('正常通过',[
        DemoNode('system_start'),DemoNode('road_nav','下个路口左转 · 80 RPM'),
        DemoNode('follow','继续循迹 · 80 RPM'),DemoNode('end')
      ],['success','success','success']),
      DemoFlow('路线不可用 / 捕线失败',[
        DemoNode('road_nav','下个路口左转 · 80 RPM'),
        DemoNode('stop'),DemoNode('end')
      ],['failure','success'])
    ]},
    tips:['连续 road_nav 可保留循迹交接；转入普通动作或结束前会安全停车。',
      '滚动掉头需要单独验证场地空间，首次必须低速架空轮胎。'],
    risk:'motion'
  },
  dm_position:{
    purpose:'让单台 DM-G6220 按限速参考轨迹到达绝对或相对角度，并在成功后保持目标位置。',
    parameters:['定位方式：相对当前位置或相对电机绝对零位。',
      '目标角度：界面使用度，编译后使用 mrad，最终目标必须位于 ±12.5 rad。',
      '轨迹速度：最大 1145.9°/s（20000 mrad/s）；安全超时为 50..30000 ms。'],
    success:'位置误差、速度和稳定时间同时满足配置后走“完成”出口，并继续保持目标。',
    failure:'定位自身超时会收回参考位置、保持当前位置并走红色出口；反馈丢失或电机错误属于全局故障。',
    example:{title:'相对转动 5°，失败时默认终止',paths:[
      DemoFlow('定位成功',[
        DemoNode('system_start'),DemoNode('dm_position','相对 5° · ≤11.5°/s'),
        DemoNode('wait','500 ms'),DemoNode('dm_disable','释放电机'),
        DemoNode('end')
      ],['success','success','success','success']),
      DemoFlow('定位超时',[
        DemoNode('dm_position','相对 5° · ≤11.5°/s')
      ],[],'红色端口留空时终止序列并重复发送失能。')
    ]},
    tips:['第一次运动应空载、固定电机外壳并限流供电。',
      '机械机构安装后必须另行收紧软限位，不能继续直接使用协议全范围。'],
    risk:'motion'
  },
  dm_speed:{
    purpose:'让 DM-G6220 按斜坡到达指定角速度，持续一段时间后减速到零并保持停止位置。',
    parameters:['目标角速度：输入框最大可填 ±1145.9°/s（±20000 mrad/s），不能为 0；实际速度还受全局参数 dm_max_velocity_mrad_s 限制。',
      '持续时间：50..30000 ms，步进 50 ms。'],
    success:'持续时间结束并完成斜坡减速后走“完成”出口，同时保持停止位置。',
    failure:'启动失败或停止阶段超时走红色出口；反馈丢失、电机状态码 8..14 或 CAN bus-off 触发全局故障。',
    example:{title:'正转 2 秒后保持并释放',paths:[
      DemoFlow('正常路径',[
        DemoNode('system_start'),DemoNode('dm_speed','11.5°/s · 2000 ms'),
        DemoNode('wait','500 ms'),DemoNode('dm_disable','释放电机'),
        DemoNode('end')
      ],['success','success','success','success'])
    ]},
    tips:['定速动作结束并不立即失能，而是保持停止位置；需要自由转动时连接“达妙失能”。'],
    risk:'motion'
  },
  dm_disable:{
    purpose:'显式停止达妙周期控制并重复发送失能命令，让电机轴退出主动保持。',
    parameters:['该动作没有可调控制参数。'],
    success:'失能命令序列发送完成后走“完成”出口。',
    failure:'CAN 发送异常时走红色出口；序列终止仍会执行失能安全兜底。',
    example:{title:'动作完成后释放电机',paths:[
      DemoFlow('正常路径',[
        DemoNode('dm_position','相对 5° · ≤11.5°/s'),
        DemoNode('dm_disable','释放电机'),DemoNode('end')
      ],['success','success'])
    ]},
    tips:['序列结束、取消、隐式终止、急停、故障和模式切换都会自动失能；该节点用于在序列中提前释放。'],
    risk:'motion'
  },
  ball_hold:{
    purpose:'启动钢球位置闭环并在后台保持指定位置，后续巡线和定距动作不会停止该闭环。',
    parameters:['目标位置：-100.0..100.0 mm，相对轨道中心 O；正方向指向双连杆驱动端。'],
    success:'视觉和达妙控制权建立后立即走“完成”出口，滚球闭环继续以 100 Hz 运行。',
    failure:'启动时视觉无有效钢球、达妙反馈不新鲜或控制权被占用时走红色出口；运行中丢球会终止整个序列并停车。',
    example:{title:'保持中心并同时巡线',paths:[
      DemoFlow('正常路径',[
        DemoNode('system_start'),DemoNode('ball_hold','保持 0 mm'),
        DemoNode('follow','循迹 · 80 RPM'),DemoNode('ball_disable','释放滚球电机'),
        DemoNode('end')
      ],['success','success','success','success'])
    ]},
    tips:['先用 vision inject 和空载双连杆验证方向，再接入真实钢球。',
      '保持节点完成不代表闭环停止；必须用滚球失能、急停或序列结束释放。'],
    risk:'motion'
  },
  ball_move:{
    purpose:'将钢球平滑移动到指定位置，并等待位置与速度连续稳定后完成。',
    parameters:['目标位置：-100.0..100.0 mm，相对轨道中心 O。',
      '整体超时：50..30000 ms，步进 50 ms。'],
    success:'位置误差和速度满足阈值并连续稳定 200 ms 后走绿色出口，随后继续保持目标位置。',
    failure:'超时、视觉超过 120 ms 无有效球、钢球接近端部或达妙控制异常时走红色出口或终止序列。',
    example:{title:'H3：从中心移动到 +50 mm 再到 -50 mm',paths:[
      DemoFlow('正常路径',[
        DemoNode('system_start'),DemoNode('ball_move','移动到 +50 mm · 5000 ms'),
        DemoNode('ball_move','移动到 -50 mm · 5000 ms'),
        DemoNode('ball_disable','释放滚球电机'),DemoNode('end')
      ],['success','success','success','success'])
    ]},
    tips:['首次测试应降低供电电流并确保钢球不会越过机械端部。',
      '目标切换由控制器限角和限斜率处理，不要在两次移动之间插入达妙定位节点。'],
    risk:'motion'
  },
  ball_disable:{
    purpose:'停止滚球闭环并重复发送达妙失能命令，释放双连杆执行机构。',
    parameters:['该动作没有可调参数。'],
    success:'控制权释放后走绿色出口。',
    failure:'控制权状态异常时走红色出口；急停仍会执行达妙失能兜底。',
    example:{title:'任务完成后释放滚球机构',paths:[
      DemoFlow('正常路径',[
        DemoNode('ball_hold','保持 0 mm'),
        DemoNode('ball_disable','释放滚球电机'),DemoNode('end')
      ],['success','success'])
    ]},
    tips:['序列结束、取消、故障和模式切换也会自动失能；该节点用于提前释放。'],
    risk:'motion'
  },
  stop:{
    purpose:'立即停止底盘、航向、循迹和道路控制，然后继续执行序列。',
    parameters:['该动作没有可调参数。'],
    success:'完成统一停车命令后立即走“完成”出口。',
    failure:'底层停止返回异常时执行安全兜底；红色出口可用于记录或恢复，留空则终止。',
    example:{title:'先明确停车再等待',paths:[
      DemoFlow('正常路径',[
        DemoNode('drive','前序直行 · 80 RPM'),DemoNode('stop'),
        DemoNode('wait','1000 ms'),DemoNode('end')
      ],['success','success','success'])
    ]},
    tips:['停车不是序列结束；需要成功结束流程时仍要连接“结束”。'],
    risk:'motion'
  },
  end:{
    purpose:'以成功结果结束整条序列，并执行统一安全清理。',
    parameters:['该动作没有可调参数，也没有输出端口。'],
    success:'到达后 ActionRunner 报告成功并停止所有剩余动作。',
    failure:'无失败出口。',
    example:{title:'标准序列尾部',paths:[
      DemoFlow('正常结束',[
        DemoNode('system_start'),DemoNode('drive_mm','300 mm · ≤80 RPM'),
        DemoNode('stop'),DemoNode('end')
      ],['success','success','success'])
    ]},
    tips:['画布必须至少包含一个从开始节点可达的结束模块。'],
    risk:'safe'
  },
  led_on:{
    purpose:'点亮 LED2、LED3 或两者，可选自动关闭。',
    parameters:['LED 目标：LED2、LED3 或两者。','自动关闭：0 表示保持；非零为 50..30000 ms。'],
    success:'设置输出后立即走“完成”出口，自动关闭计时在后台运行。',
    failure:'输出服务异常时走红色出口；留空则终止。',
    example:{title:'点亮两灯 500 ms',paths:[
      DemoFlow('正常路径',[
        DemoNode('system_start'),DemoNode('led_on','LED2+LED3 · 500 ms'),
        DemoNode('wait','500 ms'),DemoNode('end')
      ],['success','success','success'])
    ]},
    tips:['LED1 保留给比赛状态指示，不作为序列动作目标。'],
    risk:'output'
  },
  led_off:{
    purpose:'立即熄灭 LED2、LED3 或两者。',
    parameters:['LED 目标：LED2、LED3 或两者。'],
    success:'设置输出后立即走“完成”出口。',
    failure:'输出服务异常时走红色出口；留空则终止。',
    example:{title:'结束提示前关闭所有用户 LED',paths:[
      DemoFlow('正常路径',[
        DemoNode('wait','前序动作 · 500 ms'),
        DemoNode('led_off','LED2+LED3'),DemoNode('end')
      ],['success','success'])
    ]},
    tips:['可用于清除 duration=0 的持续点亮状态。'],
    risk:'output'
  },
  led_toggle:{
    purpose:'翻转 LED2、LED3 或两者的当前状态，可选自动关闭。',
    parameters:['LED 目标：LED2、LED3 或两者。','自动关闭：0 表示保持；非零为 50..30000 ms。'],
    success:'翻转完成后立即走“完成”出口。',
    failure:'输出服务异常时走红色出口；留空则终止。',
    example:{title:'配合循环闪烁三次',paths:[
      DemoFlow('循环体',[
        DemoNode('loop','循环 3 次'),
        DemoNode('led_toggle','LED2+LED3 · 0 ms'),DemoNode('wait','200 ms'),
        DemoNode('loop','返回循环')
      ],['success','success','success'])
    ]},
    tips:['需要确定最终状态时，在循环完成出口再连接“LED 熄灭”。'],
    risk:'output'
  },
  buzzer_on:{
    purpose:'开启蜂鸣器，可选在指定时间后自动关闭。',
    parameters:['自动关闭：0 表示持续开启；非零为 50..30000 ms。'],
    success:'开启后立即走“完成”出口，自动关闭计时在后台运行。',
    failure:'输出服务异常时走红色出口；留空则终止。',
    example:{title:'鸣叫 200 ms 后继续',paths:[
      DemoFlow('正常路径',[
        DemoNode('system_start'),DemoNode('buzzer_on','200 ms'),
        DemoNode('wait','300 ms'),DemoNode('drive_mm','300 mm · ≤60 RPM'),
        DemoNode('end')
      ],['success','success','success','success'])
    ]},
    tips:['自动关闭不会暂停序列；需要听觉间隔时加入等待。'],
    risk:'output'
  },
  buzzer_off:{
    purpose:'立即关闭蜂鸣器。',
    parameters:['该动作没有可调参数。'],
    success:'关闭后立即走“完成”出口。',
    failure:'输出服务异常时走红色出口；留空则终止。',
    example:{title:'在流程末尾确保静音',paths:[
      DemoFlow('正常路径',[
        DemoNode('wait','前序动作 · 500 ms'),DemoNode('buzzer_off'),
        DemoNode('end')
      ],['success','success'])
    ]},
    tips:['序列结束、取消、故障和总超时也会统一关闭蜂鸣器。'],
    risk:'output'
  },
  buzzer_toggle:{
    purpose:'翻转蜂鸣器当前状态，可选自动关闭。',
    parameters:['自动关闭：0 表示保持翻转后的状态；非零为 50..30000 ms。'],
    success:'翻转后立即走“完成”出口。',
    failure:'输出服务异常时走红色出口；留空则终止。',
    example:{title:'循环生成三段提示音',paths:[
      DemoFlow('循环体',[
        DemoNode('loop','循环 3 次'),DemoNode('buzzer_toggle','200 ms'),
        DemoNode('wait','200 ms'),DemoNode('loop','返回循环')
      ],['success','success','success'])
    ]},
    tips:['若循环次数为奇数，结束前连接“蜂鸣器关闭”以确保最终静音。'],
    risk:'output'
  }
};

function clone(value){return JSON.parse(JSON.stringify(value))}

function buildEntries(actions){
  var entries=clone(ARTICLES);
  Object.keys(actions||{}).forEach(function(type){
    var action=actions[type],guide=ACTION_GUIDES[type];
    if(!guide)return;
    entries.push({
      id:'action-'+type,
      category:'modules',
      kind:'action',
      actionType:type,
      title:action.name,
      summary:guide.purpose,
      keywords:[type,'op '+action.op,'ActionOp '+action.op,action.group,
        action.name].concat(guide.parameters),
      risk:guide.risk,
      action:{op:action.op,name:action.name,group:action.group,
        color:action.color,defaults:clone(action.defaults)},
      guide:clone(guide)
    });
  });
  return entries;
}

function normalize(text){
  return String(text==null?'':text).toLowerCase()
    .replace(/\s+/g,' ').trim();
}

function search(entries,query,category){
  var needle=normalize(query);
  return(entries||[]).filter(function(entry){
    if(category&&category!=='all'&&entry.category!==category)return false;
    if(!needle)return true;
    return normalize(JSON.stringify(entry)).indexOf(needle)>=0;
  });
}

function category(id){
  return CATEGORIES.find(function(item){return item.id===id})||null;
}

return{
  CATEGORIES:CATEGORIES,
  ARTICLES:ARTICLES,
  ACTION_GUIDES:ACTION_GUIDES,
  buildEntries:buildEntries,
  search:search,
  category:category
};
}));
