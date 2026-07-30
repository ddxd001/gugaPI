(function(root,factory){
  var api=factory();
  if(typeof module==='object'&&module.exports)module.exports=api;
  root.SequenceCore=api;
}(typeof globalThis!=='undefined'?globalThis:this,function(){
'use strict';

var FORMAT='gugapi-sequence-project',VERSION=2,ABORT=255;
var CONDS=[
  'timeout','heading_reached','line_detected','line_lost','button',
  'immediate','distance_reached'
];
var COND_LABELS={
  timeout:'定时到达',heading_reached:'航向到达',
  line_detected:'检测到线路',line_lost:'线路丢失',
  button:'按钮按下',immediate:'立即',distance_reached:'距离到达'
};
var COMPARES={
  eq:{label:'等于',symbol:'=='},ne:{label:'不等于',symbol:'!='},
  lt:{label:'小于',symbol:'<'},le:{label:'小于等于',symbol:'<='},
  gt:{label:'大于',symbol:'>'},ge:{label:'大于等于',symbol:'>='},
  contains:{label:'包含',symbol:'包含'},
  not_contains:{label:'不包含',symbol:'不包含'}
};
var BOOL_OPTIONS=[
  {value:1,label:'是 / 有效 / 按下'},
  {value:0,label:'否 / 无效 / 松开'}
];
var ROAD_OPTIONS=[
  '未知','丢线','直道','左分支','右分支','T 路口','十字路口',
  '左直角弯','右直角弯'
].map(function(label,value){return{value:value,label:label}});
var HEADING_OPTIONS=[
  '空闲','航向保持','原地转向','定距行驶','静止锁向','移动圆弧'
].map(function(label,value){return{value:value,label:label}});
var LF_OPTIONS=['空闲','标定','循迹'].map(function(label,value){
  return{value:value,label:label};
});
var APP_OPTIONS=['空闲','开发运行','故障','比赛待命','比赛运行']
  .map(function(label,value){return{value:value,label:label}});
var PATH_OPTIONS=[
  {value:1,label:'左'}, {value:2,label:'前'}, {value:4,label:'右'},
  {value:3,label:'左 + 前'}, {value:5,label:'左 + 右'},
  {value:6,label:'前 + 右'}, {value:7,label:'左 + 前 + 右'}
];
var SOURCE_GROUPS={
  constant:'内部',button:'按钮',gray:'灰度与道路',imu:'IMU',
  chassis:'底盘',state:'控制状态'
};
var SOURCES={
  constant:{label:'常量',group:'constant',kind:'bool',hidden:true,
    compares:['eq','ne'],options:BOOL_OPTIONS},
  button1_level:{label:'Button1 当前电平',group:'button',kind:'bool',
    compares:['eq','ne'],options:BOOL_OPTIONS},
  button1_pressed:{label:'Button1 新按下事件',group:'button',kind:'event',
    compares:['eq'],options:[{value:1,label:'发生'}]},
  button2_level:{label:'Button2 当前电平（仅试运行）',group:'button',
    kind:'bool',developmentOnly:true,compares:['eq','ne'],
    options:BOOL_OPTIONS},
  button2_pressed:{label:'Button2 新按下事件（仅试运行）',group:'button',
    kind:'event',developmentOnly:true,compares:['eq'],
    options:[{value:1,label:'发生'}]},
  button3_level:{label:'Button3 当前电平',group:'button',kind:'bool',
    compares:['eq','ne'],options:BOOL_OPTIONS},
  button3_pressed:{label:'Button3 新按下事件',group:'button',kind:'event',
    compares:['eq'],options:[{value:1,label:'发生'}]},
  line_detected:{label:'检测到线路',group:'gray',kind:'bool',
    compares:['eq','ne'],options:BOOL_OPTIONS},
  line_position:{label:'线路位置',group:'gray',kind:'number',
    unit:'mpos',min:-3500,max:3500,step:10,
    compares:['eq','ne','lt','le','gt','ge']},
  line_confidence:{label:'线路置信度',group:'gray',kind:'number',
    unit:'‰',min:0,max:1000,step:10,
    compares:['eq','ne','lt','le','gt','ge']},
  road_type:{label:'当前道路类型',group:'gray',kind:'enum',
    compares:['eq','ne'],options:ROAD_OPTIONS},
  road_event_type:{label:'最近/新道路事件类型',group:'gray',kind:'enum',
    compares:['eq','ne'],options:ROAD_OPTIONS},
  road_event_paths:{label:'道路事件路径',group:'gray',kind:'mask',
    compares:['contains','not_contains'],options:PATH_OPTIONS},
  imu_valid:{label:'IMU 数据有效',group:'imu',kind:'bool',
    compares:['eq','ne'],options:BOOL_OPTIONS},
  imu_yaw:{label:'Yaw',group:'imu',kind:'number',unit:'mdeg',
    min:-180000,max:180000,step:100,
    compares:['eq','ne','lt','le','gt','ge']},
  imu_pitch:{label:'Pitch',group:'imu',kind:'number',unit:'mdeg',
    min:0,max:360000,step:100,
    compares:['eq','ne','lt','le','gt','ge']},
  imu_roll:{label:'Roll',group:'imu',kind:'number',unit:'mdeg',
    min:0,max:360000,step:100,
    compares:['eq','ne','lt','le','gt','ge']},
  imu_gyro_z:{label:'Z 轴角速度',group:'imu',kind:'number',unit:'mdps',
    min:-2000000,max:2000000,step:100,
    compares:['eq','ne','lt','le','gt','ge']},
  feedback_valid:{label:'底盘反馈有效',group:'chassis',kind:'bool',
    compares:['eq','ne'],options:BOOL_OPTIONS},
  left_rpm:{label:'左轮实际转速',group:'chassis',kind:'rpm',
    unit:'RPM',compares:['eq','ne','lt','le','gt','ge']},
  right_rpm:{label:'右轮实际转速',group:'chassis',kind:'rpm',
    unit:'RPM',compares:['eq','ne','lt','le','gt','ge']},
  left_distance:{label:'左轮相对距离',group:'chassis',kind:'number',
    unit:'mm',min:-10000,max:10000,step:1,
    compares:['eq','ne','lt','le','gt','ge']},
  right_distance:{label:'右轮相对距离',group:'chassis',kind:'number',
    unit:'mm',min:-10000,max:10000,step:1,
    compares:['eq','ne','lt','le','gt','ge']},
  average_distance:{label:'平均相对距离',group:'chassis',kind:'number',
    unit:'mm',min:-10000,max:10000,step:1,
    compares:['eq','ne','lt','le','gt','ge']},
  chassis_initialized:{label:'底盘已初始化',group:'state',kind:'bool',
    compares:['eq','ne'],options:BOOL_OPTIONS},
  heading_mode:{label:'航向控制模式',group:'state',kind:'enum',
    compares:['eq','ne'],options:HEADING_OPTIONS},
  linefollow_mode:{label:'循迹控制模式',group:'state',kind:'enum',
    compares:['eq','ne'],options:LF_OPTIONS},
  app_mode:{label:'应用运行模式',group:'state',kind:'enum',
    compares:['eq','ne'],options:APP_OPTIONS}
};
var DEFAULT_COMPARE={
  source:'line_detected',compare:'eq',value:1,
  mode:'instant',timeoutMs:0,stableMs:0
};
var ROAD_ROUTES=[
  {value:'left',code:0,label:'左转'},
  {value:'straight',code:1,label:'直行'},
  {value:'right',code:2,label:'右转'},
  {value:'uturn_left_arc',code:3,label:'左掉头 · 滚动圆弧'},
  {value:'uturn_right_arc',code:4,label:'右掉头 · 滚动圆弧'},
  {value:'uturn_left_pivot',code:5,label:'左掉头 · 停车原地'},
  {value:'uturn_right_pivot',code:6,label:'右掉头 · 停车原地'}
];
var ROAD_DIRECTIONS=[
  {value:'left',label:'左转'},
  {value:'straight',label:'直行'},
  {value:'right',label:'右转'},
  {value:'uturn_left',label:'左掉头'},
  {value:'uturn_right',label:'右掉头'}
];
var ROAD_UTURN_MODES=[
  {value:'arc',label:'滚动圆弧'},
  {value:'pivot',label:'停车原地'}
];
function roadRouteValue(params){
  if(params.direction==='uturn_left'||params.direction==='uturn_right'){
    return params.direction+'_'+params.uturnMode;
  }
  return params.direction;
}
var ACTIONS={
  drive:{op:1,name:'直行',group:'运动',color:'#4f8ee8',
    help:'按有符号转速保持当前航向，可按时间或通用条件结束。',
    defaults:{rpm:80,timeoutMs:2000,completion:'timeout',
      source:'line_detected',compare:'eq',value:1,stableMs:100}},
  drive_mm:{op:8,name:'定距行驶',group:'运动',color:'#36a6c9',
    help:'按编码器距离行驶；正数前进，负数后退。',
    defaults:{distanceMm:300,maxRpm:80}},
  turn:{op:2,name:'转向',group:'运动',color:'#e69855',
    help:'相对转向；正角度左转，负角度右转。',
    defaults:{angleDeg:90,timeoutMs:5000}},
  follow:{op:3,name:'循迹',group:'运动',color:'#5fbf77',
    help:'按灰度传感器循迹，可按时间或通用条件结束。',
    defaults:{rpm:80,timeoutMs:10000,completion:'compare',
      source:'line_detected',compare:'eq',value:0,stableMs:100}},
  wait:{op:4,name:'定时等待',group:'等待与判断',color:'#778195',
    help:'停车并等待指定时间。',defaults:{timeoutMs:500}},
  condition:{op:15,name:'通用判断',group:'等待与判断',color:'#a66ce0',
    help:'立即判断，或停车等待条件持续成立。',
    defaults:DEFAULT_COMPARE},
  loop:{op:18,name:'循环',group:'流程控制',color:'#7b74d6',
    help:'按次数执行循环体；循环体的正常路径必须连回本节点。',
    defaults:{count:2}},
  road_nav:{op:19,name:'循迹通过路口',group:'电赛复合动作',color:'#32b8a0',
    help:'从当前黑线循迹到下一个路口，按指定方向通过并重新捕获黑线。',
    defaults:{direction:'straight',uturnMode:'arc',rpm:80,timeoutMs:15000}},
  dm_position:{op:20,name:'达妙定位',group:'达妙电机',color:'#d08b5b',
    help:'以参考轨迹驱动 DM-G6220 到绝对或相对角度，完成后保持目标位置。',
    defaults:{frame:'relative',angleDeg:0,maxVelocityDegS:11.5,timeoutMs:5000}},
  dm_speed:{op:21,name:'达妙定速',group:'达妙电机',color:'#c46e8f',
    help:'按指定角速度运行一段时间，然后斜坡减速并保持停止位置。',
    defaults:{velocityDegS:11.5,durationMs:1000}},
  dm_disable:{op:22,name:'达妙失能',group:'达妙电机',color:'#8c788d',
    help:'显式释放 DM-G6220；序列结束、取消、急停和故障也会自动失能。',
    defaults:{}},
  ball_hold:{op:23,name:'滚球保持',group:'H题滚球控制',color:'#4db6ac',
    help:'启动滚球闭环并保持目标位置；节点立即完成，闭环在后续巡线动作期间继续运行。',
    defaults:{targetMm:0}},
  ball_move:{op:24,name:'滚球移动',group:'H题滚球控制',color:'#26a69a',
    help:'将钢球平滑移动到目标位置，并等待位置和速度连续稳定后完成。',
    defaults:{targetMm:50,timeoutMs:5000}},
  ball_disable:{op:25,name:'滚球失能',group:'H题滚球控制',color:'#607d8b',
    help:'停止滚球闭环并失能达妙电机。',
    defaults:{}},
  stop:{op:5,name:'停车',group:'流程控制',color:'#df647c',
    help:'停止底盘、航向和循迹控制，然后继续。',defaults:{}},
  end:{op:7,name:'结束',group:'流程控制',color:'#697081',
    help:'成功结束整个序列；这是实际写入固件的动作。',defaults:{}},
  led_on:{op:9,name:'LED 点亮',group:'声光输出',color:'#e3c96f',
    help:'点亮 LED2、LED3 或两者，可设置自动关闭。',
    defaults:{target:0,durationMs:500}},
  led_off:{op:10,name:'LED 熄灭',group:'声光输出',color:'#89909e',
    help:'立即熄灭指定 LED。',defaults:{target:0}},
  led_toggle:{op:11,name:'LED 翻转',group:'声光输出',color:'#d998c5',
    help:'翻转指定 LED，可设置自动关闭。',
    defaults:{target:0,durationMs:500}},
  buzzer_on:{op:12,name:'蜂鸣器开启',group:'声光输出',color:'#e06c75',
    help:'开启蜂鸣器，可设置自动关闭。',defaults:{durationMs:200}},
  buzzer_off:{op:13,name:'蜂鸣器关闭',group:'声光输出',color:'#89909e',
    help:'立即关闭蜂鸣器。',defaults:{}},
  buzzer_toggle:{op:14,name:'蜂鸣器翻转',group:'声光输出',color:'#db8191',
    help:'翻转蜂鸣器，可设置自动关闭。',defaults:{durationMs:200}}
};
var OP_TYPES={
  1:'drive',2:'turn',3:'follow',4:'wait',5:'stop',7:'end',8:'drive_mm',
  9:'led_on',10:'led_off',11:'led_toggle',12:'buzzer_on',
  13:'buzzer_off',14:'buzzer_toggle',15:'condition',16:'drive',17:'follow',
  18:'loop',19:'road_nav',20:'dm_position',21:'dm_speed',22:'dm_disable',
  23:'ball_hold',24:'ball_move',25:'ball_disable'
};

function clone(v){return JSON.parse(JSON.stringify(v))}
function now(){return new Date().toISOString()}
function uid(prefix){
  return(prefix||'n')+'-'+Date.now().toString(36)+'-'+
    Math.random().toString(36).slice(2,8);
}
function actionNode(type,x,y,order,id){
  var a=ACTIONS[type];
  if(!a)throw new Error('未知动作类型：'+type);
  return{id:id||uid('n'),type:type,x:x||0,y:y||0,
    createdOrder:order||1,params:clone(a.defaults)};
}
function newProject(name,slot){
  var t=now();
  return{format:FORMAT,version:VERSION,name:name||'未命名流程',
    slot:slot==null?7:slot,nodes:[
      {id:'start',type:'system_start',x:70,y:220,createdOrder:0,params:{}}
    ],edges:[],createdAt:t,updatedAt:t};
}
function ports(type){
  if(type==='system_start')return['success'];
  if(type==='end')return[];
  return['success','failure'];
}
function legacyCondition(condition,mode,timeoutMs){
  var result=clone(DEFAULT_COMPARE);
  result.mode=mode||'instant';
  result.timeoutMs=Number(timeoutMs)||0;
  result.stableMs=0;
  if(condition==='line_detected'){
    result.source='line_detected';result.compare='eq';result.value=1;
  }else if(condition==='line_lost'){
    result.source='line_detected';result.compare='eq';result.value=0;
  }else if(condition==='button'){
    result.source='button1_pressed';result.compare='eq';result.value=1;
  }else if(condition==='immediate'){
    result.source='constant';result.compare='eq';result.value=1;
  }else{
    result.source='constant';result.compare='eq';result.value=0;
  }
  return result;
}
function migrateV1(input){
  var p=clone(input);
  p.version=VERSION;
  p.nodes.forEach(function(n){
    var params=n.params||{};
    if(n.type==='branch'){
      n.type='condition';
      n.params=legacyCondition(params.condition,'instant',0);
    }else if(n.type==='wait'&&params.condition&&params.condition!=='timeout'){
      n.type='condition';
      n.params=legacyCondition(params.condition,'wait',params.timeoutMs);
    }else if(n.type==='wait'){
      n.params={timeoutMs:Number(params.timeoutMs)||0};
    }else if((n.type==='drive'||n.type==='follow')&&
             params.condition&&params.condition!=='timeout'){
      var c=legacyCondition(params.condition,'wait',params.timeoutMs);
      n.params={rpm:Number(params.rpm)||0,timeoutMs:Number(params.timeoutMs)||0,
        completion:'compare',source:c.source,compare:c.compare,value:c.value,
        stableMs:0};
    }else if(n.type==='drive'||n.type==='follow'){
      n.params={rpm:Number(params.rpm)||0,timeoutMs:Number(params.timeoutMs)||0,
        completion:'timeout',source:'line_detected',compare:'eq',value:1,
        stableMs:0};
    }
  });
  return p;
}
function normalizeProject(input){
  if(!input||input.format!==FORMAT||!Array.isArray(input.nodes)||
     !Array.isArray(input.edges)){
    throw new Error('不是受支持的 gugaPI 序列工程');
  }
  if(input.version===1)input=migrateV1(input);
  if(input.version!==VERSION)throw new Error('不支持的工程版本：'+input.version);
  var p=clone(input),ids={},abortIds={};
  p.nodes.forEach(function(n){
    if(n.type==='system_abort')abortIds[n.id]=1;
  });
  p.nodes=p.nodes.filter(function(n){return n.type!=='system_abort'});
  p.edges=p.edges.filter(function(e){
    return !abortIds[e.source]&&!abortIds[e.target];
  });
  p.nodes.forEach(function(n,i){
    if(!n.id||ids[n.id])throw new Error('节点 ID 缺失或重复');
    ids[n.id]=1;
    if(n.type!=='system_start'&&!ACTIONS[n.type]){
      throw new Error('未知节点类型：'+n.type);
    }
    n.x=Number(n.x)||0;n.y=Number(n.y)||0;
    n.createdOrder=Number(n.createdOrder)||i;n.params=n.params||{};
    if(ACTIONS[n.type]){
      n.params=Object.assign(clone(ACTIONS[n.type].defaults),n.params);
    }
  });
  if(!ids.start)throw new Error('工程缺少开始系统节点');
  p.edges.forEach(function(e){
    if(!e.id)e.id=uid('e');
    if(!ids[e.source]||!ids[e.target]){
      throw new Error('连线引用了不存在的节点');
    }
    var sourceNode=p.nodes.find(function(n){return n.id===e.source});
    if(ports(sourceNode.type).indexOf(e.port)<0)throw new Error('连线端口无效');
  });
  p.name=String(p.name||'未命名流程');
  p.slot=Math.max(0,Math.min(7,Number(p.slot)||0));
  p.updatedAt=p.updatedAt||now();p.createdAt=p.createdAt||p.updatedAt;
  return p;
}
function addIssue(list,severity,code,message,nodeId,field){
  list.push({severity:severity,code:code,message:message,
    nodeId:nodeId||null,field:field||null});
}
function validateCompareParams(p,node,issues,options,forceWait){
  var source=SOURCES[p.source];
  if(!source){
    addIssue(issues,'error','condition_source','未知判断数据源',
      node.id,'source');return;
  }
  if(source.compares.indexOf(p.compare)<0){
    addIssue(issues,'error','condition_compare','比较方式不适用于该数据源',
      node.id,'compare');
  }
  var value=Number(p.value);
  var sourceMin=source.kind==='rpm'?-(options.maxRpm||1000):source.min;
  var sourceMax=source.kind==='rpm'?(options.maxRpm||1000):source.max;
  if(source.options){
    if(!source.options.some(function(x){return Number(x.value)===value})){
      addIssue(issues,'error','condition_value','判断值不在允许范围内',
        node.id,'value');
    }
  }else if(!Number.isFinite(value)||value<sourceMin||value>sourceMax){
    addIssue(issues,'error','condition_value',
      '判断值范围应为 '+sourceMin+'～'+sourceMax,node.id,'value');
  }
  var mode=forceWait?'wait':p.mode;
  if(mode!=='instant'&&mode!=='wait'){
    addIssue(issues,'error','condition_mode','判断模式无效',node.id,'mode');
  }
  var stable=Number(p.stableMs);
  if(!Number.isInteger(stable)||stable<0||stable>1000||stable%50!==0){
    addIssue(issues,'error','condition_stable',
      '连续成立时间应为 0～1000 ms，步进 50 ms',node.id,'stableMs');
  }
  if(source.kind==='event'&&stable!==0){
    addIssue(issues,'error','event_stable',
      '按钮事件不能设置连续成立时间',node.id,'stableMs');
  }
  if(mode==='wait'){
    var timeout=Number(p.timeoutMs);
    if(!Number.isInteger(timeout)||timeout<50||timeout>30000||
       timeout%50!==0){
      addIssue(issues,'error','condition_timeout',
        '条件超时应为 50～30000 ms，步进 50 ms',node.id,'timeoutMs');
    }
  }
  if(options.competition&&source.developmentOnly){
    addIssue(issues,'error','competition_button2',
      'Button2 在比赛模式固定用于停止任务，不能保存为比赛条件',
      node.id,'source');
  }
}
function validateParams(n,maxRpm,issues,options){
  var p=n.params||{},t=n.type,a=ACTIONS[t];
  if(!a)return;
  function range(key,min,max,label,nonzero){
    var v=Number(p[key]);
    if(!Number.isFinite(v)||v<min||v>max){
      addIssue(issues,'error','range',label+'范围应为 '+min+'～'+max,
        n.id,key);
    }else if(nonzero&&v===0){
      addIssue(issues,'error','zero',label+'不能为 0',n.id,key);
    }
  }
  function duration(key,allowZero){
    range(key,allowZero?0:1,30000,'时间',false);
  }
  if(t==='drive'||t==='follow'){
    range('rpm',-maxRpm,maxRpm,'转速',false);
    if(Number(p.rpm)===0)addIssue(issues,'warning','zero_speed',
      '转速为 0，不会产生有效运动',n.id,'rpm');
    if(p.completion==='compare'){
      validateCompareParams(p,n,issues,options,true);
    }else if(p.completion==='timeout'){
      duration('timeoutMs',false);
    }else{
      addIssue(issues,'error','completion','动作完成方式无效',
        n.id,'completion');
    }
  }else if(t==='drive_mm'){
    range('distanceMm',-10000,10000,'距离',true);
    range('maxRpm',1,maxRpm,'最大转速',false);
  }else if(t==='turn'){
    range('angleDeg',-180,180,'角度',false);duration('timeoutMs',false);
    if(Number(p.angleDeg)===0)addIssue(issues,'warning','zero_angle',
      '转角为 0，没有实际意义',n.id,'angleDeg');
  }else if(t==='wait'){
    duration('timeoutMs',true);
    if(Number(p.timeoutMs)===0)addIssue(issues,'warning','zero_wait',
      '等待时间为 0，将立即完成',n.id,'timeoutMs');
  }else if(t==='condition'){
    validateCompareParams(p,n,issues,options,false);
    if(p.mode==='instant'&&
       (Number(p.timeoutMs)!==0||Number(p.stableMs)!==0)){
      addIssue(issues,'warning','instant_timing',
        '立即判断不会使用超时或连续成立时间',n.id,'mode');
    }
  }else if(t==='loop'){
    var count=Number(p.count);
    if(!Number.isInteger(count)||count<1||count>1000){
      addIssue(issues,'error','loop_count',
        '循环次数应为 1～1000 的整数；1 表示循环体执行一次',
        n.id,'count');
    }
  }else if(t==='road_nav'){
    if(!ROAD_DIRECTIONS.some(function(x){return x.value===p.direction})){
      addIssue(issues,'error','road_route','路口路线无效',
        n.id,'direction');
    }
    if((p.direction==='uturn_left'||p.direction==='uturn_right')&&
       !ROAD_UTURN_MODES.some(function(x){return x.value===p.uturnMode})){
      addIssue(issues,'error','road_uturn_mode','掉头方式无效',
        n.id,'uturnMode');
    }
    range('rpm',1,maxRpm,'循迹转速',false);
    var routeTimeout=Number(p.timeoutMs);
    if(!Number.isFinite(routeTimeout)||routeTimeout<50||
       routeTimeout>30000||(routeTimeout%50)!==0){
      addIssue(issues,'error','road_timeout',
        '整体超时应为 50～30000 ms，步进 50 ms',
        n.id,'timeoutMs');
    }
  }else if(t==='dm_position'){
    if(p.frame!=='absolute'&&p.frame!=='relative'){
      addIssue(issues,'error','dm_frame','定位方式必须为绝对或相对',
        n.id,'frame');
    }
    range('angleDeg',-716.2,716.2,'目标角度',false);
    range('maxVelocityDegS',0.1,1145.9,'轨迹速度',false);
    var dmPositionTimeout=Number(p.timeoutMs);
    if(!Number.isInteger(dmPositionTimeout)||dmPositionTimeout<50||
       dmPositionTimeout>30000||dmPositionTimeout%50!==0){
      addIssue(issues,'error','dm_position_timeout',
        '定位超时应为 50～30000 ms，步进 50 ms',n.id,'timeoutMs');
    }
  }else if(t==='dm_speed'){
    range('velocityDegS',-1145.9,1145.9,'角速度',true);
    var dmDuration=Number(p.durationMs);
    if(!Number.isInteger(dmDuration)||dmDuration<50||
       dmDuration>30000||dmDuration%50!==0){
      addIssue(issues,'error','dm_speed_duration',
        '定速持续时间应为 50～30000 ms，步进 50 ms',
        n.id,'durationMs');
    }
  }else if(t==='ball_hold'||t==='ball_move'){
    range('targetMm',-100,100,'滚球目标位置',false);
    if(t==='ball_move'){
      var ballTimeout=Number(p.timeoutMs);
      if(!Number.isInteger(ballTimeout)||ballTimeout<50||
         ballTimeout>30000||ballTimeout%50!==0){
        addIssue(issues,'error','ball_move_timeout',
          '滚球移动超时应为 50～30000 ms，步进 50 ms',
          n.id,'timeoutMs');
      }
    }
  }else if(t.indexOf('led_')===0){
    var target=Number(p.target);
    if([0,2,3].indexOf(target)<0)addIssue(issues,'error','target',
      'LED 目标只能是 LED2、LED3 或两者',n.id,'target');
    if(t!=='led_off'){
      var d=Number(p.durationMs);
      if(!Number.isFinite(d)||d<0||d>30000||(d>0&&d<50)){
        addIssue(issues,'error','duration',
          '自动关闭时间应为 0 或 50～30000 ms',n.id,'durationMs');
      }
    }
  }else if(t==='buzzer_on'||t==='buzzer_toggle'){
    var bd=Number(p.durationMs);
    if(!Number.isFinite(bd)||bd<0||bd>30000||(bd>0&&bd<50)){
      addIssue(issues,'error','duration',
        '自动关闭时间应为 0 或 50～30000 ms',n.id,'durationMs');
    }
  }
}
function validate(project,options){
  options=options||{};
  var p=normalizeProject(project),maxRpm=options.maxRpm||1000;
  var issues=[],nodesBy={},out={};
  p.nodes.forEach(function(n){nodesBy[n.id]=n;out[n.id]={}});
  var actions=p.nodes.filter(function(n){return ACTIONS[n.type]});
  if(actions.length===0)addIssue(issues,'error','empty',
    '画布中没有可执行动作');
  if(actions.length>54)addIssue(issues,'error','too_many',
    '动作数量超过固件上限 54 步');
  p.edges.forEach(function(e){
    if(out[e.source][e.port])addIssue(issues,'error','duplicate_port',
      '同一输出端口只能有一条连线',e.source,e.port);
    else out[e.source][e.port]=e;
    if(e.source===e.target)addIssue(issues,'error','self_loop',
      '不允许节点直接连接自身',e.source,e.port);
  });
  var startEdge=out.start&&out.start.success;
  if(!startEdge)addIssue(issues,'error','missing_start',
    '开始节点必须连接第一个动作','start','success');
  else if(!ACTIONS[nodesBy[startEdge.target]&&nodesBy[startEdge.target].type]){
    addIssue(issues,'error','bad_start','开始节点必须连接实际动作',
      'start','success');
  }
  actions.forEach(function(n){
    validateParams(n,maxRpm,issues,options);
    if(n.type!=='end'&&!out[n.id].success)addIssue(issues,'error',
      'missing_success',n.type==='condition'?'判断的“成立”端口未连接':
      n.type==='loop'?'循环的“执行循环体”端口未连接':
      '“完成”端口必须连接后续动作',n.id,'success');
    if(n.type==='loop'&&!out[n.id].failure){
      addIssue(issues,'error','missing_failure',
        '循环的“循环完成”端口未连接',n.id,'failure');
    }
    if(out[n.id].success&&
       !ACTIONS[nodesBy[out[n.id].success.target].type]){
      addIssue(issues,'error','success_target',
        '成功端口必须连接实际动作',n.id,'success');
    }
    if(out[n.id].failure&&
       !ACTIONS[nodesBy[out[n.id].failure.target].type]){
      addIssue(issues,'error','failure_target',
        '失败端口只能连接实际动作',n.id,'failure');
    }
    if(n.type==='loop'&&out[n.id].failure&&
       !ACTIONS[nodesBy[out[n.id].failure.target].type]){
      addIssue(issues,'error','loop_done_target',
        '循环完成端口必须连接实际动作',n.id,'failure');
    }
  });
  var reachable={},q=[];
  if(startEdge)q.push(startEdge.target);
  while(q.length){
    var id=q.shift();
    if(reachable[id]||!nodesBy[id])continue;
    reachable[id]=true;
    var o=out[id]||{};
    ['success','failure'].forEach(function(k){
      if(o[k]&&nodesBy[o[k].target])q.push(o[k].target);
    });
  }
  actions.forEach(function(n){
    if(!reachable[n.id])addIssue(issues,'error','unreachable',
      '节点无法从开始节点到达',n.id);
  });
  if(!actions.some(function(n){return n.type==='end'&&reachable[n.id]})){
    addIssue(issues,'error','missing_end',
      '流程必须包含一个可达的“结束”动作');
  }
  var loopBackEdges={},hasLoop=actions.some(function(n){
    return n.type==='loop';
  });
  actions.filter(function(n){return n.type==='loop'}).forEach(function(loop){
    var body=out[loop.id]&&out[loop.id].success;
    var done=out[loop.id]&&out[loop.id].failure;
    if(!body||!done)return;
    var found=false,seenBody={},stack=[body.target],backKeys=[];
    while(stack.length){
      var id=stack.pop();
      if(id===done.target||seenBody[id])continue;
      if(id===loop.id){found=true;continue}
      seenBody[id]=1;
      var bodyOut=out[id]||{};
      ['success','failure'].forEach(function(port){
        var edge=bodyOut[port];
        if(!edge)return;
        if(edge.target===loop.id){
          found=true;
          backKeys.push(edge.source+'|'+edge.port);
        }else if(edge.target!==done.target){
          stack.push(edge.target);
        }
      });
    }
    if(!found){
      addIssue(issues,'error','loop_no_return',
        '循环体入口不存在不经过“循环完成”出口并返回循环节点的路径',
        loop.id,'success');
    }else{
      backKeys.forEach(function(key){loopBackEdges[key]=1});
    }
  });
  var color={},cycle=false;
  function visit(id){
    if(color[id]===1){cycle=true;return}
    if(color[id]===2)return;
    color[id]=1;
    var o=out[id]||{};
    ['success','failure'].forEach(function(k){
      if(o[k]&&!loopBackEdges[id+'|'+k]&&
         ACTIONS[nodesBy[o[k].target]&&nodesBy[o[k].target].type]){
        visit(o[k].target);
      }
    });
    color[id]=2;
  }
  if(startEdge)visit(startEdge.target);
  if(cycle)addIssue(issues,'warning','cycle',
    '流程包含未计数回环；固件会在总运行 300 秒时强制中止');
  if(hasLoop)addIssue(issues,'warning','loop_runtime_limit',
    '包含计数循环；整条序列运行仍受 300 秒上限保护');
  var sum=0;
  actions.forEach(function(n){
    var p2=n.params||{};
    if(['drive','turn','follow','wait','condition','road_nav',
        'dm_position','dm_speed','ball_move']
       .indexOf(n.type)>=0){
      if(n.type!=='condition'||p2.mode==='wait'){
        sum+=Math.max(0,Number(
          n.type==='dm_speed'?p2.durationMs:p2.timeoutMs)||0);
      }
    }
  });
  if(!hasLoop&&!cycle&&sum>300000)addIssue(issues,'error','global_timeout',
    '各步骤时间上限合计超过固件 300 秒全局上限');
  return{valid:!issues.some(function(i){return i.severity==='error'}),
    issues:issues,reachable:reachable,outgoing:out,nodesById:nodesBy};
}
function compareRaw(op,p,type){
  var instant=type==='condition'&&p.mode==='instant';
  return{op:op,p1:type==='condition'?0:+p.rpm,p2:0,until:-1,
    source:p.source,compare:p.compare,value:+p.value,
    mode:type==='condition'?p.mode:'wait',
    timeoutMs:instant?0:+p.timeoutMs,
    stableMs:instant?0:+p.stableMs,ons:ABORT,ont:ABORT};
}
function rawFor(n){
  var p=n.params||{},type=n.type,a=ACTIONS[type];
  var r={op:a.op,p1:0,p2:0,until:5,conditionValue:0,
    ons:ABORT,ont:ABORT};
  if(type==='drive'){
    if(p.completion==='compare')return compareRaw(16,p,type);
    r.p1=+p.rpm;r.p2=+p.timeoutMs;r.until=0;
  }else if(type==='drive_mm'){
    r.p1=+p.distanceMm;r.p2=+p.maxRpm;r.until=6;
  }else if(type==='turn'){
    r.p1=+p.angleDeg;r.p2=+p.timeoutMs;r.until=1;
  }else if(type==='follow'){
    if(p.completion==='compare')return compareRaw(17,p,type);
    r.p1=+p.rpm;r.p2=+p.timeoutMs;r.until=0;
  }else if(type==='wait'){
    r.p2=+p.timeoutMs;r.until=0;
  }else if(type==='condition'){
    return compareRaw(15,p,type);
  }else if(type==='loop'){
    r.p1=+p.count;r.p2=0;r.until=5;
  }else if(type==='road_nav'){
    var routeValue=roadRouteValue(p);
    var route=ROAD_ROUTES.find(function(x){return x.value===routeValue});
    r.p1=+p.rpm;r.p2=+p.timeoutMs;r.until=5;
    r.conditionValue=route?route.code:-1;
    r.route=routeValue;
  }else if(type==='dm_position'){
    r.p1=Math.round(Number(p.angleDeg)*Math.PI*1000/180);
    r.p2=Math.round(Number(p.maxVelocityDegS)*Math.PI*1000/180);
    r.until=p.frame==='relative'?8:7;
    r.conditionValue=+p.timeoutMs;
    r.frame=p.frame;
  }else if(type==='dm_speed'){
    r.p1=Math.round(Number(p.velocityDegS)*Math.PI*1000/180);
    r.p2=+p.durationMs;
    r.until=5;
  }else if(type==='ball_hold'){
    r.p1=Math.round(Number(p.targetMm)*10);
    r.p2=0;
    r.until=5;
  }else if(type==='ball_move'){
    r.p1=Math.round(Number(p.targetMm)*10);
    r.p2=+p.timeoutMs;
    r.until=5;
  }else if(type.indexOf('led_')===0){
    r.p1=+p.target||0;r.p2=type==='led_off'?0:(+p.durationMs||0);
  }else if(type==='buzzer_on'||type==='buzzer_toggle'){
    r.p2=+p.durationMs||0;
  }
  return r;
}
function compile(project,options){
  var p=normalizeProject(project),v=validate(p,options);
  if(!v.valid){
    var e=new Error(v.issues.filter(function(i){return i.severity==='error'})
      .map(function(i){return i.message}).join('；'));
    e.issues=v.issues;throw e;
  }
  var start=v.outgoing.start.success.target,order=[],seen={},q=[start];
  while(q.length){
    var id=q.shift();
    if(seen[id]||!ACTIONS[v.nodesById[id].type])continue;
    seen[id]=1;order.push(id);
    var o=v.outgoing[id]||{};
    ['success','failure'].forEach(function(k){
      if(o[k]&&ACTIONS[v.nodesById[o[k].target]&&
         v.nodesById[o[k].target].type]&&!seen[o[k].target]){
        q.push(o[k].target);
      }
    });
  }
  var index={};order.forEach(function(id,i){index[id]=i});
  var instrs=order.map(function(id){
    var n=v.nodesById[id],r=rawFor(n),o=v.outgoing[id]||{};
    if(n.type!=='end'){
      r.ons=index[o.success.target];
      r.ont=o.failure?index[o.failure.target]:ABORT;
    }
    return r;
  });
  return{instrs:instrs,nodeOrder:order,indexByNode:index,issues:v.issues};
}
function paramsFromRaw(type,r){
  if(r.op===16||r.op===17){
    return{rpm:r.p1,timeoutMs:r.timeoutMs,completion:'compare',
      source:r.source,compare:r.compare,value:r.value,stableMs:r.stableMs};
  }
  if(type==='drive')return{rpm:r.p1,timeoutMs:r.p2,completion:'timeout',
    source:'line_detected',compare:'eq',value:1,stableMs:0};
  if(type==='drive_mm')return{distanceMm:r.p1,maxRpm:r.p2};
  if(type==='turn')return{angleDeg:r.p1,timeoutMs:r.p2};
  if(type==='follow')return{rpm:r.p1,timeoutMs:r.p2,completion:'timeout',
    source:'line_detected',compare:'eq',value:0,stableMs:0};
  if(type==='wait')return{timeoutMs:r.p2};
  if(type==='condition')return{source:r.source,compare:r.compare,
    value:r.value,mode:r.mode,timeoutMs:r.timeoutMs,stableMs:r.stableMs};
  if(type==='loop')return{count:r.p1};
  if(type==='road_nav'){
    var route=ROAD_ROUTES.find(function(x){
      return x.value===r.route||x.code===Number(r.conditionValue);
    });
    var value=route?route.value:'straight';
    var pivot=value.indexOf('_pivot')>=0;
    return{
      direction:value.replace(/_(arc|pivot)$/,''),
      uturnMode:pivot?'pivot':'arc',
      rpm:r.p1,
      timeoutMs:r.p2
    };
  }
  if(type==='dm_position'){
    return{
      frame:Number(r.until)===8?'relative':'absolute',
      angleDeg:Number((Number(r.p1)*180/(Math.PI*1000)).toFixed(3)),
      maxVelocityDegS:Number((Number(r.p2)*180/(Math.PI*1000)).toFixed(3)),
      timeoutMs:Number(r.conditionValue)
    };
  }
  if(type==='dm_speed'){
    return{
      velocityDegS:Number((Number(r.p1)*180/(Math.PI*1000)).toFixed(3)),
      durationMs:Number(r.p2)
    };
  }
  if(type==='ball_hold'){
    return{targetMm:Number((Number(r.p1)/10).toFixed(1))};
  }
  if(type==='ball_move'){
    return{targetMm:Number((Number(r.p1)/10).toFixed(1)),
      timeoutMs:Number(r.p2)};
  }
  if(type.indexOf('led_')===0){
    var p={target:r.p1};if(type!=='led_off')p.durationMs=r.p2;return p;
  }
  if(type==='buzzer_on'||type==='buzzer_toggle'){
    return{durationMs:r.p2};
  }
  return{};
}
function legacyRawToV2(r){
  if(r.op!==6)return r;
  var condition=legacyCondition(CONDS[r.until],'instant',0);
  return{op:15,p1:0,p2:0,until:-1,source:condition.source,
    compare:condition.compare,value:condition.value,mode:'instant',
    timeoutMs:0,stableMs:0,ons:r.ons,ont:r.ont};
}
function autoLayout(project){
  var p=project,by={};p.nodes.forEach(function(n){by[n.id]=n});
  if(by.start){by.start.x=60;by.start.y=220}
  var actions=p.nodes.filter(function(n){return ACTIONS[n.type]})
    .sort(function(a,b){return a.createdOrder-b.createdOrder});
  actions.forEach(function(n,i){
    n.x=280+(i%4)*230;n.y=100+Math.floor(i/4)*190;
  });
  return p;
}
function decompile(instrs,meta){
  var normalized=(instrs||[]).map(legacyRawToV2);
  var p=newProject(meta&&meta.name,meta&&meta.slot),nodes=[];
  normalized.forEach(function(r,i){
    var type=OP_TYPES[r.op];
    if(!type)throw new Error('未知固件操作码：'+r.op);
    var n=actionNode(type,0,0,i+1,'n'+i);
    n.params=paramsFromRaw(type,r);nodes.push(n);
  });
  p.nodes=p.nodes.concat(nodes);
  if(nodes[0])p.edges.push({id:uid('e'),source:'start',
    port:'success',target:nodes[0].id});
  nodes.forEach(function(n,i){
    var r=normalized[i];if(n.type==='end')return;
    var s=r.ons===ABORT?i+1:r.ons;
    if(s<nodes.length)p.edges.push({id:uid('e'),source:n.id,
      port:'success',target:nodes[s].id});
    var f=r.ont;
    if(f!==ABORT&&f<nodes.length)p.edges.push({id:uid('e'),source:n.id,
      port:'failure',target:nodes[f].id});
  });
  return autoLayout(p);
}
function connect(p,source,port,target){
  p.edges=p.edges.filter(function(e){
    return!(e.source===source&&e.port===port);
  });
  p.edges.push({id:uid('e'),source:source,port:port,target:target});
  p.updatedAt=now();return p;
}
function chain(types,name){
  var p=newProject(name,7),prev='start';
  types.forEach(function(spec,i){
    var type=typeof spec==='string'?spec:spec.type;
    var n=actionNode(type,0,0,i+1,'n'+i);
    if(typeof spec==='object')Object.assign(n.params,spec.params||{});
    p.nodes.push(n);connect(p,prev,'success',n.id);
    prev=n.id;
  });
  return autoLayout(p);
}
function templates(){
  var a=chain([
    {type:'drive_mm',params:{distanceMm:500,maxRpm:80}},
    {type:'turn',params:{angleDeg:90,timeoutMs:5000}},'stop','end'
  ],'定距直行—左转—停车');
  var b=chain([
    {type:'follow',params:{rpm:80,timeoutMs:30000,completion:'compare',
      source:'line_detected',compare:'eq',value:0,stableMs:100}},
    'stop','end'
  ],'循迹直到丢线');
  var c=newProject('道路类型通用判断与失败恢复',7);
  var ns=[
    actionNode('condition',260,220,1,'n0'),
    actionNode('drive_mm',520,100,2,'n1'),
    actionNode('stop',520,340,3,'n2'),
    actionNode('end',780,220,4,'n3')
  ];
  ns[0].params={source:'road_type',compare:'eq',value:6,
    mode:'wait',timeoutMs:5000,stableMs:100};
  c.nodes=c.nodes.concat(ns);
  connect(c,'start','success','n0');connect(c,'n0','success','n1');
  connect(c,'n0','failure','n2');connect(c,'n1','success','n3');
  connect(c,'n1','failure','n2');connect(c,'n2','success','n3');
  var d=chain([
    {type:'led_on',params:{target:0,durationMs:500}},
    {type:'buzzer_on',params:{durationMs:200}},
    {type:'drive_mm',params:{distanceMm:200,maxRpm:60}},'stop','end'
  ],'声光提示后启动运动');
  return[a,b,c,d];
}
function serialize(p){
  var x=normalizeProject(p);x.updatedAt=now();
  return JSON.stringify(x,null,2);
}
return{
  FORMAT:FORMAT,VERSION:VERSION,ABORT:ABORT,CONDS:CONDS,
  COND_LABELS:COND_LABELS,COMPARES:COMPARES,SOURCES:SOURCES,
  SOURCE_GROUPS:SOURCE_GROUPS,ACTIONS:ACTIONS,OP_TYPES:OP_TYPES,
  ROAD_ROUTES:ROAD_ROUTES,ROAD_DIRECTIONS:ROAD_DIRECTIONS,
  ROAD_UTURN_MODES:ROAD_UTURN_MODES,
  clone:clone,uid:uid,newProject:newProject,actionNode:actionNode,
  ports:ports,normalizeProject:normalizeProject,validate:validate,
  compile:compile,decompile:decompile,autoLayout:autoLayout,
  connect:connect,templates:templates,serialize:serialize,
  migrateV1:migrateV1
};
}));
