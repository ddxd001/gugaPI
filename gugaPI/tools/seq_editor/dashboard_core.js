'use strict';

(function(root,factory){
  var api=factory();
  if(typeof module==='object'&&module.exports)module.exports=api;
  root.DashboardCore=api;
})(typeof globalThis!=='undefined'?globalThis:this,function(){
  var DEFAULT_FIELDS=[
    't','mode','step','L_tgt','L_act','R_tgt','R_act','yaw_tgt','yaw',
    'head_err','head_corr','gray_pos','gray_strength','gray_conf','gray_valid',
    'gray_state','lf_err','lf_corr','lf_weak','lf_invalid','road_type',
    'road_event_seq','road_event_type','road_paths','road_phase','road_ctrl_phase',
    'head_turn_phase','head_turn_rate_mdps','head_turn_brake_mdeg',
    'head_turn_brake_ms','head_turn_margin_mdeg','head_turn_settle_mdps',
    'head_turn_settle_rpm','fault_code','fault_count','comp_slot',
    'comp_slot_valid','comp_count','chassis_init','chassis_status',
    'feedback_status','feedback_valid','feedback_age_ms','tx_pending','tx_dropped',
    'imu_valid','imu_age_ms','imu_error_count','acc_x_mg','acc_y_mg','acc_z_mg',
    'gyro_x_mdps','gyro_y_mdps','gyro_z_mdps','pitch','roll','imu_temp_cc',
    'gray_sample_valid','gray_age_ms','gray_error_count','gray0','gray1','gray2',
    'gray3','gray4','gray5','gray6','gray7'
  ];
  var COLORS=['#55bfe9','#55c68a','#e4b657','#df8352','#ae84c6',
    '#df6670','#348ed1','#8fbcbb'];
  function s(field,label,unit,description,options){
    var result={field:field,label:label,unit:unit,description:description};
    options=options||{};
    Object.keys(options).forEach(function(key){result[key]=options[key]});
    return result;
  }
  function chart(category,key,title,subtitle,unit,series){
    series.forEach(function(item,index){
      if(!item.color)item.color=COLORS[index%COLORS.length];
    });
    return{category:category,key:key,title:title,subtitle:subtitle,
      unit:unit,series:series};
  }

  var CATEGORIES=[
    {key:'runtime',label:'运行'},
    {key:'chassis',label:'底盘'},
    {key:'line',label:'循迹与道路'},
    {key:'turn',label:'转向过程'},
    {key:'imu',label:'IMU'},
    {key:'gray',label:'灰度传感器'},
    {key:'system',label:'系统健康'}
  ];
  var CHARTS=[
    chart('runtime','runtime','运行状态','模式与序列步骤','状态',[
      s('mode','运行模式','状态码','应用当前运行模式。',{step:true,enumType:'appMode'}),
      s('step','当前序列步骤','步骤','ActionRunner 当前步骤；-1 表示没有运行。',{step:true})]),
    chart('runtime','competition','比赛序列','槽位与指令数量','计数',[
      s('comp_slot','比赛槽位','槽位','当前选择的 FRAM 序列槽。',{step:true}),
      s('comp_slot_valid','槽位有效','0/1','当前槽位是否包含有效序列。',{step:true,enumType:'boolean'}),
      s('comp_count','指令数量','步','当前比赛序列的动作数量。',{step:true})]),

    chart('chassis','motor','车轮转速','目标与实际转速','RPM',[
      s('L_tgt','左轮目标转速','RPM','底盘控制器下发给左轮的目标转速。',{dashed:true}),
      s('L_act','左轮实际转速','RPM','MotorDriver 周期反馈的左轮实测转速。'),
      s('R_tgt','右轮目标转速','RPM','底盘控制器下发给右轮的目标转速。',{dashed:true}),
      s('R_act','右轮实际转速','RPM','MotorDriver 周期反馈的右轮实测转速。')]),
    chart('chassis','heading','航向角','目标、实际与误差','deg',[
      s('yaw_tgt','目标航向角','deg','航向控制器当前锁定或转向的目标角度。',{dashed:true}),
      s('yaw','实际航向角','deg','IMU 积分得到的当前相对航向角。'),
      s('head_err','航向误差','deg','归一化后的目标角与实际角之差。')]),
    chart('chassis','heading_output','航向修正','航向闭环输出','RPM',[
      s('head_corr','航向修正量','RPM','航向闭环施加到左右轮的差速修正量。')]),
    chart('chassis','chassis','底盘状态','初始化与最近状态','状态码',[
      s('chassis_init','底盘已初始化','0/1','底盘控制模块是否完成初始化。',{step:true,enumType:'boolean'}),
      s('chassis_status','底盘最近状态','状态码','最近一次底盘操作返回的 DriverStatus。',{step:true,enumType:'driverStatus'})]),
    chart('chassis','feedback_state','电机反馈状态','反馈有效性与返回状态','状态码',[
      s('feedback_status','反馈返回状态','状态码','最近一次 MotorDriver 周期反馈的 DriverStatus。',{step:true,enumType:'driverStatus'}),
      s('feedback_valid','反馈有效','0/1','是否至少收到过一帧有效反馈。',{step:true,enumType:'boolean'})]),
    chart('chassis','feedback_age','电机反馈年龄','距离最近反馈的时间','ms',[
      s('feedback_age_ms','反馈年龄','ms','当前时刻距离最近有效电机反馈的时间。')]),

    chart('line','line_position','循迹位置','线路位置与控制误差','位置单位',[
      s('gray_pos','灰度线位置','pos','八路灰度插值得到的赛道中心位置，左正右负。'),
      s('lf_err','循迹位置误差','mpos','循迹控制器使用的中心位置误差。')]),
    chart('line','line_output','循迹修正','循迹闭环输出','RPM',[
      s('lf_corr','循迹修正量','RPM','循迹闭环输出的左右轮差速修正量。')]),
    chart('line','line_quality','线路质量','强度与位置置信度','原始值',[
      s('gray_strength','线路强度','原始值','灰度处理得到的线路总体强度。'),
      s('gray_conf','位置置信度','原始值','插值位置结果的置信度。')]),
    chart('line','line_state','线路状态','有效性与识别状态','状态码',[
      s('gray_valid','位置有效','0/1','当前灰度插值位置是否有效。',{step:true,enumType:'boolean'}),
      s('gray_state','线路识别状态','状态码','当前线路为有效、丢失、多线、宽线或传感器故障。',{step:true,enumType:'trackState'})]),
    chart('line','line_faults','循迹异常帧','弱跟踪与无效帧计数','帧',[
      s('lf_weak','弱跟踪帧数','帧','循迹控制器累计的弱跟踪帧数。',{step:true}),
      s('lf_invalid','无效帧数','帧','循迹控制器累计的无效线路帧数。',{step:true})]),
    chart('line','road_event','道路事件','类型、事件与可行方向','状态码',[
      s('road_type','当前道路类型','状态码','灰度算法当前判断的道路形态。',{step:true,enumType:'roadType'}),
      s('road_event_type','最近事件类型','状态码','最近锁存的自动路口事件类型。',{step:true,enumType:'roadType'}),
      s('road_paths','可行方向位图','位图','左、前、右可行方向位图。',{step:true,enumType:'roadPaths'})]),
    chart('line','road_sequence','道路事件序号','事件变化计数','次',[
      s('road_event_seq','道路事件序号','次','每次产生新道路事件时递增的序号。',{step:true})]),
    chart('line','road_phase','道路处理阶段','识别与控制状态机','状态码',[
      s('road_phase','道路识别阶段','状态码','道路识别状态机当前阶段。',{step:true,enumType:'roadPhase'}),
      s('road_ctrl_phase','道路控制阶段','状态码','自动路口控制器当前阶段。',{step:true})]),

    chart('turn','turn_phase','转向阶段','航向转弯状态机','状态码',[
      s('head_turn_phase','转向阶段','状态码','转向控制的驱动、刹车或稳定阶段。',{step:true,enumType:'turnPhase'})]),
    chart('turn','turn_rate','转向角速度','实际角速度与稳定阈值','mdps',[
      s('head_turn_rate_mdps','转向角速度','mdps','当前用于转向判定的角速度。'),
      s('head_turn_settle_mdps','稳定角速度阈值','mdps','进入稳定判定所要求的角速度上限。',{dashed:true})]),
    chart('turn','turn_angle','转向刹车角','刹车角与提前量','mdeg',[
      s('head_turn_brake_mdeg','刹车起始角','mdeg','当前转向计算出的刹车起始角度。'),
      s('head_turn_margin_mdeg','刹车提前量','mdeg','转向目标前的刹车角度裕量。',{dashed:true})]),
    chart('turn','turn_timing','转向刹车时间','刹车阶段持续时间','ms',[
      s('head_turn_brake_ms','刹车时间','ms','航向转向的主动刹车持续时间。')]),
    chart('turn','turn_speed','转向稳定速度','稳定阶段轮速','RPM',[
      s('head_turn_settle_rpm','稳定阶段转速','RPM','转向接近目标时使用的低速轮速。')]),

    chart('imu','accel','线加速度','IMU 三轴加速度','mg',[
      s('acc_x_mg','X 轴加速度','mg','IMU X 轴去偏置后的线加速度。'),
      s('acc_y_mg','Y 轴加速度','mg','IMU Y 轴去偏置后的线加速度。'),
      s('acc_z_mg','Z 轴加速度','mg','IMU Z 轴去偏置后的线加速度，静止时包含重力。')]),
    chart('imu','gyro','角速度','IMU 三轴角速度','mdps',[
      s('gyro_x_mdps','X 轴角速度','mdps','IMU X 轴去偏置后的角速度。'),
      s('gyro_y_mdps','Y 轴角速度','mdps','IMU Y 轴去偏置后的角速度。'),
      s('gyro_z_mdps','Z 轴角速度','mdps','IMU Z 轴去偏置后的角速度，用于航向积分。')]),
    chart('imu','attitude','车体姿态','俯仰角与横滚角','deg',[
      s('pitch','俯仰角','deg','IMU 融合得到的车体俯仰角。'),
      s('roll','横滚角','deg','IMU 融合得到的车体横滚角。')]),
    chart('imu','imu_temperature','IMU 温度','芯片内部温度','0.01 °C',[
      s('imu_temp_cc','IMU 温度裸值','0.01 °C','IMU 温度的摄氏度百分之一裸值。')]),
    chart('imu','imu_state','IMU 健康状态','有效性与错误累计','状态/次',[
      s('imu_valid','IMU 数据有效','0/1','当前 IMU 缓存数据是否有效。',{step:true,enumType:'boolean'}),
      s('imu_error_count','IMU 错误计数','次','IMU 采样或通信累计错误次数。',{step:true})]),
    chart('imu','imu_age','IMU 数据年龄','距离最近采样的时间','ms',[
      s('imu_age_ms','IMU 数据年龄','ms','当前时刻距离最近有效 IMU 采样的时间。')]),

    chart('gray','gray_raw','八路灰度原始值','通道 0 至通道 7','ADC',[
      s('gray0','灰度通道 0','ADC','最左侧灰度通道原始采样。'),
      s('gray1','灰度通道 1','ADC','灰度通道 1 原始采样。'),
      s('gray2','灰度通道 2','ADC','灰度通道 2 原始采样。'),
      s('gray3','灰度通道 3','ADC','灰度通道 3 原始采样。'),
      s('gray4','灰度通道 4','ADC','灰度通道 4 原始采样。'),
      s('gray5','灰度通道 5','ADC','灰度通道 5 原始采样。'),
      s('gray6','灰度通道 6','ADC','灰度通道 6 原始采样。'),
      s('gray7','灰度通道 7','ADC','最右侧灰度通道原始采样。')]),
    chart('gray','gray_health','灰度健康状态','采样有效性与错误累计','状态/次',[
      s('gray_sample_valid','灰度采样有效','0/1','当前八路灰度缓存是否有效。',{step:true,enumType:'boolean'}),
      s('gray_error_count','灰度错误计数','次','灰度采样累计错误次数。',{step:true})]),
    chart('gray','gray_age','灰度数据年龄','距离最近采样的时间','ms',[
      s('gray_age_ms','灰度数据年龄','ms','当前时刻距离最近有效灰度采样的时间。')]),

    chart('system','fault','系统故障','故障码与累计次数','状态/次',[
      s('fault_code','当前故障码','状态码','系统当前锁存的 FaultCode。',{step:true}),
      s('fault_count','故障累计次数','次','系统启动后设置故障的累计次数。',{step:true})]),
    chart('system','uart','调试串口负载','发送队列与丢弃计数','字节/次',[
      s('tx_pending','待发送字节','字节','调试 UART 发送环形队列中等待发送的字节数。'),
      s('tx_dropped','丢弃字节计数','次','发送队列无空间时累计丢弃的数据量。',{step:true})])
  ];
  var GROUP_FIELDS={};
  CHARTS.forEach(function(item){
    GROUP_FIELDS[item.key]=['t'].concat(item.series.map(function(entry){
      return entry.field;
    }));
  });

  var ENUM_LABELS={
    boolean:{0:'否',1:'是'},
    appMode:{0:'空闲',1:'运行',2:'故障',3:'比赛待命',4:'比赛运行'},
    driverStatus:{0:'正常',1:'错误',2:'参数无效',3:'未初始化',4:'超时',5:'忙',6:'不支持',7:'无应答'},
    trackState:{0:'未知',1:'有效线路',2:'线路丢失',3:'多条线路',4:'宽线',5:'传感器故障'},
    roadType:{0:'未知',1:'丢线',2:'直道',3:'左分支',4:'右分支',5:'T 型路口',6:'十字路口',7:'左弯角',8:'右弯角'},
    roadPhase:{0:'正常',1:'观察中',2:'已锁存'},
    turnPhase:{0:'空闲',1:'驱动',2:'刹车',3:'稳定'}
  };

  function describeValue(enumType,value){
    if(!Number.isFinite(Number(value)))return'';
    var numeric=Number(value);
    if(enumType==='roadPaths'){
      if(numeric===0)return'无可行方向';
      var paths=[];
      if(numeric&1)paths.push('左');
      if(numeric&2)paths.push('前');
      if(numeric&4)paths.push('右');
      return paths.length?paths.join(' / '):'未知位图';
    }
    var labels=ENUM_LABELS[enumType];
    return labels&&Object.prototype.hasOwnProperty.call(labels,numeric)
      ?labels[numeric]:'';
  }

  function TelemetryParser(){
    this.fields=[];
  }

  TelemetryParser.prototype.reset=function(){
    this.fields=[];
  };

  TelemetryParser.prototype.parseLine=function(line,hostTime){
    var text=String(line).trim();
    if(text.startsWith('#')){
      var fields=text.slice(1).split(',').map(function(field){return field.trim()});
      if(fields.length>=2&&fields[0]==='t'){
        this.fields=fields;
        return{type:'header',fields:fields.slice(),raw:text};
      }
      return{type:'other',raw:text};
    }
    if(this.fields.length===0||text.indexOf(',')<0){
      return{type:'other',raw:text};
    }
    var cells=text.split(',');
    if(cells.length!==this.fields.length){
      return{type:'other',raw:text};
    }
    var values={};
    for(var index=0;index<cells.length;index++){
      var cell=cells[index].trim();
      if(cell===''||!Number.isFinite(Number(cell))){
        return{type:'other',raw:text};
      }
      values[this.fields[index]]=Number(cell);
    }
    return{
      type:'sample',
      fields:this.fields.slice(),
      values:values,
      cells:cells.map(function(cell){return cell.trim()}),
      hostTime:hostTime||new Date().toISOString(),
      raw:text
    };
  };

  function SerialRouter(callbacks){
    this.callbacks=callbacks||{};
    this.parser=new TelemetryParser();
    this.pending='';
  }

  SerialRouter.prototype.reset=function(){
    this.parser.reset();
    this.pending='';
  };

  SerialRouter.prototype._emitText=function(text){
    if(text&&typeof this.callbacks.onText==='function')this.callbacks.onText(text);
  };

  SerialRouter.prototype._routeLine=function(line,newline){
    /*
     * The shell prompt has no trailing newline. On real UART hardware the
     * next asynchronous telemetry frame can therefore arrive as
     * "> #t,..." or "> 123,...". Keep the prompt on the shell path while
     * parsing the remainder as a telemetry frame.
     */
    var payload=line;
    var prompt='';
    if(payload.indexOf('> ')===0){
      prompt='> ';
      payload=payload.slice(2);
    }
    var event=this.parser.parseLine(payload,new Date().toISOString());
    if(event.type==='header'||event.type==='sample'){
      this._emitText(prompt);
      if(typeof this.callbacks.onTelemetry==='function'){
        this.callbacks.onTelemetry(event,payload+newline);
      }
    }else{
      this._emitText(line+newline);
    }
  };

  SerialRouter.prototype.push=function(chunk){
    this.pending+=String(chunk);
    while(true){
      var crIndex=this.pending.indexOf('\r');
      var lfIndex=this.pending.indexOf('\n');
      var newlineIndex=crIndex<0?lfIndex:
        (lfIndex<0?crIndex:Math.min(crIndex,lfIndex));
      if(newlineIndex<0)break;
      if(this.pending[newlineIndex]==='\r'&&
         newlineIndex===this.pending.length-1)break;
      var consumed=(this.pending[newlineIndex]==='\r'&&
        this.pending[newlineIndex+1]==='\n')?2:1;
      var line=this.pending.slice(0,newlineIndex);
      this.pending=this.pending.slice(newlineIndex+consumed);
      this._routeLine(line,'\n');
    }
    if(this.pending==='> '||this.pending==='>'){
      this._emitText(this.pending);
      this.pending='';
    }
  };

  SerialRouter.prototype.flush=function(){
    if(this.pending){
      this._routeLine(this.pending,'');
      this.pending='';
    }
  };

  function csvEscape(value){
    var text=String(value===undefined?'':value);
    return /[",\r\n]/.test(text)?'"'+text.replace(/"/g,'""')+'"':text;
  }

  function findNearestSample(samples,targetTime){
    if(!samples||samples.length===0||!Number.isFinite(targetTime))return null;
    var low=0,high=samples.length-1;
    while(low<high){
      var middle=Math.floor((low+high)/2);
      if(Number(samples[middle].hostMs)<targetTime)low=middle+1;
      else high=middle;
    }
    if(low===0)return samples[0];
    var before=samples[low-1];
    var after=samples[low];
    return Math.abs(Number(before.hostMs)-targetTime)<=
      Math.abs(Number(after.hostMs)-targetTime)?before:after;
  }

  function buildTimeline(samples,windowMs,endTime){
    var span=Number(windowMs);
    if(!Number.isFinite(span)||span<=0)span=1000;
    var list=Array.isArray(samples)?samples:[];
    var end=Number(endTime);
    if(!Number.isFinite(end)){
      end=list.length?Number(list[list.length-1].hostMs):0;
    }
    if(!Number.isFinite(end))end=0;
    var start=end-span;
    return{
      start:start,
      end:end,
      span:span,
      samples:list.filter(function(sample){
        var time=Number(sample.hostMs);
        return Number.isFinite(time)&&time>=start&&time<=end;
      })
    };
  }

  function trimTimelineSamples(samples,windowMs,endTime){
    return buildTimeline(samples,windowMs,endTime).samples;
  }

  function exportCsv(fields,samples){
    var rows=[['host_rx_iso'].concat(fields).map(csvEscape).join(',')];
    samples.forEach(function(sample){
      rows.push([sample.hostTime].concat(fields.map(function(field){
        return Object.prototype.hasOwnProperty.call(sample.values,field)
          ?sample.values[field]:'';
      })).map(csvEscape).join(','));
    });
    return rows.join('\r\n')+'\r\n';
  }

  return{
    DEFAULT_FIELDS:DEFAULT_FIELDS,
    CATEGORIES:CATEGORIES,
    CHARTS:CHARTS,
    GROUP_FIELDS:GROUP_FIELDS,
    ENUM_LABELS:ENUM_LABELS,
    describeValue:describeValue,
    TelemetryParser:TelemetryParser,
    SerialRouter:SerialRouter,
    findNearestSample:findNearestSample,
    buildTimeline:buildTimeline,
    trimTimelineSamples:trimTimelineSamples,
    exportCsv:exportCsv,
    csvEscape:csvEscape
  };
});
