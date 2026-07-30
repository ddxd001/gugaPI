'use strict';

(function(root,factory){
  var api=factory();
  if(typeof module==='object'&&module.exports)module.exports=api;
  root.BallCore=api;
})(typeof globalThis!=='undefined'?globalThis:this,function(){
  var FIELDS=[
    't','app_mode','action_running','ball_mode','ball_result','ball_status',
    'ball_target_0p1mm','ball_position_0p1mm','ball_velocity_0p1mm_s',
    'ball_error_0p1mm','ball_beam_mdeg','ball_dm_target_mrad',
    'ball_max_error_0p1mm','ball_settling','vision_state',
    'vision_confidence','vision_frame_age_ms','vision_ball_age_ms',
    'vision_injected','dm_mode','dm_enabled','dm_online','dm_fresh',
    'dm_state','dm_position_mrad','dm_velocity_mrad_s','pitch_mdeg',
    'fault_code'
  ];
  var LABELS={
    appMode:{0:'空闲',1:'调试运行',2:'故障',3:'比赛待命',4:'比赛运行'},
    ballMode:{0:'空闲',1:'保持',2:'移动',3:'失败'},
    ballResult:{0:'空闲',1:'运行中',2:'成功',3:'超时',4:'视觉丢失',
      5:'接近端点',6:'达妙错误'},
    visionState:{0:'离线',1:'未识别钢球',2:'新鲜',3:'降级',4:'过期'},
    dmMode:{0:'上电等待',1:'启动清错',2:'启动失能',3:'探测',
      4:'离线',5:'就绪',6:'使能中',7:'保持',8:'定位',
      9:'外部位置',10:'定速',11:'减速停止',12:'失能中',
      13:'置零',14:'故障锁定'}
  };
  function label(group,value){
    var table=LABELS[group]||{};
    return Object.prototype.hasOwnProperty.call(table,Number(value))
      ?table[Number(value)]:'未知('+value+')';
  }
  function scaled(sample){
    var v=(sample&&sample.values)||sample||{};
    return{
      timeMs:Number(v.t),
      targetMm:Number(v.ball_target_0p1mm)/10,
      positionMm:Number(v.ball_position_0p1mm)/10,
      velocityMmS:Number(v.ball_velocity_0p1mm_s)/10,
      errorMm:Number(v.ball_error_0p1mm)/10,
      maximumErrorMm:Number(v.ball_max_error_0p1mm)/10,
      beamDeg:Number(v.ball_beam_mdeg)/1000,
      pitchDeg:Number(v.pitch_mdeg)/1000,
      dmTargetRad:Number(v.ball_dm_target_mrad)/1000,
      dmPositionRad:Number(v.dm_position_mrad)/1000,
      dmVelocityRadS:Number(v.dm_velocity_mrad_s)/1000
    };
  }
  function motionGate(state){
    if(!state.connected)return'请先连接设备';
    if(Number(state.appMode)!==1)return'仅允许在 dev-running 调试模式操作';
    if(Number(state.faultCode)!==0)return'系统存在锁存故障';
    if(Number(state.actionRunning)!==0)return'ActionRunner 正在运行';
    if(!state.unlocked)return'请先确认机构安全并解锁';
    if(state.busy)return'正在执行其他操作';
    return'';
  }
  function validateMap(angles,positions){
    if(!Array.isArray(angles)||!Array.isArray(positions)||
       angles.length!==5||positions.length!==5)return'五点映射数量不完整';
    for(var i=0;i<5;i++){
      if(!Number.isInteger(angles[i])||angles[i]<-15000||angles[i]>15000)
        return'横梁角必须是 -15000..15000 mdeg 的整数';
      if(!Number.isInteger(positions[i])||positions[i]<-12500||
         positions[i]>12500)
        return'达妙位置必须是 -12500..12500 mrad 的整数';
      if(i>0&&angles[i]<=angles[i-1])return'横梁角必须严格递增';
    }
    var direction=0;
    for(var j=1;j<5;j++){
      var delta=positions[j]-positions[j-1];
      if(delta===0)return'达妙位置不能包含重复点';
      var next=delta>0?1:-1;
      if(direction&&direction!==next)return'达妙位置必须整体递增或整体递减';
      direction=next;
    }
    return'';
  }
  function trimSamples(samples,latestMs,windowMs){
    var cutoff=latestMs-windowMs;
    return(samples||[]).filter(function(sample){
      return Number(sample.values&&sample.values.t)>=cutoff;
    });
  }
  function csv(fields,records){
    var escape=function(value){
      var text=String(value===undefined?'':value);
      return /[",\n]/.test(text)?'"'+text.replace(/"/g,'""')+'"':text;
    };
    var header=['host_time'].concat(fields).map(escape).join(',');
    var rows=(records||[]).map(function(record){
      return[record.hostTime].concat(fields.map(function(field){
        return record.values[field];
      })).map(escape).join(',');
    });
    return[header].concat(rows).join('\r\n');
  }
  return{FIELDS:FIELDS,LABELS:LABELS,label:label,scaled:scaled,
    motionGate:motionGate,validateMap:validateMap,trimSamples:trimSamples,
    exportCsv:csv};
});
