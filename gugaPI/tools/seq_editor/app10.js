'use strict';

var BALL_PARAM_FIELDS=[
  ['ball_kp_mdeg_per_0p1mm','Kp','mdeg/0.1mm'],
  ['ball_kd_mdeg_per_0p1mm_s','Kd','mdeg/(0.1mm/s)'],
  ['ball_ki_mdeg_per_0p1mm_s','Ki','scaled'],
  ['ball_pitch_gain_permille','俯仰补偿','‰'],
  ['ball_max_angle_mdeg','最大倾角','mdeg'],
  ['ball_degraded_angle_mdeg','降级倾角','mdeg'],
  ['ball_angle_slew_mdeg_s','倾角变化率','mdeg/s'],
  ['ball_position_tolerance_0p1mm','位置容差','0.1mm'],
  ['ball_velocity_tolerance_0p1mm_s','速度容差','0.1mm/s'],
  ['ball_settle_ms','稳定时间','ms']
];
var ballState={
  visible:false,connected:false,enabled:false,requesting:false,busy:false,
  unlocked:false,calibration:false,recording:false,records:[],samples:[],
  sampleCount:0,fields:[],last:null,lastAt:0,renderPending:false,simTimer:null,
  params:{},mapAngles:[-8000,-4000,0,4000,8000],
  mapDm:[-1000,-500,0,500,1000]
};

function ballToast(message,bad){
  var toast=$('ballToast');toast.textContent=message;
  toast.className='show'+(bad?' bad':'');clearTimeout(ballToast.timer);
  ballToast.timer=setTimeout(function(){toast.className=''},2800);
}
function ballText(id,value,suffix){
  $(id).textContent=value===undefined||value===null||value===''
    ?'--':String(value)+(suffix||'');
}
function ballResponseOk(text,prefix){
  return new RegExp(prefix.replace(/[.*+?^${}()|[\]\\]/g,'\\$&')+
    '\\s*:\\s*ok(?:\\s|\\r|\\n|$)','i').test(String(text||''));
}
function ballKv(text){
  var values={};
  String(text||'').replace(/(?:^|\s)([a-zA-Z0-9_]+)=([^\s]+)/g,
    function(_,key,value){values[key]=value;return _});
  return values;
}
function ballGateState(){
  var v=ballState.last?ballState.last.values:{};
  return{connected:ballState.connected,appMode:v.app_mode,
    faultCode:v.fault_code,actionRunning:v.action_running,
    unlocked:ballState.unlocked,busy:ballState.busy};
}
function ballRenderControls(){
  var reason=BallCore.motionGate(ballGateState());
  if(reason&&ballState.unlocked&&ballState.last){
    var v=ballState.last.values;
    if(Number(v.app_mode)!==1||Number(v.fault_code)!==0) {
      ballState.unlocked=false;$('ballSafetyUnlock').checked=false;
    }
  }
  reason=BallCore.motionGate(ballGateState());
  $('ballGateReason').textContent=reason||'运动控制已解锁';
  var allowed=!reason;
  $('btnBallHold').disabled=!allowed||ballState.calibration;
  $('btnBallMove').disabled=!allowed||ballState.calibration;
  $('btnBallInject').disabled=!allowed||ballState.calibration;
  $('btnBallStop').disabled=!ballState.connected;
  $('btnBallCalStart').disabled=!allowed||ballState.calibration;
  $('btnBallCalCancel').disabled=!ballState.connected||!ballState.calibration||
    ballState.busy;
  $('btnBallJogNegative').disabled=!allowed||!ballState.calibration;
  $('btnBallJogPositive').disabled=!allowed||!ballState.calibration;
  $('btnBallMapApply').disabled=!ballState.connected||ballState.busy;
  $('btnBallParamsReload').disabled=!ballState.connected||ballState.busy;
  $('btnBallParamsApply').disabled=!ballState.connected||ballState.busy;
  $('btnBallParamsSave').disabled=!ballState.connected||ballState.busy;
  $('btnBallRecord').disabled=!ballState.connected||!ballState.enabled;
  $('btnBallExport').disabled=ballState.records.length===0;
}
function ballRenderStatus(){
  var v=ballState.last?ballState.last.values:{};
  ballText('ballAppMode',v.app_mode===undefined?null:BallCore.label('appMode',v.app_mode));
  ballText('ballActionState',v.action_running===undefined?null:
    (Number(v.action_running)?'运行中':'空闲'));
  ballText('ballMode',v.ball_mode===undefined?null:BallCore.label('ballMode',v.ball_mode));
  ballText('ballResult',v.ball_result===undefined?null:BallCore.label('ballResult',v.ball_result));
  ballText('ballTargetNow',v.ball_target_0p1mm===undefined?null:
    (Number(v.ball_target_0p1mm)/10).toFixed(1),' mm');
  ballText('ballMaximumError',v.ball_max_error_0p1mm===undefined?null:
    (Number(v.ball_max_error_0p1mm)/10).toFixed(1),' mm');
  ballText('ballSettling',v.ball_settling===undefined?null:
    (Number(v.ball_settling)?'正在稳定':'否'));
  ballText('ballPitch',v.pitch_mdeg===undefined?null:
    (Number(v.pitch_mdeg)/1000).toFixed(3),' deg');
  ballText('ballFault',v.fault_code===undefined?null:
    (Number(v.fault_code)===0?'无':'故障码 '+v.fault_code));
  ballText('ballVisionState',v.vision_state===undefined?null:
    BallCore.label('visionState',v.vision_state)+(Number(v.vision_injected)?' · 注入':''));
  ballText('ballVisionConfidence',v.vision_confidence);
  ballText('ballVisionFrameAge',v.vision_frame_age_ms,' ms');
  ballText('ballVisionBallAge',v.vision_ball_age_ms,' ms');
  ballText('ballDmMode',v.dm_mode===undefined?null:BallCore.label('dmMode',v.dm_mode));
  ballText('ballDmState',v.dm_state===undefined?null:
    ((Number(v.dm_enabled)?'已使能':'已失能')+' / '+v.dm_state));
  ballText('ballDmHealth',v.dm_online===undefined?null:
    ((Number(v.dm_online)?'在线':'离线')+' / '+(Number(v.dm_fresh)?'新鲜':'过期')));
  ballText('ballDmPosition',v.dm_position_mrad===undefined?null:
    (Number(v.dm_position_mrad)/1000).toFixed(3),' rad');
  ballText('ballDmVelocity',v.dm_velocity_mrad_s===undefined?null:
    (Number(v.dm_velocity_mrad_s)/1000).toFixed(3),' rad/s');
  var age=ballState.lastAt?Date.now()-ballState.lastAt:Infinity;
  $('ballTelemetry').textContent=ballState.requesting?'正在切换遥测':
    !ballState.enabled?'遥测未启动':age<150?'50 Hz实时':'遥测超时';
  $('ballTelemetry').className=ballState.enabled?(age<150?'live':'stale'):'';
  $('ballSamples').textContent=ballState.sampleCount+'个样本';
  $('btnBallRecord').textContent=ballState.recording?'停止记录':'开始记录';
  ballRenderControls();
}
function ballCanvas(canvas,series){
  var rect=canvas.getBoundingClientRect();if(rect.width<20||rect.height<20)return;
  var ratio=window.devicePixelRatio||1,w=Math.floor(rect.width*ratio),
    h=Math.floor(rect.height*ratio);
  if(canvas.width!==w||canvas.height!==h){canvas.width=w;canvas.height=h}
  var ctx=canvas.getContext('2d');ctx.clearRect(0,0,w,h);
  ctx.fillStyle='#10141c';ctx.fillRect(0,0,w,h);
  var samples=ballState.samples;if(samples.length<2){
    ctx.fillStyle='#687487';ctx.font=(11*ratio)+'px sans-serif';
    ctx.fillText('等待滚球遥测数据',14*ratio,h/2);return;
  }
  var latest=Number(samples[samples.length-1].values.t),start=latest-30000;
  var visible=samples.filter(function(s){return Number(s.values.t)>=start});
  var values=[];
  visible.forEach(function(sample){var scaled=BallCore.scaled(sample);
    series.forEach(function(item){var n=Number(scaled[item.key]);if(Number.isFinite(n))values.push(n)})});
  var min=Math.min.apply(null,values),max=Math.max.apply(null,values);
  if(!Number.isFinite(min)||!Number.isFinite(max)){min=-1;max=1}
  if(min===max){min-=1;max+=1}
  var pad=(max-min)*.12;min-=pad;max+=pad;
  ctx.strokeStyle='#29313e';ctx.lineWidth=ratio;
  for(var grid=1;grid<4;grid++){var gy=h*grid/4;ctx.beginPath();ctx.moveTo(0,gy);ctx.lineTo(w,gy);ctx.stroke()}
  series.forEach(function(item){
    ctx.strokeStyle=item.color;ctx.lineWidth=1.6*ratio;ctx.beginPath();
    var started=false;
    visible.forEach(function(sample){
      var scaled=BallCore.scaled(sample),value=Number(scaled[item.key]);
      if(!Number.isFinite(value))return;
      var x=(Number(sample.values.t)-start)/30000*w;
      var y=h-(value-min)/(max-min)*h;
      if(!started){ctx.moveTo(x,y);started=true}else ctx.lineTo(x,y);
    });ctx.stroke();
  });
  ctx.font=(10*ratio)+'px sans-serif';var lx=7*ratio;
  series.forEach(function(item){ctx.fillStyle=item.color;ctx.fillText(item.label,lx,13*ratio);lx+=ctx.measureText(item.label).width+13*ratio});
}
function ballDraw(){
  ballCanvas($('ballPositionChart'),[
    {key:'targetMm',label:'目标',color:'#e4b657'},
    {key:'positionMm',label:'位置',color:'#55bfe9'},
    {key:'errorMm',label:'误差',color:'#df6670'}]);
  ballCanvas($('ballVelocityChart'),[
    {key:'velocityMmS',label:'速度',color:'#55c68a'}]);
  ballCanvas($('ballAngleChart'),[
    {key:'beamDeg',label:'横梁',color:'#ae84c6'},
    {key:'pitchDeg',label:'pitch',color:'#df8352'}]);
  ballCanvas($('ballDmChart'),[
    {key:'dmTargetRad',label:'目标',color:'#e4b657'},
    {key:'dmPositionRad',label:'反馈',color:'#55bfe9'}]);
}
function ballScheduleRender(){
  if(ballState.renderPending)return;ballState.renderPending=true;
  requestAnimationFrame(function(){ballState.renderPending=false;ballRenderStatus();ballDraw()});
}
function BallTelemetry_OnEvent(event){
  if(!ballState.visible||!event)return;
  if(event.type==='header'){
    if(event.fields.join(',')===BallCore.FIELDS.join(','))ballState.fields=event.fields.slice();
    return;
  }
  if(event.type!=='sample'||event.fields.join(',')!==BallCore.FIELDS.join(','))return;
  ballState.last=event;ballState.lastAt=Date.now();
  ballState.sampleCount++;
  ballState.samples.push(event);
  ballState.samples=BallCore.trimSamples(ballState.samples,Number(event.values.t),60000);
  if(ballState.recording){
    if(ballState.records.length<100000)ballState.records.push(event);
    if(ballState.records.length>=100000){
      ballState.recording=false;
      ballToast('记录已达到100000帧并自动停止',false);
    }
  }
  ballScheduleRender();
}
async function ballStartTelemetry(){
  if(!ballState.visible||!ballState.connected||ballState.requesting)return;
  ballState.requesting=true;ballRenderStatus();
  try{
    await send('telem off',{timeoutMs:800});
    await send('telem on ball 20',{timeoutMs:1000});
    if(!ballState.visible||!ballState.connected){
      try{await send('telem off',{timeoutMs:800})}catch(error){}
      return;
    }
    ballState.enabled=true;ballState.fields=[];
    if(simMode)ballSimStart();
  }catch(error){ballToast('滚球遥测启动失败：'+error.message,true)}
  finally{ballState.requesting=false;ballRenderStatus()}
}
async function ballStopTelemetry(){
  ballState.enabled=false;ballSimStop();
  if(ballState.connected){try{await send('telem off',{timeoutMs:800})}catch(error){}}
  ballRenderStatus();
}
async function ballAction(command,success){
  if(ballState.busy)return false;ballState.busy=true;ballRenderControls();
  try{
    var response=await send(command,{timeoutMs:5500});
    var prefix=command.split(' ').slice(0,2).join(' ');
    if(!ballResponseOk(response,prefix)&&!ballResponseOk(response,command.split(' ')[0]))
      throw new Error(response.trim()||'设备拒绝命令');
    ballToast(success,false);return true;
  }catch(error){ballToast(error.message,true);return false}
  finally{ballState.busy=false;ballRenderControls()}
}
function ballTarget0p1mm(id,min,max){
  var value=Number($(id).value);
  if(!Number.isFinite(value)||value<min||value>max)return null;
  return Math.round(value*10);
}
async function ballEndCalibration(showMessage){
  if(!ballState.calibration)return;
  ballState.calibration=false;
  try{await send('dm disable',{timeoutMs:2500})}catch(error){}
  if(showMessage)ballToast('采点已结束，达妙已失能',false);
  ballRenderControls();
}
async function ballRun(command,message){
  var reason=BallCore.motionGate(ballGateState());
  if(reason){ballToast(reason,true);return}
  if(ballState.calibration)await ballEndCalibration(false);
  await ballAction(command,message);
}
function ballBuildParams(){
  $('ballParamGrid').innerHTML=BALL_PARAM_FIELDS.map(function(item){
    var meta=PARAM_META[item[0]];
    return'<label>'+item[1]+'<input id="ballParam_'+item[0]+'" type="number" min="'+
      meta.min+'" max="'+meta.max+'" step="1"><small>'+item[2]+' · '+
      meta.min+'..'+meta.max+'</small></label>';
  }).join('');
}
function ballBuildMap(){
  $('ballMapRows').innerHTML=Array.from({length:5},function(_,i){
    return'<tr><td>'+(i+1)+'</td><td><input id="ballCalAngle'+i+
      '" type="number" min="-15000" max="15000" step="1"></td><td><input id="ballCalDm'+
      i+'" type="number" min="-12500" max="12500" step="1"></td><td><button type="button" data-ball-capture="'+
      i+'">捕获当前位置</button></td></tr>';
  }).join('');
  document.querySelectorAll('[data-ball-capture]').forEach(function(button){
    button.onclick=function(){ballCapture(Number(button.dataset.ballCapture))};
  });
}
function ballFillMap(){
  for(var i=0;i<5;i++){$('ballCalAngle'+i).value=ballState.mapAngles[i];$('ballCalDm'+i).value=ballState.mapDm[i]}
}
async function ballLoadConfig(){
  if(!ballState.connected||ballState.busy)return;
  ballState.busy=true;ballRenderControls();
  try{
    var values=ballKv(await send('ball params',{timeoutMs:2500}));
    var aliases={ball_kp_mdeg_per_0p1mm:'kp',
      ball_kd_mdeg_per_0p1mm_s:'kd',ball_ki_mdeg_per_0p1mm_s:'ki',
      ball_pitch_gain_permille:'pitch_gain_permille',
      ball_max_angle_mdeg:'max_angle_mdeg',
      ball_degraded_angle_mdeg:'degraded_angle_mdeg',
      ball_angle_slew_mdeg_s:'slew_mdeg_s',
      ball_position_tolerance_0p1mm:'tolerance_0p1mm',
      ball_velocity_tolerance_0p1mm_s:'velocity_tolerance_0p1mm_s',
      ball_settle_ms:'settle_ms'};
    BALL_PARAM_FIELDS.forEach(function(item){
      var value=Number(values[aliases[item[0]]]);
      if(Number.isFinite(value)){ballState.params[item[0]]=value;$('ballParam_'+item[0]).value=value}
    });
    var mapText=String(await send('ball map',{timeoutMs:2500}));
    var match=mapText.match(/ball map((?:\s+-?\d+){10})/);
    if(match){var nums=match[1].trim().split(/\s+/).map(Number);
      for(var i=0;i<5;i++){ballState.mapAngles[i]=nums[i*2];ballState.mapDm[i]=nums[i*2+1]}ballFillMap()}
  }catch(error){ballToast('读取滚球配置失败：'+error.message,true)}
  finally{ballState.busy=false;ballRenderControls()}
}
function ballDesiredParams(){
  var desired={};
  for(var i=0;i<BALL_PARAM_FIELDS.length;i++){
    var name=BALL_PARAM_FIELDS[i][0],meta=PARAM_META[name],
      value=Number($('ballParam_'+name).value);
    if(!Number.isInteger(value)||value<meta.min||value>meta.max)
      throw new Error(BALL_PARAM_FIELDS[i][1]+'超出范围');
    desired[name]=value;
  }
  if(desired.ball_degraded_angle_mdeg>desired.ball_max_angle_mdeg)
    throw new Error('降级倾角不能大于最大倾角');
  return desired;
}
async function ballApplyParams(){
  if(!ballState.connected||ballState.busy)return;
  var desired;try{desired=ballDesiredParams()}catch(error){ballToast(error.message,true);return}
  ballState.busy=true;ballRenderControls();
  try{
    var max='ball_max_angle_mdeg',degraded='ball_degraded_angle_mdeg';
    var order=BALL_PARAM_FIELDS.map(function(item){return item[0]}).filter(function(name){return name!==max&&name!==degraded});
    if(desired[max]>=(ballState.params[max]||0))order.unshift(max,degraded);
    else order.unshift(degraded,max);
    for(var i=0;i<order.length;i++){
      var name=order[i];if(desired[name]===ballState.params[name])continue;
      var response=await send('param set '+name+' '+desired[name],{timeoutMs:2200});
      if(!ballResponseOk(response,'param set'))throw new Error('固件拒绝参数 '+name);
      ballState.params[name]=desired[name];
    }
    ballToast('滚球参数已应用到RAM，下次hold/move生效',false);
  }catch(error){ballToast(error.message,true)}
  finally{ballState.busy=false;ballRenderControls()}
}
async function ballStartCalibration(){
  var reason=BallCore.motionGate(ballGateState());if(reason){ballToast(reason,true);return}
  ballState.busy=true;ballRenderControls();
  try{
    var stopResponse=await send('ball stop',{timeoutMs:2500});
    if(!ballResponseOk(stopResponse,'ball stop'))
      throw new Error(stopResponse.trim()||'滚球停止失败');
    var response=await send('dm enable',{timeoutMs:3500});
    if(!ballResponseOk(response,'dm enable'))throw new Error(response.trim()||'达妙使能失败');
    if(!ballState.visible||!ballState.connected){
      try{await send('dm disable',{timeoutMs:1800})}catch(error){}
      return;
    }
    ballState.calibration=true;ballToast('采点已开始，可使用小步进点动机构',false);
  }catch(error){
    try{await send('dm disable',{timeoutMs:1800})}catch(disableError){}
    ballToast(error.message,true);
  }
  finally{ballState.busy=false;ballRenderControls()}
}
async function ballJog(direction){
  var reason=BallCore.motionGate(ballGateState());
  if(reason||!ballState.calibration){ballToast(reason||'请先开始采点',true);return}
  var step=Number($('ballJogStep').value),velocity=Number($('ballJogVelocity').value);
  if(!Number.isInteger(velocity)||velocity<1||velocity>20000){ballToast('点动速度范围为1..20000 mrad/s',true);return}
  await ballAction('dm position relative '+(direction*step)+' '+velocity+' 5000','点动命令已发送');
}
function ballCapture(index){
  var reason=BallCore.motionGate(ballGateState()),v=ballState.last&&ballState.last.values;
  if(reason||!ballState.calibration){ballToast(reason||'请先开始采点',true);return}
  if(!v||Number(v.dm_online)!==1||Number(v.dm_fresh)!==1){ballToast('达妙反馈无效或已过期',true);return}
  if(Math.abs(Number(v.dm_velocity_mrad_s))>50){ballToast('电机仍在运动，速度降至50 mrad/s以内再捕获',true);return}
  $('ballCalDm'+index).value=Math.round(Number(v.dm_position_mrad));
  ballToast('已捕获第'+(index+1)+'点达妙位置',false);
}
async function ballApplyMap(){
  var angles=[],positions=[];
  for(var i=0;i<5;i++){angles.push(Number($('ballCalAngle'+i).value));positions.push(Number($('ballCalDm'+i).value))}
  var error=BallCore.validateMap(angles,positions);
  if(error){$('ballMapNotice').textContent=error;ballToast(error,true);return}
  if(ballState.calibration)await ballEndCalibration(false);
  ballState.busy=true;ballRenderControls();
  try{
    var args=[];for(var j=0;j<5;j++)args.push(angles[j],positions[j]);
    var response=await send('ball map '+args.join(' '),{timeoutMs:3000});
    if(!ballResponseOk(response,'ball map'))throw new Error(response.trim()||'映射被拒绝');
    ballState.mapAngles=angles;ballState.mapDm=positions;
    $('ballMapNotice').textContent='映射已提交到RAM；验证后点击“保存到FRAM”。';
    ballToast('五点映射已原子提交到RAM',false);
  }catch(e){ballToast(e.message,true)}
  finally{ballState.busy=false;ballRenderControls()}
}
function BallPage_OnShow(){
  ballState.visible=true;ballRenderStatus();ballScheduleRender();
  if(ballState.connected){ballStartTelemetry();ballLoadConfig()}
}
function BallPage_OnHide(){
  if(!ballState.visible)return;
  ballState.visible=false;ballStopTelemetry();
  if(ballState.calibration)ballEndCalibration(false);
}
async function BallPage_BeforeDisconnect(){
  if(!ballState.connected)return;
  try{await send('ball stop',{timeoutMs:1000})}catch(error){}
  try{await send('dm disable',{timeoutMs:1000})}catch(error){}
  try{await send('telem off',{timeoutMs:700})}catch(error){}
  ballState.enabled=false;ballState.calibration=false;ballSimStop();
}

function ballSimStart(){
  ballSimStop();
  if(serialRouter)serialRouter.push('#'+BallCore.FIELDS.join(',')+'\n');
  ballState.simTimer=setInterval(function(){
    if(!ballState.visible||!simMode)return;
    var row=ballSimSample();if(serialRouter)serialRouter.push(row+'\n');
  },20);
}
function ballSimStop(){if(ballState.simTimer){clearInterval(ballState.simTimer);ballState.simTimer=null}}
var ballSim={started:Date.now(),mode:0,result:0,target:0,position:0,
  velocity:0,beam:0,dmTarget:0,dmPosition:0,dmEnabled:0,injected:0};
function ballSimStep(){
  var error=ballSim.target-ballSim.position,previous=ballSim.position;
  if(ballSim.mode===1||ballSim.mode===2)ballSim.position+=error*.035;
  ballSim.velocity=(ballSim.position-previous)*500;
  ballSim.beam=Math.max(-8000,Math.min(8000,Math.round(error*8)));
  ballSim.dmTarget=Math.round(ballSim.beam/8);
  if(ballSim.dmEnabled)ballSim.dmPosition+=(ballSim.dmTarget-ballSim.dmPosition)*.12;
  if(ballSim.mode===2&&Math.abs(error)<4&&Math.abs(ballSim.velocity)<10){ballSim.mode=1;ballSim.result=2}
}
function ballSimSample(){
  ballSimStep();var t=Date.now()-ballSim.started;
  var values={t:t,app_mode:1,action_running:0,ball_mode:ballSim.mode,
    ball_result:ballSim.result,ball_status:0,
    ball_target_0p1mm:Math.round(ballSim.target),
    ball_position_0p1mm:Math.round(ballSim.position),
    ball_velocity_0p1mm_s:Math.round(ballSim.velocity),
    ball_error_0p1mm:Math.round(ballSim.target-ballSim.position),
    ball_beam_mdeg:ballSim.beam,ball_dm_target_mrad:ballSim.dmTarget,
    ball_max_error_0p1mm:Math.round(Math.abs(ballSim.target-ballSim.position)),
    ball_settling:Math.abs(ballSim.target-ballSim.position)<10?1:0,
    vision_state:2,vision_confidence:900,vision_frame_age_ms:5,
    vision_ball_age_ms:5,vision_injected:ballSim.injected,dm_mode:ballSim.dmEnabled?9:5,
    dm_enabled:ballSim.dmEnabled,dm_online:1,dm_fresh:1,dm_state:1,
    dm_position_mrad:Math.round(ballSim.dmPosition),
    dm_velocity_mrad_s:Math.round((ballSim.dmTarget-ballSim.dmPosition)*5),
    pitch_mdeg:Math.round(Math.sin(t/1000)*900),fault_code:0};
  return BallCore.FIELDS.map(function(field){return values[field]}).join(',');
}
function ballSimCommand(cmd){
  if(cmd==='ball params'){
    if(typeof paramSimInit==='function')paramSimInit();
    var p=simParamValues||{};
    return'ball params kp='+p.ball_kp_mdeg_per_0p1mm+' kd='+p.ball_kd_mdeg_per_0p1mm_s+
      ' ki='+p.ball_ki_mdeg_per_0p1mm_s+' pitch_gain_permille='+p.ball_pitch_gain_permille+
      ' max_angle_mdeg='+p.ball_max_angle_mdeg+' degraded_angle_mdeg='+p.ball_degraded_angle_mdeg+
      ' slew_mdeg_s='+p.ball_angle_slew_mdeg_s+' tolerance_0p1mm='+p.ball_position_tolerance_0p1mm+
      ' velocity_tolerance_0p1mm_s='+p.ball_velocity_tolerance_0p1mm_s+
      ' settle_ms='+p.ball_settle_ms+' applies=next_start active='+(ballSim.mode?1:0)+'\r\n> ';
  }
  if(cmd==='ball map'||cmd.indexOf('ball map ')===0)return paramSimCommand(cmd);
  if(cmd.indexOf('ball hold ')===0||cmd.indexOf('ball move ')===0){
    var parts=cmd.split(/\s+/);ballSim.target=Number(parts[2]);ballSim.mode=cmd.indexOf('hold')>0?1:2;
    ballSim.result=cmd.indexOf('hold')>0?2:1;ballSim.dmEnabled=1;
    return(cmd.indexOf('hold')>0?'ball hold':'ball move')+': ok\r\n> ';
  }
  if(cmd==='ball stop'){ballSim.mode=0;ballSim.result=0;ballSim.dmEnabled=0;return'ball stop: ok\r\n> '}
  if(cmd.indexOf('vision inject ')===0){ballSim.injected=1;ballSim.position=Number(cmd.split(/\s+/)[2]);return'vision inject: ok\r\n> '}
  if(cmd==='dm enable'){ballSim.dmEnabled=1;return'dm enable: ok\r\n> '}
  if(cmd==='dm disable'){ballSim.dmEnabled=0;return'dm disable: ok\r\n> '}
  if(cmd.indexOf('dm position relative ')===0){var d=Number(cmd.split(/\s+/)[3]);ballSim.dmPosition+=d;ballSim.dmTarget=ballSim.dmPosition;return'dm position: ok\r\n> '}
  if(cmd==='dm status')return'dm mode='+(ballSim.dmEnabled?'hold':'ready')+' result=0 enabled='+ballSim.dmEnabled+' online=1 fresh=1 state=1 p_mrad='+Math.round(ballSim.dmPosition)+' v_mrad_s=0 age_ms=2 status=ok\r\n> ';
  return cmd+': simulated\r\n> ';
}

ballBuildParams();ballBuildMap();ballFillMap();
$('ballSafetyUnlock').onchange=function(){ballState.unlocked=this.checked;ballRenderControls()};
$('btnBallHold').onclick=function(){var target=ballTarget0p1mm('ballTargetInput',-100,100);if(target===null)return ballToast('目标位置范围为-100.0..100.0 mm',true);ballRun('ball hold '+target,'滚球保持已启动')};
$('btnBallMove').onclick=function(){var target=ballTarget0p1mm('ballTargetInput',-100,100),timeout=Number($('ballTimeoutInput').value);if(target===null||!Number.isInteger(timeout)||timeout<50||timeout>30000)return ballToast('目标位置或超时参数无效',true);ballRun('ball move '+target+' '+timeout,'滚球移动已启动')};
$('btnBallStop').onclick=async function(){
  if(!ballState.connected)return;
  ballState.calibration=false;ballRenderControls();
  try{
    var response=await send('ball stop',{timeoutMs:2500});
    if(!ballResponseOk(response,'ball stop'))
      throw new Error(response.trim()||'设备拒绝停止命令');
    try{await send('dm disable',{timeoutMs:1800})}catch(error){}
    ballToast('滚球已停止并失能',false);
  }catch(error){ballToast(error.message,true)}
};
$('btnBallInject').onclick=function(){var position=ballTarget0p1mm('ballInjectPosition',-125,125),confidence=Number($('ballInjectConfidence').value);if(position===null||!Number.isInteger(confidence)||confidence<0||confidence>1000)return ballToast('视觉注入参数无效',true);ballRun('vision inject '+position+' '+confidence,'已注入一个视觉样本')};
$('btnBallClear').onclick=function(){ballState.samples=[];ballState.records=[];
  ballState.sampleCount=0;ballState.recording=false;ballState.last=null;
  ballState.lastAt=0;ballDraw();ballRenderStatus()};
$('btnBallRecord').onclick=function(){ballState.recording=!ballState.recording;if(ballState.recording)ballState.records=[];ballRenderStatus()};
$('btnBallExport').onclick=function(){if(!ballState.records.length)return;var blob=new Blob([BallCore.exportCsv(BallCore.FIELDS,ballState.records)],{type:'text/csv;charset=utf-8'}),link=document.createElement('a');link.href=URL.createObjectURL(blob);link.download='gugapi-ball-'+new Date().toISOString().replace(/[:.]/g,'-')+'.csv';link.click();setTimeout(function(){URL.revokeObjectURL(link.href)},1000)};
$('btnBallParamsReload').onclick=ballLoadConfig;
$('btnBallParamsApply').onclick=ballApplyParams;
$('btnBallParamsSave').onclick=function(){ballAction('param save','当前参数和映射已保存到FRAM')};
$('btnBallCalStart').onclick=ballStartCalibration;
$('btnBallCalCancel').onclick=function(){ballEndCalibration(true)};
$('btnBallJogNegative').onclick=function(){ballJog(-1)};
$('btnBallJogPositive').onclick=function(){ballJog(1)};
$('btnBallMapApply').onclick=ballApplyMap;

var ballPreviousSerialState=onSerialStateChange;
onSerialStateChange=function(connected){
  if(typeof ballPreviousSerialState==='function')ballPreviousSerialState(connected);
  ballState.connected=connected;ballState.unlocked=false;ballState.calibration=false;
  $('ballSafetyUnlock').checked=false;
  $('ballConnection').textContent=connected?(simMode?'模拟连接':'真实串口已连接'):'未连接';
  $('ballConnection').className=connected?'ok':'';
  if(connected&&ballState.visible){ballStartTelemetry();ballLoadConfig()}
  else if(!connected){ballState.enabled=false;ballSimStop()}
  ballRenderStatus();
};
ballRenderStatus();
