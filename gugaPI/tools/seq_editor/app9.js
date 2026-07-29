'use strict';

var lineSensorState={visible:false,connected:false,busy:false,polling:false,selected:'ir3',
  active:null,fram:null,dirty:false,pollTimer:null,pollCount:0,calActive:false,
  calStep:'none',calSamples:'0/64',
  calReady:{white:false,black:false,center:false,left:false,right:false}};

function lineSensorKv(text){
  var values={};
  String(text||'').replace(/(?:^|\s)([a-zA-Z0-9_]+)=([^\s]+)/g,
    function(_,key,value){values[key]=value;return _});
  return values;
}
function lineSensorOk(text){return /:\s*ok(?:\s|\r|\n|$)/i.test(String(text||''))}
function lineSensorLabel(source){return source==='adc8'?'八路 ADC':'三路串口红外'}
function lineSensorToast(message,bad){
  var toast=$('lineSensorToast');toast.textContent=message;
  toast.className='show'+(bad?' bad':'');clearTimeout(lineSensorToast.timer);
  lineSensorToast.timer=setTimeout(function(){toast.className=''},2600);
}
function lineSensorSetText(id,value,suffix){
  $(id).textContent=value===undefined||value===null?'--':String(value)+(suffix||'');
}
function lineSensorRender(){
  document.querySelectorAll('.line-source-card').forEach(function(card){
    card.classList.toggle('selected',card.dataset.source===lineSensorState.selected);
    card.classList.toggle('active',card.dataset.source===lineSensorState.active);
  });
  lineSensorSetText('lineActiveSource',lineSensorState.active?lineSensorLabel(lineSensorState.active):'--');
  lineSensorSetText('lineFramSource',lineSensorState.fram?lineSensorLabel(lineSensorState.fram):'--');
  lineSensorSetText('lineDirtyState',lineSensorState.dirty?'未保存':'已同步');
  $('btnLineApply').disabled=!lineSensorState.connected||lineSensorState.busy||
    lineSensorState.selected===lineSensorState.active;
  $('btnLineSave').disabled=!lineSensorState.connected||lineSensorState.busy||!lineSensorState.dirty;
  $('btnLineSensorRefresh').disabled=!lineSensorState.connected||lineSensorState.busy;
  $('btnLineClearStats').disabled=!lineSensorState.connected||lineSensorState.busy;
  $('btnLineCalBegin').disabled=!lineSensorState.connected||lineSensorState.busy;
  $('btnLineCalCancel').disabled=!lineSensorState.connected||lineSensorState.busy||!lineSensorState.calActive;
  var allReady=['white','black','center','left','right'].every(function(step){return lineSensorState.calReady[step]});
  $('btnLineCalCommit').disabled=!lineSensorState.connected||lineSensorState.busy||!lineSensorState.calActive||!allReady;
  document.querySelectorAll('[data-cal-step]').forEach(function(button){
    var step=button.dataset.calStep,done=lineSensorState.calReady[step];
    var capturing=lineSensorState.calActive&&lineSensorState.calStep===step;
    button.disabled=!lineSensorState.connected||lineSensorState.busy||!lineSensorState.calActive;
    button.classList.toggle('done',done);
    button.classList.toggle('capturing',capturing);
    button.querySelector('em').textContent=done?'已完成':capturing?lineSensorState.calSamples:'等待';
  });
}
function lineSensorRenderStatus(values){
  if(values.active){
    lineSensorState.active=values.active;
    if(lineSensorState.fram===null)lineSensorState.fram=values.configured||values.active;
    $('lineLiveStatus').textContent=values.valid==='1'?(values.calibrated==='1'?'有效':'未标定'):'无有效帧';
    lineSensorSetText('lineAge',values.age_ms,' ms');
  }
  lineSensorRender();
}
function lineSensorRenderRaw(values){
  lineSensorSetText('lineRawOffset',values.offset_raw);
  lineSensorSetText('linePosition',values.position_mpos,' mpos');
  lineSensorSetText('lineAllBlack',values.all_black==='1'?'是 (1)':values.all_black==='0'?'否 (0)':undefined);
  lineSensorSetText('lineAdc1',values.adc1);lineSensorSetText('lineAdc2',values.adc2);lineSensorSetText('lineAdc3',values.adc3);
}
function lineSensorRenderStats(values){
  lineSensorSetText('lineFrames',values.frames);lineSensorSetText('lineCrcErrors',values.crc_errors);
  lineSensorSetText('lineHeaderErrors',values.header_errors);lineSensorSetText('lineUartErrors',values.uart_errors);
  lineSensorSetText('lineDropped',values.dropped);lineSensorSetText('linePeriod',values.average_ms,' ms');
}
function lineSensorRenderCal(values){
  if(!Object.prototype.hasOwnProperty.call(values,'active'))return;
  lineSensorState.calActive=values.active==='1';
  lineSensorState.calStep=values.step||'none';
  lineSensorState.calSamples=values.samples||'0/64';
  var ready=String(values.ready||'00000').padEnd(5,'0');
  ['white','black','center','left','right'].forEach(function(step,index){lineSensorState.calReady[step]=ready[index]==='1'});
  var capturing=lineSensorState.calStep!=='none';
  $('lineCalState').textContent=capturing?'正在采集 '+lineSensorState.calStep+' '+lineSensorState.calSamples:
    lineSensorState.calActive?'标定进行中':'未开始';
  var allReady=['white','black','center','left','right'].every(function(step){return lineSensorState.calReady[step]});
  $('lineCalNotice').className=allReady?'ok':'';
  $('lineCalNotice').textContent=allReady?'五步数据已齐全。提交时固件会检查黑白范围、中心误差和左右符号。':'依次摆放传感器并点击对应步骤；采集期间请保持位置不动。';
  lineSensorRender();
}
function lineSensorSchedulePoll(delay){
  clearTimeout(lineSensorState.pollTimer);
  lineSensorState.pollTimer=null;
  if(!lineSensorState.visible||!lineSensorState.connected)return;
  lineSensorState.pollTimer=setTimeout(function(){
    lineSensorState.pollTimer=null;
    lineSensorPoll();
  },delay);
}
async function lineSensorPoll(){
  if(!lineSensorState.visible||!lineSensorState.connected||
     lineSensorState.busy||lineSensorState.polling)return;
  lineSensorState.polling=true;
  try{
    lineSensorRenderStatus(lineSensorKv(await send('linesensor status',{timeoutMs:1600})));
    if(lineSensorState.busy)return;
    lineSensorRenderRaw(lineSensorKv(await send('irsensor raw',{timeoutMs:1600})));
    if(lineSensorState.busy)return;
    if((lineSensorState.pollCount++%2)===0){
      lineSensorRenderStats(lineSensorKv(await send('irsensor stats',{timeoutMs:1600})));
      if(lineSensorState.busy)return;
      lineSensorRenderCal(lineSensorKv(await send('irsensor calib status',{timeoutMs:1600})));
      if(lineSensorState.busy)return;
      var paramStatus=lineSensorKv(await send('param status',{timeoutMs:1600}));
      lineSensorState.dirty=paramStatus.dirty==='1';
    }
  }catch(error){lineSensorToast('刷新失败：'+error.message,true)}
  finally{
    lineSensorState.polling=false;
    lineSensorRender();
    lineSensorSchedulePoll(900);
  }
}
async function lineSensorAction(command,success){
  if(lineSensorState.busy)return false;lineSensorState.busy=true;lineSensorRender();
  try{
    var response=await send(command,{timeoutMs:3500});
    if(!lineSensorOk(response))throw new Error(response.trim()||'设备拒绝命令');
    lineSensorToast(success,false);return true;
  }catch(error){lineSensorToast(error.message,true);return false}
  finally{
    lineSensorState.busy=false;
    lineSensorRender();
    lineSensorSchedulePoll(40);
  }
}
function LineSensorPage_OnShow(){
  lineSensorState.visible=true;
  lineSensorRender();
  lineSensorSchedulePoll(0);
}
function LineSensorPage_OnHide(){
  lineSensorState.visible=false;
  clearTimeout(lineSensorState.pollTimer);
  lineSensorState.pollTimer=null;
}

document.querySelectorAll('.line-source-card').forEach(function(card){card.onclick=function(){lineSensorState.selected=card.dataset.source;lineSensorRender()}});
$('btnLineSensorRefresh').onclick=function(){lineSensorSchedulePoll(0)};
$('btnLineApply').onclick=async function(){
  var source=lineSensorState.selected;
  if(await lineSensorAction('linesensor source '+source,'已应用到 RAM，尚未写入 FRAM'))lineSensorState.active=source;
};
$('btnLineSave').onclick=async function(){
  if(await lineSensorAction('param save','当前传感器和参数已保存到 FRAM')){lineSensorState.fram=lineSensorState.active;lineSensorState.dirty=false}
};
$('btnLineClearStats').onclick=function(){lineSensorAction('irsensor clear','通信统计已清零')};
$('btnLineCalBegin').onclick=async function(){
  if(await lineSensorAction('irsensor calib begin','标定已开始')){
    lineSensorState.calActive=true;lineSensorState.calStep='none';lineSensorState.calSamples='0/64';
    Object.keys(lineSensorState.calReady).forEach(function(key){lineSensorState.calReady[key]=false});
    lineSensorRender();
  }
};
$('btnLineCalCancel').onclick=async function(){
  if(await lineSensorAction('irsensor calib cancel','已取消本次标定')){
    lineSensorState.calActive=false;lineSensorState.calStep='none';lineSensorRender();
  }
};
$('btnLineCalCommit').onclick=async function(){
  if(await lineSensorAction('irsensor calib commit','标定结果已提交到 RAM，请验证后显式保存')){
    lineSensorState.calActive=false;lineSensorState.calStep='none';lineSensorState.dirty=true;lineSensorRender();
  }
};
document.querySelectorAll('[data-cal-step]').forEach(function(button){button.onclick=function(){lineSensorAction('irsensor calib capture '+button.dataset.calStep,'已开始采集：'+button.querySelector('b').textContent)}});

var lineSensorPreviousSerialState=onSerialStateChange;
onSerialStateChange=function(connected){
  if(typeof lineSensorPreviousSerialState==='function')lineSensorPreviousSerialState(connected);
  lineSensorState.connected=connected;lineSensorState.fram=connected?null:lineSensorState.fram;
  var badge=$('lineSensorConnection');badge.textContent=connected?(simMode?'模拟连接':'真实串口已连接'):'未连接';badge.className=connected?(simMode?'sim':'ok'):'';
  if(connected&&lineSensorState.visible)lineSensorPoll();lineSensorRender();
};

var lineSensorSim={source:'ir3',saved:'ir3',dirty:false,seq:0,started:Date.now(),
  calActive:false,ready:{white:false,black:false,center:false,left:false,right:false}};
function lineSensorSimCommand(cmd){
  var t=(Date.now()-lineSensorSim.started)/1000;
  if(cmd==='linesensor status')return'linesensor active='+lineSensorSim.source+' configured='+lineSensorSim.source+' ready=1 valid=1 fresh=1 calibrated=1 road_capable='+(lineSensorSim.source==='adc8'?1:0)+' age_ms=2 status=ok\r\n> ';
  if(cmd.indexOf('linesensor source ')===0){lineSensorSim.source=cmd.split(' ')[2];lineSensorSim.dirty=true;if(typeof simParamDirty!=='undefined')simParamDirty=true;return'linesensor source: ok active='+lineSensorSim.source+' dirty=1\r\n> '}
  if(cmd==='irsensor raw'){lineSensorSim.seq++;var offset=Math.round(Math.sin(t*1.5)*420);return'irsensor raw seq='+lineSensorSim.seq+' offset_raw='+offset+' position_mpos='+Math.round(offset*5)+' all_black=0 adc1='+(1800+Math.round(Math.sin(t)*500))+' adc2='+(2700+Math.round(Math.sin(t+2)*600))+' adc3='+(1600+Math.round(Math.sin(t+4)*450))+' on=2300 off=1900\r\n> '}
  if(cmd==='irsensor stats')return'irsensor stats bytes='+(lineSensorSim.seq*15)+' frames='+lineSensorSim.seq+' header_errors=0 crc_errors=0 uart_errors=0 rx_timeouts=0 overrun_errors=0 framing_errors=0 parity_errors=0 noise_errors=0 dropped=0 period_ms=10 average_ms=10 valid_permille=1000 crc_error_permille=0 payload_errors=0 resync_bytes=0 dma_wraps='+Math.floor(lineSensorSim.seq*15/128)+' dma_produced='+(lineSensorSim.seq*15)+' dma_consumed='+(lineSensorSim.seq*15)+' dma_lag=0 dma_max_lag=15 dma_overwrites=0 dma_faults=0 comm=healthy error_streak=0 age_ms=1 latency_us=1200 latency_max_us=1800\r\n> ';
  if(cmd==='irsensor clear'){lineSensorSim.seq=0;return'irsensor clear: ok\r\n> '}
  if(cmd==='irsensor calib begin'){lineSensorSim.calActive=true;Object.keys(lineSensorSim.ready).forEach(function(k){lineSensorSim.ready[k]=false});return'irsensor calib begin: ok\r\n> '}
  if(cmd.indexOf('irsensor calib capture ')===0){var step=cmd.split(' ')[3];lineSensorSim.ready[step]=true;return'irsensor calib capture: ok\r\n> '}
  if(cmd==='irsensor calib status'){var order=['white','black','center','left','right'];return'irsensor calib active='+(lineSensorSim.calActive?1:0)+' step=none samples=0/64 ready='+order.map(function(k){return lineSensorSim.ready[k]?1:0}).join('')+' white=900 black=3200 center=2 left=-600 right=610 status=ok\r\n> '}
  if(cmd==='irsensor calib commit'){lineSensorSim.calActive=false;lineSensorSim.dirty=true;return'irsensor calib commit: ok\r\n> '}
  if(cmd==='irsensor calib cancel'){lineSensorSim.calActive=false;return'irsensor calib cancel: ok\r\n> '}
  return cmd+': simulated\r\n> ';
}
