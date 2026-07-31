'use strict';

var lineSensorState={visible:false,connected:false,busy:false,polling:false,
  dirty:false,pollTimer:null,pollCount:0,
  gray:{session:false,running:false,phase:'idle',samples:0,target:64,last:'ok',
    whiteReady:false,blackReady:false,liveValid:false,age:0,
    white:Array(8).fill(0),black:Array(8).fill(0),whiteNoise:Array(8).fill(0),
    blackNoise:Array(8).fill(0),span:Array(8).fill(0),fault:0}};

function lineSensorKv(text){return window.LineSensorCore?LineSensorCore.parseKv(text):{}}
function lineSensorList(value){return window.LineSensorCore?LineSensorCore.parseList(value,8):[]}
function lineSensorOk(text){return /:\s*ok(?:\s|\r|\n|$)/i.test(String(text||''))}
function lineSensorToast(message,bad){
  var toast=$('lineSensorToast');toast.textContent=message;
  toast.className='show'+(bad?' bad':'');clearTimeout(lineSensorToast.timer);
  lineSensorToast.timer=setTimeout(function(){toast.className=''},3000);
}
function lineSensorSetText(id,value,suffix){
  $(id).textContent=value===undefined||value===null?'--':String(value)+(suffix||'');
}
function lineSensorCommand(action){return LineSensorCore.calibrationCommand(action)}

function lineSensorBuildAdcUi(){
  var bars='',rows='';
  for(var i=0;i<8;i++){
    bars+='<div class="gray-channel"><div class="gray-bar-track"><span id="grayRawBar'+i+'" class="gray-bar-raw"></span><span id="grayNormBar'+i+'" class="gray-bar-normalized"></span></div><label>CH'+i+'</label><code id="grayRawValue'+i+'">--</code></div>';
    rows+='<div class="gray-preview-row"><span>CH'+i+'</span><span id="grayWhite'+i+'">--</span><span id="grayBlack'+i+'">--</span><span id="graySpan'+i+'">--</span><span id="grayNoise'+i+'">--</span><span id="grayResult'+i+'" class="wait">等待</span></div>';
  }
  $('grayChannelBars').innerHTML=bars;$('grayPreviewRows').innerHTML=rows;
}

function lineSensorRender(){
  var gray=lineSensorState.gray,connected=lineSensorState.connected,
    ready=gray.liveValid&&connected;
  lineSensorSetText('lineDirtyState',lineSensorState.dirty?'未保存':'已同步');
  $('btnLineSave').disabled=!connected||lineSensorState.busy||!lineSensorState.dirty;
  $('btnLineSensorRefresh').disabled=!connected||lineSensorState.busy;
  $('btnGrayCalBegin').disabled=!ready||lineSensorState.busy||gray.running;
  $('btnGrayCalWhite').disabled=!ready||!gray.session||gray.running||lineSensorState.busy;
  $('btnGrayCalBlack').disabled=!ready||!gray.session||gray.running||lineSensorState.busy;
  $('btnGrayCalCommit').disabled=!connected||!gray.session||gray.running||
    !gray.whiteReady||!gray.blackReady||gray.fault!==0||lineSensorState.busy;
  $('btnGrayCalCancel').disabled=!connected||!gray.session||lineSensorState.busy;
  $('grayCalProgress').max=gray.target||64;$('grayCalProgress').value=gray.samples||0;
  $('grayCalSamples').textContent=(gray.samples||0)+'/'+(gray.target||64);
  $('grayCalState').textContent=gray.running?'正在采集'+(gray.phase==='white'?'白底':'黑线'):
    gray.session?'标定会话进行中':'未开始';
}

function lineSensorRenderStatus(values){
  var age=Number(values.age_ms),fresh=values.fresh==='1'&&Number.isFinite(age)&&age<=200;
  lineSensorSetText('lineSensorReady',values.ready==='1'?'就绪':'未就绪');
  lineSensorSetText('lineSensorValid',values.valid==='1'?'有效':'无效');
  lineSensorSetText('lineSensorAge',values.age_ms,' ms');
  $('lineLiveStatus').textContent=values.valid==='1'&&fresh?
    (values.calibrated==='1'?'有效':'未标定'):'无有效数据';
}

function lineSensorRenderGrayLive(values){
  var raw=lineSensorList(values.raw),normalized=lineSensorList(values.normalized),
    gray=lineSensorState.gray,age=Number(values.age_ms);
  gray.age=Number.isFinite(age)?age:Infinity;
  gray.liveValid=values.valid==='1'&&Number.isFinite(age)&&age<=200&&raw.length===8;
  $('lineLiveStatus').textContent=gray.liveValid?(values.processed==='1'?'有效':'原始数据有效'):
    values.valid!=='1'?'八路帧无效':Number.isFinite(age)&&age>200?'数据过期 ('+age+' ms)':'数据格式无效';
  lineSensorSetText('graySequence',values.seq);lineSensorSetText('grayAge',values.age_ms,' ms');
  lineSensorSetText('grayPosition',values.position);lineSensorSetText('grayStrength',values.strength);
  lineSensorSetText('graySampleStatus',values.status);lineSensorSetText('grayFaultMask',values.fault);
  lineSensorSetText('grayAnomalyMask',values.anomaly);
  for(var i=0;i<8;i++){
    var rawValue=raw[i]||0,normValue=normalized.length===8?(normalized[i]||0):0;
    $('grayRawBar'+i).style.height=Math.max(0,Math.min(100,rawValue*100/4095))+'%';
    $('grayNormBar'+i).style.height=Math.max(0,Math.min(100,normValue/10))+'%';
    $('grayRawValue'+i).textContent=raw.length===8?rawValue:'--';
  }
}

function lineSensorRenderGrayCal(values){
  var gray=lineSensorState.gray;
  gray.session=values.session==='1';gray.running=values.running==='1';gray.phase=values.phase||'idle';
  var parts=String(values.samples||'0').split('/');
  gray.samples=Number(parts[0])||0;gray.target=Number(values.target_samples||parts[1])||64;
  gray.last=values.last||'ok';gray.whiteReady=values.white==='1';gray.blackReady=values.black==='1';
  gray.fault=parseInt(values.fault||'0',0)||0;
  var notice=$('grayCalNotice');notice.className='';
  if(gray.last==='timeout'){
    notice.className='error';notice.textContent='采集超时：500 ms 内没有新的完整八路帧。请检查 gray live、ADC 中断和复用器接线后重试。';
  }else if(!gray.liveValid){
    notice.className='error';notice.textContent=Number.isFinite(gray.age)&&gray.age>200?
      '八路数据已过期（'+gray.age+' ms，要求不超过 200 ms），不能开始采集。':
      '没有有效的完整八路帧，不能开始采集。请检查 ADC 和复用器。';
  }else if(gray.running){
    notice.textContent='正在采集'+(gray.phase==='white'?'白底':'黑线')+'，请保持传感器位置稳定。';
  }else if(gray.whiteReady&&gray.blackReady&&gray.fault===0){
    notice.className='ok';notice.textContent='8 路黑白样本均通过跨度和噪声校验，可以提交到 RAM。';
  }else if(gray.whiteReady&&gray.blackReady){
    notice.className='error';notice.textContent='存在失败通道，请查看下方跨度/噪声并重新采集。';
  }else if(gray.session){notice.textContent='请依次采集白底和黑线；重复点击可覆盖对应暂存样本。';
  }else{notice.textContent='请先确认 8 路实时值持续更新，再开始新标定。'}
}

function lineSensorRenderGrayPreview(values){
  var gray=lineSensorState.gray,fields=['white','black','white_noise','black_noise','span'];
  fields.forEach(function(field){
    var parsed=lineSensorList(values[field]);
    if(parsed.length===8){
      var key=field.replace(/_([a-z])/g,function(_,c){return c.toUpperCase()});gray[key]=parsed;
    }
  });
  gray.whiteReady=values.white_ready==='1';gray.blackReady=values.black_ready==='1';
  gray.fault=parseInt(values.fault||'0',0)||0;
  for(var i=0;i<8;i++){
    var noise=Math.max(gray.whiteNoise[i]||0,gray.blackNoise[i]||0),
      ready=gray.whiteReady&&gray.blackReady,failed=ready&&((gray.fault&(1<<i))!==0),result=$('grayResult'+i);
    $('grayWhite'+i).textContent=gray.whiteReady?gray.white[i]:'--';
    $('grayBlack'+i).textContent=gray.blackReady?gray.black[i]:'--';
    $('graySpan'+i).textContent=ready?gray.span[i]:'--';$('grayNoise'+i).textContent=ready?noise:'--';
    result.textContent=!ready?'等待':failed?'失败':'通过';result.className=!ready?'wait':failed?'fail':'pass';
  }
}

function lineSensorSchedulePoll(delay){
  clearTimeout(lineSensorState.pollTimer);lineSensorState.pollTimer=null;
  if(!lineSensorState.visible||!lineSensorState.connected)return;
  lineSensorState.pollTimer=setTimeout(function(){lineSensorState.pollTimer=null;lineSensorPoll()},delay);
}
async function lineSensorPoll(){
  if(!lineSensorState.visible||!lineSensorState.connected||lineSensorState.busy||lineSensorState.polling)return;
  lineSensorState.polling=true;
  try{
    var includeSlow=lineSensorState.gray.running||(lineSensorState.pollCount++%2)===0;
    var commands=LineSensorCore.pollCommands(includeSlow);
    for(var i=0;i<commands.length;i++){
      var command=commands[i],values=lineSensorKv(await send(command,{timeoutMs:1600}));
      if(command==='linesensor status')lineSensorRenderStatus(values);
      else if(command==='gray live')lineSensorRenderGrayLive(values);
      else if(command==='gray calib status')lineSensorRenderGrayCal(values);
      else if(command==='gray calib preview')lineSensorRenderGrayPreview(values);
      else if(command==='param status')lineSensorState.dirty=values.dirty==='1';
    }
  }catch(error){lineSensorToast('刷新失败：'+error.message,true)}
  finally{lineSensorState.polling=false;lineSensorRender();lineSensorSchedulePoll(lineSensorState.gray.running?180:900)}
}
async function lineSensorAction(command,success){
  if(lineSensorState.busy)return false;lineSensorState.busy=true;lineSensorRender();
  try{
    var response=await send(command,{timeoutMs:3500});
    if(!lineSensorOk(response))throw new Error(response.trim()||'设备拒绝命令');
    lineSensorToast(success,false);return true;
  }catch(error){lineSensorToast(error.message,true);return false}
  finally{lineSensorState.busy=false;lineSensorRender();lineSensorSchedulePoll(40)}
}

function LineSensorPage_OnShow(){lineSensorState.visible=true;lineSensorRender();lineSensorSchedulePoll(0)}
function LineSensorPage_OnHide(){lineSensorState.visible=false;clearTimeout(lineSensorState.pollTimer);lineSensorState.pollTimer=null}

lineSensorBuildAdcUi();
$('btnLineSensorRefresh').onclick=function(){lineSensorSchedulePoll(0)};
$('btnLineSave').onclick=async function(){
  if(!window.confirm('确认把当前 RAM 中的全部参数（包括 ADC8 标定）写入 FRAM？'))return;
  if(await lineSensorAction('param save','当前传感器和参数已保存到 FRAM'))lineSensorState.dirty=false;
};
$('btnGrayCalBegin').onclick=async function(){if(await lineSensorAction(lineSensorCommand('begin'),'八路标定会话已开始')){var g=lineSensorState.gray;g.session=true;g.running=false;g.whiteReady=false;g.blackReady=false;g.fault=0;lineSensorRender()}};
$('btnGrayCalWhite').onclick=async function(){if(await lineSensorAction(lineSensorCommand('white'),'正在采集白底 64 帧')){var g=lineSensorState.gray;g.running=true;g.phase='white';g.samples=0;lineSensorRender()}};
$('btnGrayCalBlack').onclick=async function(){if(await lineSensorAction(lineSensorCommand('black'),'正在采集黑线 64 帧')){var g=lineSensorState.gray;g.running=true;g.phase='black';g.samples=0;lineSensorRender()}};
$('btnGrayCalCancel').onclick=async function(){if(await lineSensorAction(lineSensorCommand('cancel'),'已取消八路标定')){lineSensorState.gray.session=false;lineSensorState.gray.running=false;lineSensorRender()}};
$('btnGrayCalCommit').onclick=async function(){if(await lineSensorAction(lineSensorCommand('commit'),'八路标定已提交到 RAM，请验证后显式保存')){lineSensorState.gray.session=false;lineSensorState.gray.running=false;lineSensorState.dirty=true;lineSensorRender()}};

var lineSensorPreviousSerialState=onSerialStateChange;
onSerialStateChange=function(connected){
  if(typeof lineSensorPreviousSerialState==='function')lineSensorPreviousSerialState(connected);
  lineSensorState.connected=connected;
  var badge=$('lineSensorConnection');badge.textContent=connected?(simMode?'模拟连接':'真实串口已连接'):'未连接';
  badge.className=connected?(simMode?'sim':'ok'):'';$('graySimControls').hidden=!(connected&&simMode);
  if(connected&&lineSensorState.visible)lineSensorPoll();lineSensorRender();
};

var lineSensorSim={dirty:false,seq:0,started:Date.now(),graySession:false,
  grayRunning:false,grayPhase:'idle',graySamples:0,grayWhite:false,grayBlack:false,
  grayFault:0,grayLast:'ok'};
function lineSensorSimCommand(cmd){
  var sim=lineSensorSim,t=(Date.now()-sim.started)/1000;
  if(cmd==='linesensor status')return'linesensor source=adc8 ready=1 valid=1 fresh=1 calibrated=1 age_ms=2 status=ok\r\n> ';
  if(cmd==='gray live'){
    sim.seq++;var raw=[],norm=[];
    for(var g=0;g<8;g++){raw.push(2100+Math.round(Math.sin(t*2+g*.55)*900));norm.push(Math.max(0,Math.min(1000,Math.round((3500-raw[g])/2.8))))}
    return'gray live valid=1 processed=1 seq='+sim.seq+' age_ms=2 frame_ms=7 raw='+raw.join(',')+' normalized='+norm.join(',')+' position='+Math.round(Math.sin(t)*1200)+' strength=1800 line=1 fault=0x00 saturation=0x00 anomaly=0x00 status=ok process_status=ok\r\n> ';
  }
  if(cmd==='gray calib begin'){sim.graySession=true;sim.grayRunning=false;sim.grayWhite=false;sim.grayBlack=false;sim.grayFault=0;sim.grayLast='ok';return'gray calib begin: ok\r\n> '}
  if(cmd.indexOf('gray calib white')===0||cmd.indexOf('gray calib black')===0){sim.graySession=true;sim.grayRunning=true;sim.grayPhase=cmd.indexOf('white')>=0?'white':'black';sim.graySamples=0;sim.grayLast='busy';return'gray calib capture: ok\r\n> '}
  if(cmd==='gray calib status'){
    if(sim.grayRunning){sim.graySamples=Math.min(64,sim.graySamples+16);if(sim.graySamples===64){sim.grayRunning=false;if(sim.grayPhase==='white')sim.grayWhite=true;else sim.grayBlack=true;sim.grayPhase='idle';sim.grayLast='ok'}}
    return'gray calib session='+(sim.graySession?1:0)+' running='+(sim.grayRunning?1:0)+' mode='+(sim.grayPhase==='white'?2:sim.grayPhase==='black'?3:0)+' phase='+sim.grayPhase+' samples='+sim.graySamples+' target_samples=64 white='+(sim.grayWhite?1:0)+' black='+(sim.grayBlack?1:0)+' fault=0x'+sim.grayFault.toString(16).padStart(2,'0')+' last='+(sim.grayRunning?'busy':sim.grayLast)+'\r\n> ';
  }
  if(cmd==='gray calib preview'){
    var white='3300,3280,3260,3240,3220,3200,3180,3160',black='900,920,940,960,980,1000,1020,1040',span=sim.grayFault?'2400,2360,2320,100,2240,2200,2160,2120':'2400,2360,2320,2280,2240,2200,2160,2120';
    return'gray calib preview session='+(sim.graySession?1:0)+' white_ready='+(sim.grayWhite?1:0)+' black_ready='+(sim.grayBlack?1:0)+' white='+white+' black='+black+' white_noise=18,17,16,20,19,18,17,16 black_noise=15,14,13,17,16,15,14,13 span='+span+' fault=0x'+sim.grayFault.toString(16).padStart(2,'0')+'\r\n> ';
  }
  if(cmd==='gray calib commit'){sim.graySession=false;sim.dirty=true;return'gray calib commit: ok\r\n> '}
  if(cmd==='gray calib cancel'){sim.graySession=false;sim.grayRunning=false;return'gray calib cancel: ok\r\n> '}
  return cmd+': simulated\r\n> ';
}

$('btnGraySimFailure').onclick=function(){var s=lineSensorSim;s.graySession=true;s.grayRunning=false;s.grayWhite=true;s.grayBlack=true;s.grayFault=0x08;s.grayLast='invalid-arg';lineSensorSchedulePoll(0)};
$('btnGraySimTimeout').onclick=function(){var s=lineSensorSim;s.graySession=true;s.grayRunning=false;s.graySamples=16;s.grayWhite=false;s.grayBlack=false;s.grayFault=0;s.grayLast='timeout';lineSensorSchedulePoll(0)};
