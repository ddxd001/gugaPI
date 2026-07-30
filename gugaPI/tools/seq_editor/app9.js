'use strict';

var lineSensorState={visible:false,connected:false,busy:false,polling:false,
  selected:'ir3',active:null,fram:null,dirty:false,pollTimer:null,pollCount:0,
  ir:{active:false,step:'none',samples:'0/64',lastCount:0,lastProgress:0,
    ready:{white:false,black:false,center:false,left:false,right:false}},
  gray:{session:false,running:false,phase:'idle',samples:0,target:64,last:'ok',
    whiteReady:false,blackReady:false,liveValid:false,age:0,
    white:Array(8).fill(0),black:Array(8).fill(0),whiteNoise:Array(8).fill(0),
    blackNoise:Array(8).fill(0),span:Array(8).fill(0),fault:0}};
var lineSensorCalSteps=['white','black','center','left','right'];
var lineSensorCalLabels={white:'白底',black:'全黑',center:'线路居中',left:'线路最左',right:'线路最右'};

function lineSensorKv(text){
  return window.LineSensorCore?LineSensorCore.parseKv(text):{};
}
function lineSensorList(value){
  return window.LineSensorCore?LineSensorCore.parseList(value,8):[];
}
function lineSensorOk(text){return /:\s*ok(?:\s|\r|\n|$)/i.test(String(text||''))}
function lineSensorLabel(source){return source==='adc8'?'八路 ADC':'三路串口红外'}
function lineSensorToast(message,bad){
  var toast=$('lineSensorToast');toast.textContent=message;
  toast.className='show'+(bad?' bad':'');clearTimeout(lineSensorToast.timer);
  lineSensorToast.timer=setTimeout(function(){toast.className=''},3000);
}
function lineSensorSetText(id,value,suffix){
  $(id).textContent=value===undefined||value===null?'--':String(value)+(suffix||'');
}
function lineSensorCommand(source,action){
  return LineSensorCore.calibrationCommand(source,action);
}
function lineSensorHex(value){
  var parsed=Number(value);return Number.isFinite(parsed)?'0x'+parsed.toString(16).toUpperCase().padStart(2,'0'):'--';
}

function lineSensorBuildAdcUi(){
  var bars='',rows='';
  for(var i=0;i<8;i++){
    bars+='<div class="gray-channel"><div class="gray-bar-track"><span id="grayRawBar'+i+'" class="gray-bar-raw"></span><span id="grayNormBar'+i+'" class="gray-bar-normalized"></span></div><label>CH'+i+'</label><code id="grayRawValue'+i+'">--</code></div>';
    rows+='<div class="gray-preview-row"><span>CH'+i+'</span><span id="grayWhite'+i+'">--</span><span id="grayBlack'+i+'">--</span><span id="graySpan'+i+'">--</span><span id="grayNoise'+i+'">--</span><span id="grayResult'+i+'" class="wait">等待</span></div>';
  }
  $('grayChannelBars').innerHTML=bars;$('grayPreviewRows').innerHTML=rows;
}

function lineSensorRender(){
  var source=lineSensorState.active||'ir3',isAdc=source==='adc8';
  document.querySelectorAll('.line-source-card').forEach(function(card){
    card.classList.toggle('selected',card.dataset.source===lineSensorState.selected);
    card.classList.toggle('active',card.dataset.source===lineSensorState.active);
  });
  lineSensorSetText('lineActiveSource',lineSensorState.active?lineSensorLabel(lineSensorState.active):'--');
  lineSensorSetText('lineFramSource',lineSensorState.fram?lineSensorLabel(lineSensorState.fram):'--');
  lineSensorSetText('lineDirtyState',lineSensorState.dirty?'未保存':'已同步');
  var pending=lineSensorState.active&&lineSensorState.selected!==lineSensorState.active;
  $('lineSourcePending').hidden=!pending;
  $('lineSourcePending').textContent=pending?'已选择“'+lineSensorLabel(lineSensorState.selected)+'”，请先应用到 RAM；当前仍显示“'+lineSensorLabel(lineSensorState.active)+'”的数据和标定。':'';
  $('lineIrCalibration').hidden=isAdc;$('lineAdcCalibration').hidden=!isAdc;
  $('lineIrLive').hidden=isAdc;$('lineAdcLive').hidden=!isAdc;
  $('btnLineApply').disabled=!lineSensorState.connected||lineSensorState.busy||lineSensorState.selected===lineSensorState.active;
  $('btnLineSave').disabled=!lineSensorState.connected||lineSensorState.busy||!lineSensorState.dirty;
  $('btnLineSensorRefresh').disabled=!lineSensorState.connected||lineSensorState.busy;
  $('btnLineClearStats').disabled=!lineSensorState.connected||lineSensorState.busy||isAdc;
  lineSensorRenderIrControls(isAdc);
  lineSensorRenderGrayControls(isAdc);
}

function lineSensorRenderIrControls(isAdc){
  var ir=lineSensorState.ir;
  $('btnLineCalBegin').disabled=isAdc||!lineSensorState.connected||lineSensorState.busy||ir.active;
  $('btnLineCalCancel').disabled=isAdc||!lineSensorState.connected||lineSensorState.busy||!ir.active;
  var allReady=lineSensorCalSteps.every(function(step){return ir.ready[step]});
  $('btnLineCalCommit').disabled=isAdc||!lineSensorState.connected||lineSensorState.busy||!ir.active||!allReady;
  document.querySelectorAll('[data-cal-step]').forEach(function(button){
    var step=button.dataset.calStep,done=ir.ready[step],capturing=ir.active&&ir.step===step;
    button.disabled=isAdc||!lineSensorState.connected||lineSensorState.busy||!ir.active||ir.step!=='none';
    button.classList.toggle('done',done);button.classList.toggle('capturing',capturing);
    button.querySelector('em').textContent=done?'已完成':capturing?ir.samples:'等待';
  });
}

function lineSensorRenderGrayControls(isAdc){
  var gray=lineSensorState.gray,connected=lineSensorState.connected,
    ready=isAdc&&gray.liveValid&&connected;
  $('btnGrayCalBegin').disabled=!ready||!lineSensorState.connected||lineSensorState.busy||gray.running;
  $('btnGrayCalWhite').disabled=!ready||!gray.session||gray.running||lineSensorState.busy;
  $('btnGrayCalBlack').disabled=!ready||!gray.session||gray.running||lineSensorState.busy;
  $('btnGrayCalCommit').disabled=!connected||!gray.session||gray.running||!gray.whiteReady||!gray.blackReady||gray.fault!==0||lineSensorState.busy;
  $('btnGrayCalCancel').disabled=!connected||!gray.session||lineSensorState.busy;
  $('grayCalProgress').max=gray.target||64;$('grayCalProgress').value=gray.samples||0;
  $('grayCalSamples').textContent=(gray.samples||0)+'/'+(gray.target||64);
  $('grayCalState').textContent=gray.running?'正在采集'+(gray.phase==='white'?'白底':'黑线'):
    gray.session?'标定会话进行中':'未开始';
}

function lineSensorRenderStatus(values){
  if(values.active){
    if(lineSensorState.active===null)lineSensorState.selected=values.active;
    lineSensorState.active=values.active;
    if(lineSensorState.fram===null)lineSensorState.fram=values.configured||values.active;
    $('lineLiveStatus').textContent=values.valid==='1'?(values.calibrated==='1'?'有效':'未标定'):'无有效数据';
    if(values.age_ms!==undefined)lineSensorSetText('lineAge',values.age_ms,' ms');
  }
  lineSensorRender();
}

function lineSensorRenderIrRaw(values){
  lineSensorSetText('lineRawOffset',values.offset_raw);lineSensorSetText('linePosition',values.position_mpos,' mpos');
  lineSensorSetText('lineAllBlack',values.all_black==='1'?'是 (1)':values.all_black==='0'?'否 (0)':undefined);
  lineSensorSetText('lineAdc1',values.adc1);lineSensorSetText('lineAdc2',values.adc2);lineSensorSetText('lineAdc3',values.adc3);
}
function lineSensorRenderIrStats(values){
  lineSensorSetText('lineFrames',values.frames);lineSensorSetText('lineCrcErrors',values.crc_errors);
  lineSensorSetText('lineHeaderErrors',values.header_errors);lineSensorSetText('lineUartErrors',values.uart_errors);
  lineSensorSetText('lineDropped',values.dropped);lineSensorSetText('linePeriod',values.average_ms,' ms');
}
function lineSensorRenderIrCal(values){
  if(!Object.prototype.hasOwnProperty.call(values,'active'))return;
  var ir=lineSensorState.ir;ir.active=values.active==='1';ir.step=values.step||'none';ir.samples=values.samples||'0/64';
  var count=Number(String(ir.samples).split('/')[0])||0;
  if(ir.step!=='none'&&count!==ir.lastCount){ir.lastCount=count;ir.lastProgress=Date.now()}
  var ready=String(values.ready||'00000').padEnd(5,'0');
  lineSensorCalSteps.forEach(function(step,index){ir.ready[step]=ready[index]==='1'});
  var capturing=ir.step!=='none',allReady=lineSensorCalSteps.every(function(step){return ir.ready[step]});
  var next=lineSensorCalSteps.find(function(step){return !ir.ready[step]});
  var stalled=capturing&&ir.lastProgress&&Date.now()-ir.lastProgress>2500;
  $('lineCalState').textContent=capturing?'正在采集 '+lineSensorCalLabels[ir.step]+' '+ir.samples:ir.active?'标定进行中':'未开始';
  $('lineCalNotice').className=allReady?'ok':stalled?'error':'';
  $('lineCalNotice').textContent=allReady?'五步数据已齐全，请提交到 RAM。':stalled?'超过 2.5 秒没有新帧，请检查 CRC、UART 和接线。':capturing?'正在自动采集“'+lineSensorCalLabels[ir.step]+'”，请保持位置不动。':ir.active&&next?'请摆放到“'+lineSensorCalLabels[next]+'”并开始采集。':'点击“开始新标定”后自动采集白底。';
  lineSensorRender();
}

function lineSensorRenderGrayLive(values){
  var raw=lineSensorList(values.raw),normalized=lineSensorList(values.normalized),gray=lineSensorState.gray;
  var age=Number(values.age_ms);gray.age=Number.isFinite(age)?age:Infinity;
  gray.liveValid=values.valid==='1'&&Number.isFinite(age)&&age<=200&&raw.length===8;
  $('lineLiveStatus').textContent=gray.liveValid?(values.processed==='1'?'有效':'原始数据有效'):
    values.valid!=='1'?'八路帧无效':Number.isFinite(age)&&age>200?'数据过期 ('+age+' ms)':'数据格式无效';
  lineSensorSetText('graySequence',values.seq);lineSensorSetText('grayAge',values.age_ms,' ms');
  lineSensorSetText('grayPosition',values.position);lineSensorSetText('grayStrength',values.strength);
  lineSensorSetText('graySampleStatus',values.status);lineSensorSetText('grayFaultMask',values.fault);lineSensorSetText('grayAnomalyMask',values.anomaly);
  for(var i=0;i<8;i++){
    var rawValue=raw[i]||0,normValue=normalized.length===8?(normalized[i]||0):0;
    $('grayRawBar'+i).style.height=Math.max(0,Math.min(100,rawValue*100/4095))+'%';
    $('grayNormBar'+i).style.height=Math.max(0,Math.min(100,normValue/10))+'%';
    $('grayRawValue'+i).textContent=raw.length===8?rawValue:'--';
  }
  lineSensorRender();
}

function lineSensorRenderGrayCal(values){
  var gray=lineSensorState.gray;
  gray.session=values.session==='1';gray.running=values.running==='1';gray.phase=values.phase||'idle';
  var sampleParts=String(values.samples||'0').split('/');
  gray.samples=Number(sampleParts[0])||0;gray.target=Number(values.target_samples||sampleParts[1])||64;gray.last=values.last||'ok';
  gray.whiteReady=values.white==='1';gray.blackReady=values.black==='1';gray.fault=parseInt(values.fault||'0',0)||0;
  var notice=$('grayCalNotice');notice.className='';
  if(gray.last==='timeout'){notice.className='error';notice.textContent='采集超时：500 ms 内没有新的完整八路帧。请检查 gray live、ADC 中断和复用器接线后重试。'}
  else if(!gray.liveValid){notice.className='error';notice.textContent=Number.isFinite(gray.age)&&gray.age>200?'八路数据已过期（'+gray.age+' ms，要求不超过 200 ms），不能开始采集。':'没有有效的完整八路帧，不能开始采集。请检查 ADC 和复用器。'}
  else if(gray.running){notice.textContent='正在采集'+(gray.phase==='white'?'白底':'黑线')+'，请保持传感器位置稳定。'}
  else if(gray.whiteReady&&gray.blackReady&&gray.fault===0){notice.className='ok';notice.textContent='8 路黑白样本均通过跨度和噪声校验，可以提交到 RAM。'}
  else if(gray.whiteReady&&gray.blackReady){notice.className='error';notice.textContent='存在失败通道，请查看下方跨度/噪声并重新采集。'}
  else if(gray.session){notice.textContent='请依次采集白底和黑线；重复点击可覆盖对应暂存样本。'}
  else{notice.textContent='请先确认 8 路实时值持续更新，再开始新标定。'}
  lineSensorRender();
}

function lineSensorRenderGrayPreview(values){
  var gray=lineSensorState.gray,fields=['white','black','white_noise','black_noise','span'];
  fields.forEach(function(field){var parsed=lineSensorList(values[field]);if(parsed.length===8){var key=field.replace(/_([a-z])/g,function(_,c){return c.toUpperCase()});gray[key]=parsed}});
  gray.whiteReady=values.white_ready==='1';gray.blackReady=values.black_ready==='1';gray.fault=parseInt(values.fault||'0',0)||0;
  for(var i=0;i<8;i++){
    var noise=Math.max(gray.whiteNoise[i]||0,gray.blackNoise[i]||0),ready=gray.whiteReady&&gray.blackReady;
    var failed=ready&&((gray.fault&(1<<i))!==0),result=$('grayResult'+i);
    $('grayWhite'+i).textContent=gray.whiteReady?gray.white[i]:'--';$('grayBlack'+i).textContent=gray.blackReady?gray.black[i]:'--';
    $('graySpan'+i).textContent=ready?gray.span[i]:'--';$('grayNoise'+i).textContent=ready?noise:'--';
    result.textContent=!ready?'等待':failed?'失败':'通过';result.className=!ready?'wait':failed?'fail':'pass';
  }
  lineSensorRender();
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
    lineSensorRenderStatus(lineSensorKv(await send('linesensor status',{timeoutMs:1600})));
    var source=lineSensorState.active||'ir3';
    var capturing=source==='adc8'?lineSensorState.gray.running:lineSensorState.ir.step!=='none';
    var includeSlow=capturing||(lineSensorState.pollCount++%2)===0;
    var commands=LineSensorCore.pollCommands(source,includeSlow).slice(1);
    for(var i=0;i<commands.length;i++){
      var command=commands[i],values=lineSensorKv(await send(command,{timeoutMs:1600}));
      if(command==='gray live')lineSensorRenderGrayLive(values);
      else if(command==='gray calib status')lineSensorRenderGrayCal(values);
      else if(command==='gray calib preview')lineSensorRenderGrayPreview(values);
      else if(command==='irsensor raw')lineSensorRenderIrRaw(values);
      else if(command==='irsensor stats')lineSensorRenderIrStats(values);
      else if(command==='irsensor calib status')lineSensorRenderIrCal(values);
      else if(command==='param status')lineSensorState.dirty=values.dirty==='1';
    }
  }catch(error){lineSensorToast('刷新失败：'+error.message,true)}
  finally{lineSensorState.polling=false;lineSensorRender();var fast=lineSensorState.gray.running||lineSensorState.ir.step!=='none';lineSensorSchedulePoll(fast?180:900)}
}
async function lineSensorAction(command,success){
  if(lineSensorState.busy)return false;lineSensorState.busy=true;lineSensorRender();
  try{var response=await send(command,{timeoutMs:3500});if(!lineSensorOk(response))throw new Error(response.trim()||'设备拒绝命令');lineSensorToast(success,false);return true}
  catch(error){lineSensorToast(error.message,true);return false}
  finally{lineSensorState.busy=false;lineSensorRender();lineSensorSchedulePoll(40)}
}
async function lineSensorStartIrCapture(step){
  var ir=lineSensorState.ir;if(!ir.active||ir.step!=='none')return false;
  ir.step=step;ir.samples='0/64';ir.lastCount=0;ir.lastProgress=Date.now();lineSensorRender();
  var ok=await lineSensorAction(lineSensorCommand('ir3',step),'正在自动采集：'+lineSensorCalLabels[step]);
  if(!ok){ir.step='none';lineSensorRender()}return ok;
}

function LineSensorPage_OnShow(){lineSensorState.visible=true;lineSensorRender();lineSensorSchedulePoll(0)}
function LineSensorPage_OnHide(){lineSensorState.visible=false;clearTimeout(lineSensorState.pollTimer);lineSensorState.pollTimer=null}

lineSensorBuildAdcUi();
document.querySelectorAll('.line-source-card').forEach(function(card){card.onclick=function(){lineSensorState.selected=card.dataset.source;lineSensorRender()}});
$('btnLineSensorRefresh').onclick=function(){lineSensorSchedulePoll(0)};
$('btnLineApply').onclick=async function(){var source=lineSensorState.selected;if(await lineSensorAction('linesensor source '+source,'已应用到 RAM，尚未写入 FRAM')){lineSensorState.active=source;lineSensorRender()}};
$('btnLineSave').onclick=async function(){if(!window.confirm('确认把当前 RAM 中的全部参数（包括线路传感器标定）写入 FRAM？'))return;if(await lineSensorAction('param save','当前传感器和参数已保存到 FRAM')){lineSensorState.fram=lineSensorState.active;lineSensorState.dirty=false}};
$('btnLineClearStats').onclick=function(){lineSensorAction('irsensor clear','通信统计已清零')};
$('btnLineCalBegin').onclick=async function(){if(await lineSensorAction(lineSensorCommand('ir3','begin'),'红外标定已开始')){var ir=lineSensorState.ir;ir.active=true;ir.step='none';lineSensorCalSteps.forEach(function(key){ir.ready[key]=false});lineSensorRender();await lineSensorStartIrCapture('white')}};
$('btnLineCalCancel').onclick=async function(){if(await lineSensorAction(lineSensorCommand('ir3','cancel'),'已取消红外标定')){lineSensorState.ir.active=false;lineSensorState.ir.step='none';lineSensorRender()}};
$('btnLineCalCommit').onclick=async function(){if(await lineSensorAction(lineSensorCommand('ir3','commit'),'红外标定已提交到 RAM，请验证后显式保存')){lineSensorState.ir.active=false;lineSensorState.ir.step='none';lineSensorState.dirty=true;lineSensorRender()}};
document.querySelectorAll('[data-cal-step]').forEach(function(button){button.onclick=function(){lineSensorStartIrCapture(button.dataset.calStep)}});
$('btnGrayCalBegin').onclick=async function(){if(await lineSensorAction(lineSensorCommand('adc8','begin'),'八路标定会话已开始')){var gray=lineSensorState.gray;gray.session=true;gray.running=false;gray.whiteReady=false;gray.blackReady=false;gray.fault=0;lineSensorRender()}};
$('btnGrayCalWhite').onclick=async function(){if(await lineSensorAction(lineSensorCommand('adc8','white'),'正在采集白底 64 帧')){lineSensorState.gray.running=true;lineSensorState.gray.phase='white';lineSensorState.gray.samples=0;lineSensorRender()}};
$('btnGrayCalBlack').onclick=async function(){if(await lineSensorAction(lineSensorCommand('adc8','black'),'正在采集黑线 64 帧')){lineSensorState.gray.running=true;lineSensorState.gray.phase='black';lineSensorState.gray.samples=0;lineSensorRender()}};
$('btnGrayCalCancel').onclick=async function(){if(await lineSensorAction(lineSensorCommand('adc8','cancel'),'已取消八路标定')){lineSensorState.gray.session=false;lineSensorState.gray.running=false;lineSensorRender()}};
$('btnGrayCalCommit').onclick=async function(){if(await lineSensorAction(lineSensorCommand('adc8','commit'),'八路标定已提交到 RAM，请验证后显式保存')){lineSensorState.gray.session=false;lineSensorState.gray.running=false;lineSensorState.dirty=true;lineSensorRender()}};

var lineSensorPreviousSerialState=onSerialStateChange;
onSerialStateChange=function(connected){
  if(typeof lineSensorPreviousSerialState==='function')lineSensorPreviousSerialState(connected);
  lineSensorState.connected=connected;lineSensorState.fram=connected?null:lineSensorState.fram;
  var badge=$('lineSensorConnection');badge.textContent=connected?(simMode?'模拟连接':'真实串口已连接'):'未连接';badge.className=connected?(simMode?'sim':'ok'):'';
  $('graySimControls').hidden=!(connected&&simMode);
  if(connected&&lineSensorState.visible)lineSensorPoll();lineSensorRender();
};

var lineSensorSim={source:'ir3',dirty:false,seq:0,started:Date.now(),irActive:false,
  irReady:{white:false,black:false,center:false,left:false,right:false},graySession:false,
  grayRunning:false,grayPhase:'idle',graySamples:0,grayWhite:false,grayBlack:false,
  grayFault:0,grayLast:'ok'};
function lineSensorSimCommand(cmd){
  var sim=lineSensorSim,t=(Date.now()-sim.started)/1000;
  if(cmd==='linesensor status')return'linesensor active='+sim.source+' configured='+sim.source+' ready=1 valid=1 fresh=1 calibrated=1 road_capable='+(sim.source==='adc8'?1:0)+' age_ms=2 status=ok\r\n> ';
  if(cmd.indexOf('linesensor source ')===0){sim.source=cmd.split(' ')[2];sim.dirty=true;if(typeof simParamDirty!=='undefined')simParamDirty=true;return'linesensor source: ok active='+sim.source+' dirty=1\r\n> '}
  if(cmd==='gray live'){sim.seq++;var raw=[],norm=[];for(var g=0;g<8;g++){raw.push(2100+Math.round(Math.sin(t*2+g*.55)*900));norm.push(Math.max(0,Math.min(1000,Math.round((3500-raw[g])/2.8))))}return'gray live valid=1 processed=1 seq='+sim.seq+' age_ms=2 frame_ms=7 raw='+raw.join(',')+' normalized='+norm.join(',')+' position='+Math.round(Math.sin(t)*1200)+' strength=1800 line=1 fault=0x00 saturation=0x00 anomaly=0x00 status=ok process_status=ok\r\n> '}
  if(cmd==='gray calib begin'){sim.graySession=true;sim.grayRunning=false;sim.grayWhite=false;sim.grayBlack=false;sim.grayFault=0;sim.grayLast='ok';return'gray calib begin: ok\r\n> '}
  if(cmd.indexOf('gray calib white')===0||cmd.indexOf('gray calib black')===0){sim.graySession=true;sim.grayRunning=true;sim.grayPhase=cmd.indexOf('white')>=0?'white':'black';sim.graySamples=0;sim.grayLast='busy';return'gray calib capture: ok\r\n> '}
  if(cmd==='gray calib status'){if(sim.grayRunning){sim.graySamples=Math.min(64,sim.graySamples+16);if(sim.graySamples===64){sim.grayRunning=false;if(sim.grayPhase==='white')sim.grayWhite=true;else sim.grayBlack=true;sim.grayPhase='idle';sim.grayLast='ok'}}return'gray calib session='+(sim.graySession?1:0)+' running='+(sim.grayRunning?1:0)+' mode='+(sim.grayPhase==='white'?2:sim.grayPhase==='black'?3:0)+' phase='+sim.grayPhase+' samples='+sim.graySamples+' target_samples=64 white='+(sim.grayWhite?1:0)+' black='+(sim.grayBlack?1:0)+' fault=0x'+sim.grayFault.toString(16).padStart(2,'0')+' last='+(sim.grayRunning?'busy':sim.grayLast)+'\r\n> '}
  if(cmd==='gray calib preview'){var white='3300,3280,3260,3240,3220,3200,3180,3160',black='900,920,940,960,980,1000,1020,1040',span=sim.grayFault?'2400,2360,2320,100,2240,2200,2160,2120':'2400,2360,2320,2280,2240,2200,2160,2120';return'gray calib preview session='+(sim.graySession?1:0)+' white_ready='+(sim.grayWhite?1:0)+' black_ready='+(sim.grayBlack?1:0)+' white='+white+' black='+black+' white_noise=18,17,16,20,19,18,17,16 black_noise=15,14,13,17,16,15,14,13 span='+span+' fault=0x'+sim.grayFault.toString(16).padStart(2,'0')+'\r\n> '}
  if(cmd==='gray calib commit'){sim.graySession=false;sim.dirty=true;return'gray calib commit: ok\r\n> '}
  if(cmd==='gray calib cancel'){sim.graySession=false;sim.grayRunning=false;return'gray calib cancel: ok\r\n> '}
  if(cmd==='irsensor raw'){sim.seq++;var offset=Math.round(Math.sin(t*1.5)*420);return'irsensor raw seq='+sim.seq+' offset_raw='+offset+' position_mpos='+Math.round(offset*5)+' all_black=0 adc1=1800 adc2=2700 adc3=1600 on=2300 off=1900\r\n> '}
  if(cmd==='irsensor stats')return'irsensor stats frames='+sim.seq+' header_errors=0 crc_errors=0 uart_errors=0 dropped=0 average_ms=10\r\n> ';
  if(cmd==='irsensor clear'){sim.seq=0;return'irsensor clear: ok\r\n> '}
  if(cmd==='irsensor calib begin'){sim.irActive=true;lineSensorCalSteps.forEach(function(k){sim.irReady[k]=false});return'irsensor calib begin: ok\r\n> '}
  if(cmd.indexOf('irsensor calib capture ')===0){sim.irReady[cmd.split(' ')[3]]=true;return'irsensor calib capture: ok\r\n> '}
  if(cmd==='irsensor calib status')return'irsensor calib active='+(sim.irActive?1:0)+' step=none samples=0/64 ready='+lineSensorCalSteps.map(function(k){return sim.irReady[k]?1:0}).join('')+' status=ok\r\n> ';
  if(cmd==='irsensor calib commit'){sim.irActive=false;sim.dirty=true;return'irsensor calib commit: ok\r\n> '}
  if(cmd==='irsensor calib cancel'){sim.irActive=false;return'irsensor calib cancel: ok\r\n> '}
  return cmd+': simulated\r\n> ';
}

$('btnGraySimFailure').onclick=function(){var sim=lineSensorSim;sim.graySession=true;sim.grayRunning=false;sim.grayWhite=true;sim.grayBlack=true;sim.grayFault=0x08;sim.grayLast='invalid-arg';lineSensorSchedulePoll(0)};
$('btnGraySimTimeout').onclick=function(){var sim=lineSensorSim;sim.graySession=true;sim.grayRunning=false;sim.graySamples=16;sim.grayWhite=false;sim.grayBlack=false;sim.grayFault=0;sim.grayLast='timeout';lineSensorSchedulePoll(0)};
