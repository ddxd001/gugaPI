'use strict';
const OP={0:'none',1:'drive',2:'turn',3:'follow',4:'wait',5:'stop',6:'branch',7:'end',8:'drive_mm',9:'led_on',10:'led_off',11:'led_toggle',12:'buzzer_on',13:'buzzer_off',14:'buzzer_toggle',15:'condition',16:'drive_if',17:'follow_if',18:'loop',19:'road_nav'};
const OPL={1:'直行',2:'转向',3:'循迹',4:'等待',5:'停车',6:'条件分支',7:'结束',8:'定距行驶',9:'LED 点亮',10:'LED 熄灭',11:'LED 翻转',12:'蜂鸣器开启',13:'蜂鸣器关闭',14:'蜂鸣器翻转',15:'通用判断',16:'条件直行',17:'条件循迹',18:'循环',19:'循迹通过路口'};
const OPC={1:'#89b4fa',2:'#fab387',3:'#a6e3a1',4:'#9399b2',5:'#f38ba8',6:'#cba6f7',7:'#6c7086',8:'#74c7ec',9:'#f9e2af',10:'#7f849c',11:'#f5c2e7',12:'#f38ba8',13:'#7f849c',14:'#eba0ac',15:'#cba6f7',16:'#89b4fa',17:'#a6e3a1',18:'#7b74d6',19:'#32b8a0'};
const COND=['timeout','heading_reached','line_detected','line_lost','button','immediate','distance_reached'];
const ROAD_ROUTE=['left','straight','right','uturn_left_arc','uturn_right_arc','uturn_left_pivot','uturn_right_pivot'];
let port=null,reader=null,writer=null,readableClosed=null,writableClosed=null;
let curSlot=-1,instrs=[],selIdx=-1,slots=Array(8).fill(null);
let rxBuf='',rxResolve=null,rxExpectedCommand='',simMode=false;
var onSerialData=null,onSerialStateChange=null,commandQueue=Promise.resolve();
var serialRouter=null;
let simSlots=Array(8).fill(null),simInstrs=[],simRun={running:false,current:0,started:0,result:'idle'};
simSlots[0]=[{op:8,p1:500,p2:80,until:6,ons:1,ont:255},{op:2,p1:90,p2:5000,until:1,ons:2,ont:255},{op:5,p1:0,p2:0,until:5,ons:3,ont:255},{op:7,p1:0,p2:0,until:5,ons:255,ont:255}];
simSlots[1]=[{op:3,p1:80,p2:30000,until:3,ons:1,ont:255},{op:5,p1:0,p2:0,until:5,ons:2,ont:255},{op:7,p1:0,p2:0,until:5,ons:255,ont:255}];
const $=id=>document.getElementById(id);
function logc(cls,msg){var el=$('log');el.innerHTML+='<span class="'+cls+'">'+msg+'</span>\n';el.scrollTop=el.scrollHeight}
function shellTextReceived(text){
  if(!text)return;
  logc('rx',text);
  if(typeof onSerialData==='function')onSerialData(text,'rx');
  rxBuf+=text;
  var responseComplete=typeof DashboardCore!=='undefined'&&
    typeof DashboardCore.isShellCommandResponseComplete==='function'?
      DashboardCore.isShellCommandResponseComplete(rxBuf,rxExpectedCommand):
      (/(?:^|\n)> $/.test(rxBuf)||rxBuf==='> ');
  if(responseComplete){
    if(rxResolve)rxResolve(rxBuf);
  }
}
function telemetryReceived(event,raw){
  if(typeof DashboardTelemetry_OnEvent==='function'){
    DashboardTelemetry_OnEvent(event,raw);
  }
  if(typeof Terminal_ShouldShowTelemetry==='function'&&
     Terminal_ShouldShowTelemetry()&&typeof onSerialData==='function'){
    onSerialData(raw,'telemetry');
  }
}
if(typeof DashboardCore!=='undefined'){
  serialRouter=new DashboardCore.SerialRouter({
    onText:shellTextReceived,
    onTelemetry:telemetryReceived
  });
}
function routeSerialData(data){
  if(serialRouter)serialRouter.push(data);
  else shellTextReceived(String(data).replace(/\r/g,''));
}
async function disconnectSerial(){
  try{
    if(typeof Dashboard_BeforeDisconnect==='function')await Dashboard_BeforeDisconnect();
    if(reader){await reader.cancel();if(readableClosed)await readableClosed.catch(function(){});reader.releaseLock()}
    if(writer){await writer.close();if(writableClosed)await writableClosed.catch(function(){});writer.releaseLock()}
    if(port)await port.close();
  }catch(e){logc('tx','[断开异常] '+e.message)}
  reader=null;writer=null;readableClosed=null;writableClosed=null;port=null;
  if(serialRouter)serialRouter.reset();
  $('btnConnect').textContent='连接串口';$('btnRefresh').disabled=true;
  $('statusText').textContent='';
  if(typeof onSerialStateChange==='function')onSerialStateChange(false);
}
$('btnConnect').onclick=async function(){
  if(port){await disconnectSerial();return}
  try{
    port=await navigator.serial.requestPort();await port.open({baudRate:115200});
    var dec=new TextDecoderStream();readableClosed=port.readable.pipeTo(dec.writable);reader=dec.readable.getReader();
    var enc=new TextEncoderStream();writableClosed=enc.readable.pipeTo(port.writable);writer=enc.writable.getWriter();
    $('btnConnect').textContent='断开串口';$('btnRefresh').disabled=false;
    $('statusText').textContent='已连接';logc('tx','[串口已连接]');
    if(typeof onSerialStateChange==='function')onSerialStateChange(true);
    (async function(){
      try{while(true){var r=await reader.read();if(r.done)break;if(r.value)routeSerialData(r.value)}}
      catch(e){if(typeof onSerialData==='function')onSerialData('[串口读取中断] '+e.message+'\n','error')}
    })();
    await refreshSlots();
  }catch(e){
    port=null;reader=null;writer=null;readableClosed=null;writableClosed=null;
    logc('tx','[连接失败] '+e.message);
    if(typeof onSerialStateChange==='function')onSerialStateChange(false);
  }
};
$('btnSim').onclick=async function(){
  if(simMode){
    simMode=false;port=null;writer=null;$('btnSim').textContent='模拟设备';
    $('btnConnect').disabled=false;$('btnRefresh').disabled=true;$('statusText').textContent='';
    logc('tx','[退出模拟]');
    if(typeof onSerialStateChange==='function')onSerialStateChange(false);
    return;
  }
  simMode=true;port={fake:true};writer={fake:true};$('btnSim').textContent='退出模拟';
  $('btnConnect').disabled=true;$('btnRefresh').disabled=false;$('statusText').textContent='模拟模式';
  logc('tx','[进入模拟]');
  if(typeof onSerialStateChange==='function')onSerialStateChange(true);
  await refreshSlots();
};
function send(cmd,options){var task=commandQueue.then(function(){return sendNow(cmd,options)});commandQueue=task.catch(function(){});return task}
async function sendNow(cmd,options){
  if(!writer)return'';
  logc('tx','> '+cmd);
  if(typeof onSerialData==='function')onSerialData('> '+cmd+'\n','tx');
  if(simMode)return simResponse(cmd);
  rxBuf='';
  rxExpectedCommand=cmd;
  await writer.write(cmd+'\r\n');
  var timeoutMs=options&&options.timeoutMs?options.timeoutMs:1500;
  return new Promise(function(resolve){
    var settled=false,timer;
    function finish(text){
      if(settled)return;
      settled=true;
      clearTimeout(timer);
      if(rxResolve===finish){
        rxResolve=null;
        rxExpectedCommand='';
      }
      resolve(text);
    }
    rxResolve=finish;
    timer=setTimeout(function(){finish(rxBuf)},timeoutMs);
    if(typeof DashboardCore!=='undefined'&&
       typeof DashboardCore.isShellCommandResponseComplete==='function'&&
       DashboardCore.isShellCommandResponseComplete(rxBuf,cmd)){
      finish(rxBuf);
    }
  });
}
function rawTarget(token){
  return token==='abort'||token==='next'?255:Number(token);
}
function parseRawInstruction(tokens,hasIndex){
  var p=tokens.slice(hasIndex?1:0),opName=p[0];
  var opKey=Object.keys(OP).find(function(k){return OP[k]===opName});
  if(opKey===undefined)return null;
  var op=Number(opKey);
  if(op===15&&p.length>=9){
    return{op:15,p1:0,p2:0,until:-1,source:p[1],compare:p[2],
      value:Number(p[3]),mode:p[4],timeoutMs:Number(p[5]),
      stableMs:Number(p[6]),ons:rawTarget(p[7]),ont:rawTarget(p[8])};
  }
  if((op===16||op===17)&&p.length>=9){
    return{op:op,p1:Number(p[1]),p2:0,until:-1,source:p[2],
      compare:p[3],value:Number(p[4]),mode:'wait',
      timeoutMs:Number(p[5]),stableMs:Number(p[6]),
      ons:rawTarget(p[7]),ont:rawTarget(p[8])};
  }
  if(op===19&&p.length>=6){
    return{op:19,p1:Number(p[2]),p2:Number(p[3]),until:5,
      route:p[1],conditionValue:ROAD_ROUTE.indexOf(p[1]),
      ons:rawTarget(p[4]),ont:rawTarget(p[5])};
  }
  if(p.length<6)return null;
  return{op:op,p1:Number(p[1]),p2:Number(p[2]),
    until:COND.indexOf(p[3]),conditionValue:0,
    ons:rawTarget(p[4]),ont:rawTarget(p[5])};
}
function formatRawInstruction(x,includeIndex,index){
  var prefix=includeIndex?index+' ':'';
  if(x.op===15){
    return prefix+'condition '+x.source+' '+x.compare+' '+x.value+' '+
      x.mode+' '+x.timeoutMs+' '+x.stableMs+' '+x.ons+' '+x.ont;
  }
  if(x.op===16||x.op===17){
    return prefix+OP[x.op]+' '+x.p1+' '+x.source+' '+x.compare+' '+
      x.value+' '+x.timeoutMs+' '+x.stableMs+' '+x.ons+' '+x.ont;
  }
  if(x.op===19){
    return prefix+'road_nav '+x.route+' '+x.p1+' '+x.p2+' '+
      x.ons+' '+x.ont;
  }
  return prefix+OP[x.op]+' '+x.p1+' '+x.p2+' '+COND[x.until]+' '+
    x.ons+' '+x.ont;
}
function parseSimAdd(cmd){
  var p=cmd.trim().split(/\s+/),op=p[2];
  if(op==='condition'){
    return{op:15,p1:0,p2:0,until:-1,source:p[3],compare:p[4],
      value:Number(p[5]),mode:p[6],timeoutMs:Number(p[7]),
      stableMs:Number(p[8]),ons:rawTarget(p[9]),ont:rawTarget(p[10])};
  }
  if(op==='drive_if'||op==='follow_if'){
    return{op:op==='drive_if'?16:17,p1:Number(p[3]),p2:0,until:-1,
      source:p[4],compare:p[5],value:Number(p[6]),mode:'wait',
      timeoutMs:Number(p[7]),stableMs:Number(p[8]),
      ons:rawTarget(p[9]),ont:rawTarget(p[10])};
  }
  if(op==='road_nav'){
    return{op:19,p1:Number(p[4]),p2:Number(p[5]),until:5,
      route:p[3],conditionValue:ROAD_ROUTE.indexOf(p[3]),
      ons:rawTarget(p[6]),ont:rawTarget(p[7])};
  }
  var opKey=Object.keys(OP).find(function(k){return OP[k]===op});
  return{op:Number(opKey),p1:Number(p[3]),p2:Number(p[4]),
    until:COND.indexOf(p[5]),conditionValue:0,
    ons:rawTarget(p[6]),ont:rawTarget(p[7])};
}
function simResponse(cmd){
  var resp='';
  if(cmd.indexOf('seq dump ')===0){
    var s=Number(cmd.split(' ')[2]),a=simSlots[s];
    resp='SEQ '+s+' '+(a?a.length:0)+'\r\n';
    (a||[]).forEach(function(x){
      resp+=formatRawInstruction(x,false,0)+'\r\n';
    });
    resp+='END\r\n> ';
  }else if(cmd==='seq list'){
    for(var i=0;i<8;i++){
      var sl=simSlots[i];
      resp+='seq '+i+' '+(sl&&sl.length?'ok':'empty')+
        ' count='+(sl?sl.length:0)+'\r\n';
    }
    resp+='> ';
  }else if(cmd==='run clear'){
    simInstrs=[];simRun={running:false,current:0,started:0,result:'idle'};
    resp='run clear: ok\r\n> ';
  }else if(cmd.indexOf('run add ')===0){
    simInstrs.push(parseSimAdd(cmd));resp='run add: ok\r\n> ';
  }else if(cmd==='run'){
    resp='usage: run add ... loop|road_nav\r\n> ';
  }else if(cmd==='run validate'||cmd==='run validate competition'){
    resp=simInstrs.length?'run validate ok count='+simInstrs.length+
      '\r\n> ':'run validate error index=255 field=table reason=empty\r\n> ';
  }else if(cmd==='run dump'){
    resp='seq '+simInstrs.length+'\r\n';
    simInstrs.forEach(function(x,i){
      resp+=formatRawInstruction(x,true,i)+'\r\n';
    });
    resp+='> ';
  }else if(cmd==='run start'){
    simRun={running:true,current:0,started:Date.now(),result:'running'};
    resp='run start: ok\r\n> ';
  }else if(cmd==='run status'){
    if(simRun.running){
      var elapsed=Date.now()-simRun.started;
      simRun.current=Math.min(Math.max(0,simInstrs.length-1),
        Math.floor(elapsed/450));
      if(elapsed>Math.max(800,simInstrs.length*450)){
        simRun.running=false;simRun.current=simInstrs.length;
        simRun.result='success';
      }
    }
    resp='run '+simRun.current+'/'+simInstrs.length+
      ' running='+(simRun.running?1:0)+
      ' last='+(simRun.result==='success'?1:0)+
      (simRun.running&&simInstrs[simRun.current]?
        ' cur='+OP[simInstrs[simRun.current].op]:'')+
      ' result='+simRun.result+
      ' status=ok reason=none fail_index=255 drive=0/0\r\n> ';
  }else if(cmd==='run cancel'){
    simRun.running=false;simRun.result='cancelled';
    resp='run cancel: ok\r\n> ';
  }else if(cmd==='estop'){
    simRun.running=false;simRun.result='cancelled';
    resp='estop: ok\r\n> ';
  }else if(cmd.indexOf('seq save ')===0){
    var saveSlot=Number(cmd.split(' ')[2]);
    simSlots[saveSlot]=simInstrs.map(function(x){
      return Object.assign({},x);
    });
    resp='seq save: ok\r\n> ';
  }else if(cmd.indexOf('seq del ')===0){
    simSlots[Number(cmd.split(' ')[2])]=null;resp='seq del: ok\r\n> ';
  }else if(cmd==='param get max_wheel_rpm'){
    resp='max_wheel_rpm = 1000\r\n> ';
  }else if(typeof paramSimCommand==='function'&&
           (cmd==='param'||cmd.indexOf('param ')===0||
            cmd==='comp status'||cmd==='reset')){
    resp=paramSimCommand(cmd);
  }else if(typeof lineSensorSimCommand==='function'&&
           (cmd==='linesensor'||cmd.indexOf('linesensor ')===0||
            cmd==='irsensor'||cmd.indexOf('irsensor ')===0)){
    resp=lineSensorSimCommand(cmd);
  }else if(cmd==='help'){
    resp='commands: version reset sched led buzzer param gray imu motor '+
      'chassis heading run lf road comp seq linesensor irsensor estop\r\n> ';
  }else{
    resp=cmd+': simulated\r\n> ';
  }
  logc('rx',resp.replace(/\r/g,''));
  if(typeof onSerialData==='function')onSerialData(resp.replace(/\r/g,''),'rx');
  return new Promise(function(resolve){
    setTimeout(function(){resolve(resp)},35);
  });
}
function parse(text){
  var lines=String(text).split(/\r?\n/),out=[],inside=false;
  for(var i=0;i<lines.length;i++){
    var line=lines[i].trim();
    if(/^SEQ\s/.test(line)){inside=true;continue}
    if(!inside)continue;
    if(line==='END'||line==='>')break;
    var raw=parseRawInstruction(line.split(/\s+/),false);
    if(raw)out.push(raw);
  }
  return out;
}
async function refreshSlots(){for(var i=0;i<8;i++){var data=parse(await send('seq dump '+i));slots[i]=data.length?data:null;var el=$('slotInfo'+i);if(el){el.textContent=data.length?data.length+' 步':'空';el.style.color=data.length?'#a6e3a1':'#6c7086'}}if(typeof seqRefreshMaxRpm==='function')await seqRefreshMaxRpm()}
function buildList(){$('slotList').innerHTML='';for(var i=0;i<8;i++){var c=document.createElement('button');c.className='slot-card';c.dataset.slot=i;c.type='button';c.innerHTML='<span>槽位 '+i+'</span><span id="slotInfo'+i+'">...</span>';c.onclick=(function(slot){return function(){selectSlot(slot)}})(i);$('slotList').appendChild(c)}}
buildList();
function selectSlot(s){curSlot=s;document.querySelectorAll('.slot-card').forEach(function(c){c.classList.toggle('active',+c.dataset.slot===s)});instrs=slots[s]?slots[s].map(function(x){return Object.assign({},x)}):[];selIdx=-1;if(window.SequenceCore&&typeof setSeqProject==='function'){setSeqProject(instrs.length?SequenceCore.decompile(instrs,{name:'FRAM 槽位 '+s,slot:s}):SequenceCore.newProject('槽位 '+s+' 新流程',s));seqFit()}else{render();edit()}}
$('btnRefresh').onclick=refreshSlots;
