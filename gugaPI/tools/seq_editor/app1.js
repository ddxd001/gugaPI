'use strict';
const OP={0:'none',1:'drive',2:'turn',3:'follow',4:'wait',5:'stop',6:'branch',7:'end'};
const OPL={1:'Drive',2:'Turn',3:'Follow',4:'Wait',5:'Stop',6:'Branch',7:'End'};
const OPC={1:'#89b4fa',2:'#fab387',3:'#a6e3a1',4:'#9399b2',5:'#f38ba8',6:'#cba6f7',7:'#6c7086'};
const COND=['timeout','heading_reached','line_detected','line_lost','button','immediate'];
let port=null,reader=null,writer=null;
let readableClosed=null,writableClosed=null;
let curSlot=-1,instrs=[],selIdx=-1;
let slots=Array(8).fill(null);
let rxBuf='',rxResolve=null,simMode=false;
var onSerialData=null,onSerialStateChange=null;
var commandQueue=Promise.resolve();
let simSlots=Array(8).fill(null);
simSlots[0]=[{op:1,p1:60,p2:3000,until:0,ons:255,ont:255},{op:2,p1:90,p2:8000,until:1,ons:255,ont:255},{op:1,p1:60,p2:3000,until:0,ons:255,ont:255},{op:5,p1:0,p2:0,until:5,ons:255,ont:255},{op:7,p1:0,p2:0,until:5,ons:255,ont:255}];
simSlots[1]=[{op:1,p1:80,p2:5000,until:0,ons:255,ont:255},{op:2,p1:-90,p2:8000,until:1,ons:255,ont:255},{op:3,p1:80,p2:30000,until:3,ons:255,ont:255},{op:5,p1:0,p2:0,until:5,ons:255,ont:255},{op:7,p1:0,p2:0,until:5,ons:255,ont:255}];
simSlots[2]=[{op:1,p1:60,p2:5000,until:2,ons:255,ont:4},{op:2,p1:90,p2:8000,until:1,ons:5,ont:3},{op:5,p1:0,p2:0,until:5,ons:255,ont:255},{op:7,p1:0,p2:0,until:5,ons:255,ont:255},{op:1,p1:80,p2:5000,until:0,ons:255,ont:255},{op:7,p1:0,p2:0,until:5,ons:255,ont:255}];
let simInstrs=[];
const $=id=>document.getElementById(id);
function logc(cls,msg){var el=$('log');el.innerHTML+='<span class="'+cls+'">'+msg+'</span>\n';el.scrollTop=el.scrollHeight}
$('btnConnect').onclick=async()=>{
  if(port){try{if(reader){await reader.cancel();if(readableClosed)await readableClosed.catch(function(){});reader.releaseLock()}if(writer){await writer.close();if(writableClosed)await writableClosed.catch(function(){});writer.releaseLock()}await port.close()}catch(e){logc('tx','[DISC ERR] '+e.message)}reader=null;writer=null;readableClosed=null;writableClosed=null;port=null;$('btnConnect').textContent='Connect';$('btnRefresh').disabled=true;$('statusText').textContent='';logc('tx','[DISC]');if(typeof onSerialStateChange==='function')onSerialStateChange(false);return}
  try{port=await navigator.serial.requestPort();await port.open({baudRate:115200});
    var dec=new TextDecoderStream();readableClosed=port.readable.pipeTo(dec.writable);reader=dec.readable.getReader();
    var enc=new TextEncoderStream();writableClosed=enc.readable.pipeTo(port.writable);writer=enc.writable.getWriter();
    $('btnConnect').textContent='Disconnect';$('btnRefresh').disabled=false;$('statusText').textContent='Connected';logc('tx','[CONN]');
    if(typeof onSerialStateChange==='function')onSerialStateChange(true);
    (async()=>{try{while(true){var r=await reader.read();if(r.done)break;if(r.value){logc('rx',r.value.replace(/\r/g,''));if(typeof onSerialData==='function')onSerialData(r.value.replace(/\r/g,''),'rx');rxBuf+=r.value;if(/(?:^|\r?\n)> $/.test(rxBuf)){if(rxResolve)rxResolve(rxBuf)}}}}catch(e){if(typeof onSerialData==='function')onSerialData('[串口读取中断] '+e.message+'\n','error')}})();
    await refreshSlots();
  }catch(e){port=null;reader=null;writer=null;readableClosed=null;writableClosed=null;logc('tx','[ERR]'+e.message);if(typeof onSerialStateChange==='function')onSerialStateChange(false);if(typeof onSerialData==='function')onSerialData('[连接失败] '+e.message+'\n','error')}};
$('btnSim').onclick=async()=>{
  if(simMode){simMode=false;port=null;writer=null;$('btnSim').textContent='Simulate';$('btnConnect').disabled=false;$('btnRefresh').disabled=true;$('statusText').textContent='';logc('tx','[SIM OFF]');if(typeof onSerialStateChange==='function')onSerialStateChange(false);return}
  simMode=true;port={fake:true};writer={fake:true};$('btnSim').textContent='Stop Sim';$('btnConnect').disabled=true;$('btnRefresh').disabled=false;$('statusText').textContent='SIM MODE';logc('tx','[SIM ON]');if(typeof onSerialStateChange==='function')onSerialStateChange(true);await refreshSlots()};
function send(cmd,options){var task=commandQueue.then(function(){return sendNow(cmd,options)});commandQueue=task.catch(function(){});return task}
async function sendNow(cmd,options){if(!writer)return'';logc('tx','> '+cmd);if(typeof onSerialData==='function')onSerialData('> '+cmd+'\n','tx');if(simMode)return simResponse(cmd);rxBuf='';await writer.write(cmd+'\r\n');var timeoutMs=options&&options.timeoutMs?options.timeoutMs:1500;return new Promise(function(resolve){var settled=false;var timer=null;var finish=function(text){if(settled)return;settled=true;if(timer)clearTimeout(timer);if(rxResolve===finish)rxResolve=null;resolve(text)};rxResolve=finish;timer=setTimeout(function(){finish(rxBuf)},timeoutMs)})}
function simResponse(cmd){var resp='';if(cmd.startsWith('seq dump')){var s=+cmd.split(' ')[2];var a=simSlots[s];if(a&&a.length){resp='SEQ '+s+' '+a.length+'\r\n';a.forEach(function(i){resp+=OP[i.op]+' '+i.p1+' '+i.p2+' '+COND[i.until]+' '+(i.ons===255?'next':i.ons)+' '+(i.ont===255?'abort':i.ont)+'\r\n'});resp+='END\r\n> '}else{resp='SEQ '+s+' 0\r\nEND\r\n> '}}else if(cmd.startsWith('seq list')){for(var i=0;i<8;i++){var a2=simSlots[i];resp+='seq '+i+' '+(a2&&a2.length?'ok':'empty')+' count='+(a2?a2.length:0)+'\r\n'}resp+='> '}else if(cmd==='run clear'){simInstrs=[];resp='run clear: ok\r\n> '}else if(cmd.startsWith('run add')){var p=cmd.split(/\s+/);simInstrs.push({op:Object.keys(OP).find(function(k){return OP[k]===p[2]})||0,p1:+p[3]||0,p2:+p[4]||0,until:COND.indexOf(p[5])>=0?COND.indexOf(p[5]):0,ons:p[6]==='next'?255:p[6]==='abort'?255:+p[6]||0,ont:p[7]==='next'?255:p[7]==='abort'?255:+p[7]||0});resp='run add: ok\r\n> '}else if(cmd.startsWith('seq save')){var s2=+cmd.split(' ')[2];simSlots[s2]=simInstrs.map(function(x){return Object.assign({},x)});resp='seq save: ok\r\n> '}else if(cmd.startsWith('seq run')){resp='seq run: ok\r\n> '}else if(cmd.startsWith('comp start')){resp='comp start: ok\r\n> '}else if(cmd.startsWith('seq del')){var s3=+cmd.split(' ')[2];simSlots[s3]=null;resp='seq del: ok\r\n> '}else if(typeof paramSimCommand==='function'&&(cmd==='param'||cmd.startsWith('param ')||cmd==='comp status'||cmd==='reset')){resp=paramSimCommand(cmd)}else if(cmd==='help'){resp='commands: version reset sched txstat led buzzer button fram param oled imu gray lora motor chassis heading run lf road comp telem seq i2c\r\n> '}else{resp=cmd+': simulated\r\n> '}logc('rx',resp.replace(/\r/g,''));if(typeof onSerialData==='function')onSerialData(resp.replace(/\r/g,''),'rx');return new Promise(function(r){setTimeout(function(){r(resp)},50)})}
function parse(text){var lines=text.split('\n');var r=[];var s=false;for(var i=0;i<lines.length;i++){var l=lines[i].trim();if(!l)continue;if(l.startsWith('SEQ')){s=true;continue}if(l==='END'||l==='>'||l==='')break;if(!s)continue;var p=l.split(/\s+/);if(p.length>=6){var op=0;for(var k in OP)if(OP[k]===p[0])op=+k;r.push({op:op,p1:+p[1]||0,p2:+p[2]||0,until:COND.indexOf(p[3])>=0?COND.indexOf(p[3]):0,ons:p[4]==='next'?255:p[4]==='abort'?255:+p[4]||0,ont:p[5]==='next'?255:p[5]==='abort'?255:+p[5]||0})}}return r}
async function refreshSlots(){for(var i=0;i<8;i++){var r=await send('seq dump '+i);var p=parse(r);slots[i]=p.length?p:null;var el=$('slotInfo'+i);if(el){el.textContent=p.length?p.length+' instr':'empty';el.style.color=p.length?'#a6e3a1':'#6c7086'}}}
function buildList(){$('slotList').innerHTML='';for(var i=0;i<8;i++){var c=document.createElement('div');c.className='slot-card';c.dataset.slot=i;(function(s){c.onclick=function(){selectSlot(s)}})(i);c.innerHTML='<span>#'+i+'</span><span id="slotInfo'+i+'">...</span>';$('slotList').appendChild(c)}}
buildList();
function selectSlot(s){curSlot=s;document.querySelectorAll('.slot-card').forEach(function(c){c.classList.toggle('active',+c.dataset.slot===s)});instrs=slots[s]?slots[s].map(function(x){return Object.assign({},x)}):[];selIdx=-1;render();edit()}
