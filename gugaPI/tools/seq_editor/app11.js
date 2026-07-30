'use strict';

var serialLogState={
  active:false,starting:false,stopping:false,connected:false,
  writable:null,writeChain:Promise.resolve(),chunks:[],
  sequence:0,entries:0,payloadBytes:0,startedAt:null,mode:'',lastMessage:''
};

function serialLogFormatBytes(value){
  if(value<1024)return value+' B';
  if(value<1024*1024)return(value/1024).toFixed(1)+' KiB';
  return(value/(1024*1024)).toFixed(1)+' MiB';
}

function serialLogRender(){
  var button=$('btnSerialLog');
  var status=$('serialLogStatus');
  button.disabled=serialLogState.starting||serialLogState.stopping||
    (!serialLogState.connected&&!serialLogState.active);
  button.textContent=serialLogState.active?'停止串口记录':'记录全量串口';
  button.classList.toggle('recording',serialLogState.active);
  if(serialLogState.starting){
    status.textContent='正在选择日志文件…';
  }else if(serialLogState.stopping){
    status.textContent='正在写入日志尾部…';
  }else if(serialLogState.active){
    status.textContent='记录中 · '+serialLogState.entries+' 条 · '+
      serialLogFormatBytes(serialLogState.payloadBytes);
  }else{
    status.textContent=serialLogState.lastMessage||'串口日志未开启';
  }
}

function serialLogDownload(chunks,filename){
  var blob=new Blob(chunks,{type:'application/x-ndjson;charset=utf-8'});
  var link=document.createElement('a');
  link.href=URL.createObjectURL(blob);
  link.download=filename;
  link.click();
  setTimeout(function(){URL.revokeObjectURL(link.href)},1000);
}

async function SerialLog_Start(){
  if(serialLogState.active||serialLogState.starting||!serialLogState.connected)return;
  if(serialLogState.writable||serialLogState.chunks.length){
    await SerialLog_Stop('restart');
  }
  serialLogState.starting=true;
  serialLogState.lastMessage='';
  serialLogRender();
  var startedAt=new Date();
  var filename=SerialLogCore.suggestedFilename(startedAt);
  var writable=null;
  var mode='memory';
  try{
    if(typeof window.showSaveFilePicker==='function'){
      var handle=await window.showSaveFilePicker({
        suggestedName:filename,
        types:[{description:'gugaPI 串口日志',accept:{
          'application/x-ndjson':['.jsonl']
        }}]
      });
      writable=await handle.createWritable();
      mode='file';
    }
    var header=SerialLogCore.encodeHeader(startedAt,{
      baud_rate:115200,encoding:'utf-8',source:simMode?'simulator':'web-serial'
    });
    if(writable)await writable.write(header);
    serialLogState.active=true;
    serialLogState.writable=writable;
    serialLogState.writeChain=Promise.resolve();
    serialLogState.chunks=writable?[]:[header];
    serialLogState.sequence=0;
    serialLogState.entries=0;
    serialLogState.payloadBytes=0;
    serialLogState.startedAt=startedAt;
    serialLogState.mode=mode;
    serialLogState.lastMessage=mode==='file'?'已开始持续写入本地文件':
      '浏览器不支持实时文件写入，停止时下载日志';
  }catch(error){
    if(writable){try{await writable.abort()}catch(abortError){}}
    serialLogState.lastMessage=error&&error.name==='AbortError'?
      '已取消选择日志文件':'启动记录失败：'+error.message;
  }finally{
    serialLogState.starting=false;
    serialLogRender();
  }
}

function SerialLog_Record(direction,data){
  if(!serialLogState.active||data===undefined||data===null||data==='')return;
  var text=String(data);
  serialLogState.sequence++;
  serialLogState.entries++;
  serialLogState.payloadBytes+=SerialLogCore.byteLength(text);
  var line=SerialLogCore.encodeEntry(
    serialLogState.sequence,direction,text,new Date());
  if(serialLogState.writable){
    serialLogState.writeChain=serialLogState.writeChain.then(function(){
      return serialLogState.writable.write(line);
    }).catch(function(error){
      serialLogState.active=false;
      serialLogState.lastMessage='日志写入失败：'+error.message;
      serialLogRender();
    });
  }else{
    serialLogState.chunks.push(line);
  }
  serialLogRender();
}

async function SerialLog_Stop(reason){
  if((!serialLogState.active&&!serialLogState.writable&&
      serialLogState.chunks.length===0)||serialLogState.stopping)return;
  serialLogState.active=false;
  serialLogState.stopping=true;
  serialLogRender();
  var footer=SerialLogCore.encodeFooter(
    new Date(),reason||'user',serialLogState.entries,
    serialLogState.payloadBytes);
  var filename=SerialLogCore.suggestedFilename(serialLogState.startedAt||new Date());
  try{
    if(serialLogState.writable){
      await serialLogState.writeChain;
      await serialLogState.writable.write(footer);
      await serialLogState.writable.close();
    }else{
      serialLogState.chunks.push(footer);
      serialLogDownload(serialLogState.chunks,filename);
    }
    serialLogState.lastMessage='日志已保存 · '+serialLogState.entries+' 条 · '+
      serialLogFormatBytes(serialLogState.payloadBytes);
  }catch(error){
    serialLogState.lastMessage='结束记录失败：'+error.message;
  }finally{
    serialLogState.writable=null;
    serialLogState.writeChain=Promise.resolve();
    serialLogState.chunks=[];
    serialLogState.stopping=false;
    serialLogRender();
  }
}

function SerialLog_OnConnection(connected){
  serialLogState.connected=connected;
  serialLogRender();
}

$('btnSerialLog').onclick=function(){
  if(serialLogState.active)SerialLog_Stop('user');
  else SerialLog_Start();
};

var serialLogPreviousSerialState=onSerialStateChange;
onSerialStateChange=function(connected){
  if(typeof serialLogPreviousSerialState==='function'){
    serialLogPreviousSerialState(connected);
  }
  SerialLog_OnConnection(connected);
};
SerialLog_OnConnection(!!writer||simMode);
