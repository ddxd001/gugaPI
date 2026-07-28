'use strict';
var seqMaxRpm=1000,seqLastCompile=null,seqPollTimer=null;
var seqEmergencyStopPromise=null,seqOperationEpoch=0;
function commandOk(response,label){return String(response).toLowerCase().indexOf(String(label).toLowerCase()+': ok')>=0}
async function seqRefreshMaxRpm(){if(!writer&&!simMode)return;try{var r=await send('param get max_wheel_rpm'),m=String(r).match(/max_wheel_rpm\s*=\s*(\d+)/);if(m)seqMaxRpm=Math.max(1,Number(m[1]));render();edit()}catch(e){}}
function seqShowNotice(message,type){var el=$('seqNotice');if(!el)return;el.textContent=message;el.className=type||'';clearTimeout(el._timer);el._timer=setTimeout(function(){el.textContent='';el.className=''},3500)}
function seqField(label,key,type,options,help){var n=seqNode(seqSelectedNode),v=n.params[key],html='<label>'+label+'</label>';if(type==='select')html+='<select class="seq-prop" data-key="'+key+'">'+options.map(function(o){return'<option value="'+o.value+'"'+(String(o.value)===String(v)?' selected':'')+'>'+o.label+'</option>'}).join('')+'</select>';else html+='<div class="seq-input-unit"><input class="seq-prop" data-key="'+key+'" type="number" min="'+options.min+'" max="'+options.max+'" step="'+(options.step||1)+'" value="'+v+'"><span>'+options.unit+'</span></div>';if(help)html+='<small>'+help+'</small>';return html}
function seqSourceOptions(){
  return Object.keys(SC.SOURCES).filter(function(key){
    return !SC.SOURCES[key].hidden;
  }).map(function(key){
    var source=SC.SOURCES[key];
    return{value:key,label:SC.SOURCE_GROUPS[source.group]+' · '+source.label};
  });
}
function seqCompareOptions(sourceKey){
  var source=SC.SOURCES[sourceKey]||SC.SOURCES.line_detected;
  return source.compares.map(function(key){
    return{value:key,label:SC.COMPARES[key].label+' ('+
      SC.COMPARES[key].symbol+')'};
  });
}
function seqConditionValueField(n){
  var source=SC.SOURCES[n.params.source]||SC.SOURCES.line_detected;
  if(source.options){
    return seqField('判断值','value','select',source.options);
  }
  var min=source.kind==='rpm'?-seqMaxRpm:source.min;
  var max=source.kind==='rpm'?seqMaxRpm:source.max;
  return seqField('判断值','value','number',{
    min:min,max:max,step:source.step||1,unit:source.unit||''
  });
}
function seqConditionProperties(n,showMode){
  var p=n.params,source=SC.SOURCES[p.source]||SC.SOURCES.line_detected;
  var h=seqField('数据源','source','select',seqSourceOptions(),
    source.developmentOnly?'Button2 在比赛模式固定用于停止，只能试运行。':'');
  h+=seqField('比较方式','compare','select',seqCompareOptions(p.source));
  h+=seqConditionValueField(n);
  if(showMode){
    h+=seqField('执行方式','mode','select',[
      {value:'instant',label:'立即判断'},
      {value:'wait',label:'停车等待成立'}
    ]);
  }
  var waits=!showMode||p.mode==='wait';
  if(waits){
    h+=seqField('安全超时','timeoutMs','number',{
      min:50,max:30000,step:50,unit:'ms'
    },'超时或数据无效时走红色端口。');
    if(source.kind!=='event'){
      h+=seqField('连续成立','stableMs','number',{
        min:0,max:1000,step:50,unit:'ms'
      },'条件连续成立达到该时间后才通过；0 表示立即通过。');
    }
  }
  return h;
}
function seqProperties(n){
  var h='';
  if(n.type==='drive'||n.type==='follow'){
    h+=seqField(n.type==='drive'?'目标转速':'基础转速','rpm','number',{
      min:-seqMaxRpm,max:seqMaxRpm,unit:'RPM'
    },n.type==='drive'?'正数前进，负数后退。':'');
    h+=seqField('完成方式','completion','select',[
      {value:'timeout',label:'运行指定时间'},
      {value:'compare',label:'满足通用条件'}
    ]);
    if(n.params.completion==='compare')h+=seqConditionProperties(n,false);
    else h+=seqField('运行时间','timeoutMs','number',{
      min:1,max:30000,unit:'ms'
    });
  }else if(n.type==='drive_mm'){
    h+=seqField('行驶距离','distanceMm','number',{
      min:-10000,max:10000,unit:'mm'
    },'正数前进，负数后退，不能为 0。')+
      seqField('最大转速','maxRpm','number',{
        min:1,max:seqMaxRpm,unit:'RPM'
      });
  }else if(n.type==='turn'){
    h+=seqField('相对转角','angleDeg','number',{
      min:-180,max:180,unit:'°'
    },'正数左转，负数右转。')+
      seqField('安全超时','timeoutMs','number',{
        min:1,max:30000,unit:'ms'
      });
  }else if(n.type==='wait'){
    h+=seqField('等待时间','timeoutMs','number',{
      min:0,max:30000,unit:'ms'
    });
  }else if(n.type==='condition'){
    h+=seqConditionProperties(n,true);
  }else if(n.type==='loop'){
    h+=seqField('循环次数','count','number',{
      min:1,max:1000,step:1,unit:'次'
    },'循环体实际执行次数；1 表示执行一次，0 不表示无限循环。');
  }else if(n.type==='road_nav'){
    h+=seqField('路口方向','direction','select',SC.ROAD_DIRECTIONS);
    if(n.params.direction==='uturn_left'||
       n.params.direction==='uturn_right'){
      h+=seqField('掉头方式','uturnMode','select',SC.ROAD_UTURN_MODES,
        '滚动圆弧保持连续运动；停车原地会先停车再转向。');
    }
    h+=seqField('循迹速度','rpm','number',{
      min:1,max:seqMaxRpm,step:1,unit:'RPM'
    },'仅支持正向循迹。');
    h+=seqField('整体超时','timeoutMs','number',{
      min:50,max:30000,step:50,unit:'ms'
    },'覆盖寻找路口、转向和重新捕线的全过程。');
  }else if(n.type.indexOf('led_')===0){
    h+=seqField('LED 目标','target','select',[
      {value:0,label:'LED2 + LED3'},{value:2,label:'仅 LED2'},
      {value:3,label:'仅 LED3'}
    ]);
    if(n.type!=='led_off')h+=seqField('自动关闭','durationMs','number',{
      min:0,max:30000,unit:'ms'
    },'0 表示保持；非零时最少 50 ms。');
  }else if(n.type==='buzzer_on'||n.type==='buzzer_toggle'){
    h+=seqField('自动关闭','durationMs','number',{
      min:0,max:30000,unit:'ms'
    },'0 表示保持；非零时最少 50 ms。');
  }else h+='<p class="seq-muted">该动作没有可调参数。</p>';
  return h;
}
function edit(){var el=$('editorContent');if(!el||!seqProject)return;var validation=SC.validate(seqProject,{maxRpm:seqMaxRpm}),node=seqNode(seqSelectedNode);if(seqSelectedEdge){el.innerHTML='<div class="seq-panel-title">连线</div><p class="seq-muted">选中连线后可按 Delete 删除，也可从同一端口重新拖线。</p><button class="danger" id="btnDeleteEdge">删除连线</button>';var de=$('btnDeleteEdge');if(de)de.onclick=seqDeleteSelection;return}if(!node){var errs=validation.issues;el.innerHTML='<div class="seq-panel-title">流程检查</div><div class="seq-project-meta"><b>'+seqEsc(seqProject.name)+'</b><span>绑定槽位 '+seqProject.slot+' · '+seqProject.nodes.filter(function(n){return SC.ACTIONS[n.type]}).length+' 步</span></div><div class="seq-issues">'+(errs.length?errs.map(function(i){return'<button class="seq-issue '+i.severity+'" data-node="'+(i.nodeId||'')+'">'+(i.severity==='error'?'错误':'提醒')+' · '+seqEsc(i.message)+'</button>'}).join(''):'<div class="seq-pass">✓ 参数、连线和可达性检查通过</div>')+'</div><div class="seq-help"><b>端口说明</b><p>绿色是动作完成路径；红色是失败或超时路径。失败端口不连时默认中止，判断的两个端口都必须连接。循环的“执行循环体”和“循环完成”都是绿色正常路径，且都必须连接。</p><b>试运行与保存</b><p>“试运行当前画布”只临时写入 RAM；“保存到 FRAM”才会覆盖所选槽位。</p></div>';el.querySelectorAll('.seq-issue[data-node]').forEach(function(x){x.onclick=function(){if(x.dataset.node)seqSelectNode(x.dataset.node)}});return}if(node.type.indexOf('system_')===0){el.innerHTML='<div class="seq-panel-title">'+(node.type==='system_start'?'开始节点':'中止节点')+'</div><p class="seq-muted">'+(node.type==='system_start'?'所有可执行动作必须从这里可达。':'失败路径连接到这里会编译为 ontimeout=abort。')+'</p>';return}var a=SC.ACTIONS[node.type],nodeIssues=validation.issues.filter(function(i){return i.nodeId===node.id});el.innerHTML='<div class="seq-panel-title">'+a.name+'</div><div class="seq-tech">Shell: '+node.type+' · ActionOp '+a.op+'</div><p class="seq-action-help">'+a.help+'</p>'+seqProperties(node)+(nodeIssues.length?'<div class="seq-inline-errors">'+nodeIssues.map(function(i){return'<p class="'+i.severity+'">'+seqEsc(i.message)+'</p>'}).join('')+'</div>':'')+'<div class="btn-row"><button id="btnDuplicateNode">复制</button><button id="btnDeleteNode" class="danger">删除</button></div>';
 el.querySelectorAll('.seq-prop').forEach(function(input){input.onchange=function(){var before=seqSnapshot(),key=input.dataset.key,value=input.tagName==='SELECT'?input.value:Number(input.value);node.params[key]=value;if(key==='source'){var source=SC.SOURCES[value];node.params.compare=source.compares[0];if(source.options)node.params.value=Number(source.options[0].value);else node.params.value=Math.max(source.kind==='rpm'?-seqMaxRpm:source.min,Math.min(source.kind==='rpm'?seqMaxRpm:source.max,Number(node.params.value)||0));if(source.kind==='event')node.params.stableMs=0}if(key==='mode'&&value==='instant'){node.params.timeoutMs=0;node.params.stableMs=0}if(key==='mode'&&value==='wait'&&Number(node.params.timeoutMs)<50){node.params.timeoutMs=5000;node.params.stableMs=100}if(key==='completion'&&value==='compare'){if(Number(node.params.timeoutMs)<50)node.params.timeoutMs=5000;if(Number(node.params.stableMs)<0)node.params.stableMs=100}seqCommit('已更新“'+a.name+'”参数',before)}});$('btnDuplicateNode').onclick=seqDuplicate;$('btnDeleteNode').onclick=seqDeleteSelection}
function seqRawCanonical(x){
  if(Number(x.op)>=15&&Number(x.op)<=17){
    return[Number(x.op),Number(x.p1)||0,String(x.source),
      String(x.compare),Number(x.value),String(x.mode||'wait'),
      Number(x.timeoutMs),Number(x.stableMs),Number(x.ons),Number(x.ont)]
      .join('|');
  }
  if(Number(x.op)===19){
    return[19,Number(x.conditionValue),Number(x.p1),Number(x.p2),
      Number(x.ons),Number(x.ont)].join('|');
  }
  return[Number(x.op),Number(x.p1),Number(x.p2),Number(x.until),
    Number(x.ons),Number(x.ont)].join('|');
}
function seqRawEqual(a,b){
  return a.length===b.length&&a.every(function(x,i){
    return seqRawCanonical(x)===seqRawCanonical(b[i]);
  });
}
function seqParseRunDump(text){
  var lines=String(text).split(/\r?\n/),out=[];
  lines.forEach(function(line){
    var p=line.trim().split(/\s+/);
    if(p.length>=7&&/^\d+$/.test(p[0])){
      var raw=parseRawInstruction(p,true);
      if(raw)out.push(raw);
    }
  });
  return out;
}
function seqDelay(ms){return new Promise(function(resolve){setTimeout(resolve,ms)})}
async function seqEnsureQuietShell(){
  if(simMode)return true;
  var lastResponse='';
  for(var attempt=0;attempt<2;attempt++){
    lastResponse=await send('telem off',{timeoutMs:1800});
    if(commandOk(lastResponse,'telem off'))return true;
    var status=await send('telem status',{timeoutMs:1800});
    if(/telem enabled=0/i.test(status))return true;
    lastResponse=status||lastResponse;
    await seqDelay(60);
  }
  throw new Error('停止实时遥测失败：'+
    (lastResponse.trim()||'设备未确认'));
}
function seqAssertOperationActive(operationEpoch){
  if((operationEpoch!==undefined)&&(operationEpoch!==seqOperationEpoch)){
    var error=new Error('当前操作已被紧急停止取消');
    error.seqCancelled=true;
    throw error;
  }
}
function seqCommandForRaw(x){
  var ons=x.ons===255?'next':x.ons;
  var ont=x.ont===255?'abort':x.ont;
  if(x.op===15){
    return'run add condition '+x.source+' '+x.compare+' '+x.value+' '+
      x.mode+' '+x.timeoutMs+' '+x.stableMs+' '+ons+' '+ont;
  }
  if(x.op===16||x.op===17){
    return'run add '+OP[x.op]+' '+x.p1+' '+x.source+' '+x.compare+' '+
      x.value+' '+x.timeoutMs+' '+x.stableMs+' '+ons+' '+ont;
  }
  if(x.op===19){
    return'run add road_nav '+x.route+' '+x.p1+' '+x.p2+' '+ons+' '+ont;
  }
  return'run add '+OP[x.op]+' '+x.p1+' '+x.p2+' '+COND[x.until]+' '+
    ons+' '+ont;
}
async function seqUploadOnce(instrs,operationEpoch,competition){
  seqAssertOperationActive(operationEpoch);
  var r=await send('run clear',{timeoutMs:2200});
  seqAssertOperationActive(operationEpoch);
  if(!commandOk(r,'run clear')){
    throw new Error('清空 RAM 表失败：'+(r.trim()||'设备无响应'));
  }
  for(var i=0;i<instrs.length;i++){
    var x=instrs[i];
    var cmd=seqCommandForRaw(x);
    r=await send(cmd,{timeoutMs:2200});
    seqAssertOperationActive(operationEpoch);
    if(!commandOk(r,'run add')){
      throw new Error('第 '+i+' 步下发失败：'+
        (r.trim()||'设备无响应'));
    }
  }
  r=await send(competition?'run validate competition':'run validate',
    {timeoutMs:2200});
  seqAssertOperationActive(operationEpoch);
  if(!/run validate ok/i.test(r)){
    throw new Error('固件校验失败：'+(r.trim()||'设备无响应'));
  }
  var back=seqParseRunDump(await send('run dump',{timeoutMs:2200}));
  seqAssertOperationActive(operationEpoch);
  if(!seqRawEqual(instrs,back)){
    throw new Error('RAM 回读内容与画布编译结果不一致');
  }
  return true;
}
async function seqUpload(instrs,operationEpoch,competition){
  await seqEnsureQuietShell();
  seqAssertOperationActive(operationEpoch);
  if(instrs.some(function(x){return Number(x.op)===19})){
    var roadHelp=await send('run',{timeoutMs:2200});
    seqAssertOperationActive(operationEpoch);
    if(!/\broad_nav\b/.test(String(roadHelp))){
      throw new Error('当前固件不支持“循迹通过路口”（操作码 19）；'+
        '旧固件会拒绝该序列，请先升级固件');
    }
  }
  var lastError=null;
  for(var attempt=0;attempt<2;attempt++){
    try{return await seqUploadOnce(instrs,operationEpoch,competition)}
    catch(error){
      if(error.seqCancelled)throw error;
      lastError=error;
      if(attempt===0)await seqDelay(80);
    }
  }
  throw lastError;
}
async function seqRestoreRam(raw){try{if(raw&&raw.length)await seqUpload(raw);else await send('run clear')}catch(e){logc('tx','[RAM RESTORE FAILED] '+e.message)}}
function seqCompileCurrent(competition){
  seqLastCompile=SC.compile(seqProject,{
    maxRpm:seqMaxRpm,competition:!!competition
  });
  return seqLastCompile;
}
async function seqWaitForEmergencyStop(){
  if(seqEmergencyStopPromise)await seqEmergencyStopPromise;
}
function seqRunStartError(response){
  var detail=String(response).trim()||'设备无响应';
  if(/run start:\s*not-initialized/i.test(detail)){
    return '设备存在锁存故障，急停不会清除安全故障；请排除故障并复位设备后再试';
  }
  return '启动失败：'+detail;
}
async function runSlot(){if((!writer&&!simMode)||!seqProject){seqShowNotice('请先连接串口或开启模拟模式','error');return}var backup=[];try{await seqWaitForEmergencyStop();var operationEpoch=seqOperationEpoch;var compiled=seqCompileCurrent();backup=seqParseRunDump(await send('run dump'));seqAssertOperationActive(operationEpoch);await seqUpload(compiled.instrs,operationEpoch);var start=await send('run start');seqAssertOperationActive(operationEpoch);if(!commandOk(start,'run start'))throw new Error(seqRunStartError(start));seqShowNotice('当前画布已在 RAM 中启动，未写入 FRAM','ok');seqStartPolling()}catch(e){try{await seqWaitForEmergencyStop()}catch(stopError){if(e.seqCancelled)e=stopError}await seqRestoreRam(backup);seqShowNotice(e.message,'error');logc('tx','[试运行失败] '+e.message)}}
async function saveSlot(){if((!writer&&!simMode)||!seqProject){seqShowNotice('请先连接串口或开启模拟模式','error');return}var slot=Number(seqProject.slot);if(slots[slot]&&slots[slot].length&&!window.confirm('槽位 '+slot+' 已有 '+slots[slot].length+' 步，确认覆盖 FRAM？'))return;try{await seqWaitForEmergencyStop();var operationEpoch=seqOperationEpoch;var compiled=seqCompileCurrent(true);await seqUpload(compiled.instrs,operationEpoch,true);var r=await send('seq save '+slot,{timeoutMs:2500});seqAssertOperationActive(operationEpoch);var back=parse(await send('seq dump '+slot,{timeoutMs:2500}));seqAssertOperationActive(operationEpoch);if(!commandOk(r,'seq save')&&!seqRawEqual(compiled.instrs,back))throw new Error('FRAM 保存失败：'+(r.trim()||'设备无响应'));if(!seqRawEqual(compiled.instrs,back))throw new Error('FRAM 保存后的回读比对失败');slots[slot]=back;seqShowNotice('已保存到 FRAM 槽位 '+slot+'，并通过回读比对','ok');await refreshSlots()}catch(e){seqShowNotice(e.message,'error');logc('tx','[保存失败] '+e.message)}}
function seqParseStatus(text){var m=String(text).match(/run\s+(\d+)\/(\d+)\s+running=(\d).*?result=([^\s]+)/s);return m?{current:+m[1],count:+m[2],running:m[3]==='1',result:m[4]}:null}
function seqStartPolling(){clearInterval(seqPollTimer);seqPollTimer=setInterval(async function(){try{var s=seqParseStatus(await send('run status'));if(!s)return;if(seqLastCompile&&s.current<seqLastCompile.nodeOrder.length)seqActiveNode=seqLastCompile.nodeOrder[s.current];else seqActiveNode=null;render();if(!s.running){clearInterval(seqPollTimer);seqPollTimer=null;seqShowNotice(s.result==='success'?'序列执行完成':'序列已停止：'+s.result,s.result==='success'?'ok':'error')}}catch(e){}},500)}
async function seqPerformEmergencyStop(){
  clearInterval(seqPollTimer);
  seqPollTimer=null;
  seqActiveNode=null;
  render();
  if(!writer&&!simMode){
    seqShowNotice('当前未连接设备','error');
    return;
  }
  var response=await send('estop',{timeoutMs:2200});
  if(!commandOk(response,'estop')){
    throw new Error('紧急停止未获设备确认：'+
      (response.trim()||'设备无响应'));
  }
  seqShowNotice('设备已确认紧急停止','error');
}
async function seqEmergencyStop(){
  if(seqEmergencyStopPromise)return seqEmergencyStopPromise;
  seqOperationEpoch++;
  seqEmergencyStopPromise=seqPerformEmergencyStop();
  try{
    return await seqEmergencyStopPromise;
  }catch(error){
    seqShowNotice(error.message,'error');
    logc('tx','[紧急停止失败] '+error.message);
  }finally{
    seqEmergencyStopPromise=null;
  }
}
function seqDownload(){var blob=new Blob([SC.serialize(seqProject)],{type:'application/json'}),a=document.createElement('a');a.href=URL.createObjectURL(blob);a.download=(seqProject.name||'gugapi-sequence').replace(/[\\/:*?"<>|]/g,'_')+'.json';a.click();setTimeout(function(){URL.revokeObjectURL(a.href)},1000)}
function seqBindUi(){seqBuildPalette();seqInitCanvas();$('btnSeqUndo').onclick=seqUndo;$('btnSeqRedo').onclick=seqRedo;$('btnSeqFit').onclick=seqFit;$('btnSeqLayout').onclick=seqAutoLayout;$('btnSeqNew').onclick=function(){if(window.confirm('新建工程会替换当前本地画布，继续吗？'))setSeqProject(SC.newProject('未命名流程',curSlot>=0?curSlot:7))};$('btnSeqExport').onclick=seqDownload;$('btnSeqImport').onclick=function(){$('seqFileInput').click()};$('seqFileInput').onchange=async function(){try{var text=await this.files[0].text();setSeqProject(SC.normalizeProject(JSON.parse(text)));seqShowNotice('工程导入成功','ok')}catch(e){seqShowNotice('导入失败：'+e.message,'error')}this.value=''};$('seqProjectName').onchange=function(){var before=seqSnapshot();seqProject.name=this.value.trim()||'未命名流程';seqCommit('已修改工程名称',before)};$('seqProjectSlot').onchange=function(){var before=seqSnapshot();seqProject.slot=Number(this.value);curSlot=seqProject.slot;seqCommit('已绑定槽位 '+seqProject.slot,before)};$('seqTemplate').onchange=function(){if(this.value==='')return;var p=SC.templates()[Number(this.value)];if(p){p.slot=seqProject.slot;setSeqProject(p);seqShowNotice('已载入模板“'+p.name+'”','ok')}this.value=''};$('btnSeqRun').onclick=runSlot;$('btnSeqSave').onclick=saveSlot;$('btnEmergencyStop').onclick=seqEmergencyStop;$('btnSeqHelp').onclick=function(){$('seqGuide').classList.add('show')};$('btnGuideClose').onclick=function(){$('seqGuide').classList.remove('show');localStorage.setItem('gugapi-seq-guide-seen','1')};var saved=localStorage.getItem(seqStorageKey());try{setSeqProject(saved?JSON.parse(saved):SC.templates()[0])}catch(e){setSeqProject(SC.templates()[0])}if(!localStorage.getItem('gugapi-seq-guide-seen'))$('seqGuide').classList.add('show')}
var oldRender=render;render=function(){oldRender();if(seqProject){$('seqProjectName').value=seqProject.name;$('seqProjectSlot').value=String(seqProject.slot)}};
seqBindUi();
seqFixWirePointerEvents();
