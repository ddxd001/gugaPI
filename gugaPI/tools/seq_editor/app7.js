'use strict';

var DASH_MAX_RECORDS=100000;
var DASH_MAX_HISTORY_MS=60000;
var DASH_COLORS=['#88c0d0','#bf616a','#a3be8c','#ebcb8b','#b48ead','#d08770'];
var dashState={
  visible:false,connected:false,enabled:false,requesting:false,fields:[],
  samples:[],lastSample:null,lastSampleAt:0,recording:false,records:[],
  recordFields:[],recordGroup:null,
  renderPending:false,simTimer:null,simStart:0,simStopped:false,
  activeGroup:null,pendingGroup:null,hoverX:null,hoverY:null
};

function dashValue(sample,name,fallback){
  return sample&&Object.prototype.hasOwnProperty.call(sample.values,name)
    ?sample.values[name]:fallback;
}

function dashToast(message,bad){
  var toast=$('dashToast');
  toast.textContent=message;
  toast.className='show'+(bad?' bad':'');
  clearTimeout(dashToast.timer);
  dashToast.timer=setTimeout(function(){toast.className=''},2600);
}

function dashSetTelemetryState(text,state){
  $('dashTelemetryText').textContent=text;
  $('dashTelemetryDot').className=state||'';
  $('btnDashTelemetry').textContent='停止展示';
}

function dashUpdateControls(){
  $('btnDashTelemetry').disabled=!dashState.enabled||dashState.requesting;
  $('dashPeriod').disabled=dashState.requesting;
  $('btnDashRecord').disabled=!dashState.connected||!dashState.enabled;
  $('btnDashExport').disabled=dashState.records.length===0;
  $('btnDashEstop').disabled=!dashState.connected||dashState.requesting;
  $('btnDashRecord').textContent=dashState.recording?'停止记录':'开始记录';
}

function dashRenderStatus(){
  var sample=dashState.lastSample;
  var age=dashState.lastSampleAt?Date.now()-dashState.lastSampleAt:Infinity;
  var liveLimit=Math.max(300,Number($('dashPeriod').value)*3);
  $('dashSampleCount').textContent=dashState.samples.length+' samples';
  if(dashState.requesting){
    dashSetTelemetryState('正在切换数据…','stale');
  }else if(!dashState.activeGroup){
    dashSetTelemetryState('未选择数据','');
  }else if(!dashState.connected){
    dashSetTelemetryState('串口未连接','');
  }else if(!dashState.enabled){
    dashSetTelemetryState(dashState.visible?'等待启动':'页面已暂停','');
  }else if(age<=liveLimit){
    dashSetTelemetryState('实时 · '+Math.round(age)+' ms','live');
  }else{
    dashSetTelemetryState('数据超时','stale');
  }
  dashUpdateDetailValues(sample);
  dashUpdateControls();
}

function dashFormat(value,digits){
  return value===null||value===undefined||!Number.isFinite(value)
    ?'--':Number(value).toFixed(digits);
}

function dashScheduleRender(){
  if(dashState.renderPending)return;
  dashState.renderPending=true;
  requestAnimationFrame(function(){
    dashState.renderPending=false;
    dashDrawAll();
    dashRenderStatus();
  });
}

function dashCanvasSize(canvas){
  var rect=canvas.getBoundingClientRect();
  if(rect.width<10||rect.height<10)return null;
  var ratio=window.devicePixelRatio||1;
  var width=Math.floor(rect.width*ratio);
  var height=Math.floor(rect.height*ratio);
  if(canvas.width!==width||canvas.height!==height){
    canvas.width=width;canvas.height=height;
  }
  return{width:width,height:height,ratio:ratio};
}

function dashDrawEmptyState(ctx,width,height,ratio,title,detail){
  var centerX=width/2,centerY=height/2;
  ctx.save();
  ctx.textAlign='center';ctx.textBaseline='middle';
  ctx.fillStyle='#87919e';ctx.font='600 '+(12*ratio)+'px sans-serif';
  ctx.fillText(title,centerX,centerY-7*ratio);
  ctx.fillStyle='#58626e';ctx.font=(9*ratio)+'px Consolas,monospace';
  ctx.fillText(detail,centerX,centerY+13*ratio);
  ctx.restore();
}

var dashCharts=[
  {key:'motor',title:'车轮转速',subtitle:'目标与实际转速',unit:'RPM',series:[
    {field:'L_tgt',label:'左轮目标转速',unit:'RPM',description:'底盘控制器下发给左轮的目标转速。',color:'#55bfe9',dashed:true},
    {field:'L_act',label:'左轮实际转速',unit:'RPM',description:'MotorDriver 周期反馈的左轮实测转速。',color:'#348ed1'},
    {field:'R_tgt',label:'右轮目标转速',unit:'RPM',description:'底盘控制器下发给右轮的目标转速。',color:'#e4b657',dashed:true},
    {field:'R_act',label:'右轮实际转速',unit:'RPM',description:'MotorDriver 周期反馈的右轮实测转速。',color:'#df8352'}]},
  {key:'heading',title:'航向控制',subtitle:'目标 / 实际 / 误差',unit:'DEG',series:[
    {field:'yaw_tgt',label:'目标航向角',unit:'deg',description:'航向控制器当前锁定或转向的目标角度。',color:'#55bfe9',dashed:true},
    {field:'yaw',label:'实际航向角',unit:'deg',description:'IMU 积分得到的当前相对航向角。',color:'#55c68a'},
    {field:'head_err',label:'航向误差',unit:'deg',description:'归一化后的目标角与实际角之差。',color:'#df6670'}]},
  {key:'line',title:'循迹控制',subtitle:'位置误差与闭环修正',unit:'CONTROL',series:[
    {field:'gray_pos',label:'灰度线位置',unit:'pos',description:'八路灰度插值得到的赛道中心位置，左正右负。',color:'#ae84c6'},
    {field:'lf_err',label:'循迹位置误差',unit:'mpos',description:'循迹控制器使用的中心位置误差。',color:'#55bfe9'},
    {field:'lf_corr',label:'循迹修正量',unit:'RPM',description:'循迹闭环输出的左右轮差速修正量。',color:'#e4b657'}]},
  {key:'accel',title:'线加速度',subtitle:'IMU 三轴加速度',unit:'MG',series:[
    {field:'acc_x_mg',label:'X 轴加速度',unit:'mg',description:'IMU X 轴去偏置后的线加速度。',color:'#df6670'},
    {field:'acc_y_mg',label:'Y 轴加速度',unit:'mg',description:'IMU Y 轴去偏置后的线加速度。',color:'#55c68a'},
    {field:'acc_z_mg',label:'Z 轴加速度',unit:'mg',description:'IMU Z 轴去偏置后的线加速度，静止时包含重力。',color:'#55bfe9'}]},
  {key:'gyro',title:'角速度',subtitle:'IMU 三轴角速度',unit:'MDPS',series:[
    {field:'gyro_x_mdps',label:'X 轴角速度',unit:'mdps',description:'IMU X 轴去偏置后的角速度。',color:'#df6670'},
    {field:'gyro_y_mdps',label:'Y 轴角速度',unit:'mdps',description:'IMU Y 轴去偏置后的角速度。',color:'#55c68a'},
    {field:'gyro_z_mdps',label:'Z 轴角速度',unit:'mdps',description:'IMU Z 轴去偏置后的角速度，用于航向积分。',color:'#55bfe9'}]}
];

function dashActiveChart(){
  return dashCharts.find(function(chart){return chart.key===dashState.activeGroup})||null;
}

async function dashSelectGroup(key){
  if(dashState.requesting||dashState.activeGroup===key)return;
  var chart=dashCharts.find(function(item){return item.key===key});
  if(!chart)return;
  if(!dashState.connected){
    dashState.activeGroup=key;
    dashResetPlot();
    dashBuildDataBrowser();
    dashRenderStatus();
    return;
  }
  await dashStartGroup(key,true);
}

function dashBuildDataBrowser(){
  var active=dashActiveChart();
  $('dashGroupList').innerHTML=dashCharts.map(function(chart){
    return'<button class="dash-group-button'+
      (active&&chart.key===active.key?' active':'')+
      (chart.key===dashState.pendingGroup?' pending':'')+
      '" type="button" data-group="'+chart.key+'"><span class="dash-group-accent"></span>'+
      '<span class="dash-group-copy"><strong>'+chart.title+'</strong><small>'+
      chart.subtitle+'</small></span><span class="dash-group-count">'+
      chart.series.length+' CH</span></button>';
  }).join('');
  $('dashGroupList').querySelectorAll('[data-group]').forEach(function(button){
    button.onclick=function(){dashSelectGroup(button.getAttribute('data-group'))};
  });

  $('dashChartTitle').textContent=active?active.title:'未选择数据';
  $('dashChartSubtitle').textContent=active?
    (active.subtitle+' · '+active.unit):'SELECT A CHART FROM THE LEFT';
  $('dashDetailsTitle').textContent=active?active.title:'未选择';
  $('dashDataDetailsEmpty').hidden=!!active;
  $('dashDataDetails').innerHTML=active?active.series.map(function(item){
    return'<div class="dash-detail-item"><div class="dash-detail-head">'+
      '<i class="dash-detail-dot" style="background:'+item.color+'"></i>'+
      '<strong>'+item.label+'</strong><code>'+item.field+'</code></div>'+
      '<div class="dash-detail-value"><strong data-value-field="'+item.field+
      '">--</strong><span>'+item.unit+'</span></div><p>'+item.description+
      '</p></div>';
  }).join(''):'';
  dashUpdateDetailValues(dashState.lastSample);
}

function dashUpdateDetailValues(sample){
  document.querySelectorAll('#dashDataDetails [data-value-field]').forEach(function(element){
    var value=dashValue(sample,element.getAttribute('data-value-field'),null);
    element.textContent=dashFormat(value,Math.abs(value||0)<100?1:0);
  });
}

function dashRenderTooltip(sample,series,cssX,cssY){
  var tooltip=$('dashChartTooltip');
  if(!sample){tooltip.hidden=true;return}
  var deviceTime=dashValue(sample,'t',null);
  tooltip.innerHTML='<div class="dash-tooltip-time">'+
    (deviceTime===null?sample.hostTime:('DEVICE '+deviceTime+' ms'))+'</div>'+
    series.map(function(item){
      return'<div class="dash-tooltip-row"><span style="color:'+item.color+'">'+
        item.label+'</span><strong>'+dashFormat(
          dashValue(sample,item.field,null),Math.abs(dashValue(sample,item.field,0))<100?1:0)+
        '</strong></div>';
    }).join('');
  tooltip.hidden=false;
  var wrap=$('dashMainChartWrap');
  var maxLeft=Math.max(6,wrap.clientWidth-tooltip.offsetWidth-8);
  var maxTop=Math.max(6,wrap.clientHeight-tooltip.offsetHeight-8);
  tooltip.style.left=Math.max(6,Math.min(maxLeft,cssX+14))+'px';
  tooltip.style.top=Math.max(6,Math.min(maxTop,cssY+12))+'px';
}

function dashDrawChart(canvas,chart){
  var size=dashCanvasSize(canvas);
  if(!size)return;
  var series=chart.series;
  var ctx=canvas.getContext('2d');
  var width=size.width,height=size.height,ratio=size.ratio;
  var left=55*ratio,right=20*ratio,top=19*ratio,bottom=31*ratio;
  var now=Date.now(),windowMs=Number($('dashWindow').value)*1000;
  var visible=dashState.samples.filter(function(sample){
    return sample.hostMs>=now-windowMs;
  });
  ctx.clearRect(0,0,width,height);
  ctx.fillStyle='#11151a';ctx.fillRect(0,0,width,height);
  ctx.font=(9*ratio)+'px Consolas,monospace';
  ctx.strokeStyle='#29313a';ctx.lineWidth=ratio;
  var min=0,max=0,hasValue=false;
  visible.forEach(function(sample){
    series.forEach(function(item){
      var value=dashValue(sample,item.field,null);
      if(value!==null&&Number.isFinite(value)){
        min=Math.min(min,value);max=Math.max(max,value);hasValue=true;
      }
    });
  });
  if(!hasValue){min=-1;max=1}
  if(min===max){min-=1;max+=1}
  var pad=(max-min)*.1;min-=pad;max+=pad;
  for(var grid=0;grid<=5;grid++){
    var y=top+(height-top-bottom)*grid/5;
    ctx.beginPath();ctx.moveTo(left,y);ctx.lineTo(width-right,y);ctx.stroke();
    ctx.fillStyle='#687482';ctx.textAlign='right';ctx.textBaseline='middle';
    ctx.fillText((max-(max-min)*grid/5).toFixed(Math.abs(max-min)<20?1:0),left-6*ratio,y);
  }
  var firstTime=visible.length?visible[0].hostMs:now-windowMs;
  var displaySpan=Math.min(windowMs,Math.max(2000,now-firstTime));
  var displayStart=now-displaySpan;
  for(var vertical=0;vertical<=6;vertical++){
    var gridX=left+(width-left-right)*vertical/6;
    ctx.beginPath();ctx.moveTo(gridX,top);ctx.lineTo(gridX,height-bottom);ctx.stroke();
  }
  ctx.textAlign='center';ctx.textBaseline='top';ctx.fillStyle='#606c79';
  ctx.fillText('-'+(displaySpan/1000).toFixed(displaySpan<10000?1:0)+'s',left,height-bottom+6*ratio);
  ctx.fillText('now',width-right,height-bottom+5*ratio);
  $('dashVisibleRange').textContent='WINDOW '+(displaySpan/1000).toFixed(1)+' s · '+
    visible.length+' SAMPLES';

  if(!visible.length){
    dashDrawEmptyState(ctx,width,height,ratio,'等待遥测数据','连接设备并开启遥测，或使用模拟模式');
    dashRenderTooltip(null,series,0,0);
    return;
  }
  if(!series.length){
    dashDrawEmptyState(ctx,width,height,ratio,'没有可显示的曲线','请选择至少一个可用字段');
    dashRenderTooltip(null,series,0,0);
    return;
  }
  if(!hasValue){
    dashDrawEmptyState(ctx,width,height,ratio,'当前字段不可用','请烧录包含扩展 telemetry 字段的固件');
    dashRenderTooltip(null,series,0,0);
    return;
  }

  series.forEach(function(item,index){
    ctx.strokeStyle=item.color||DASH_COLORS[index%DASH_COLORS.length];
    ctx.lineWidth=1.7*ratio;ctx.lineCap='round';ctx.lineJoin='round';
    ctx.setLineDash(item.dashed?[6*ratio,5*ratio]:[]);
    ctx.beginPath();
    var started=false,lastPoint=null;
    visible.forEach(function(sample){
      var value=dashValue(sample,item.field,null);
      if(value===null||!Number.isFinite(value)){started=false;return}
      if(sample.hostMs<displayStart)return;
      var x=left+(width-left-right)*(sample.hostMs-displayStart)/displaySpan;
      var y=top+(height-top-bottom)*(max-value)/(max-min);
      if(!started){ctx.moveTo(x,y);started=true}else ctx.lineTo(x,y);
      lastPoint={x:x,y:y};
    });
    ctx.stroke();
    ctx.setLineDash([]);
    if(lastPoint){
      ctx.beginPath();ctx.arc(lastPoint.x,lastPoint.y,2.8*ratio,0,Math.PI*2);
      ctx.fillStyle=item.color||DASH_COLORS[index%DASH_COLORS.length];ctx.fill();
    }
  });

  if(dashState.hoverX!==null){
    var hoverCanvasX=Math.max(left,Math.min(width-right,dashState.hoverX*ratio));
    var targetTime=displayStart+
      (hoverCanvasX-left)/(width-left-right)*displaySpan;
    var hoverSample=DashboardCore.findNearestSample(visible,targetTime);
    if(hoverSample){
      var actualX=left+(width-left-right)*
        (hoverSample.hostMs-displayStart)/displaySpan;
      ctx.strokeStyle='#aeb7c2';ctx.lineWidth=ratio;ctx.setLineDash([3*ratio,3*ratio]);
      ctx.beginPath();ctx.moveTo(actualX,top);ctx.lineTo(actualX,height-bottom);ctx.stroke();
      ctx.setLineDash([]);
      series.forEach(function(item){
        var value=dashValue(hoverSample,item.field,null);
        if(value===null||!Number.isFinite(value))return;
        var pointY=top+(height-top-bottom)*(max-value)/(max-min);
        ctx.beginPath();ctx.arc(actualX,pointY,3.2*ratio,0,Math.PI*2);
        ctx.fillStyle=item.color;ctx.fill();
        ctx.strokeStyle='#11151a';ctx.lineWidth=ratio;ctx.stroke();
      });
      dashRenderTooltip(
        hoverSample,series,actualX/ratio,dashState.hoverY===null?20:dashState.hoverY);
    }
  }else{
    dashRenderTooltip(null,series,0,0);
  }
}

function dashDrawAll(){
  if(!$('dashboardView').hidden){
    var chart=dashActiveChart();
    if(chart){
      dashDrawChart($('dashMainChart'),chart);
    }else{
      var canvas=$('dashMainChart'),size=dashCanvasSize(canvas);
      if(size){
        var ctx=canvas.getContext('2d');
        ctx.fillStyle='#11151a';ctx.fillRect(0,0,size.width,size.height);
        dashDrawEmptyState(ctx,size.width,size.height,size.ratio,
          '未选择数据','从左侧选择一张图表开始按需遥测');
      }
      $('dashVisibleRange').textContent='NO DATA SELECTED';
      $('dashChartTooltip').hidden=true;
    }
  }
}

function dashResetPlot(){
  dashState.fields=[];
  dashState.samples=[];
  dashState.lastSample=null;
  dashState.lastSampleAt=0;
  dashState.hoverX=null;
  dashState.hoverY=null;
  $('dashChartTooltip').hidden=true;
}

function DashboardTelemetry_OnEvent(event){
  if(!dashState.activeGroup||(!dashState.enabled&&!dashState.requesting))return;
  if(event.type==='header'){
    var expected=DashboardCore.GROUP_FIELDS[dashState.activeGroup]||[];
    if(event.fields.join(',')!==expected.join(','))return;
    dashState.fields=event.fields.slice();
    dashState.enabled=true;
    dashUpdateControls();
    return;
  }
  if(event.type!=='sample')return;
  event.hostMs=Date.now();
  dashState.lastSample=event;
  dashState.lastSampleAt=event.hostMs;
  dashState.enabled=true;
  dashState.samples.push(event);
  var oldest=event.hostMs-DASH_MAX_HISTORY_MS;
  while(dashState.samples.length&&dashState.samples[0].hostMs<oldest){
    dashState.samples.shift();
  }
  if(dashState.recording){
    if(dashState.records.length>=DASH_MAX_RECORDS){
      dashState.recording=false;
      dashToast('记录达到 100,000 帧，已自动停止',true);
    }else{
      dashState.records.push(event);
    }
  }
  dashScheduleRender();
}

async function dashStartGroup(key,clearHistory){
  if(!dashState.connected||dashState.requesting)return false;
  var previousGroup=dashState.activeGroup;
  var previousEnabled=dashState.enabled;
  dashState.pendingGroup=key;
  dashState.requesting=true;
  dashBuildDataBrowser();dashRenderStatus();
  try{
    var command='telem on '+key+' '+$('dashPeriod').value;
    var response=await send(command,{timeoutMs:2500});
    if(!simMode&&(!/telem:\s*ok/i.test(response)||
       response.indexOf('profile='+key)<0)){
      throw new Error(response.trim()||'设备未确认');
    }
    if(dashState.recording){
      dashState.recording=false;
      dashToast('切换图表，当前记录已停止',false);
    }
    dashState.activeGroup=key;
    dashState.enabled=true;
    if(clearHistory||previousGroup!==key)dashResetPlot();
    dashState.fields=DashboardCore.GROUP_FIELDS[key].slice();
    if(simMode){
      dashSimStart(key,Number($('dashPeriod').value));
    }
    dashBuildDataBrowser();
    dashToast('正在显示：'+dashActiveChart().title,false);
    return true;
  }catch(error){
    dashState.activeGroup=previousGroup;
    dashState.enabled=previousEnabled;
    dashToast('图表切换失败：'+error.message,true);
    return false;
  }finally{
    dashState.pendingGroup=null;dashState.requesting=false;
    dashBuildDataBrowser();
    dashRenderStatus();
  }
}

async function dashStopStream(clearSelection){
  if(dashState.requesting)return;
  dashState.requesting=true;dashRenderStatus();
  try{
    if(dashState.connected&&!simMode&&dashState.enabled){
      var response=await send('telem off',{timeoutMs:1200});
      if(!/telem off:\s*ok/i.test(response)){
        throw new Error(response.trim()||'设备未确认');
      }
    }
  }catch(error){
    dashToast('停止遥测失败：'+error.message,true);
  }finally{
    dashState.enabled=false;
    dashState.recording=false;
    dashSimStop();
    if(clearSelection){
      dashState.activeGroup=null;
      dashResetPlot();
      dashBuildDataBrowser();
    }
    dashState.requesting=false;
    dashRenderStatus();dashDrawAll();
    if(!clearSelection&&dashState.visible&&dashState.connected&&
       dashState.activeGroup&&!dashState.enabled){
      dashStartGroup(dashState.activeGroup,false);
    }
  }
}

function dashSimStart(group,period){
  dashSimStop();
  dashState.simStart=Date.now();
  dashState.simStopped=false;
  var fields=DashboardCore.GROUP_FIELDS[group];
  if(serialRouter)serialRouter.push('#'+fields.join(',')+'\n');
  dashState.simTimer=setInterval(dashSimFrame,period);
  dashSimFrame();
}

function dashSimStop(){
  if(dashState.simTimer)clearInterval(dashState.simTimer);
  dashState.simTimer=null;
}

function dashSimFrame(){
  if(!serialRouter||!dashState.activeGroup)return;
  var elapsed=Date.now()-dashState.simStart,t=elapsed/1000;
  var values={};
  var fields=DashboardCore.GROUP_FIELDS[dashState.activeGroup];
  fields.forEach(function(field){values[field]=0});
  values.t=elapsed;
  values.L_tgt=dashState.simStopped?0:120;values.R_tgt=dashState.simStopped?0:115;
  values.L_act=dashState.simStopped?0:118+Math.sin(t*2)*3;
  values.R_act=dashState.simStopped?0:113+Math.cos(t*2)*3;
  values.yaw_tgt=45;values.yaw=45+Math.sin(t*.7)*2;values.head_err=45-values.yaw;
  values.gray_pos=Math.sin(t*1.3)*350;values.lf_err=-values.gray_pos;
  values.lf_corr=values.gray_pos*.16;
  values.acc_x_mg=Math.sin(t)*35;values.acc_y_mg=Math.cos(t*.8)*28;
  values.acc_z_mg=1000+Math.sin(t*2)*10;
  values.gyro_x_mdps=Math.sin(t*.9)*900;values.gyro_y_mdps=Math.cos(t)*700;
  values.gyro_z_mdps=Math.sin(t*.7)*12000;
  var row=fields.map(function(field){return values[field]}).join(',');
  serialRouter.push(row+'\n');
}

function Dashboard_OnShow(){
  dashState.visible=true;
  dashBuildDataBrowser();dashDrawAll();
  if(dashState.connected&&dashState.activeGroup&&!dashState.enabled&&!dashState.requesting){
    dashStartGroup(dashState.activeGroup,false);
  }
}

function Dashboard_OnHide(){
  dashState.visible=false;
  if(dashState.enabled&&!dashState.requesting)dashStopStream(false);
}

function Dashboard_OnConnection(connected){
  dashState.connected=connected;
  if(!connected){
    dashState.enabled=false;dashState.requesting=false;dashState.recording=false;
    dashState.lastSample=null;dashState.lastSampleAt=0;dashSimStop();
  }else if(dashState.visible&&dashState.activeGroup){
    dashStartGroup(dashState.activeGroup,true);
  }
  dashRenderStatus();
}

async function Dashboard_BeforeDisconnect(){
  if(dashState.enabled&&!simMode&&dashState.activeGroup){
    try{await send('telem off',{timeoutMs:700})}catch(error){}
  }
  dashState.enabled=false;dashSimStop();
}

function dashToggleRecord(){
  if(!dashState.recording){
    dashState.records=[];
    dashState.recordFields=dashState.fields.slice();
    dashState.recordGroup=dashState.activeGroup;
    dashState.recording=true;
    dashToast('开始记录遥测',false);
  }else{
    dashState.recording=false;
    dashToast('记录已停止，共 '+dashState.records.length+' 帧',false);
  }
  dashUpdateControls();
}

function dashExport(){
  if(!dashState.records.length||!dashState.recordFields.length)return;
  var csv=DashboardCore.exportCsv(dashState.recordFields,dashState.records);
  var blob=new Blob([csv],{type:'text/csv;charset=utf-8'});
  var link=document.createElement('a');
  link.href=URL.createObjectURL(blob);
  link.download='gugapi-'+(dashState.recordGroup||'telemetry')+'-'+
    new Date().toISOString().replace(/[:.]/g,'-')+'.csv';
  document.body.appendChild(link);link.click();link.remove();
  setTimeout(function(){URL.revokeObjectURL(link.href)},0);
}

async function dashEstop(){
  if(!dashState.connected||dashState.requesting)return;
  dashState.requesting=true;$('btnDashEstop').textContent='停止中…';dashUpdateControls();
  if(simMode)dashState.simStopped=true;
  try{
    var response=await send('estop',{timeoutMs:2500});
    if(!simMode&&!/estop:\s*ok/i.test(response)){
      throw new Error(response.trim()||'设备未确认');
    }
    if(dashState.activeGroup!=='motor'){
      dashToast('软件全停命令已确认',false);
      return;
    }
    var deadline=Date.now()+2000;
    while(Date.now()<deadline){
      var latest=dashState.lastSample;
      if(latest&&dashValue(latest,'L_tgt',1)===0&&dashValue(latest,'R_tgt',1)===0){
        dashToast('软件全停已确认：目标轮速为 0',false);
        return;
      }
      await new Promise(function(resolve){setTimeout(resolve,80)});
    }
    dashToast('停止命令已发送，未在 2 秒内收到零目标确认',true);
  }catch(error){
    dashToast('软件全停失败：'+error.message,true);
  }finally{
    dashState.requesting=false;$('btnDashEstop').textContent='STOP ALL';dashUpdateControls();
  }
}

dashBuildDataBrowser();
$('btnDashTelemetry').onclick=function(){dashStopStream(true)};
$('dashPeriod').onchange=function(){
  if(dashState.enabled&&dashState.activeGroup){
    dashStartGroup(dashState.activeGroup,false);
  }
};
$('dashWindow').onchange=dashDrawAll;
$('btnDashRecord').onclick=dashToggleRecord;
$('btnDashExport').onclick=dashExport;
$('btnDashClear').onclick=function(){dashState.samples=[];dashDrawAll();dashToast('曲线已清空',false)};
$('btnDashEstop').onclick=dashEstop;
$('dashMainChart').addEventListener('pointermove',function(event){
  var rect=$('dashMainChart').getBoundingClientRect();
  dashState.hoverX=event.clientX-rect.left;
  dashState.hoverY=event.clientY-rect.top;
  dashScheduleRender();
});
$('dashMainChart').addEventListener('pointerleave',function(){
  dashState.hoverX=null;dashState.hoverY=null;
  $('dashChartTooltip').hidden=true;
  dashScheduleRender();
});
window.addEventListener('resize',dashDrawAll);
setInterval(function(){
  dashRenderStatus();
  if(!$('dashboardView').hidden)dashDrawAll();
},250);

var dashboardPreviousSerialState=onSerialStateChange;
onSerialStateChange=function(connected){
  if(typeof dashboardPreviousSerialState==='function')dashboardPreviousSerialState(connected);
  Dashboard_OnConnection(connected);
};
Dashboard_OnConnection(!!writer||simMode);
