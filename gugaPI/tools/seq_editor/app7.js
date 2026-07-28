'use strict';

var DASH_MAX_RECORDS=100000;
var DASH_MAX_HISTORY_MS=60000;
var DASH_MODE={0:'IDLE',1:'RUNNING',2:'FAULT',3:'ARMED',4:'COMP RUNNING'};
var DASH_FAULT={0:'NONE',1:'ASSERT',2:'DRIVER INIT',3:'DRIVER TIMEOUT',
  4:'SENSOR LOST',5:'UART OVERFLOW',6:'UNKNOWN'};
var DASH_STATUS={0:'ok',1:'error',2:'invalid-arg',3:'not-initialized',
  4:'timeout',5:'busy',6:'unsupported',7:'nack'};
var DASH_COLORS=['#88c0d0','#bf616a','#a3be8c','#ebcb8b','#b48ead','#d08770'];
var DASH_HEADER=DashboardCore.DEFAULT_FIELDS;
var DASH_LAYOUT_STORAGE_KEY='gugapi.dashboard.layout.v2';
var dashState={
  shown:false,connected:false,enabled:false,requesting:false,fields:[],
  samples:[],lastSample:null,lastSampleAt:0,recording:false,records:[],
  renderPending:false,simTimer:null,simStart:0,simStopped:false,
  activeGroup:'motor',visibleSeries:{},hoverX:null,hoverY:null
};

function dashValue(sample,name,fallback){
  return sample&&Object.prototype.hasOwnProperty.call(sample.values,name)
    ?sample.values[name]:fallback;
}

function dashClass(element,name){
  element.classList.remove('ok','warn','bad');
  if(name)element.classList.add(name);
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
  $('btnDashTelemetry').textContent=dashState.enabled?'关闭遥测':'开启遥测';
}

function dashUpdateControls(){
  var available=dashState.connected&&!dashState.requesting;
  $('btnDashTelemetry').disabled=!available;
  $('dashPeriod').disabled=!available;
  $('btnDashRecord').disabled=!dashState.connected||!dashState.enabled;
  $('btnDashExport').disabled=dashState.records.length===0;
  $('btnDashEstop').disabled=!dashState.connected||dashState.requesting;
  $('btnDashRecord').textContent=dashState.recording?'停止记录':'开始记录';
}

function dashRenderStatus(){
  var sample=dashState.lastSample;
  var age=dashState.lastSampleAt?Date.now()-dashState.lastSampleAt:Infinity;
  $('dashSampleCount').textContent=dashState.samples.length+' samples';
  if(!dashState.connected){
    dashSetTelemetryState('遥测未启动','');
    $('dashSampleAge').textContent='未连接';
    dashClass($('dashSampleAge'),'bad');
  }else if(!dashState.enabled){
    dashSetTelemetryState('遥测已关闭','');
    $('dashSampleAge').textContent='等待开启';
    dashClass($('dashSampleAge'),'warn');
  }else if(age<=300){
    dashSetTelemetryState('实时 · '+Math.round(age)+' ms','live');
    $('dashSampleAge').textContent='在线 · '+Math.round(age)+' ms';
    dashClass($('dashSampleAge'),'ok');
  }else{
    dashSetTelemetryState('数据超时','stale');
    $('dashSampleAge').textContent=isFinite(age)?'超时 · '+Math.round(age)+' ms':'等待首帧';
    dashClass($('dashSampleAge'),'bad');
  }

  if(!sample){
    ['dashMode','dashFault','dashTask','dashFeedback','dashTx'].forEach(function(id){
      $(id).textContent='--';dashClass($(id),'');
    });
    dashUpdateLegendValues(null);
    dashUpdateControls();
    return;
  }

  var mode=dashValue(sample,'mode',null);
  $('dashMode').textContent=mode===null?'不可用':(DASH_MODE[mode]||('MODE '+mode));
  dashClass($('dashMode'),mode===2?'bad':(mode===1||mode===4?'warn':'ok'));

  var fault=dashValue(sample,'fault_code',null);
  var faultCount=dashValue(sample,'fault_count',null);
  $('dashFault').textContent=fault===null?'不可用':
    (DASH_FAULT[fault]||('FAULT '+fault))+(faultCount?' ×'+faultCount:'');
  dashClass($('dashFault'),fault===0?'ok':(fault===null?'warn':'bad'));

  var slot=dashValue(sample,'comp_slot',null);
  var count=dashValue(sample,'comp_count',null);
  var step=dashValue(sample,'step',-1);
  $('dashTask').textContent=slot===null?
    ('STEP '+(step<0?'--':step)):
    ('SLOT '+slot+' · '+(step<0?'待命':'STEP '+step)+' / '+count);
  dashClass($('dashTask'),dashValue(sample,'comp_slot_valid',1)?'ok':'warn');

  var chassis=dashValue(sample,'chassis_init',null);
  var feedbackValid=dashValue(sample,'feedback_valid',0);
  var feedbackAge=dashValue(sample,'feedback_age_ms',0);
  var feedbackStatus=dashValue(sample,'feedback_status',null);
  if(chassis===null){
    $('dashFeedback').textContent='不可用';
    dashClass($('dashFeedback'),'warn');
  }else if(chassis&&feedbackValid&&feedbackStatus===0&&feedbackAge<=100){
    $('dashFeedback').textContent='正常 · '+feedbackAge+' ms';
    dashClass($('dashFeedback'),'ok');
  }else{
    $('dashFeedback').textContent=!chassis?'未初始化':
      ((DASH_STATUS[feedbackStatus]||feedbackStatus)+' · '+feedbackAge+' ms');
    dashClass($('dashFeedback'),feedbackAge<=300?'warn':'bad');
  }

  var pending=dashValue(sample,'tx_pending',null);
  var dropped=dashValue(sample,'tx_dropped',null);
  $('dashTx').textContent=pending===null?'不可用':
    (pending+' B · dropped '+dropped);
  dashClass($('dashTx'),dropped>0?'warn':'ok');

  $('dashYaw').textContent=dashFormat(dashValue(sample,'yaw',null),1);
  $('dashPitch').textContent=dashFormat(dashValue(sample,'pitch',null),1);
  $('dashRoll').textContent=dashFormat(dashValue(sample,'roll',null),1);
  var temp=dashValue(sample,'imu_temp_cc',null);
  $('dashTemp').textContent=temp===null?'--':(temp/100).toFixed(1);
  dashRenderGray(sample);
  dashUpdateLegendValues(sample);
  dashUpdateControls();
}

function dashFormat(value,digits){
  return value===null||value===undefined||!Number.isFinite(value)
    ?'--':Number(value).toFixed(digits);
}

function dashBuildGrayBars(){
  var container=$('dashGrayBars');
  container.innerHTML='';
  for(var index=0;index<8;index++){
    var channel=document.createElement('div');
    channel.className='dash-gray-channel';
    channel.innerHTML='<span>CH'+index+'</span>'+
      '<div class="dash-gray-track"><div class="dash-gray-fill"></div></div>'+
      '<strong>--</strong>';
    container.appendChild(channel);
  }
}

function dashRenderGray(sample){
  var channels=$('dashGrayBars').children;
  for(var index=0;index<channels.length;index++){
    var value=dashValue(sample,'gray'+index,null);
    channels[index].querySelector('.dash-gray-fill').style.width=
      value===null?'0%':Math.max(0,Math.min(100,value/4095*100))+'%';
    channels[index].querySelector('strong').textContent=value===null?'--':value;
  }
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
    {field:'L_tgt',label:'L target',color:'#55bfe9',dashed:true},
    {field:'L_act',label:'L actual',color:'#348ed1'},
    {field:'R_tgt',label:'R target',color:'#e4b657',dashed:true},
    {field:'R_act',label:'R actual',color:'#df8352'}]},
  {key:'heading',title:'航向控制',subtitle:'目标 / 实际 / 误差',unit:'DEG',series:[
    {field:'yaw_tgt',label:'yaw target',color:'#55bfe9',dashed:true},
    {field:'yaw',label:'yaw',color:'#55c68a'},
    {field:'head_err',label:'heading error',color:'#df6670'}]},
  {key:'line',title:'循迹控制',subtitle:'位置误差与闭环修正',unit:'CONTROL',series:[
    {field:'gray_pos',label:'gray position',color:'#ae84c6'},
    {field:'lf_err',label:'LF error',color:'#55bfe9'},
    {field:'lf_corr',label:'LF correction',color:'#e4b657'}]},
  {key:'accel',title:'线加速度',subtitle:'IMU 三轴加速度',unit:'MG',series:[
    {field:'acc_x_mg',label:'acc X',color:'#df6670'},
    {field:'acc_y_mg',label:'acc Y',color:'#55c68a'},
    {field:'acc_z_mg',label:'acc Z',color:'#55bfe9'}]},
  {key:'gyro',title:'角速度',subtitle:'IMU 三轴角速度',unit:'MDPS',series:[
    {field:'gyro_x_mdps',label:'gyro X',color:'#df6670'},
    {field:'gyro_y_mdps',label:'gyro Y',color:'#55c68a'},
    {field:'gyro_z_mdps',label:'gyro Z',color:'#55bfe9'}]}
];

function dashActiveChart(){
  return dashCharts.find(function(chart){return chart.key===dashState.activeGroup})||
    dashCharts[0];
}

function dashFieldAvailable(field){
  return dashState.fields.length===0||dashState.fields.indexOf(field)>=0;
}

function dashSeriesVisible(item){
  return dashState.visibleSeries[item.field]!==false;
}

function dashLoadLayout(){
  try{
    var saved=JSON.parse(localStorage.getItem(DASH_LAYOUT_STORAGE_KEY)||'{}');
    if(dashCharts.some(function(chart){return chart.key===saved.activeGroup})){
      dashState.activeGroup=saved.activeGroup;
    }
    if(saved.visibleSeries&&typeof saved.visibleSeries==='object'){
      dashState.visibleSeries=saved.visibleSeries;
    }
  }catch(error){}
}

function dashSaveLayout(){
  try{
    localStorage.setItem(DASH_LAYOUT_STORAGE_KEY,JSON.stringify({
      activeGroup:dashState.activeGroup,
      visibleSeries:dashState.visibleSeries
    }));
  }catch(error){}
}

function dashToggleSeries(field){
  var chart=dashActiveChart();
  var item=chart.series.find(function(series){return series.field===field});
  if(!item||!dashFieldAvailable(field))return;
  var visible=chart.series.filter(function(series){
    return dashFieldAvailable(series.field)&&dashSeriesVisible(series);
  });
  if(dashSeriesVisible(item)&&visible.length<=1){
    dashToast('至少保留一条可见曲线',true);
    return;
  }
  dashState.visibleSeries[field]=!dashSeriesVisible(item);
  dashSaveLayout();
  dashBuildDataBrowser();
  dashDrawAll();
}

function dashSelectGroup(key){
  if(dashState.activeGroup===key)return;
  dashState.activeGroup=key;
  dashState.hoverX=null;dashState.hoverY=null;
  $('dashChartTooltip').hidden=true;
  dashSaveLayout();
  dashBuildDataBrowser();
  dashDrawAll();
}

function dashBuildDataBrowser(){
  var active=dashActiveChart();
  var availableSeries=active.series.filter(function(item){
    return dashFieldAvailable(item.field);
  });
  if(availableSeries.length&&
     !availableSeries.some(function(item){return dashSeriesVisible(item)})){
    dashState.visibleSeries[availableSeries[0].field]=true;
    dashSaveLayout();
  }
  $('dashGroupList').innerHTML=dashCharts.map(function(chart){
    var available=chart.series.filter(function(item){
      return dashFieldAvailable(item.field);
    }).length;
    return'<button class="dash-group-button'+(chart.key===active.key?' active':'')+
      '" type="button" data-group="'+chart.key+'"><span class="dash-group-accent"></span>'+
      '<span class="dash-group-copy"><strong>'+chart.title+'</strong><small>'+
      chart.unit+'</small></span><span class="dash-group-count">'+available+'/'+
      chart.series.length+'</span></button>';
  }).join('');
  $('dashGroupList').querySelectorAll('[data-group]').forEach(function(button){
    button.onclick=function(){dashSelectGroup(button.getAttribute('data-group'))};
  });

  $('dashChartTitle').textContent=active.title;
  $('dashChartSubtitle').textContent=active.subtitle+' · '+active.unit;
  $('dashSeriesList').innerHTML=active.series.map(function(item){
    var available=dashFieldAvailable(item.field);
    var visible=dashSeriesVisible(item);
    return'<button class="dash-series-toggle'+(visible?'':' off')+
      (available?'':' unavailable')+'" type="button" data-field="'+item.field+
      '" style="color:'+item.color+'"><span class="dash-series-dot"></span>'+
      '<span class="dash-series-name">'+item.label+'</span><small>'+
      (available?(visible?'ON':'OFF'):'N/A')+'</small></button>';
  }).join('');
  $('dashSeriesList').querySelectorAll('[data-field]').forEach(function(button){
    button.onclick=function(){dashToggleSeries(button.getAttribute('data-field'))};
  });

  var visibleSeries=active.series.filter(dashSeriesVisible);
  $('dashMainLegend').innerHTML=visibleSeries.map(function(item){
    return'<div class="dash-main-legend-item" data-field="'+item.field+
      '" role="button" tabindex="0"><span class="dash-main-legend-line" style="background:'+
      item.color+'"></span><span>'+item.label+'</span>'+
      '<strong data-value-field="'+item.field+'">--</strong></div>';
  }).join('');
  $('dashCurrentValues').innerHTML=visibleSeries.map(function(item){
    return'<div><span class="dash-current-name"><i class="dash-current-dot" style="background:'+
      item.color+'"></i>'+item.label+'</span><strong data-value-field="'+item.field+
      '">--</strong></div>';
  }).join('');
  $('dashMainLegend').querySelectorAll('[data-field]').forEach(function(item){
    item.onclick=function(){dashToggleSeries(item.getAttribute('data-field'))};
    item.onkeydown=function(event){
      if(event.key==='Enter'||event.key===' '){
        event.preventDefault();dashToggleSeries(item.getAttribute('data-field'));
      }
    };
  });
  dashUpdateLegendValues(dashState.lastSample);
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
  var series=chart.series.filter(function(item){
    return dashSeriesVisible(item)&&dashFieldAvailable(item.field);
  });
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
    dashDrawChart($('dashMainChart'),dashActiveChart());
  }
}

function dashUpdateLegendValues(sample){
  document.querySelectorAll('[data-value-field]').forEach(function(element){
    var value=dashValue(sample,element.getAttribute('data-value-field'),null);
    element.textContent=dashFormat(value,Math.abs(value||0)<100?1:0);
  });
}

function DashboardTelemetry_OnEvent(event){
  if(event.type==='header'){
    dashState.fields=event.fields.slice();
    dashState.enabled=true;
    dashBuildDataBrowser();
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

async function dashSetTelemetry(enabled){
  if(!dashState.connected||dashState.requesting)return;
  dashState.requesting=true;dashUpdateControls();
  try{
    var command=enabled?'telem on '+$('dashPeriod').value:'telem off';
    var response=await send(command,{timeoutMs:2500});
    if(!simMode&&!new RegExp(enabled?'telem: ok':'telem off: ok','i').test(response)){
      throw new Error(response.trim()||'设备未确认');
    }
    dashState.enabled=enabled;
    if(!enabled)dashState.recording=false;
    if(simMode){
      if(enabled)dashSimStart(Number($('dashPeriod').value));
      else dashSimStop();
    }
    dashToast(enabled?'遥测已开启':'遥测已关闭',false);
  }catch(error){
    dashToast('遥测切换失败：'+error.message,true);
  }finally{
    dashState.requesting=false;
    dashRenderStatus();
  }
}

function dashSimStart(period){
  dashSimStop();
  dashState.simStart=Date.now();
  dashState.simStopped=false;
  if(serialRouter)serialRouter.push('#'+DASH_HEADER.join(',')+'\n');
  dashState.simTimer=setInterval(dashSimFrame,period);
  dashSimFrame();
}

function dashSimStop(){
  if(dashState.simTimer)clearInterval(dashState.simTimer);
  dashState.simTimer=null;
}

function dashSimFrame(){
  if(!serialRouter)return;
  var elapsed=Date.now()-dashState.simStart,t=elapsed/1000;
  var values={};
  DASH_HEADER.forEach(function(field){values[field]=0});
  values.t=elapsed;values.mode=dashState.simStopped?0:1;values.step=dashState.simStopped?-1:2;
  values.L_tgt=dashState.simStopped?0:120;values.R_tgt=dashState.simStopped?0:115;
  values.L_act=dashState.simStopped?0:118+Math.sin(t*2)*3;
  values.R_act=dashState.simStopped?0:113+Math.cos(t*2)*3;
  values.yaw_tgt=45;values.yaw=45+Math.sin(t*.7)*2;values.head_err=45-values.yaw;
  values.gray_pos=Math.sin(t*1.3)*350;values.lf_err=-values.gray_pos;
  values.lf_corr=values.gray_pos*.16;values.gray_strength=2450;values.gray_conf=890;
  values.gray_valid=1;values.chassis_init=1;values.chassis_status=0;
  values.feedback_status=0;values.feedback_valid=1;values.feedback_age_ms=12;
  values.comp_slot=2;values.comp_slot_valid=1;values.comp_count=6;
  values.imu_valid=1;values.imu_age_ms=5;
  values.acc_x_mg=Math.sin(t)*35;values.acc_y_mg=Math.cos(t*.8)*28;
  values.acc_z_mg=1000+Math.sin(t*2)*10;
  values.gyro_x_mdps=Math.sin(t*.9)*900;values.gyro_y_mdps=Math.cos(t)*700;
  values.gyro_z_mdps=Math.sin(t*.7)*12000;
  values.pitch=Math.sin(t*.4)*4;values.roll=Math.cos(t*.5)*3;
  values.imu_temp_cc=2840;values.gray_sample_valid=1;values.gray_age_ms=7;
  for(var channel=0;channel<8;channel++){
    values['gray'+channel]=Math.round(700+3100*Math.max(0,
      Math.sin(t*.8+channel*.65)));
  }
  var row=DASH_HEADER.map(function(field){return values[field]}).join(',');
  serialRouter.push(row+'\n');
}

function Dashboard_OnShow(){
  dashState.shown=true;
  dashDrawAll();
  if(dashState.connected&&!dashState.enabled&&!dashState.requesting){
    dashSetTelemetry(true);
  }
}

function Dashboard_OnConnection(connected){
  dashState.connected=connected;
  if(!connected){
    dashState.enabled=false;dashState.requesting=false;dashState.recording=false;
    dashState.lastSample=null;dashState.lastSampleAt=0;dashSimStop();
  }else if(dashState.shown){
    dashSetTelemetry(true);
  }
  dashRenderStatus();
}

async function Dashboard_BeforeDisconnect(){
  if(dashState.enabled&&!simMode){
    try{await send('telem off',{timeoutMs:700})}catch(error){}
  }
  dashState.enabled=false;dashSimStop();
}

function dashToggleRecord(){
  if(!dashState.recording){
    dashState.records=[];
    dashState.recording=true;
    dashToast('开始记录遥测',false);
  }else{
    dashState.recording=false;
    dashToast('记录已停止，共 '+dashState.records.length+' 帧',false);
  }
  dashUpdateControls();
}

function dashExport(){
  if(!dashState.records.length||!dashState.fields.length)return;
  var csv=DashboardCore.exportCsv(dashState.fields,dashState.records);
  var blob=new Blob([csv],{type:'text/csv;charset=utf-8'});
  var link=document.createElement('a');
  link.href=URL.createObjectURL(blob);
  link.download='gugapi-telemetry-'+new Date().toISOString().replace(/[:.]/g,'-')+'.csv';
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

dashLoadLayout();
dashBuildGrayBars();
dashBuildDataBrowser();
$('btnDashTelemetry').onclick=function(){dashSetTelemetry(!dashState.enabled)};
$('dashPeriod').onchange=function(){if(dashState.enabled)dashSetTelemetry(true)};
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
