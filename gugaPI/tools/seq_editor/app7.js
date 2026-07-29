'use strict';

var DASH_MAX_RECORDS=100000;
var DASH_MAX_HISTORY_MS=60000;
var DASH_COLORS=['#88c0d0','#bf616a','#a3be8c','#ebcb8b','#b48ead','#d08770'];
var dashState={
  visible:false,connected:false,enabled:false,requesting:false,fields:[],
  samples:[],lastSample:null,lastSampleAt:0,recording:false,records:[],
  recordFields:[],recordGroup:null,
  renderPending:false,simTimer:null,simStart:0,simStopped:false,
  activeGroup:null,pendingGroup:null,hoverX:null,hoverY:null,timelineEndMs:0,
  categoryOpen:{runtime:true,chassis:true,line:false,turn:false,imu:false,
    gray:false,infrared:false,system:false}
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
  $('btnDashTelemetry').textContent=dashState.enabled?'停止采集':'继续采集';
}

function dashUpdateControls(){
  $('btnDashTelemetry').disabled=!dashState.activeGroup||
    !dashState.connected||dashState.requesting;
  $('dashPeriod').disabled=dashState.requesting;
  $('btnDashRecord').disabled=!dashState.connected||!dashState.enabled;
  $('btnDashExport').disabled=dashState.records.length===0;
  $('btnDashRecord').textContent=dashState.recording?'停止记录':'开始记录';
}

function dashRenderStatus(){
  var sample=dashState.lastSample;
  var age=dashState.lastSampleAt?Date.now()-dashState.lastSampleAt:Infinity;
  var liveLimit=Math.max(300,Number($('dashPeriod').value)*3);
  $('dashSampleCount').textContent=dashState.samples.length+' 个样本';
  if(dashState.requesting){
    dashSetTelemetryState('正在切换数据…','stale');
  }else if(!dashState.activeGroup){
    dashSetTelemetryState('未选择数据','');
  }else if(!dashState.connected){
    dashSetTelemetryState('串口未连接','');
  }else if(!dashState.enabled){
    dashSetTelemetryState(dashState.visible?'采集已停止':'页面已暂停','');
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

function dashDigits(item,value){
  if(item&&item.step)return 0;
  return Math.abs(value||0)<100?1:0;
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

var dashCategories=DashboardCore.CATEGORIES;
var dashCharts=DashboardCore.CHARTS;

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
  $('dashGroupList').innerHTML=dashCategories.map(function(category){
    var charts=dashCharts.filter(function(item){return item.category===category.key});
    var open=dashState.categoryOpen[category.key]!==false;
    return'<section class="dash-category'+(open?' open':'')+'"><button '+
      'class="dash-category-toggle" type="button" data-category="'+category.key+
      '" aria-expanded="'+open+'"><span>'+category.label+'</span><small>'+charts.length+
      ' 张图</small><i></i></button><div class="dash-category-items">'+
      charts.map(function(chart){
        return'<button class="dash-group-button'+
          (active&&chart.key===active.key?' active':'')+
          (chart.key===dashState.pendingGroup?' pending':'')+
          '" type="button" data-group="'+chart.key+'"><span class="dash-group-accent"></span>'+
          '<span class="dash-group-copy"><strong>'+chart.title+'</strong><small>'+
          chart.subtitle+'</small></span><span class="dash-group-count">'+
          chart.series.length+' 项</span></button>';
      }).join('')+'</div></section>';
  }).join('');
  $('dashGroupList').querySelectorAll('[data-category]').forEach(function(button){
    button.onclick=function(){
      var key=button.getAttribute('data-category');
      dashState.categoryOpen[key]=dashState.categoryOpen[key]===false;
      dashBuildDataBrowser();
    };
  });
  $('dashGroupList').querySelectorAll('[data-group]').forEach(function(button){
    button.onclick=function(){dashSelectGroup(button.getAttribute('data-group'))};
  });

  $('dashChartTitle').textContent=active?active.title:'未选择数据';
  $('dashChartSubtitle').textContent=active?
    (active.subtitle+' · '+active.unit):'请从左侧选择一张图表';
  $('dashDetailsTitle').textContent=active?active.title:'未选择';
  $('dashDataDetailsEmpty').hidden=!!active;
  $('dashDataDetails').innerHTML=active?active.series.map(function(item){
    return'<div class="dash-detail-item"><div class="dash-detail-head">'+
      '<i class="dash-detail-dot" style="background:'+item.color+'"></i>'+
      '<strong>'+item.label+'</strong><code>'+item.field+'</code></div>'+
      '<div class="dash-detail-value"><strong data-value-field="'+item.field+
      '">--</strong><span>'+item.unit+'</span></div><div class="dash-detail-meaning" '+
      'data-meaning-field="'+item.field+'"></div><p>'+item.description+
      '</p></div>';
  }).join(''):'';
  dashUpdateDetailValues(dashState.lastSample);
}

function dashUpdateDetailValues(sample){
  var chart=dashActiveChart();
  document.querySelectorAll('#dashDataDetails [data-value-field]').forEach(function(element){
    var field=element.getAttribute('data-value-field');
    var value=dashValue(sample,field,null);
    var item=chart&&chart.series.find(function(entry){return entry.field===field});
    element.textContent=dashFormat(value,dashDigits(item,value));
  });
  document.querySelectorAll('#dashDataDetails [data-meaning-field]').forEach(function(element){
    var field=element.getAttribute('data-meaning-field');
    var item=chart&&chart.series.find(function(entry){return entry.field===field});
    var meaning=item?DashboardCore.describeValue(
      item.enumType,dashValue(sample,field,null)):'';
    element.textContent=meaning?('含义：'+meaning):'';
    element.hidden=!meaning;
  });
}

function dashRenderTooltip(sample,series,cssX,cssY){
  var tooltip=$('dashChartTooltip');
  if(!sample){tooltip.hidden=true;return}
  var deviceTime=dashValue(sample,'t',null);
  tooltip.innerHTML='<div class="dash-tooltip-time">'+
    (deviceTime===null?sample.hostTime:('设备时间 '+deviceTime+' ms'))+'</div>'+
    series.map(function(item){
      return'<div class="dash-tooltip-row"><span style="color:'+item.color+'">'+
        item.label+'</span><strong>'+dashFormat(
          dashValue(sample,item.field,null),dashDigits(
            item,dashValue(sample,item.field,0)))+
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
  var windowMs=Number($('dashWindow').value)*1000;
  var timeline=DashboardCore.buildTimeline(
    dashState.samples,windowMs,dashState.timelineEndMs||undefined);
  var visible=timeline.samples;
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
  var displaySpan=timeline.span;
  var displayStart=timeline.start;
  for(var vertical=0;vertical<=6;vertical++){
    var gridX=left+(width-left-right)*vertical/6;
    ctx.beginPath();ctx.moveTo(gridX,top);ctx.lineTo(gridX,height-bottom);ctx.stroke();
  }
  ctx.textAlign='center';ctx.textBaseline='top';ctx.fillStyle='#606c79';
  ctx.fillText('-'+(displaySpan/1000).toFixed(displaySpan<10000?1:0)+'s',left,height-bottom+6*ratio);
  ctx.fillText('最新',width-right,height-bottom+5*ratio);
  $('dashVisibleRange').textContent='窗口 '+(displaySpan/1000).toFixed(1)+' 秒 · '+
    visible.length+' 个样本';

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
      if(!started){
        ctx.moveTo(x,y);started=true;
      }else if(item.step&&lastPoint){
        ctx.lineTo(x,lastPoint.y);ctx.lineTo(x,y);
      }else{
        ctx.lineTo(x,y);
      }
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
      $('dashVisibleRange').textContent='尚未选择数据';
      $('dashChartTooltip').hidden=true;
    }
  }
}

function dashResetPlot(){
  dashState.fields=[];
  dashState.samples=[];
  dashState.lastSample=null;
  dashState.lastSampleAt=0;
  dashState.timelineEndMs=0;
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
  var sampleExpected=DashboardCore.GROUP_FIELDS[dashState.activeGroup]||[];
  if(event.fields.join(',')!==sampleExpected.join(','))return;
  event.hostMs=Date.now();
  dashState.lastSample=event;
  dashState.lastSampleAt=event.hostMs;
  dashState.timelineEndMs=event.hostMs;
  dashState.enabled=true;
  dashState.samples.push(event);
  var windowMs=Math.min(DASH_MAX_HISTORY_MS,
    Number($('dashWindow').value)*1000);
  dashState.samples=DashboardCore.trimTimelineSamples(
    dashState.samples,windowMs,dashState.timelineEndMs);
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

function dashDelay(ms){
  return new Promise(function(resolve){setTimeout(resolve,ms)});
}

function dashTelemStatusMatches(response,key,period,enabled){
  var text=String(response||'');
  return text.indexOf('telem enabled='+(enabled?'1':'0'))>=0&&
    (!enabled||(text.indexOf('profile='+key)>=0&&
      text.indexOf('period_ms='+period)>=0));
}

async function dashConfirmTelemetryStopped(){
  var lastResponse='';
  for(var attempt=0;attempt<2;attempt++){
    lastResponse=await send('telem off',{timeoutMs:1800});
    if(/telem off:\s*ok/i.test(lastResponse))return true;
    var status=await send('telem status',{timeoutMs:1800});
    if(dashTelemStatusMatches(status,'',0,false))return true;
    lastResponse=status||lastResponse;
    await dashDelay(60);
  }
  throw new Error(lastResponse.trim()||'设备未确认遥测已停止');
}

async function dashActivateTelemetryProfile(key,period){
  var command='telem on '+key+' '+period;
  var lastResponse='';
  for(var attempt=0;attempt<2;attempt++){
    lastResponse=await send(command,{timeoutMs:2500});
    if(/telem:\s*ok/i.test(lastResponse)&&
       lastResponse.indexOf('profile='+key)>=0&&
       lastResponse.indexOf('period_ms='+period)>=0){
      return true;
    }
    var status=await send('telem status',{timeoutMs:1800});
    if(dashTelemStatusMatches(status,key,period,true))return true;
    lastResponse=status||lastResponse;
    if(attempt===0){
      await dashConfirmTelemetryStopped();
      await dashDelay(60);
    }
  }
  throw new Error(lastResponse.trim()||'设备未确认新的图表数据');
}

async function dashStartGroup(key,clearHistory){
  if(!dashState.connected||dashState.requesting)return false;
  var previousGroup=dashState.activeGroup;
  var previousEnabled=dashState.enabled;
  var requestedPeriod=Number($('dashPeriod').value);
  var oldStreamStopped=false;
  dashState.pendingGroup=key;
  dashState.requesting=true;
  dashBuildDataBrowser();dashRenderStatus();
  try{
    if(!simMode){
      if(previousEnabled){
        await dashConfirmTelemetryStopped();
        oldStreamStopped=true;
        await dashDelay(40);
      }
      await dashActivateTelemetryProfile(key,requestedPeriod);
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
    dashState.enabled=previousEnabled&&!oldStreamStopped;
    if(!simMode&&oldStreamStopped&&previousEnabled&&previousGroup){
      try{
        await dashActivateTelemetryProfile(previousGroup,requestedPeriod);
        dashState.enabled=true;
      }catch(restoreError){
        dashState.enabled=false;
        error=new Error(error.message+'；原图表恢复失败：'+restoreError.message);
      }
    }
    dashToast('图表切换失败：'+error.message,true);
    return false;
  }finally{
    dashState.pendingGroup=null;dashState.requesting=false;
    dashBuildDataBrowser();
    dashRenderStatus();
  }
}

async function dashStopStream(resumeOnReturn){
  if(dashState.requesting)return;
  var stopped=!dashState.connected||simMode||!dashState.enabled;
  dashState.requesting=true;dashRenderStatus();
  try{
    if(dashState.connected&&!simMode&&dashState.enabled){
      await dashConfirmTelemetryStopped();
      stopped=true;
    }
  }catch(error){
    dashToast('停止遥测失败：'+error.message,true);
  }finally{
    dashState.enabled=!stopped&&dashState.enabled;
    if(stopped){
      dashState.recording=false;
      dashSimStop();
    }
    dashState.requesting=false;
    dashRenderStatus();dashDrawAll();
    if(resumeOnReturn&&dashState.visible&&dashState.connected&&
       dashState.activeGroup&&!dashState.enabled){
      dashStartGroup(dashState.activeGroup,false);
    }
  }
}

function dashToggleStream(){
  if(dashState.enabled)return dashStopStream(false);
  if(dashState.activeGroup)return dashStartGroup(dashState.activeGroup,false);
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
  values.mode=1;values.step=Math.floor(t/3)%8;
  values.comp_slot=7;values.comp_slot_valid=1;values.comp_count=12;
  values.L_tgt=dashState.simStopped?0:120;values.R_tgt=dashState.simStopped?0:115;
  values.L_act=dashState.simStopped?0:118+Math.sin(t*2)*3;
  values.R_act=dashState.simStopped?0:113+Math.cos(t*2)*3;
  values.yaw_tgt=45;values.yaw=45+Math.sin(t*.7)*2;values.head_err=45-values.yaw;
  values.head_corr=values.head_err*8;
  values.chassis_init=1;values.chassis_status=0;
  values.feedback_status=0;values.feedback_valid=1;values.feedback_age_ms=12;
  values.gray_pos=Math.sin(t*1.3)*350;values.lf_err=-values.gray_pos;
  values.lf_corr=values.gray_pos*.16;
  values.gray_strength=760+Math.sin(t)*45;values.gray_conf=880+Math.cos(t)*35;
  values.gray_valid=1;values.gray_state=1;values.lf_weak=Math.floor(t/5);
  values.lf_invalid=Math.floor(t/9);values.road_type=2;
  values.road_event_seq=Math.floor(t/6);values.road_event_type=2;
  values.road_paths=2;values.road_phase=0;values.road_ctrl_phase=0;
  values.head_turn_phase=0;values.head_turn_rate_mdps=Math.sin(t)*1800;
  values.head_turn_brake_mdeg=42000;values.head_turn_brake_ms=120;
  values.head_turn_margin_mdeg=3000;values.head_turn_settle_mdps=2500;
  values.head_turn_settle_rpm=35;
  values.acc_x_mg=Math.sin(t)*35;values.acc_y_mg=Math.cos(t*.8)*28;
  values.acc_z_mg=1000+Math.sin(t*2)*10;
  values.gyro_x_mdps=Math.sin(t*.9)*900;values.gyro_y_mdps=Math.cos(t)*700;
  values.gyro_z_mdps=Math.sin(t*.7)*12000;
  values.pitch=Math.sin(t*.4)*3;values.roll=Math.cos(t*.45)*2;
  values.imu_temp_cc=2860;values.imu_valid=1;values.imu_age_ms=8;
  values.imu_error_count=0;values.gray_sample_valid=1;values.gray_age_ms=6;
  values.gray_error_count=0;
  for(var channel=0;channel<8;channel++){
    values['gray'+channel]=900+channel*70+Math.sin(t*1.2+channel*.5)*120;
  }
  values.ir_offset_raw=Math.round(Math.sin(t*1.6)*420);
  values.ir_position_mpos=Math.round(values.ir_offset_raw*3000/600);
  values.ir_all_black=(Math.floor(t)%12===10)?1:0;
  values.ir_adc1=1250+Math.round((1+Math.sin(t*1.4))*900);
  values.ir_adc2=1350+Math.round((1+Math.sin(t*1.4+1.8))*850);
  values.ir_adc3=1200+Math.round((1+Math.sin(t*1.4+3.6))*920);
  values.ir_valid=1;values.ir_age_ms=2;values.ir_period_ms=10;
  values.ir_crc_errors=Math.floor(t/45);values.ir_dropped=0;
  values.fault_code=0;values.fault_count=0;
  values.tx_pending=18+Math.round(Math.abs(Math.sin(t))*12);values.tx_dropped=0;
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
  if(dashState.enabled&&!dashState.requesting)dashStopStream(true);
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

dashBuildDataBrowser();
$('btnDashTelemetry').onclick=dashToggleStream;
$('dashPeriod').onchange=function(){
  if(dashState.enabled&&dashState.activeGroup){
    dashStartGroup(dashState.activeGroup,false);
  }
};
$('dashWindow').onchange=function(){
  if(dashState.timelineEndMs){
    dashState.samples=DashboardCore.trimTimelineSamples(
      dashState.samples,Number(this.value)*1000,dashState.timelineEndMs);
  }
  dashDrawAll();
};
$('btnDashRecord').onclick=dashToggleRecord;
$('btnDashExport').onclick=dashExport;
$('btnDashClear').onclick=function(){dashState.samples=[];dashDrawAll();dashToast('曲线已清空',false)};
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
},250);

var dashboardPreviousSerialState=onSerialStateChange;
onSerialStateChange=function(connected){
  if(typeof dashboardPreviousSerialState==='function')dashboardPreviousSerialState(connected);
  Dashboard_OnConnection(connected);
};
Dashboard_OnConnection(!!writer||simMode);
