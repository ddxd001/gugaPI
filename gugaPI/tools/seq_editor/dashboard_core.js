'use strict';

(function(root,factory){
  var api=factory();
  if(typeof module==='object'&&module.exports)module.exports=api;
  root.DashboardCore=api;
})(typeof globalThis!=='undefined'?globalThis:this,function(){
  var DEFAULT_FIELDS=[
    't','mode','step','L_tgt','L_act','R_tgt','R_act','yaw_tgt','yaw',
    'head_err','head_corr','gray_pos','gray_strength','gray_conf','gray_valid',
    'gray_state','lf_err','lf_corr','lf_weak','lf_invalid','road_type',
    'road_event_seq','road_event_type','road_paths','road_phase','road_ctrl_phase',
    'head_turn_phase','head_turn_rate_mdps','head_turn_brake_mdeg',
    'head_turn_brake_ms','head_turn_margin_mdeg','head_turn_settle_mdps',
    'head_turn_settle_rpm','fault_code','fault_count','comp_slot',
    'comp_slot_valid','comp_count','chassis_init','chassis_status',
    'feedback_status','feedback_valid','feedback_age_ms','tx_pending','tx_dropped',
    'imu_valid','imu_age_ms','imu_error_count','acc_x_mg','acc_y_mg','acc_z_mg',
    'gyro_x_mdps','gyro_y_mdps','gyro_z_mdps','pitch','roll','imu_temp_cc',
    'gray_sample_valid','gray_age_ms','gray_error_count','gray0','gray1','gray2',
    'gray3','gray4','gray5','gray6','gray7'
  ];

  function TelemetryParser(){
    this.fields=[];
  }

  TelemetryParser.prototype.reset=function(){
    this.fields=[];
  };

  TelemetryParser.prototype.parseLine=function(line,hostTime){
    var text=String(line).trim();
    if(text.startsWith('#')){
      var fields=text.slice(1).split(',').map(function(field){return field.trim()});
      if(fields.length>=2&&fields[0]==='t'){
        this.fields=fields;
        return{type:'header',fields:fields.slice(),raw:text};
      }
      return{type:'other',raw:text};
    }
    if(this.fields.length===0||text.indexOf(',')<0){
      return{type:'other',raw:text};
    }
    var cells=text.split(',');
    if(cells.length!==this.fields.length){
      return{type:'other',raw:text};
    }
    var values={};
    for(var index=0;index<cells.length;index++){
      var cell=cells[index].trim();
      if(cell===''||!Number.isFinite(Number(cell))){
        return{type:'other',raw:text};
      }
      values[this.fields[index]]=Number(cell);
    }
    return{
      type:'sample',
      fields:this.fields.slice(),
      values:values,
      cells:cells.map(function(cell){return cell.trim()}),
      hostTime:hostTime||new Date().toISOString(),
      raw:text
    };
  };

  function SerialRouter(callbacks){
    this.callbacks=callbacks||{};
    this.parser=new TelemetryParser();
    this.pending='';
  }

  SerialRouter.prototype.reset=function(){
    this.parser.reset();
    this.pending='';
  };

  SerialRouter.prototype._emitText=function(text){
    if(text&&typeof this.callbacks.onText==='function')this.callbacks.onText(text);
  };

  SerialRouter.prototype._routeLine=function(line,newline){
    /*
     * The shell prompt has no trailing newline. On real UART hardware the
     * next asynchronous telemetry frame can therefore arrive as
     * "> #t,..." or "> 123,...". Keep the prompt on the shell path while
     * parsing the remainder as a telemetry frame.
     */
    var payload=line;
    var prompt='';
    if(payload.indexOf('> ')===0){
      prompt='> ';
      payload=payload.slice(2);
    }
    var event=this.parser.parseLine(payload,new Date().toISOString());
    if(event.type==='header'||event.type==='sample'){
      this._emitText(prompt);
      if(typeof this.callbacks.onTelemetry==='function'){
        this.callbacks.onTelemetry(event,payload+newline);
      }
    }else{
      this._emitText(line+newline);
    }
  };

  SerialRouter.prototype.push=function(chunk){
    this.pending+=String(chunk);
    while(true){
      var crIndex=this.pending.indexOf('\r');
      var lfIndex=this.pending.indexOf('\n');
      var newlineIndex=crIndex<0?lfIndex:
        (lfIndex<0?crIndex:Math.min(crIndex,lfIndex));
      if(newlineIndex<0)break;
      if(this.pending[newlineIndex]==='\r'&&
         newlineIndex===this.pending.length-1)break;
      var consumed=(this.pending[newlineIndex]==='\r'&&
        this.pending[newlineIndex+1]==='\n')?2:1;
      var line=this.pending.slice(0,newlineIndex);
      this.pending=this.pending.slice(newlineIndex+consumed);
      this._routeLine(line,'\n');
    }
    if(this.pending==='> '||this.pending==='>'){
      this._emitText(this.pending);
      this.pending='';
    }
  };

  SerialRouter.prototype.flush=function(){
    if(this.pending){
      this._routeLine(this.pending,'');
      this.pending='';
    }
  };

  function csvEscape(value){
    var text=String(value===undefined?'':value);
    return /[",\r\n]/.test(text)?'"'+text.replace(/"/g,'""')+'"':text;
  }

  function findNearestSample(samples,targetTime){
    if(!samples||samples.length===0||!Number.isFinite(targetTime))return null;
    var low=0,high=samples.length-1;
    while(low<high){
      var middle=Math.floor((low+high)/2);
      if(Number(samples[middle].hostMs)<targetTime)low=middle+1;
      else high=middle;
    }
    if(low===0)return samples[0];
    var before=samples[low-1];
    var after=samples[low];
    return Math.abs(Number(before.hostMs)-targetTime)<=
      Math.abs(Number(after.hostMs)-targetTime)?before:after;
  }

  function exportCsv(fields,samples){
    var rows=[['host_rx_iso'].concat(fields).map(csvEscape).join(',')];
    samples.forEach(function(sample){
      rows.push([sample.hostTime].concat(fields.map(function(field){
        return Object.prototype.hasOwnProperty.call(sample.values,field)
          ?sample.values[field]:'';
      })).map(csvEscape).join(','));
    });
    return rows.join('\r\n')+'\r\n';
  }

  return{
    DEFAULT_FIELDS:DEFAULT_FIELDS,
    TelemetryParser:TelemetryParser,
    SerialRouter:SerialRouter,
    findNearestSample:findNearestSample,
    exportCsv:exportCsv,
    csvEscape:csvEscape
  };
});
