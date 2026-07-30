(function(root,factory){
  'use strict';
  var api=factory();
  if(typeof module==='object'&&module.exports)module.exports=api;
  else root.LineSensorCore=api;
})(typeof self!=='undefined'?self:this,function(){
  'use strict';

  function parseKv(text){
    var values={};
    String(text||'').replace(/(?:^|\s)([a-zA-Z0-9_]+)=([^\s]+)/g,
      function(_,key,value){values[key]=value;return _});
    return values;
  }

  function parseList(value,count){
    var result=String(value||'').split(',').filter(function(item){return item!==''})
      .map(function(item){return Number(item)});
    if(count!==undefined&&result.length!==count)return[];
    return result.every(Number.isFinite)?result:[];
  }

  function pollCommands(source,includeSlow){
    var commands=['linesensor status'];
    if(source==='adc8'){
      commands.push('gray live','gray calib status');
      if(includeSlow)commands.push('gray calib preview','param status');
    }else{
      commands.push('irsensor raw');
      if(includeSlow)commands.push('irsensor stats','irsensor calib status','param status');
    }
    return commands;
  }

  function calibrationCommand(source,action){
    if(source==='adc8'){
      var gray={begin:'gray calib begin',white:'gray calib white 64',
        black:'gray calib black 64',status:'gray calib status',
        preview:'gray calib preview',commit:'gray calib commit',
        cancel:'gray calib cancel'};
      return gray[action]||'';
    }
    var infrared={begin:'irsensor calib begin',status:'irsensor calib status',
      commit:'irsensor calib commit',cancel:'irsensor calib cancel'};
    if(['white','black','center','left','right'].indexOf(action)>=0){
      return'irsensor calib capture '+action;
    }
    return infrared[action]||'';
  }

  return{parseKv:parseKv,parseList:parseList,pollCommands:pollCommands,
    calibrationCommand:calibrationCommand};
});
