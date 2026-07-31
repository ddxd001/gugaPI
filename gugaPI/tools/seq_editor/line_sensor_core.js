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

  function pollCommands(includeSlow){
    var commands=['linesensor status','gray live','gray calib status'];
    if(includeSlow)commands.push('gray calib preview','param status');
    return commands;
  }

  function calibrationCommand(action){
    var gray={begin:'gray calib begin',white:'gray calib white 64',
      black:'gray calib black 64',status:'gray calib status',
      preview:'gray calib preview',commit:'gray calib commit',
      cancel:'gray calib cancel'};
    return gray[action]||'';
  }

  return{parseKv:parseKv,parseList:parseList,pollCommands:pollCommands,
    calibrationCommand:calibrationCommand};
});
