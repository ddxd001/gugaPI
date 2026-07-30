(function(root,factory){
  if(typeof module==='object'&&module.exports)module.exports=factory();
  else root.SerialLogCore=factory();
}(typeof self!=='undefined'?self:this,function(){
'use strict';

function toIso(value){
  var date=value instanceof Date?value:new Date(value===undefined?Date.now():value);
  return date.toISOString();
}

function fileTimestamp(value){
  return toIso(value).replace(/[:.]/g,'-');
}

function suggestedFilename(value){
  return'gugapi-serial-'+fileTimestamp(value)+'.jsonl';
}

function encode(object){
  return JSON.stringify(object)+'\n';
}

function encodeHeader(value,metadata){
  return encode({
    type:'session',format:'gugapi-serial-log',version:1,
    started_at:toIso(value),metadata:metadata||{}
  });
}

function encodeEntry(sequence,direction,data,value){
  if(direction!=='tx'&&direction!=='rx')throw new Error('invalid direction');
  return encode({
    type:'serial',sequence:sequence,host_time:toIso(value),
    direction:direction,data:String(data)
  });
}

function encodeFooter(value,reason,entries,payloadBytes){
  return encode({
    type:'end',ended_at:toIso(value),reason:reason||'stopped',
    entries:entries||0,payload_bytes:payloadBytes||0
  });
}

function byteLength(value){
  var text=String(value);
  if(typeof TextEncoder!=='undefined')return new TextEncoder().encode(text).length;
  if(typeof Buffer!=='undefined')return Buffer.byteLength(text,'utf8');
  return unescape(encodeURIComponent(text)).length;
}

return{
  toIso:toIso,fileTimestamp:fileTimestamp,
  suggestedFilename:suggestedFilename,
  encodeHeader:encodeHeader,encodeEntry:encodeEntry,encodeFooter:encodeFooter,
  byteLength:byteLength
};
}));
