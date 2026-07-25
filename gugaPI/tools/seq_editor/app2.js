var vb={x:0,y:0,w:800,h:600};
var panning=false,px=0,py=0;
function updVB(){$('flow').setAttribute('viewBox',vb.x+' '+vb.y+' '+vb.w+' '+vb.h)}
var cv=$('canvas');
cv.addEventListener('mousedown',function(e){if(e.button!==0)return;panning=true;px=e.clientX;py=e.clientY});
window.addEventListener('mousemove',function(e){if(!panning)return;var dx=e.clientX-px,dy=e.clientY-py;px=e.clientX;py=e.clientY;var r=cv.getBoundingClientRect();vb.x-=dx*(vb.w/r.width);vb.y-=dy*(vb.h/r.height);updVB()});
window.addEventListener('mouseup',function(){panning=false});
cv.addEventListener('wheel',function(e){e.preventDefault();var r=cv.getBoundingClientRect();var mx=(e.clientX-r.left)/r.width,my=(e.clientY-r.top)/r.height;var f=e.deltaY>0?1.15:0.87;vb.x+=(vb.w-vb.w*f)*mx;vb.y+=(vb.h-vb.h*f)*my;vb.w*=f;vb.h*=f;updVB()},{passive:false});
var NW=120,NH=50,DY=100,DX=170;
function render(){
  var svg=$('flow');
  if(!instrs.length){svg.innerHTML='<text x="50" y="50" fill="#6c7086" font-size="16">No instructions</text>';vb={x:0,y:0,w:800,h:600};updVB();return}
  var pos={},visited={},edges=[];
  var minX=0,maxX=0,maxY=0;
  function place(idx,x,y){
    if(visited[idx])return;
    visited[idx]=true;pos[idx]={x:x,y:y};
    if(x<minX)minX=x;if(x>maxX)maxX=x;if(y>maxY)maxY=y;
    var ins=instrs[idx];var ns=ins.ons===255?idx+1:ins.ons;var nt=ins.ont;
    if(ns<instrs.length){edges.push({from:idx,to:ns,type:'ok'});place(ns,x-DX,y+DY)}
    else if(ins.op===7){edges.push({from:idx,to:'done',type:'ok'})}
    if(nt===255){edges.push({from:idx,to:'abort',type:'fail'})}
    else if(nt<instrs.length){edges.push({from:idx,to:nt,type:'fail'});if(!visited[nt])place(nt,x+DX,y+DY)}
  }
  place(0,0,0);
  var hasDone=edges.some(function(e){return e.to==='done'});
  var hasAbort=edges.some(function(e){return e.to==='abort'});
  if(hasDone)pos['done']={x:-DX,y:maxY+DY};
  if(hasAbort)pos['abort']={x:DX,y:maxY+DY};
  var uc=0;
  for(var i=0;i<instrs.length;i++)if(!visited[i]){uc++;pos[i]={x:0,y:maxY+DY*2+uc*DY}}
  var W=Math.max(800,maxX-minX+300),H=Math.max(600,maxY+DY*3+100);
  vb={x:minX-100,y:-50,w:W,h:H};updVB();
  var h='<defs><marker id="mao" markerWidth="10" markerHeight="10" refX="8" refY="5" orient="auto"><path d="M0,0L10,5L0,10Z" fill="#a6e3a1"/></marker><marker id="mat" markerWidth="10" markerHeight="10" refX="8" refY="5" orient="auto"><path d="M0,0L10,5L0,10Z" fill="#f38ba8"/></marker></defs>';
  edges.forEach(function(e){var from=pos[e.from],to=pos[e.to];if(!from||!to)return;var fx=from.x+NW/2,fy=from.y+NH;var tx=to.x+NW/2,ty=to.y;var col=e.type==='ok'?'#a6e3a1':'#f38ba8';var dash=e.type==='fail'?'stroke-dasharray="5,3"':'';var lab=e.type==='ok'?'ok':'fail';var ly=(fy+ty)/2;if(Math.abs(fx-tx)>10){h+='<path d="M'+fx+','+fy+' Q'+tx+','+ly+' '+tx+','+ty+'" stroke="'+col+'" stroke-width="2" fill="none" '+dash+' marker-end="url(#'+(e.type==='ok'?'mao':'mat')+')" /><text class="arrow-label" x="'+((fx+tx)/2+10)+'" y="'+ly+'" fill="'+col+'">'+lab+'</text>'}else{h+='<path d="M'+fx+','+fy+' L'+tx+','+ty+'" stroke="'+col+'" stroke-width="2" fill="none" '+dash+' marker-end="url(#'+(e.type==='ok'?'mao':'mat')+')" /><text class="arrow-label" x="'+(fx+15)+'" y="'+(ly-3)+'" fill="'+col+'">'+lab+'</text>'}});
  if(hasDone)h+='<g><rect x="'+pos['done'].x+'" y="'+pos['done'].y+'" width="'+NW+'" height="'+NH+'" rx="6" fill="#a6e3a1" opacity="0.3" stroke="#a6e3a1" stroke-width="2"/><text x="'+(pos['done'].x+NW/2)+'" y="'+(pos['done'].y+30)+'" text-anchor="middle" fill="#a6e3a1" font-size="13" font-weight="600">DONE</text></g>';
  if(hasAbort)h+='<g><rect x="'+pos['abort'].x+'" y="'+pos['abort'].y+'" width="'+NW+'" height="'+NH+'" rx="6" fill="#f38ba8" opacity="0.3" stroke="#f38ba8" stroke-width="2"/><text x="'+(pos['abort'].x+NW/2)+'" y="'+(pos['abort'].y+30)+'" text-anchor="middle" fill="#f38ba8" font-size="13" font-weight="600">ABORT</text></g>';
  instrs.forEach(function(ins,i){var p=pos[i];if(!p)return;var c=OPC[ins.op]||'#585b70';var l=OPL[ins.op]||'?';var sel=i===selIdx;var d='';if(ins.op===1)d=ins.p1+'RPM '+ins.p2+'ms';else if(ins.op===2)d=ins.p1+'deg '+ins.p2+'ms';else if(ins.op===3)d=ins.p1+'RPM';else if(ins.op===4)d=ins.p2+'ms';else if(ins.op===6)d=COND[ins.until];var unreach=!visited[i];h+='<g class="instr-node" style="cursor:pointer" onclick="sel('+i+')"><rect x="'+p.x+'" y="'+p.y+'" width="'+NW+'" height="'+NH+'" rx="6" fill="'+c+'" opacity="'+(sel?1:0.85)+'" stroke="'+(sel?'#fff':unreach?'#6c7086':'none')+'" stroke-width="'+(sel?2:unreach?1:0)+'"'+(unreach?' stroke-dasharray="3,3"':'')+'/><text x="'+(p.x+NW/2)+'" y="'+(p.y+20)+'" text-anchor="middle" fill="#1e1e2e" font-size="13" font-weight="600">'+i+': '+l+'</text><text x="'+(p.x+NW/2)+'" y="'+(p.y+37)+'" text-anchor="middle" fill="#1e1e2e" font-size="10">'+d+'</text></g>'});
  svg.innerHTML=h}
function sel(i){selIdx=i;render();edit()}
