var vb={x:0,y:0,w:800,h:600};
var panning=false,px=0,py=0;
function updVB(){$('flow').setAttribute('viewBox',vb.x+' '+vb.y+' '+vb.w+' '+vb.h)}
var cv=$('canvas');
cv.addEventListener('mousedown',function(e){if(e.button!==0)return;panning=true;px=e.clientX;py=e.clientY});
window.addEventListener('mousemove',function(e){if(!panning)return;var dx=e.clientX-px,dy=e.clientY-py;px=e.clientX;py=e.clientY;var r=cv.getBoundingClientRect();vb.x-=dx*(vb.w/r.width);vb.y-=dy*(vb.h/r.height);updVB()});
window.addEventListener('mouseup',function(){panning=false});
cv.addEventListener('wheel',function(e){e.preventDefault();var r=cv.getBoundingClientRect();var mx=(e.clientX-r.left)/r.width,my=(e.clientY-r.top)/r.height;var f=e.deltaY>0?1.15:0.87;vb.x+=(vb.w-vb.w*f)*mx;vb.y+=(vb.h-vb.h*f)*my;vb.w*=f;vb.h*=f;updVB()},{passive:false});
var NW=120,NH=50,DY=90,GAP=40;
var BL={0:['timeout','no timeout'],1:['reached','not reached'],2:['line found','no line'],3:['line lost','line exists'],4:['button','no button'],5:['always','never']};

function buildGraph(){
var nodes=[],edges=[],ns={};
function add(id){if(!ns[id]){ns[id]=1;nodes.push(id)}}
add(0);
for(var i=0;i<instrs.length;i++){
var ins=instrs[i],isB=ins.op===6;
var s=ins.ons===255?i+1:ins.ons,t=ins.ont;
var sl,fl;
if(isB){var b=BL[ins.until]||['true','false'];sl=b[0];fl=b[1]}else{sl='ok';fl='fail'}
if(s<instrs.length){add(s);edges.push({f:i,t:s,tp:'succ',lb:sl,br:isB})}
else if(ins.op===7){add('done');edges.push({f:i,t:'done',tp:'succ',lb:sl,br:isB})}
if(t===255){add('abort');edges.push({f:i,t:'abort',tp:'fail',lb:fl,br:isB})}
else if(t<instrs.length){add(t);edges.push({f:i,t:t,tp:'fail',lb:fl,br:isB})}
}
var reach={0:1},q=[0];
while(q.length){var n=q.shift();edges.forEach(function(e){if(e.f===n&&!reach[e.t]){reach[e.t]=1;q.push(e.t)}})}
return{nodes:nodes,edges:edges,reach:reach}
}
function sugiyama(){
var g=buildGraph(),nodes=g.nodes,edges=g.edges,reach=g.reach;
var rn=nodes.filter(function(n){return reach[n]});
var layer={};
var ch=true,it=0;
while(ch&&it<100){ch=false;it++;edges.forEach(function(e){if(reach[e.f]&&reach[e.t]){var nl=(layer[e.f]||0)+1;if((layer[e.t]||0)<nl){layer[e.t]=nl;ch=true}}})}
var maxL=0;rn.forEach(function(n){if((layer[n]||0)>maxL)maxL=layer[n]||0});
var layers=[];for(var l=0;l<=maxL;l++)layers[l]=[];
rn.forEach(function(n){layers[layer[n]||0].push(n)});
layers=layers.filter(function(l){return l.length>0});
var lm={};layers.forEach(function(ln,li){ln.forEach(function(n){lm[n]=li})});
var did=10000,dm={};
edges.forEach(function(e){if(!reach[e.f]||!reach[e.t])return;var lf=lm[e.f],lt=lm[e.t];if(lt>lf+1){e.path=[e.f];for(var d=lf+1;d<lt;d++){var id=++did;dm[id]=1;layers[d].push(id);lm[id]=d;e.path.push(id)}e.path.push(e.t)}});
function bary(node,ali){
var sum=0,cnt=0;
layers[ali].forEach(function(o,i){edges.forEach(function(e){if(!e.path){if(e.f===node&&e.t===o){sum+=i;cnt++}if(e.f===o&&e.t===node){sum+=i;cnt++}}})});
edges.forEach(function(e){if(e.path){e.path.forEach(function(pid,idx){if(pid===node&&idx>0){var pv=e.path[idx-1];if(lm[pv]===ali){var pi=layers[ali].indexOf(pv);if(pi>=0){sum+=pi;cnt++}}}if(pid===node&&idx<e.path.length-1){var nx=e.path[idx+1];if(lm[nx]===ali){var ni=layers[ali].indexOf(nx);if(ni>=0){sum+=ni;cnt++}}}})}});
return cnt>0?sum/cnt:-1}
for(var pass=0;pass<24;pass++){
if(pass%2===0){for(var li=1;li<layers.length;li++){layers[li].sort(function(a,b){return bary(a,li-1)-bary(b,li-1)})}}
else{for(var li2=layers.length-2;li2>=0;li2--){layers[li2].sort(function(a,b){return bary(a,li2+1)-bary(b,li2+1)})}}
}
var pos={},maxW=0;
layers.forEach(function(ln){var w=ln.length*(NW+GAP);if(w>maxW)maxW=w});
layers.forEach(function(ln,li){var w=ln.length*(NW+GAP);var sx=(maxW-w)/2;ln.forEach(function(n,i){pos[n]={x:sx+i*(NW+GAP),y:li*DY}})});
var uc=0,ml=layers.length;
nodes.forEach(function(n){if(!reach[n]&&n!==0){pos[n]={x:maxW/2-NW/2,y:(ml+uc)*DY};uc++}});
var hd=edges.some(function(e){return e.t==='done'}),ha=edges.some(function(e){return e.t==='abort'});
var ty=ml*DY;
if(hd){var dc=layers.filter(function(l){return l.indexOf('done')>=0}).length;if(dc===0){pos['done']={x:0,y:ty};ml++}}
if(ha){var ac=layers.filter(function(l){return l.indexOf('abort')>=0}).length;if(ac===0){pos['abort']={x:NW+GAP,y:ty};ml++}}
return{pos:pos,edges:edges,reach:reach,hasDone:hd,hasAbort:ha}
}

function render(){
var svg=$('flow');
if(!instrs.length){svg.innerHTML='<text x="50" y="50" fill="#6c7086" font-size="16">No instructions</text>';vb={x:0,y:0,w:800,h:600};updVB();return}
var L=sugiyama(),pos=L.pos,edges=L.edges,reach=L.reach;
var mnX=1e9,mxX=-1e9,mnY=1e9,mxY=-1e9;
Object.keys(pos).forEach(function(id){var p=pos[id];if(p.x<mnX)mnX=p.x;if(p.x>mxX)mxX=p.x;if(p.y<mnY)mnY=p.y;if(p.y>mxY)mxY=p.y});
if(mnX>1e8){mnX=0;mxX=800;mnY=0;mxY=600}
var pad=80;
vb={x:mnX-pad,y:mnY-pad,w:(mxX-mnX)+NW+pad*2,h:(mxY-mnY)+NH+pad*2};updVB();
var h='<defs>';
h+='<marker id="mao" markerWidth="10" markerHeight="10" refX="8" refY="5" orient="auto"><path d="M0,0L10,5L0,10Z" fill="#a6e3a1"/></marker>';
h+='<marker id="mat" markerWidth="10" markerHeight="10" refX="8" refY="5" orient="auto"><path d="M0,0L10,5L0,10Z" fill="#f38ba8"/></marker>';
h+='<marker id="mab" markerWidth="10" markerHeight="10" refX="8" refY="5" orient="auto"><path d="M0,0L10,5L0,10Z" fill="#cdd6f4"/></marker>';
h+='</defs>';
edges.forEach(function(e){
if(!reach[e.f])return;
var fp=pos[e.f],tp=pos[e.t];if(!fp||!tp)return;
var fx,fy,tx=tp.x+NW/2,ty=tp.y;
if(e.br){if(e.tp==='succ'){fx=fp.x+NW/2-NW/4;fy=fp.y+NH/2+NH/3}else{fx=fp.x+NW/2+NW/4;fy=fp.y+NH/2+NH/3}}
else{fx=fp.x+NW/2;fy=fp.y+NH}
var col=e.br?'#cdd6f4':(e.tp==='succ'?'#a6e3a1':'#f38ba8');
var dash=e.tp==='fail'&&!e.br?'stroke-dasharray="5,3"':'';
var mk='url(#'+(e.br?'mab':(e.tp==='succ'?'mao':'mat'))+')';
var ly=(fy+ty)/2;
if(e.path&&e.path.length>2){
var d='M'+fx+','+fy;
for(var pi=1;pi<e.path.length;pi++){var pp=pos[e.path[pi]];if(pp){d+=' L'+(pp.x+NW/2)+','+pp.y}}
h+='<path d="'+d+'" stroke="'+col+'" stroke-width="2" fill="none" '+dash+' marker-end="'+mk+'" />';
var lp=pos[e.path[1]];h+='<text class="arrow-label" x="'+(fx+12)+'" y="'+(fy+DY/2-5)+'" fill="'+col+'" style="font-size:10px;font-weight:600">'+e.lb+'</text>';
}else if(Math.abs(fx-tx)>NW*0.6){
var midY=ty-DY/2;
h+='<path d="M'+fx+','+fy+' L'+fx+','+midY+' L'+tx+','+midY+' L'+tx+','+ty+'" stroke="'+col+'" stroke-width="2" fill="none" '+dash+' marker-end="'+mk+'" />';
h+='<text class="arrow-label" x="'+((fx+tx)/2+8)+'" y="'+(midY-3)+'" fill="'+col+'" style="font-size:10px;font-weight:600">'+e.lb+'</text>';
}else{
h+='<path d="M'+fx+','+fy+' L'+tx+','+ty+'" stroke="'+col+'" stroke-width="2" fill="none" '+dash+' marker-end="'+mk+'" />';
h+='<text class="arrow-label" x="'+(fx+12)+'" y="'+(ly-3)+'" fill="'+col+'" style="font-size:10px;font-weight:600">'+e.lb+'</text>';
}});
if(L.hasDone){var dp=pos['done'];if(dp)h+='<g><rect x="'+dp.x+'" y="'+dp.y+'" width="'+NW+'" height="'+NH+'" rx="25" fill="#a6e3a1" opacity="0.2" stroke="#a6e3a1" stroke-width="2"/><text x="'+(dp.x+NW/2)+'" y="'+(dp.y+30)+'" text-anchor="middle" fill="#a6e3a1" font-size="13" font-weight="600">DONE</text></g>'}
if(L.hasAbort){var ap=pos['abort'];if(ap)h+='<g><rect x="'+ap.x+'" y="'+ap.y+'" width="'+NW+'" height="'+NH+'" rx="25" fill="#f38ba8" opacity="0.2" stroke="#f38ba8" stroke-width="2"/><text x="'+(ap.x+NW/2)+'" y="'+(ap.y+30)+'" text-anchor="middle" fill="#f38ba8" font-size="13" font-weight="600">ABORT</text></g>'}
instrs.forEach(function(ins,i){
var p=pos[i];if(!p)return;
var c=OPC[ins.op]||'#585b70',l=OPL[ins.op]||'?',sel=i===selIdx,unreach=!reach[i]&&i!==0;
var d='';
if(ins.op===1)d=ins.p1+'RPM '+ins.p2+'ms';else if(ins.op===2)d=ins.p1+'deg '+ins.p2+'ms';
else if(ins.op===3)d=ins.p1+'RPM';else if(ins.op===4)d=ins.p2+'ms';else if(ins.op===6)d=COND[ins.until];
var sw=sel?2:(unreach?1:0),st=sel?'#fff':(unreach?'#6c7086':'none'),da=unreach?' stroke-dasharray="3,3"':'';
if(ins.op===6){
var cx=p.x+NW/2,cy=p.y+NH/2,dw=NW+20,dh=NH+20;
var pts=cx+','+(cy-dh/2)+' '+(cx+dw/2)+','+cy+' '+cx+','+(cy+dh/2)+' '+(cx-dw/2)+','+cy;
h+='<g class="instr-node" style="cursor:pointer" onclick="sel('+i+')"><polygon points="'+pts+'" fill="'+c+'" opacity="'+(sel?0.9:0.75)+'" stroke="'+st+'" stroke-width="'+sw+'"'+da+'/><text x="'+cx+'" y="'+(cy-2)+'" text-anchor="middle" fill="#1e1e2e" font-size="12" font-weight="600">'+i+': '+l+'</text><text x="'+cx+'" y="'+(cy+14)+'" text-anchor="middle" fill="#1e1e2e" font-size="9">'+d+'</text></g>';
}else if(ins.op===5||ins.op===7){
h+='<g class="instr-node" style="cursor:pointer" onclick="sel('+i+')"><rect x="'+p.x+'" y="'+p.y+'" width="'+NW+'" height="'+NH+'" rx="25" fill="'+c+'" opacity="'+(sel?1:0.85)+'" stroke="'+st+'" stroke-width="'+sw+'"'+da+'/><text x="'+(p.x+NW/2)+'" y="'+(p.y+20)+'" text-anchor="middle" fill="#1e1e2e" font-size="13" font-weight="600">'+i+': '+l+'</text><text x="'+(p.x+NW/2)+'" y="'+(p.y+37)+'" text-anchor="middle" fill="#1e1e2e" font-size="10">'+d+'</text></g>';
}else{
h+='<g class="instr-node" style="cursor:pointer" onclick="sel('+i+')"><rect x="'+p.x+'" y="'+p.y+'" width="'+NW+'" height="'+NH+'" rx="6" fill="'+c+'" opacity="'+(sel?1:0.85)+'" stroke="'+st+'" stroke-width="'+sw+'"'+da+'/><text x="'+(p.x+NW/2)+'" y="'+(p.y+20)+'" text-anchor="middle" fill="#1e1e2e" font-size="13" font-weight="600">'+i+': '+l+'</text><text x="'+(p.x+NW/2)+'" y="'+(p.y+37)+'" text-anchor="middle" fill="#1e1e2e" font-size="10">'+d+'</text></g>';
}});
svg.innerHTML=h;
}
function sel(i){selIdx=i;render();edit()}
