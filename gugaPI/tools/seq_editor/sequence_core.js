(function(root,factory){
  var api=factory();
  if(typeof module==='object'&&module.exports)module.exports=api;
  root.SequenceCore=api;
}(typeof globalThis!=='undefined'?globalThis:this,function(){
'use strict';
var FORMAT='gugapi-sequence-project',VERSION=1,ABORT=255;
var CONDS=['timeout','heading_reached','line_detected','line_lost','button','immediate','distance_reached'];
var COND_LABELS={timeout:'定时到达',heading_reached:'航向到达',line_detected:'检测到线路',line_lost:'线路丢失',button:'按钮按下',immediate:'立即',distance_reached:'距离到达'};
var ACTIONS={
 drive:{op:1,name:'直行',group:'运动',color:'#4f8ee8',help:'按有符号转速保持当前航向，满足条件后完成。',defaults:{rpm:80,timeoutMs:2000,condition:'timeout'}},
 drive_mm:{op:8,name:'定距行驶',group:'运动',color:'#36a6c9',help:'按编码器距离行驶；正数前进，负数后退。',defaults:{distanceMm:300,maxRpm:80}},
 turn:{op:2,name:'转向',group:'运动',color:'#e69855',help:'相对转向；正角度左转，负角度右转。',defaults:{angleDeg:90,timeoutMs:5000}},
 follow:{op:3,name:'循迹',group:'运动',color:'#5fbf77',help:'按灰度传感器循迹，支持丢线、按钮或定时结束。',defaults:{rpm:80,timeoutMs:10000,condition:'line_lost'}},
 wait:{op:4,name:'等待',group:'等待与判断',color:'#778195',help:'停止运动并等待定时、线路或按钮条件。',defaults:{timeoutMs:500,condition:'timeout'}},
 branch:{op:6,name:'条件分支',group:'流程控制',color:'#a66ce0',help:'立即采样条件，从“成立”或“不成立”端口继续。',defaults:{condition:'line_detected'}},
 stop:{op:5,name:'停车',group:'流程控制',color:'#df647c',help:'停止底盘、航向和循迹控制，然后继续。',defaults:{}},
 end:{op:7,name:'结束',group:'流程控制',color:'#697081',help:'成功结束整个序列；这是实际写入固件的动作。',defaults:{}},
 led_on:{op:9,name:'LED 点亮',group:'声光输出',color:'#e3c96f',help:'点亮 LED2、LED3 或两者，可设置自动关闭。',defaults:{target:0,durationMs:500}},
 led_off:{op:10,name:'LED 熄灭',group:'声光输出',color:'#89909e',help:'立即熄灭指定 LED。',defaults:{target:0}},
 led_toggle:{op:11,name:'LED 翻转',group:'声光输出',color:'#d998c5',help:'翻转指定 LED，可设置自动关闭。',defaults:{target:0,durationMs:500}},
 buzzer_on:{op:12,name:'蜂鸣器开启',group:'声光输出',color:'#e06c75',help:'开启蜂鸣器，可设置自动关闭。',defaults:{durationMs:200}},
 buzzer_off:{op:13,name:'蜂鸣器关闭',group:'声光输出',color:'#89909e',help:'立即关闭蜂鸣器。',defaults:{}},
 buzzer_toggle:{op:14,name:'蜂鸣器翻转',group:'声光输出',color:'#db8191',help:'翻转蜂鸣器，可设置自动关闭。',defaults:{durationMs:200}}
};
var OP_TYPES={};Object.keys(ACTIONS).forEach(function(k){OP_TYPES[ACTIONS[k].op]=k});
function clone(v){return JSON.parse(JSON.stringify(v))}
function now(){return new Date().toISOString()}
function uid(prefix){return(prefix||'n')+'-'+Date.now().toString(36)+'-'+Math.random().toString(36).slice(2,8)}
function actionNode(type,x,y,order,id){var a=ACTIONS[type];if(!a)throw new Error('未知动作类型：'+type);return{id:id||uid('n'),type:type,x:x||0,y:y||0,createdOrder:order||1,params:clone(a.defaults)}}
function newProject(name,slot){var t=now();return{format:FORMAT,version:VERSION,name:name||'未命名流程',slot:slot==null?7:slot,nodes:[{id:'start',type:'system_start',x:70,y:220,createdOrder:0,params:{}},{id:'abort',type:'system_abort',x:760,y:420,createdOrder:999999,params:{}}],edges:[],createdAt:t,updatedAt:t}}
function ports(type){if(type==='system_start')return['success'];if(type==='system_abort'||type==='end')return[];return type==='branch'?['success','failure']:['success','failure']}
function conditionAllowed(type){if(type==='drive')return['timeout','line_detected','line_lost','button'];if(type==='follow')return['timeout','line_lost','button','line_detected'];if(type==='wait')return['timeout','line_detected','line_lost','button'];if(type==='branch')return['line_detected','line_lost','button','immediate','timeout'];return[]}
function normalizeProject(input){
  if(!input||input.format!==FORMAT||input.version!==VERSION||!Array.isArray(input.nodes)||!Array.isArray(input.edges))throw new Error('不是受支持的 gugaPI 序列工程 v1');
  var p=clone(input),ids={};
  p.nodes.forEach(function(n,i){if(!n.id||ids[n.id])throw new Error('节点 ID 缺失或重复');ids[n.id]=1;if(n.type!=='system_start'&&n.type!=='system_abort'&&!ACTIONS[n.type])throw new Error('未知节点类型：'+n.type);n.x=Number(n.x)||0;n.y=Number(n.y)||0;n.createdOrder=Number(n.createdOrder)||i;n.params=n.params||{}});
  if(!ids.start||!ids.abort)throw new Error('工程缺少开始或中止系统节点');
  p.edges.forEach(function(e){if(!e.id)e.id=uid('e');if(!ids[e.source]||!ids[e.target])throw new Error('连线引用了不存在的节点');if(ports(p.nodes.find(function(n){return n.id===e.source}).type).indexOf(e.port)<0)throw new Error('连线端口无效')});
  p.name=String(p.name||'未命名流程');p.slot=Math.max(0,Math.min(7,Number(p.slot)||0));p.updatedAt=p.updatedAt||now();p.createdAt=p.createdAt||p.updatedAt;return p;
}
function addIssue(list,severity,code,message,nodeId,field){list.push({severity:severity,code:code,message:message,nodeId:nodeId||null,field:field||null})}
function validateParams(n,maxRpm,issues){
  var p=n.params||{},t=n.type,a=ACTIONS[t];if(!a)return;
  function range(key,min,max,label,nonzero){var v=Number(p[key]);if(!Number.isFinite(v)||v<min||v>max)addIssue(issues,'error','range',label+'范围应为 '+min+'～'+max,n.id,key);else if(nonzero&&v===0)addIssue(issues,'error','zero',label+'不能为 0',n.id,key)}
  function duration(allowZero){range('timeoutMs',allowZero?0:1,30000,'时间',false)}
  if(t==='drive'||t==='follow'){range('rpm',-maxRpm,maxRpm,'转速',false);duration(false);if(conditionAllowed(t).indexOf(p.condition)<0)addIssue(issues,'error','condition','完成条件不适用于'+a.name,n.id,'condition');if(Number(p.rpm)===0)addIssue(issues,'warning','zero_speed','转速为 0，不会产生有效运动',n.id,'rpm')}
  else if(t==='drive_mm'){range('distanceMm',-10000,10000,'距离',true);range('maxRpm',1,maxRpm,'最大转速',false)}
  else if(t==='turn'){range('angleDeg',-180,180,'角度',false);duration(false);if(Number(p.angleDeg)===0)addIssue(issues,'warning','zero_angle','转角为 0，没有实际意义',n.id,'angleDeg')}
  else if(t==='wait'){duration(true);if(conditionAllowed(t).indexOf(p.condition)<0)addIssue(issues,'error','condition','等待条件无效',n.id,'condition');if(p.condition==='timeout'&&Number(p.timeoutMs)===0)addIssue(issues,'warning','zero_wait','等待时间为 0，将立即完成',n.id,'timeoutMs')}
  else if(t==='branch'){if(conditionAllowed(t).indexOf(p.condition)<0)addIssue(issues,'error','condition','分支条件无效',n.id,'condition')}
  else if(t.indexOf('led_')===0){var target=Number(p.target);if([0,2,3].indexOf(target)<0)addIssue(issues,'error','target','LED 目标只能是 LED2、LED3 或两者',n.id,'target');if(t!=='led_off'){var d=Number(p.durationMs);if(!Number.isFinite(d)||d<0||d>30000||(d>0&&d<50))addIssue(issues,'error','duration','自动关闭时间应为 0 或 50～30000 ms',n.id,'durationMs')}}
  else if(t==='buzzer_on'||t==='buzzer_toggle'){var bd=Number(p.durationMs);if(!Number.isFinite(bd)||bd<0||bd>30000||(bd>0&&bd<50))addIssue(issues,'error','duration','自动关闭时间应为 0 或 50～30000 ms',n.id,'durationMs')}
}
function validate(project,options){
  var p=normalizeProject(project),maxRpm=(options&&options.maxRpm)||1000,issues=[],nodesBy={},out={};p.nodes.forEach(function(n){nodesBy[n.id]=n;out[n.id]={}});
  var actions=p.nodes.filter(function(n){return ACTIONS[n.type]});
  if(actions.length===0)addIssue(issues,'error','empty','画布中没有可执行动作');
  if(actions.length>64)addIssue(issues,'error','too_many','动作数量超过固件上限 64 步');
  p.edges.forEach(function(e){if(out[e.source][e.port])addIssue(issues,'error','duplicate_port','同一输出端口只能有一条连线',e.source,e.port);else out[e.source][e.port]=e;if(e.source===e.target)addIssue(issues,'error','self_loop','不允许节点直接连接自身',e.source,e.port)});
  var startEdge=out.start&&out.start.success;if(!startEdge)addIssue(issues,'error','missing_start','开始节点必须连接第一个动作','start','success');else if(!ACTIONS[nodesBy[startEdge.target]&&nodesBy[startEdge.target].type])addIssue(issues,'error','bad_start','开始节点必须连接实际动作','start','success');
  actions.forEach(function(n){validateParams(n,maxRpm,issues);if(n.type!=='end'&&!out[n.id].success)addIssue(issues,'error','missing_success',n.type==='branch'?'分支的“条件成立”端口未连接':'“完成”端口必须连接后续动作',n.id,'success');if(n.type==='branch'&&!out[n.id].failure)addIssue(issues,'error','missing_failure','分支的“条件不成立”端口未连接',n.id,'failure');if(out[n.id].success&&!ACTIONS[nodesBy[out[n.id].success.target].type])addIssue(issues,'error','success_target','成功端口必须连接实际动作',n.id,'success');if(out[n.id].failure&&out[n.id].failure.target!=='abort'&&!ACTIONS[nodesBy[out[n.id].failure.target].type])addIssue(issues,'error','failure_target','失败端口只能连接动作或中止节点',n.id,'failure')});
  var reachable={},q=[];if(startEdge){q.push(startEdge.target)}while(q.length){var id=q.shift();if(reachable[id]||!nodesBy[id])continue;reachable[id]=true;var o=out[id]||{};['success','failure'].forEach(function(k){if(o[k]&&nodesBy[o[k].target]&&nodesBy[o[k].target].type!=='system_abort')q.push(o[k].target)})}
  actions.forEach(function(n){if(!reachable[n.id])addIssue(issues,'error','unreachable','节点无法从开始节点到达',n.id)});
  if(!actions.some(function(n){return n.type==='end'&&reachable[n.id]}))addIssue(issues,'error','missing_end','流程必须包含一个可达的“结束”动作');
  var color={},cycle=false;function visit(id){if(color[id]===1){cycle=true;return}if(color[id]===2)return;color[id]=1;var o=out[id]||{};['success','failure'].forEach(function(k){if(o[k]&&ACTIONS[nodesBy[o[k].target]&&nodesBy[o[k].target].type])visit(o[k].target)});color[id]=2}if(startEdge)visit(startEdge.target);if(cycle)addIssue(issues,'warning','cycle','流程包含循环；固件会在总运行 60 秒时强制中止');
  var sum=0;actions.forEach(function(n){var p2=n.params||{};if(['drive','turn','follow','wait'].indexOf(n.type)>=0)sum+=Math.max(0,Number(p2.timeoutMs)||0);if(['led_on','led_toggle','buzzer_on','buzzer_toggle'].indexOf(n.type)>=0)sum+=0});if(!cycle&&sum>60000)addIssue(issues,'error','global_timeout','各步骤时间上限合计超过固件 60 秒全局上限');
  return{valid:!issues.some(function(i){return i.severity==='error'}),issues:issues,reachable:reachable,outgoing:out,nodesById:nodesBy};
}
function rawFor(n){var p=n.params||{},type=n.type,a=ACTIONS[type],r={op:a.op,p1:0,p2:0,until:5,ons:ABORT,ont:ABORT};if(type==='drive'){r.p1=+p.rpm;r.p2=+p.timeoutMs;r.until=CONDS.indexOf(p.condition)}else if(type==='drive_mm'){r.p1=+p.distanceMm;r.p2=+p.maxRpm;r.until=6}else if(type==='turn'){r.p1=+p.angleDeg;r.p2=+p.timeoutMs;r.until=1}else if(type==='follow'){r.p1=+p.rpm;r.p2=+p.timeoutMs;r.until=CONDS.indexOf(p.condition)}else if(type==='wait'){r.p2=+p.timeoutMs;r.until=CONDS.indexOf(p.condition)}else if(type==='branch'){r.until=CONDS.indexOf(p.condition)}else if(type.indexOf('led_')===0){r.p1=+p.target||0;r.p2=type==='led_off'?0:(+p.durationMs||0)}else if(type==='buzzer_on'||type==='buzzer_toggle'){r.p2=+p.durationMs||0}return r}
function compile(project,options){var p=normalizeProject(project),v=validate(p,options);if(!v.valid){var e=new Error(v.issues.filter(function(i){return i.severity==='error'}).map(function(i){return i.message}).join('；'));e.issues=v.issues;throw e}var start=v.outgoing.start.success.target,order=[],seen={},q=[start];while(q.length){var id=q.shift();if(seen[id]||!ACTIONS[v.nodesById[id].type])continue;seen[id]=1;order.push(id);var o=v.outgoing[id]||{};['success','failure'].forEach(function(k){if(o[k]&&ACTIONS[v.nodesById[o[k].target]&&v.nodesById[o[k].target].type]&&!seen[o[k].target])q.push(o[k].target)})}var index={};order.forEach(function(id,i){index[id]=i});var instrs=order.map(function(id){var n=v.nodesById[id],r=rawFor(n),o=v.outgoing[id]||{};if(n.type!=='end'){r.ons=index[o.success.target];if(o.failure&&o.failure.target!=='abort')r.ont=index[o.failure.target];else r.ont=ABORT}return r});return{instrs:instrs,nodeOrder:order,indexByNode:index,issues:v.issues}}
function paramsFromRaw(type,r){if(type==='drive')return{rpm:r.p1,timeoutMs:r.p2,condition:CONDS[r.until]};if(type==='drive_mm')return{distanceMm:r.p1,maxRpm:r.p2};if(type==='turn')return{angleDeg:r.p1,timeoutMs:r.p2};if(type==='follow')return{rpm:r.p1,timeoutMs:r.p2,condition:CONDS[r.until]};if(type==='wait')return{timeoutMs:r.p2,condition:CONDS[r.until]};if(type==='branch')return{condition:CONDS[r.until]};if(type.indexOf('led_')===0){var p={target:r.p1};if(type!=='led_off')p.durationMs=r.p2;return p}if(type==='buzzer_on'||type==='buzzer_toggle')return{durationMs:r.p2};return{}}
function autoLayout(project){var p=project,by={};p.nodes.forEach(function(n){by[n.id]=n});var start=by.start;if(start){start.x=60;start.y=220}var actions=p.nodes.filter(function(n){return ACTIONS[n.type]}).sort(function(a,b){return a.createdOrder-b.createdOrder});actions.forEach(function(n,i){n.x=280+(i%4)*230;n.y=100+Math.floor(i/4)*190});if(by.abort){by.abort.x=280+Math.min(3,actions.length%4)*230;by.abort.y=100+(Math.floor(actions.length/4)+1)*190}return p}
function decompile(instrs,meta){var p=newProject(meta&&meta.name,meta&&meta.slot),nodes=[];(instrs||[]).forEach(function(r,i){var type=OP_TYPES[r.op];if(!type)throw new Error('未知固件操作码：'+r.op);var n=actionNode(type,0,0,i+1,'n'+i);n.params=paramsFromRaw(type,r);nodes.push(n)});p.nodes=p.nodes.concat(nodes);if(nodes[0])p.edges.push({id:uid('e'),source:'start',port:'success',target:nodes[0].id});nodes.forEach(function(n,i){var r=instrs[i];if(n.type==='end')return;var s=r.ons===ABORT?i+1:r.ons;if(s<nodes.length)p.edges.push({id:uid('e'),source:n.id,port:'success',target:nodes[s].id});var f=r.ont;if(f!==ABORT&&f<nodes.length)p.edges.push({id:uid('e'),source:n.id,port:'failure',target:nodes[f].id});else p.edges.push({id:uid('e'),source:n.id,port:'failure',target:'abort'})});return autoLayout(p)}
function connect(p,source,port,target){p.edges=p.edges.filter(function(e){return!(e.source===source&&e.port===port)});p.edges.push({id:uid('e'),source:source,port:port,target:target});p.updatedAt=now();return p}
function chain(types,name){var p=newProject(name,7),prev='start';types.forEach(function(spec,i){var type=typeof spec==='string'?spec:spec.type,n=actionNode(type,0,0,i+1,'n'+i);if(typeof spec==='object')Object.assign(n.params,spec.params||{});p.nodes.push(n);connect(p,prev,'success',n.id);if(prev!=='start'&&p.nodes.find(function(x){return x.id===prev}).type!=='end')connect(p,prev,'failure','abort');prev=n.id});return autoLayout(p)}
function templates(){var a=chain([{type:'drive_mm',params:{distanceMm:500,maxRpm:80}},{type:'turn',params:{angleDeg:90,timeoutMs:5000}},'stop','end'],'定距直行—左转—停车');var b=chain([{type:'follow',params:{rpm:80,timeoutMs:30000,condition:'line_lost'}},'stop','end'],'循迹直到丢线');var c=newProject('检测线路后的条件分支与失败恢复',7),ns=[actionNode('wait',260,220,1,'n0'),actionNode('branch',490,220,2,'n1'),actionNode('drive_mm',720,100,3,'n2'),actionNode('stop',720,340,4,'n3'),actionNode('end',960,220,5,'n4')];ns[0].params={timeoutMs:5000,condition:'line_detected'};c.nodes=c.nodes.concat(ns);connect(c,'start','success','n0');connect(c,'n0','success','n1');connect(c,'n0','failure','n3');connect(c,'n1','success','n2');connect(c,'n1','failure','n3');connect(c,'n2','success','n4');connect(c,'n2','failure','n3');connect(c,'n3','success','n4');connect(c,'n3','failure','abort');var d=chain([{type:'led_on',params:{target:0,durationMs:500}},{type:'buzzer_on',params:{durationMs:200}},{type:'drive_mm',params:{distanceMm:200,maxRpm:60}},'stop','end'],'声光提示后启动运动');return[a,b,c,d]}
function serialize(p){var x=normalizeProject(p);x.updatedAt=now();return JSON.stringify(x,null,2)}
return{FORMAT:FORMAT,VERSION:VERSION,ABORT:ABORT,CONDS:CONDS,COND_LABELS:COND_LABELS,ACTIONS:ACTIONS,OP_TYPES:OP_TYPES,clone:clone,uid:uid,newProject:newProject,actionNode:actionNode,ports:ports,conditionAllowed:conditionAllowed,normalizeProject:normalizeProject,validate:validate,compile:compile,decompile:decompile,autoLayout:autoLayout,connect:connect,templates:templates,serialize:serialize};
}));
