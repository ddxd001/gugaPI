'use strict';
const assert=require('assert');
const path=require('path');
const SC=require(path.resolve(__dirname,'../gugaPI/tools/seq_editor/sequence_core.js'));

function add(p,type,id,order){const n=SC.actionNode(type,order*180,100,order,id);p.nodes.push(n);return n}
function simple(type){
  const p=SC.newProject(type,7);
  const first=add(p,type,'first',1);
  SC.connect(p,'start','success',first.id);
  if(type==='end')return p;
  if(type==='loop'){
    const body=add(p,'stop','body',2);
    const end=add(p,'end','end',3);
    SC.connect(p,first.id,'success',body.id);
    SC.connect(p,first.id,'failure',end.id);
    SC.connect(p,body.id,'success',first.id);
    SC.connect(p,body.id,'failure','abort');
    return p;
  }
  const end=add(p,'end','end',2);
  SC.connect(p,first.id,'success',end.id);
  SC.connect(p,first.id,'failure','abort');
  if(type==='condition'){
    const no=add(p,'end','end-no',3);
    SC.connect(p,first.id,'failure',no.id);
  }
  return p;
}

assert.strictEqual(SC.FORMAT,'gugapi-sequence-project');
assert.strictEqual(SC.VERSION,2);
assert.strictEqual(Object.keys(SC.ACTIONS).length,16);

for(const type of Object.keys(SC.ACTIONS)){
  const p=simple(type);
  const checked=SC.validate(p,{maxRpm:1000});
  assert(checked.valid,type+' defaults must form a valid project: '+checked.issues.map(x=>x.message));
  const built=SC.compile(p,{maxRpm:1000});
  const expectedOp=type==='follow'?17:SC.ACTIONS[type].op;
  assert.strictEqual(built.instrs[0].op,expectedOp,type+' opcode');
  assert(Number.isInteger(built.instrs[0].p1)&&Number.isInteger(built.instrs[0].p2));
  assert(Number.isInteger(built.instrs[0].until));
}

for(const template of SC.templates()){
  const checked=SC.validate(template,{maxRpm:1000});
  assert(checked.valid,template.name+': '+checked.issues.map(x=>x.message));
  assert(SC.compile(template).instrs.length>0);
}

const source=SC.templates()[2];
const json=SC.serialize(source);
const restored=SC.normalizeProject(JSON.parse(json));
assert.strictEqual(restored.format,SC.FORMAT);
assert.strictEqual(restored.nodes.length,source.nodes.length);
assert.strictEqual(restored.edges.length,source.edges.length);

const compiled=SC.compile(source);
const decoded=SC.decompile(compiled.instrs,{name:'往返',slot:3});
const recompiled=SC.compile(decoded);
assert.deepStrictEqual(recompiled.instrs,compiled.instrs,'compile/decompile round trip');

const unreachable=SC.clone(simple('drive'));
unreachable.nodes.push(SC.actionNode('wait',900,500,9,'orphan'));
let result=SC.validate(unreachable);
assert(!result.valid&&result.issues.some(x=>x.code==='unreachable'));

const tooMany=SC.newProject('超长',7);
let previous='start';
for(let i=0;i<65;i++){
  const n=add(tooMany,i===64?'end':'stop','m'+i,i+1);
  SC.connect(tooMany,previous,'success',n.id);
  if(previous!=='start')SC.connect(tooMany,previous,'failure','abort');
  previous=n.id;
}
result=SC.validate(tooMany);
assert(!result.valid&&result.issues.some(x=>x.code==='too_many'));

const cycle=SC.newProject('循环',7);
const branch=add(cycle,'condition','branch',1);
branch.params.source='constant';
branch.params.compare='eq';
branch.params.value=1;
const wait=add(cycle,'wait','wait',2);
const finish=add(cycle,'end','finish',3);
SC.connect(cycle,'start','success','branch');
SC.connect(cycle,'branch','success','wait');
SC.connect(cycle,'branch','failure','finish');
SC.connect(cycle,'wait','success','branch');
SC.connect(cycle,'wait','failure','abort');
result=SC.validate(cycle);
assert(result.valid&&result.issues.some(x=>x.code==='cycle'));
assert.doesNotThrow(()=>SC.compile(cycle));

const self=SC.clone(cycle);
SC.connect(self,'wait','success','wait');
result=SC.validate(self);
assert(!result.valid&&result.issues.some(x=>x.code==='self_loop'));

const countedLoop=simple('loop');
countedLoop.nodes.find(n=>n.id==='first').params.count=3;
result=SC.validate(countedLoop);
assert(result.valid);
assert(result.issues.some(x=>x.code==='loop_runtime_limit'));
assert(!result.issues.some(x=>x.code==='cycle'),
  'valid counted-loop back edge must not produce generic cycle warning');
const loopRaw=SC.compile(countedLoop).instrs;
assert.strictEqual(loopRaw[0].op,18);
assert.strictEqual(loopRaw[0].p1,3);
assert.strictEqual(loopRaw[0].p2,0);
assert.strictEqual(loopRaw[0].until,5);
assert.strictEqual(loopRaw[0].ons,1);
assert.strictEqual(loopRaw[0].ont,2);
assert.deepStrictEqual(
  SC.compile(SC.decompile(loopRaw,{name:'计数循环',slot:7})).instrs,
  loopRaw,'counted loop compile/decompile round trip');
const loopJson=SC.serialize(countedLoop);
assert.strictEqual(
  SC.compile(SC.normalizeProject(JSON.parse(loopJson))).instrs[0].op,18,
  'loop survives JSON v2 export/import');

for(const route of SC.ROAD_ROUTES){
  const road=simple('road_nav');
  const roadNode=road.nodes.find(n=>n.id==='first');
  if(route.value.startsWith('uturn_')){
    roadNode.params.direction=route.value.replace(/_(arc|pivot)$/,'');
    roadNode.params.uturnMode=route.value.endsWith('_pivot')?'pivot':'arc';
  }else{
    roadNode.params.direction=route.value;
  }
  roadNode.params.rpm=80;
  roadNode.params.timeoutMs=15000;
  result=SC.validate(road,{maxRpm:500});
  assert(result.valid,route.value+': '+result.issues.map(x=>x.message));
  const roadRaw=SC.compile(road,{maxRpm:500}).instrs;
  assert.strictEqual(roadRaw[0].op,19);
  assert.strictEqual(roadRaw[0].conditionValue,route.code);
  assert.strictEqual(roadRaw[0].route,route.value);
  assert.strictEqual(roadRaw[0].p1,80);
  assert.strictEqual(roadRaw[0].p2,15000);
  assert.strictEqual(roadRaw[0].until,5);
  assert.deepStrictEqual(
    SC.compile(SC.decompile(roadRaw)).instrs,roadRaw,
    route.value+' road-nav round trip');
  assert.deepStrictEqual(
    SC.compile(SC.normalizeProject(JSON.parse(SC.serialize(road)))).instrs,
    roadRaw,route.value+' JSON v2 round trip');
}

for(const patch of [
  {rpm:0},{rpm:501},{timeoutMs:75},{timeoutMs:30050},
  {direction:'reverse'}
]){
  const invalidRoad=simple('road_nav');
  Object.assign(invalidRoad.nodes.find(n=>n.id==='first').params,patch);
  result=SC.validate(invalidRoad,{maxRpm:500});
  assert(!result.valid,'invalid road-nav params must be rejected: '+
    JSON.stringify(patch));
}

const missingLoopExit=SC.clone(countedLoop);
missingLoopExit.edges=missingLoopExit.edges.filter(
  e=>!(e.source==='first'&&e.port==='failure'));
result=SC.validate(missingLoopExit);
assert(!result.valid&&result.issues.some(x=>x.code==='missing_failure'));

for(const invalidCount of [0,1001]){
  const invalidLoop=SC.clone(countedLoop);
  invalidLoop.nodes.find(n=>n.id==='first').params.count=invalidCount;
  result=SC.validate(invalidLoop);
  assert(!result.valid&&result.issues.some(x=>x.code==='loop_count'));
}

const noLoopReturn=SC.clone(countedLoop);
SC.connect(noLoopReturn,'body','success','end');
result=SC.validate(noLoopReturn);
assert(!result.valid&&result.issues.some(x=>x.code==='loop_no_return'));

const nested=SC.newProject('嵌套循环',7);
const outer=add(nested,'loop','outer',1);outer.params.count=2;
const inner=add(nested,'loop','inner',2);inner.params.count=3;
add(nested,'stop','inner-body',3);
add(nested,'stop','inner-done',4);
add(nested,'end','outer-done',5);
SC.connect(nested,'start','success','outer');
SC.connect(nested,'outer','success','inner');
SC.connect(nested,'outer','failure','outer-done');
SC.connect(nested,'inner','success','inner-body');
SC.connect(nested,'inner','failure','inner-done');
SC.connect(nested,'inner-body','success','inner');
SC.connect(nested,'inner-body','failure','abort');
SC.connect(nested,'inner-done','success','outer');
SC.connect(nested,'inner-done','failure','abort');
result=SC.validate(nested);
assert(result.valid,result.issues.map(x=>x.message).join('; '));
assert(!result.issues.some(x=>x.code==='cycle'));
const nestedRaw=SC.compile(nested).instrs;
assert.strictEqual(nestedRaw.filter(x=>x.op===18).length,2);
assert.deepStrictEqual(SC.compile(SC.decompile(nestedRaw)).instrs,nestedRaw,
  'nested loops round trip');

const overTime=SC.newProject('超时',7);
const waits=[];
for(let i=0;i<11;i++){
  const waitNode=add(overTime,'wait','w'+i,i+1);
  waitNode.params.timeoutMs=30000;
  waits.push(waitNode);
}
add(overTime,'end','wend',12);
SC.connect(overTime,'start','success','w0');
for(let i=0;i<waits.length;i++){
  SC.connect(overTime,'w'+i,'success',i+1<waits.length?'w'+(i+1):'wend');
  SC.connect(overTime,'w'+i,'failure','abort');
}
result=SC.validate(overTime);
assert(!result.valid&&result.issues.some(x=>x.code==='global_timeout'));
const atLimit=SC.clone(overTime);
atLimit.nodes.find(n=>n.id==='w10').params.timeoutMs=0;
result=SC.validate(atLimit);
assert(result.valid&&!result.issues.some(x=>x.code==='global_timeout'),
  '300 second static total must remain valid');

const roadOverTime=SC.newProject('路口动作超时合计',7);
const roadSteps=[];
for(let i=0;i<11;i++){
  const road=add(roadOverTime,'road_nav','road'+i,i+1);
  road.params.timeoutMs=30000;
  roadSteps.push(road);
}
add(roadOverTime,'end','road-end',12);
SC.connect(roadOverTime,'start','success','road0');
for(let i=0;i<roadSteps.length;i++){
  SC.connect(roadOverTime,'road'+i,'success',
    i+1<roadSteps.length?'road'+(i+1):'road-end');
  SC.connect(roadOverTime,'road'+i,'failure','abort');
}
result=SC.validate(roadOverTime);
assert(!result.valid&&result.issues.some(x=>x.code==='global_timeout'),
  'road-nav overall timeout must participate in the 300 second static sum');

const stable=SC.templates()[0];
const before=SC.compile(stable);
const targetId=before.nodeOrder[2];
const inserted=SC.actionNode('led_on',410,40,99,'inserted');
stable.nodes.push(inserted);
const edge=stable.edges.find(e=>e.source===before.nodeOrder[0]&&e.port==='success');
const oldTarget=edge.target;
edge.target='inserted';
SC.connect(stable,'inserted','success',oldTarget);
SC.connect(stable,'inserted','failure','abort');
const after=SC.compile(stable);
assert.strictEqual(after.nodeOrder[after.indexByNode[targetId]],targetId,'node identity survives insertion');
const rawFromFirst=after.instrs[after.indexByNode[before.nodeOrder[0]]];
assert.strictEqual(after.nodeOrder[rawFromFirst.ons],'inserted','edge targets are recompiled, not shifted indices');

const compareProject=simple('condition');
const compareNode=compareProject.nodes.find(n=>n.id==='first');
Object.assign(compareNode.params,{source:'imu_yaw',compare:'ge',value:90000,
  mode:'wait',timeoutMs:5000,stableMs:100});
const compareRaw=SC.compile(compareProject).instrs[0];
assert.strictEqual(compareRaw.op,15);
assert.strictEqual(compareRaw.source,'imu_yaw');
assert.strictEqual(compareRaw.value,90000);
assert.deepStrictEqual(
  SC.compile(SC.decompile(SC.compile(compareProject).instrs)).instrs,
  SC.compile(compareProject).instrs,
  'generic condition round trip');

const moving=simple('drive');
const movingNode=moving.nodes.find(n=>n.id==='first');
Object.assign(movingNode.params,{completion:'compare',
  source:'average_distance',compare:'ge',value:300,
  timeoutMs:5000,stableMs:100});
assert.strictEqual(SC.compile(moving).instrs[0].op,16);

const button2=simple('condition');
const button2Node=button2.nodes.find(n=>n.id==='first');
Object.assign(button2Node.params,{source:'button2_pressed',compare:'eq',
  value:1,mode:'wait',timeoutMs:1000,stableMs:0});
assert(SC.validate(button2,{maxRpm:1000}).valid);
assert(!SC.validate(button2,{maxRpm:1000,competition:true}).valid);

const v1=SC.clone(SC.newProject('legacy',3));
v1.version=1;
const legacy={
  id:'legacy-branch',type:'branch',x:200,y:100,createdOrder:1,
  params:{condition:'line_lost'}
};
const legacyEnd={
  id:'legacy-end',type:'end',x:400,y:100,createdOrder:2,params:{}
};
v1.nodes.push(legacy,legacyEnd);
v1.edges.push(
  {id:'a',source:'start',port:'success',target:'legacy-branch'},
  {id:'b',source:'legacy-branch',port:'success',target:'legacy-end'},
  {id:'c',source:'legacy-branch',port:'failure',target:'abort'}
);
const migrated=SC.normalizeProject(v1);
const migratedNode=migrated.nodes.find(n=>n.id==='legacy-branch');
assert.strictEqual(migrated.version,2);
assert.strictEqual(migratedNode.type,'condition');
assert.strictEqual(migratedNode.params.source,'line_detected');
assert.strictEqual(migratedNode.params.value,0);

console.log('sequence core ok: v2 migration, 16 actions, road-nav, counted/nested loops and round trips');
