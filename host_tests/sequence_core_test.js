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
  const end=add(p,'end','end',2);
  SC.connect(p,first.id,'success',end.id);
  SC.connect(p,first.id,'failure','abort');
  if(type==='branch'){
    const no=add(p,'end','end-no',3);
    SC.connect(p,first.id,'failure',no.id);
  }
  return p;
}

assert.strictEqual(SC.FORMAT,'gugapi-sequence-project');
assert.strictEqual(SC.VERSION,1);
assert.strictEqual(Object.keys(SC.ACTIONS).length,14);

for(const type of Object.keys(SC.ACTIONS)){
  const p=simple(type);
  const checked=SC.validate(p,{maxRpm:1000});
  assert(checked.valid,type+' defaults must form a valid project: '+checked.issues.map(x=>x.message));
  const built=SC.compile(p,{maxRpm:1000});
  assert.strictEqual(built.instrs[0].op,SC.ACTIONS[type].op,type+' opcode');
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
const branch=add(cycle,'branch','branch',1);
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

const overTime=SC.newProject('超时',7);
const waits=[add(overTime,'wait','w0',1),add(overTime,'wait','w1',2),add(overTime,'wait','w2',3),add(overTime,'end','wend',4)];
waits.slice(0,3).forEach(n=>{n.params.timeoutMs=30000;n.params.condition='timeout'});
SC.connect(overTime,'start','success','w0');
SC.connect(overTime,'w0','success','w1');SC.connect(overTime,'w0','failure','abort');
SC.connect(overTime,'w1','success','w2');SC.connect(overTime,'w1','failure','abort');
SC.connect(overTime,'w2','success','wend');SC.connect(overTime,'w2','failure','abort');
result=SC.validate(overTime);
assert(!result.valid&&result.issues.some(x=>x.code==='global_timeout'));

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

console.log('sequence core ok: 14 actions, templates, graph validation and round trips');
