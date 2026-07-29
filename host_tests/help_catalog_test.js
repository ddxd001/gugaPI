'use strict';

const assert=require('assert');
const fs=require('fs');
const path=require('path');

const root=path.resolve(__dirname,'..');
const SC=require(path.join(
  root,'gugaPI/tools/seq_editor/sequence_core.js'));
const Help=require(path.join(
  root,'gugaPI/tools/seq_editor/help_catalog.js'));

const actionTypes=Object.keys(SC.ACTIONS).sort();
const guideTypes=Object.keys(Help.ACTION_GUIDES).sort();
assert.deepStrictEqual(guideTypes,actionTypes,
  'every sequence action must have exactly one help guide');
assert.strictEqual(new Set(Help.CATEGORIES.map(item=>item.id)).size,
  Help.CATEGORIES.length,'help category ids must be unique');
assert.strictEqual(new Set(Help.ARTICLES.map(item=>item.id)).size,
  Help.ARTICLES.length,'help article ids must be unique');

const entries=Help.buildEntries(SC.ACTIONS);
const actionEntries=entries.filter(entry=>entry.kind==='action');
assert.strictEqual(actionEntries.length,actionTypes.length);
assert.strictEqual(new Set(entries.map(entry=>entry.id)).size,entries.length,
  'help entry ids must be unique');

for(const type of actionTypes){
  const action=SC.ACTIONS[type];
  const guide=Help.ACTION_GUIDES[type];
  const entry=actionEntries.find(item=>item.actionType===type);
  assert(entry,type+' action entry missing');
  assert.strictEqual(entry.action.name,action.name,type+' name drift');
  assert.strictEqual(entry.action.group,action.group,type+' group drift');
  assert.strictEqual(entry.action.op,action.op,type+' opcode drift');
  assert.deepStrictEqual(entry.action.defaults,action.defaults,
    type+' defaults drift');
  assert(/[\u3400-\u9fff]/.test(guide.purpose),
    type+' purpose needs Chinese explanation');
  assert(Array.isArray(guide.parameters)&&guide.parameters.length>0,
    type+' needs parameter explanation');
  assert(guide.success&&guide.failure,
    type+' needs both output behavior explanations');
  assert(guide.example&&guide.example.title&&
    Array.isArray(guide.example.paths)&&guide.example.paths.length>0,
    type+' needs a wiring example');
  for(const flow of guide.example.paths){
    assert(Array.isArray(flow.nodes)&&flow.nodes.length>0,
      type+' example path needs typed nodes');
    assert(Array.isArray(flow.ports)&&
      flow.ports.length===Math.max(0,flow.nodes.length-1),
      type+' example path ports must match its links');
    for(const node of flow.nodes){
      assert(node.type==='system_start'||SC.ACTIONS[node.type],
        type+' example uses unknown node type '+node.type);
    }
    for(const port of flow.ports){
      assert(port==='success'||port==='failure',
        type+' example uses invalid port '+port);
    }
  }
  assert(Array.isArray(guide.tips)&&guide.tips.length>0,
    type+' needs safety or usage tips');
}

assert(Help.search(entries,'ActionOp 19','all')
  .some(entry=>entry.id==='action-road_nav'));
assert(Help.search(entries,'循环体入口不存在','troubleshooting')
  .some(entry=>entry.id==='faq-sequence'));
assert(Help.search(entries,'MotorDriver','troubleshooting')
  .some(entry=>entry.id==='faq-device'));
assert(Help.search(entries,'直行','modules')
  .some(entry=>entry.id==='action-drive'));
assert(Help.search(entries,'直行','parameters').length===0,
  'category filtering must be applied with text search');

const html=fs.readFileSync(path.join(
  root,'gugaPI/tools/seq_editor/index.html'),'utf8');
const app4=fs.readFileSync(path.join(
  root,'gugaPI/tools/seq_editor/app4.js'),'utf8');
const app8=fs.readFileSync(path.join(
  root,'gugaPI/tools/seq_editor/app8.js'),'utf8');
assert(html.includes('id="tabHelp"')&&html.includes('id="helpView"'),
  'help tab and view must exist');
assert(html.indexOf('help_catalog.js')<html.indexOf('app8.js'),
  'help catalog must load before help page code');
assert(app4.includes("switchTab('help')")&&
  app4.includes("HelpPage_OnShow"),
  'shared tab lifecycle must include help page');
assert(!/\bsend\s*\(/.test(app8),
  'help page must not send serial commands');
assert(!/setSeqProject|seqProject\s*=/.test(app8),
  'help page must not modify the sequence project');

console.log('help catalog ok: '+Help.CATEGORIES.length+' categories, '+
  Help.ARTICLES.length+' articles, '+actionEntries.length+' action guides');
