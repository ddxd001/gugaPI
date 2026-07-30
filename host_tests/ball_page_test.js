'use strict';

const assert=require('assert');
const fs=require('fs');
const path=require('path');
const core=require('../gugaPI/tools/seq_editor/ball_core.js');
const root=path.resolve(__dirname,'..');
const shell=fs.readFileSync(path.join(root,'gugaPI/app/app_shell.cpp'),'utf8');
const html=fs.readFileSync(path.join(root,'gugaPI/tools/seq_editor/index.html'),'utf8');
const app=fs.readFileSync(path.join(root,'gugaPI/tools/seq_editor/app10.js'),'utf8');
const tabs=fs.readFileSync(path.join(root,'gugaPI/tools/seq_editor/app4.js'),'utf8');

assert.strictEqual(core.FIELDS.length,28);
assert.deepStrictEqual(core.FIELDS.slice(0,3),['t','app_mode','action_running']);
assert(core.FIELDS.includes('ball_position_0p1mm'));
assert(core.FIELDS.includes('vision_ball_age_ms'));
assert(core.FIELDS.includes('dm_position_mrad'));
const ballHeader='#'+core.FIELDS.join(',');
const headerStrings=[...shell.matchAll(/"([^"]*)"/g)].map(match=>match[1]);
assert(headerStrings.join('').includes(ballHeader+'\\n'),
  'firmware ball telemetry header must match host schema');
assert(/TELEM_PROFILE_BALL/.test(shell));
assert(/profile == TELEM_PROFILE_BALL\) \? 20U : 50U/.test(shell),
  'ball profile must allow 20 ms without lowering other profile limits');
assert(/StrEqual\(text, "ball"\)/.test(shell));

const scaled=core.scaled({values:{t:20,ball_target_0p1mm:500,
  ball_position_0p1mm:-125,ball_velocity_0p1mm_s:80,
  ball_error_0p1mm:625,ball_max_error_0p1mm:700,
  ball_beam_mdeg:2500,pitch_mdeg:-1000,ball_dm_target_mrad:500,
  dm_position_mrad:450,dm_velocity_mrad_s:100}});
assert.strictEqual(scaled.targetMm,50);
assert.strictEqual(scaled.positionMm,-12.5);
assert.strictEqual(scaled.beamDeg,2.5);
assert.strictEqual(scaled.dmPositionRad,.45);

assert.strictEqual(core.motionGate({connected:false}),'请先连接设备');
assert(core.motionGate({connected:true,appMode:3,faultCode:0,
  actionRunning:0,unlocked:true,busy:false}).includes('dev-running'));
assert(core.motionGate({connected:true,appMode:1,faultCode:2,
  actionRunning:0,unlocked:true,busy:false}).includes('故障'));
assert(core.motionGate({connected:true,appMode:1,faultCode:0,
  actionRunning:1,unlocked:true,busy:false}).includes('ActionRunner'));
assert.strictEqual(core.motionGate({connected:true,appMode:1,faultCode:0,
  actionRunning:0,unlocked:true,busy:false}),'');

assert.strictEqual(core.validateMap(
  [-8000,-4000,0,4000,8000],[-1000,-500,0,500,1000]),'');
assert.strictEqual(core.validateMap(
  [-8000,-4000,0,4000,8000],[1000,500,0,-500,-1000]),'');
assert(core.validateMap([-8000,-4000,0,4000,8000],[0,1,1,2,3]));
assert(core.validateMap([-8000,-4000,0,4000,8000],[0,2,1,3,4]));

const samples=[
  {values:{t:0}},{values:{t:30000}},{values:{t:61000}}
];
assert.deepStrictEqual(core.trimSamples(samples,61000,60000)
  .map(sample=>sample.values.t),[30000,61000]);
const csv=core.exportCsv(['t','ball_position_0p1mm'],[
  {hostTime:'2026-07-30T00:00:00Z',
    values:{t:10,ball_position_0p1mm:25}}
]);
assert(csv.includes('host_time,t,ball_position_0p1mm'));
assert(csv.includes('2026-07-30T00:00:00Z,10,25'));

const ids=new Set([...html.matchAll(/\bid="([^"]+)"/g)].map(match=>match[1]));
for(const match of app.matchAll(/\$\('([^']+)'\)/g)){
  assert(ids.has(match[1]),'ball page references missing HTML id '+match[1]);
}
assert(ids.has('tabBall')&&ids.has('ballView'));
assert(tabs.includes("switchTab('ball')"));
assert(app.includes("send('telem on ball 20'"));
assert(app.includes("'dm position relative '"));
assert(app.includes("'ball map '+args.join(' ')"));
assert(app.includes('Math.abs(Number(v.dm_velocity_mrad_s))>50'));
assert(app.includes('BallPage_BeforeDisconnect'));
assert(app.includes("$('btnBallStop').disabled=!ballState.connected"),
  'stop must remain available regardless of unlock or operation busy state');
assert(html.indexOf('ball_core.js')<html.indexOf('app1.js'));
assert(html.indexOf('app10.js')>html.indexOf('app9.js'));

console.log('ball page ok: 50 Hz telemetry, safety gate, controls, capture and CSV');
