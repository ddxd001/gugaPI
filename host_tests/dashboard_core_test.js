'use strict';

const assert=require('assert');
const fs=require('fs');
const path=require('path');
const core=require('../gugaPI/tools/seq_editor/dashboard_core.js');

const shellSource=fs.readFileSync(
  path.resolve(__dirname,'../gugaPI/app/app_shell.cpp'),'utf8');
const dashboardSource=fs.readFileSync(
  path.resolve(__dirname,'../gugaPI/tools/seq_editor/app7.js'),'utf8');
const htmlSource=fs.readFileSync(
  path.resolve(__dirname,'../gugaPI/tools/seq_editor/index.html'),'utf8');
const headerFunction=shellSource.match(
  /void TelemSendHeader\(void\)\s*\{([\s\S]*?)\n\}/);
assert(headerFunction,'firmware telemetry header function is missing');
const headerTokens=[...headerFunction[1].matchAll(/"([^"]*)"/g)]
  .map(match=>match[1]);
const firmwareHeaders=[];
let joinedHeader='';
for(const token of headerTokens){
  if(token.startsWith('#'))joinedHeader=token;
  else if(joinedHeader)joinedHeader+=token;
  if(joinedHeader.endsWith('\\n')){
    firmwareHeaders.push(joinedHeader.replace(/\\n$/,''));
    joinedHeader='';
  }
}
const fullHeader=firmwareHeaders.find(header=>
  header.startsWith('#t,mode,step,'));
assert(fullHeader,'full compatibility telemetry header is missing');
assert.deepStrictEqual(fullHeader.slice(1).split(','),core.DEFAULT_FIELDS,
  'full dashboard schema must match the firmware telemetry header');
for(const [group,fields] of Object.entries(core.GROUP_FIELDS)){
  const expected='#'+fields.join(',');
  assert(firmwareHeaders.includes(expected),
    group+' firmware telemetry header must match dashboard group schema');
}
assert(/telem on <motor\|heading\|line\|accel\|gyro>/.test(shellSource),
  'firmware grouped telemetry usage is missing');

const htmlIds=new Set([...htmlSource.matchAll(/\bid="([^"]+)"/g)]
  .map(match=>match[1]));
for(const match of dashboardSource.matchAll(/\$\('([^']+)'\)/g)){
  assert(htmlIds.has(match[1]),'dashboard references missing HTML id '+match[1]);
}
assert(htmlSource.indexOf('dashboard_core.js')<htmlSource.indexOf('app1.js'),
  'telemetry router must load before app1.js');
assert(htmlSource.indexOf('app7.js')>htmlSource.indexOf('app4.js'),
  'dashboard UI must load after shared tab and terminal code');
assert(/activeGroup:null/.test(dashboardSource),
  'dashboard must start without a selected telemetry group');
assert(!/localStorage/.test(dashboardSource),
  'dashboard selection must not persist across application restarts');
for(const group of Object.keys(core.GROUP_FIELDS)){
  assert(dashboardSource.includes("'telem on '+key+' '"),
    'dashboard must build grouped telemetry commands');
  assert(dashboardSource.includes("key:'"+group+"'"),
    'dashboard chart metadata is missing group '+group);
}

const parser=new core.TelemetryParser();
let event=parser.parseLine('#t,mode,L_tgt','2026-07-28T00:00:00.000Z');
assert.strictEqual(event.type,'header');
assert.deepStrictEqual(event.fields,['t','mode','L_tgt']);

event=parser.parseLine('100,3,-120','2026-07-28T00:00:00.100Z');
assert.strictEqual(event.type,'sample');
assert.strictEqual(event.values.t,100);
assert.strictEqual(event.values.mode,3);
assert.strictEqual(event.values.L_tgt,-120);
assert.strictEqual(event.hostTime,'2026-07-28T00:00:00.100Z');

assert.strictEqual(parser.parseLine('100,3').type,'other',
  'wrong field count must not be consumed as telemetry');
assert.strictEqual(parser.parseLine('100,armed,-120').type,'other',
  'non-numeric CSV must remain visible to the Shell');

const text=[];
const telemetry=[];
const router=new core.SerialRouter({
  onText:value=>text.push(value),
  onTelemetry:value=>telemetry.push(value)
});
router.push('#t,mode,L_t');
router.push('gt\r');
router.push('\n200,1,80\r\ncomp status: ok\r');
router.push('\n> ');
assert.strictEqual(telemetry.length,2);
assert.strictEqual(telemetry[0].type,'header');
assert.strictEqual(telemetry[1].values.L_tgt,80);
assert.strictEqual(text.join(''),'comp status: ok\n> ',
  'telemetry must be removed without breaking the prompt');

router.reset();
router.push('ordinary shell text\r\n');
router.flush();
assert(text.join('').includes('ordinary shell text\n'));
router.reset();
text.length=0;
telemetry.length=0;
router.push('telem: ok period=100 ms\r\n> #t,mode,L_tgt\r\n');
router.push('300,1,90\r\n> 400,1,91\r\n');
assert.strictEqual(telemetry.length,3,
  'prompt-prefixed telemetry header/data must be routed on real UART');
assert.strictEqual(telemetry[0].type,'header');
assert.strictEqual(telemetry[1].values.L_tgt,90);
assert.strictEqual(telemetry[2].values.L_tgt,91);
assert(text.join('').includes('> '),
  'prompt prefix must remain visible to shell command handling');

const csv=core.exportCsv(
  ['t','mode'],
  [{
    hostTime:'2026-07-28T00:00:00.100Z',
    values:{t:100,mode:3}
  }]
);
assert(csv.startsWith('host_rx_iso,t,mode\r\n'));
assert(csv.includes('2026-07-28T00:00:00.100Z,100,3\r\n'));
assert.strictEqual(core.csvEscape('a,b'),'"a,b"');
assert.strictEqual(core.csvEscape('a"b'),'"a""b"');

const timedSamples=[{hostMs:100},{hostMs:200},{hostMs:450},{hostMs:900}];
assert.strictEqual(core.findNearestSample(timedSamples,440),timedSamples[2]);
assert.strictEqual(core.findNearestSample(timedSamples,325),timedSamples[1],
  'equal-distance hover selects the earlier sample');
assert.strictEqual(core.findNearestSample(timedSamples,10),timedSamples[0]);
assert.strictEqual(core.findNearestSample(timedSamples,1200),timedSamples[3]);
assert.strictEqual(core.findNearestSample([],100),null);

console.log('dashboard core ok');
