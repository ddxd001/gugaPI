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
const serverSource=fs.readFileSync(
  path.resolve(__dirname,'../gugaPI/tools/seq_editor/server.ps1'),'utf8');
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
  assert(shellSource.includes('"'+group+'"'),
    group+' firmware telemetry profile parser is missing');
}
assert(/telem on <profile>/.test(shellSource),
  'firmware grouped telemetry usage is missing');
assert(/TELEM_PROFILE_LINE[^_]/.test(shellSource),
  'legacy combined line telemetry profile must remain available');

const groupedFields=[];
for(const fields of Object.values(core.GROUP_FIELDS)){
  groupedFields.push(...fields.slice(1));
}
assert.deepStrictEqual([...new Set(groupedFields)].sort(),
  core.DEFAULT_FIELDS.slice(1).sort(),
  'on-demand groups must cover every full telemetry field exactly once');
assert.strictEqual(groupedFields.length,new Set(groupedFields).size,
  'a telemetry field must not be duplicated across dashboard charts');
assert(core.CHARTS.length>=30,'complete diagnostic chart catalog is missing');
assert.deepStrictEqual(core.CATEGORIES.map(item=>item.label),
  ['运行','底盘','循迹与道路','转向过程','IMU','灰度传感器','系统健康']);
for(const chart of core.CHARTS){
  assert.deepStrictEqual(core.GROUP_FIELDS[chart.key],
    ['t',...chart.series.map(item=>item.field)]);
  assert(chart.title&&chart.subtitle&&chart.unit);
  for(const item of chart.series){
    assert(item.label&&item.unit&&item.description&&item.color,
      chart.key+' metadata is incomplete for '+item.field);
  }
}

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
assert(/event\.fields\.join\(','\)!==sampleExpected\.join\(','\)/.test(dashboardSource),
  'samples from a stale telemetry profile must be rejected');
for(const group of Object.keys(core.GROUP_FIELDS)){
  assert(dashboardSource.includes("'telem on '+key+' '"),
    'dashboard must build grouped telemetry commands');
  assert(core.CHARTS.some(chart=>chart.key===group),
    'dashboard chart catalog is missing group '+group);
}
assert(!htmlSource.includes('STOP ALL'),
  'dashboard must use the single global Chinese emergency stop control');
for(const legacyEnglish of ['REALTIME INSPECTOR','DATA DETAILS',
  'SELECT A CHART FROM THE LEFT','NO DATA SELECTED']){
  assert(!htmlSource.includes(legacyEnglish)&&!dashboardSource.includes(legacyEnglish),
    'dashboard still contains untranslated label '+legacyEnglish);
}
assert.strictEqual(core.describeValue('boolean',1),'是');
assert.strictEqual(core.describeValue('appMode',3),'比赛待命');
assert.strictEqual(core.describeValue('roadPaths',7),'左 / 前 / 右');
assert(/text\/css; charset=utf-8/.test(serverSource),
  'development server must return CSS with a browser-safe MIME type');
assert(/application\/json; charset=utf-8/.test(serverSource));
assert(/image\/svg\+xml/.test(serverSource));

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
