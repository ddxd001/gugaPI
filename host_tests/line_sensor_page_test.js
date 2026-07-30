'use strict';

const assert=require('assert');
const fs=require('fs');
const path=require('path');

const root=path.resolve(__dirname,'..');
const html=fs.readFileSync(path.join(root,'gugaPI/tools/seq_editor/index.html'),'utf8');
const page=fs.readFileSync(path.join(root,'gugaPI/tools/seq_editor/app9.js'),'utf8');
const shell=fs.readFileSync(path.join(root,'gugaPI/app/app_shell.cpp'),'utf8');
const ids=new Set([...html.matchAll(/\bid="([^"]+)"/g)].map(match=>match[1]));

for(const id of ['lineIrCalibration','lineAdcCalibration','lineIrLive',
  'lineAdcLive','lineSourcePending','grayChannelBars','grayPreviewRows',
  'grayCalProgress','grayCalNotice','btnGrayCalBegin','btnGrayCalWhite',
  'btnGrayCalBlack','btnGrayCalCommit','btnGrayCalCancel','btnLineSave',
  'graySimControls','btnGraySimFailure','btnGraySimTimeout']){
  assert(ids.has(id),'line sensor page is missing '+id);
}
assert(html.indexOf('line_sensor_core.js')<html.indexOf('app9.js'),
  'line sensor core must load before its page controller');
assert(/var source=lineSensorState\.active\|\|'ir3'/.test(page),
  'the visible pane must follow the applied source');
assert(/selected!==lineSensorState\.active/.test(page)&&
       page.includes('请先应用到 RAM'),
  'a selected but unapplied source needs an explicit warning');
assert(page.includes("values.valid==='1'&&Number.isFinite(age)&&age<=200"),
  'ADC8 calibration controls must require a fresh valid frame');
assert(page.includes("gray.last==='timeout'")&&page.includes('500 ms'),
  'the UI must explain stalled ADC8 capture');
assert(page.includes("lineSensorCommand('adc8','commit')")&&
       page.includes('提交到 RAM')&&page.includes("lineSensorAction('param save'"),
  'RAM commit and FRAM save must remain separate actions');
assert(page.includes('btnGraySimFailure')&&page.includes('btnGraySimTimeout'),
  'offline failure and timeout demonstrations are missing');
assert(shell.includes('"gray live valid="'),
  'gray live is missing field valid');
for(const field of [' seq=',' age_ms=',' raw=',' normalized=',
  ' position=',' fault=',' anomaly=',' status=',' process_status=']){
  assert(shell.includes('"'+field+'"'),
    'gray live is missing field '+field.trim());
}

console.log('line sensor page ok');
