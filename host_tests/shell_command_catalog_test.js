'use strict';

const assert=require('assert');
const fs=require('fs');
const path=require('path');
const vm=require('vm');

const root=path.resolve(__dirname,'..');
const shellSource=fs.readFileSync(path.join(root,'gugaPI/app/app_shell.cpp'),'utf8');
const infraredUartSource=fs.readFileSync(
  path.join(root,'gugaPI/drivers/infrared_line/infrared_line_uart.cpp'),'utf8');
const infraredAppSource=fs.readFileSync(
  path.join(root,'gugaPI/app/app_infrared_sensor.cpp'),'utf8');
const debugConfigSource=fs.readFileSync(
  path.join(root,'gugaPI/config/debug_config.h'),'utf8');
const catalogSource=fs.readFileSync(
  path.join(root,'gugaPI/tools/seq_editor/command_catalog.js'),'utf8');
const context={};
vm.createContext(context);
vm.runInContext(catalogSource,context,{filename:'command_catalog.js'});

const catalog=context.SHELL_COMMAND_LIBRARY;
const meta=context.SHELL_CATALOG_META;
assert(Array.isArray(catalog)&&catalog.length>0,'catalog must not be empty');

const registered=new Set(['help']);
for(const match of shellSource.matchAll(/Shell_RegisterCommand\(\s*"([^"]+)"/g)){
  registered.add(match[1]);
}
const catalogNames=new Set(catalog.map(entry=>entry.name));
assert.deepStrictEqual(
  [...catalogNames].sort(),
  [...registered].sort(),
  'catalog top-level names must match firmware registrations plus built-in help');

const active=catalog.filter(entry=>entry.profiles.includes(meta.activeProfile));
assert.strictEqual(active.length,29,'development profile top-level command count changed');
assert(!catalogNames.has('adc')&&!catalogNames.has('pwm'),
  'unregistered adc/pwm placeholders must not return to the catalog');

const maxArgsMatch=debugConfigSource.match(
  /#define\s+DEBUG_SHELL_MAX_ARGS\s+\((\d+)U\)/);
assert(maxArgsMatch,'DEBUG_SHELL_MAX_ARGS must remain a numeric constant');
assert(Number(maxArgsMatch[1])>=12,
  'Shell argv capacity must cover condition and full CAN commands');

assert(infraredUartSource.includes(
  'IncrementSaturated(&context->rx_timeout_count);'),
  'normal UART receive timeout must have its own diagnostic counter');
assert(!/RX_TIMEOUT_ERROR[^}]+uart_error_count/s.test(infraredUartSource),
  'normal UART receive timeout must not increment the hardware error total');
assert(infraredAppSource.includes('RecordCommunicationErrors(uart_delta)'),
  'isolated UART errors must feed the recoverable communication state');
assert(infraredAppSource.includes('(dma_overwrites > g_previousDmaOverwrites)')&&
       infraredAppSource.includes('(dma_faults > g_previousDmaFaults)'),
  'DMA overwrite and global DMA faults must immediately fault transport');
assert(infraredAppSource.includes('g_data.error_streak >= 3U'),
  'transport must stop after three consecutive invalid events');
for(const field of ['rx_timeouts','overrun_errors','framing_errors',
                    'parity_errors','noise_errors','valid_permille',
                    'crc_error_permille']){
  assert(shellSource.includes('" '+field+'="'),
    'irsensor stats is missing '+field);
}

for(const entry of catalog){
  assert(/[\u3400-\u9fff]/.test(entry.title+entry.summary),
    entry.name+' needs a Chinese title or summary');
  assert(Array.isArray(entry.forms)&&entry.forms.length>0,
    entry.name+' needs at least one usage form');
  for(const form of entry.forms){
    assert(form.syntax===entry.name||form.syntax.startsWith(entry.name+' '),
      entry.name+' has a mismatched usage form: '+form.syntax);
    assert(/[\u3400-\u9fff]/.test(form.description),
      form.syntax+' needs a Chinese explanation');
    assert(['R','W','M'].includes(form.risk),
      form.syntax+' has an invalid risk marker');
  }
}

const sourceUsage=[];
for(const match of shellSource.matchAll(/Shell_WriteLine\(\s*"  ([^"]+)"\s*\)/g)){
  const syntax=match[1];
  const top=syntax.split(/\s+/)[0];
  if(registered.has(top))sourceUsage.push(syntax);
}
const catalogUsage=new Set(catalog.flatMap(entry=>entry.forms.map(form=>form.syntax)));
const missingUsage=[...new Set(sourceUsage)].filter(syntax=>!catalogUsage.has(syntax));
assert.deepStrictEqual(missingUsage,[],
  'catalog is missing firmware usage forms');

const activeForms=active.reduce((total,entry)=>total+entry.forms.length,0);
console.log('shell catalog ok: '+catalog.length+' groups, '+active.length+
  ' active, '+activeForms+' active usage forms');
