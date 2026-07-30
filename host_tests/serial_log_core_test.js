'use strict';

const assert=require('assert');
const fs=require('fs');
const path=require('path');
const core=require('../gugaPI/tools/seq_editor/serial_log_core.js');

const start=new Date('2026-07-30T12:34:56.789Z');
assert.strictEqual(
  core.suggestedFilename(start),
  'gugapi-serial-2026-07-30T12-34-56-789Z.jsonl');

const header=JSON.parse(core.encodeHeader(start,{baud_rate:115200}));
assert.deepStrictEqual(header,{
  type:'session',format:'gugapi-serial-log',version:1,
  started_at:'2026-07-30T12:34:56.789Z',metadata:{baud_rate:115200}
});

const raw='gray live\r\n> 温度=25\r\n';
const entry=JSON.parse(core.encodeEntry(7,'rx',raw,start));
assert.strictEqual(entry.sequence,7);
assert.strictEqual(entry.direction,'rx');
assert.strictEqual(entry.data,raw,'JSONL must preserve CR/LF and Unicode');
assert.strictEqual(core.byteLength('温度'),6);
assert.throws(()=>core.encodeEntry(1,'other','x',start),/invalid direction/);

const footer=JSON.parse(core.encodeFooter(start,'serial-disconnect',9,123));
assert.strictEqual(footer.reason,'serial-disconnect');
assert.strictEqual(footer.entries,9);
assert.strictEqual(footer.payload_bytes,123);

const root=path.resolve(__dirname,'../gugaPI/tools/seq_editor');
const transport=fs.readFileSync(path.join(root,'app1.js'),'utf8');
const ui=fs.readFileSync(path.join(root,'app11.js'),'utf8');
const html=fs.readFileSync(path.join(root,'index.html'),'utf8');

assert(transport.indexOf("SerialLog_Record('rx',data)")<
  transport.indexOf('serialRouter.push(data)'),
  'RX must be captured before telemetry/shell routing removes data');
assert(transport.includes("SerialLog_Record('tx',cmd+'\\r\\n')"),
  'every Shell command must enter the transcript');
assert(transport.includes("SerialLog_Record('rx',resp)"),
  'simulated replies must be testable through the same recorder');
assert(transport.includes("SerialLog_Stop('serial-disconnect')"),
  'disconnect must finalize the local file');
assert(ui.includes('window.showSaveFilePicker')&&ui.includes('createWritable'),
  'supported browsers must stream directly to a user-selected local file');
assert(ui.includes('application/x-ndjson')&&ui.includes("mode='memory'"),
  'unsupported browsers must retain a download fallback');
assert(html.includes('id="btnSerialLog"')&&html.includes('id="serialLogStatus"'));
assert(html.indexOf('serial_log_core.js')<html.indexOf('app1.js'));
assert(html.indexOf('app11.js')>html.indexOf('app10.js'));

console.log('serial log core ok: exact TX/RX JSONL and live-file hooks');
