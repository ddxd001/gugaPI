'use strict';

const assert=require('assert');
const core=require('../gugaPI/tools/seq_editor/line_sensor_core.js');

assert.deepStrictEqual(core.pollCommands('adc8',false),
  ['linesensor status','gray live','gray calib status']);
assert.deepStrictEqual(core.pollCommands('adc8',true),
  ['linesensor status','gray live','gray calib status',
    'gray calib preview','param status']);
assert(core.pollCommands('adc8',true).every(command=>
  !command.startsWith('irsensor ')),
  'adc8 polling must never send infrared commands');
assert(core.pollCommands('ir3',true).every(command=>
  !command.startsWith('gray ')),
  'ir3 polling must never send grayscale commands');

assert.strictEqual(core.calibrationCommand('adc8','begin'),'gray calib begin');
assert.strictEqual(core.calibrationCommand('adc8','white'),'gray calib white 64');
assert.strictEqual(core.calibrationCommand('adc8','black'),'gray calib black 64');
assert.strictEqual(core.calibrationCommand('adc8','preview'),'gray calib preview');
assert.strictEqual(core.calibrationCommand('ir3','white'),
  'irsensor calib capture white');
assert.strictEqual(core.calibrationCommand('ir3','center'),
  'irsensor calib capture center');

const values=core.parseKv(
  'valid=1 seq=9 raw=1,2,3,4,5,6,7,8 status=ok');
assert.deepStrictEqual(values,
  {valid:'1',seq:'9',raw:'1,2,3,4,5,6,7,8',status:'ok'});
assert.deepStrictEqual(core.parseList(values.raw,8),[1,2,3,4,5,6,7,8]);
assert.deepStrictEqual(core.parseList('1,2',8),[]);
assert.deepStrictEqual(core.parseList('1,bad',2),[]);

console.log('line sensor core ok');
