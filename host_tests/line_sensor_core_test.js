'use strict';

const assert=require('assert');
const core=require('../gugaPI/tools/seq_editor/line_sensor_core.js');

assert.deepStrictEqual(core.pollCommands(false),
  ['linesensor status','gray live','gray calib status']);
assert.deepStrictEqual(core.pollCommands(true),
  ['linesensor status','gray live','gray calib status',
    'gray calib preview','param status']);
assert(core.pollCommands(true).every(command=>!command.startsWith('ir')),
  'polling must expose only ADC8 commands');

assert.strictEqual(core.calibrationCommand('begin'),'gray calib begin');
assert.strictEqual(core.calibrationCommand('white'),'gray calib white 64');
assert.strictEqual(core.calibrationCommand('black'),'gray calib black 64');
assert.strictEqual(core.calibrationCommand('preview'),'gray calib preview');
assert.strictEqual(core.calibrationCommand('center'),'');

const values=core.parseKv(
  'valid=1 seq=9 raw=1,2,3,4,5,6,7,8 status=ok');
assert.deepStrictEqual(values,
  {valid:'1',seq:'9',raw:'1,2,3,4,5,6,7,8',status:'ok'});
assert.deepStrictEqual(core.parseList(values.raw,8),[1,2,3,4,5,6,7,8]);
assert.deepStrictEqual(core.parseList('1,2',8),[]);
assert.deepStrictEqual(core.parseList('1,bad',2),[]);

console.log('line sensor core ok');
