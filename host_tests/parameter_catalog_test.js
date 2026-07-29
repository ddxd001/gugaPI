'use strict';

const assert=require('assert');
const fs=require('fs');
const path=require('path');
const vm=require('vm');

const root=path.resolve(__dirname,'..');
const storeSource=fs.readFileSync(
  path.join(root,'gugaPI/app/config_store.cpp'),'utf8');
const parameterSource=fs.readFileSync(
  path.join(root,'gugaPI/tools/seq_editor/app5.js'),'utf8');

const metadataSource=parameterSource.slice(0,parameterSource.indexOf('var paramPageState='));
const context={};
vm.createContext(context);
vm.runInContext(metadataSource,context,{filename:'app5.js metadata'});

const descriptorNames=[...storeSource.matchAll(
  /\{\s*"([a-zA-Z0-9_]+)"\s*,\s*PARAM_(?:U8|U16|U32|I32)\s*,/g
)].map(match=>match[1]);

assert.strictEqual(descriptorNames.length,107,
  'firmware parameter count changed; audit the host metadata');
assert.strictEqual(new Set(descriptorNames).size,descriptorNames.length,
  'firmware parameter descriptors contain duplicates');
assert.strictEqual(new Set(context.PARAM_ORDER).size,context.PARAM_ORDER.length,
  'host parameter metadata contains duplicates');
assert.deepStrictEqual(
  [...context.PARAM_ORDER].sort(),
  [...descriptorNames].sort(),
  'host parameter names must match ConfigStore descriptors');

const v15Parameters={
  road_align_distance_mm:{defaultValue:0,min:0,max:300},
  road_align_rpm:{defaultValue:30,min:1,max:300},
  road_turn_outer_max_rpm:{defaultValue:220,min:1,max:1000},
  road_turn_inner_reverse_max_rpm:{defaultValue:120,min:0,max:1000}
};
for(const [name,expected] of Object.entries(v15Parameters)){
  assert(context.PARAM_META[name],name+' is missing from the host parameter tree');
  for(const [field,value] of Object.entries(expected)){
    assert.strictEqual(context.PARAM_META[name][field],value,
      name+' '+field+' must match firmware v15');
  }
}

const v16Parameters={
  dm_position_kp_milli:{defaultValue:4000,min:0,max:10000},
  dm_position_kd_milli:{defaultValue:400,min:0,max:2000},
  dm_speed_kd_milli:{defaultValue:500,min:0,max:2000},
  dm_max_velocity_mrad_s:{defaultValue:2000,min:0,max:20000},
  dm_max_tracking_error_mrad:{defaultValue:250,min:1,max:250},
  dm_speed_slew_mrad_s2:{defaultValue:2000,min:1,max:10000},
  dm_position_tolerance_mrad:{defaultValue:10,min:1,max:100},
  dm_velocity_tolerance_mrad_s:{defaultValue:80,min:1,max:500},
  dm_settle_ms:{defaultValue:200,min:50,max:1000},
  dm_feedback_timeout_ms:{defaultValue:100,min:50,max:500}
};
for(const [name,expected] of Object.entries(v16Parameters)){
  assert(context.PARAM_META[name],name+' is missing from the host parameter tree');
  for(const [field,value] of Object.entries(expected)){
    assert.strictEqual(context.PARAM_META[name][field],value,
      name+' '+field+' must match firmware v16');
  }
}

const grayWhite=[3253,3217,3189,3316,3151,3011,2802,3188];
const grayBlack=[1010,934,737,2010,1548,1347,753,1362];
for(let index=0;index<8;index++){
  assert.strictEqual(context.PARAM_META['gray_white_'+index].defaultValue,
    grayWhite[index],'gray white defaults must match the commissioned car');
  assert.strictEqual(context.PARAM_META['gray_black_'+index].defaultValue,
    grayBlack[index],'gray black defaults must match the commissioned car');
}
assert.strictEqual(context.PARAM_META.gray_track_mask.defaultValue,0x7E,
  'host default grayscale tracking mask must match firmware migration');

assert(/static const uint16_t kVersion = 16U;/.test(storeSource),
  'firmware ConfigStore version changed');
assert(/static const uint16_t kPayloadLength = 243U;/.test(storeSource),
  'firmware ConfigStore payload length changed');
assert(/len=243/.test(parameterSource),
  'host simulator must report the v16 payload length');

const current={};
for(const name of context.PARAM_ORDER){
  current[name]=context.PARAM_META[name].defaultValue;
}
// The supplied export uses 6000/6500. Importing it over this valid live pair
// would fail if the JSON's release field were sent first.
current.ina_uv_trip_mv=9800;
current.ina_uv_release_mv=10500;
const plan=context.paramPlanImport(current,
  {ina_uv_release_mv:6500,ina_uv_trip_mv:6000},{});
assert.strictEqual(plan.ok,true,plan.error);
assert.deepStrictEqual(Array.from(plan.steps,item=>item.name),
  ['ina_uv_trip_mv','ina_uv_release_mv'],
  'JSON import must arrange coupled parameters in a firmware-safe order');
let intermediate=Object.assign({},current);
for(const step of plan.steps){
  intermediate=context.paramCandidateWithValue(intermediate,step.name,step.value);
  assert.strictEqual(context.paramCandidateError(intermediate,{}),'',
    'every planned param set must satisfy ConfigStore validation');
}
assert.strictEqual(intermediate.ina_uv_trip_mv,6000);
assert.strictEqual(intermediate.ina_uv_release_mv,6500);

const invalidPlan=context.paramPlanImport(current,
  {ina_uv_trip_mv:7000,ina_uv_release_mv:6500},{});
assert.strictEqual(invalidPlan.ok,false,
  'invalid final parameter combinations must be rejected before any write');

const radiusPlan=context.paramPlanImport(current,{wheel_radius_um:33050},{});
assert.strictEqual(radiusPlan.ok,true,radiusPlan.error);
assert.strictEqual(radiusPlan.finalValues.wheel_radius_mm,33,
  'import planner must model ConfigStore wheel-radius synchronization');

console.log('parameter catalog ok: 107 parameters, ConfigStore v16 payload 243');
