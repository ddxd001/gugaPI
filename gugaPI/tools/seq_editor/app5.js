'use strict';

// QGroundControl-inspired parameter browser for the gugaPI ConfigStore shell.
// Values sent to the board always remain the raw integers shown by `param get`.

var PARAM_GROUPS=[
  {id:'all',section:'浏览',label:'全部参数'},
  {id:'chassis',section:'底盘',label:'几何与编码器'},
  {id:'motor',section:'底盘',label:'电机方向'},
  {id:'speed',section:'底盘',label:'速度环'},
  {id:'position',section:'底盘',label:'位置环'},
  {id:'distance',section:'底盘',label:'距离速度规划'},
  {id:'gy931',section:'姿态与航向',label:'GY931 零点'},
  {id:'imu',section:'姿态与航向',label:'IMU 偏置'},
  {id:'heading',section:'姿态与航向',label:'航向控制'},
  {id:'dm',section:'执行器',label:'达妙电机'},
  {id:'power',section:'保护',label:'电源保护'},
  {id:'gray_cal',section:'灰度与循迹',label:'灰度标定'},
  {id:'gray_proc',section:'灰度与循迹',label:'灰度判定'},
  {id:'linefollow',section:'灰度与循迹',label:'循迹控制'},
  {id:'other',section:'其他',label:'未分类参数'}
];

var PARAM_META={};
var PARAM_ORDER=[];
var PARAM_EXPORT_BATCH_SIZE=16;
function addParamMeta(name,label,group,def,min,max,unit,desc,restart,kind,preview){
  PARAM_META[name]={name:name,label:label,group:group,defaultValue:def,min:min,max:max,
    unit:unit||'',description:desc||'',restart:!!restart,kind:kind||'number',
    preview:preview||''};
  PARAM_ORDER.push(name);
}

addParamMeta('left_counts_per_rev','左轮编码器每圈计数','chassis',1456,1,100000000,'counts/rev','左轮对应 MotorDriver M2，默认值包含 QEI 四倍频。修改后需重启，使 gugaPI 与 MotorDriver 使用相同计数。',true);
addParamMeta('right_counts_per_rev','右轮编码器每圈计数','chassis',1456,1,100000000,'counts/rev','右轮对应 MotorDriver M1，默认值包含 QEI 四倍频。修改后需重启，使 gugaPI 与 MotorDriver 使用相同计数。',true);
addParamMeta('wheel_radius_um','有效滚动半径','chassis',33050,1000,1000000,'um','实测有效滚动半径，底盘距离换算优先使用该精确值。',false);
addParamMeta('wheel_radius_mm','兼容车轮半径','chassis',33,1,1000,'mm','兼容旧脚本的整数毫米入口；写入后会同步覆盖精确滚动半径。',false);
addParamMeta('wheel_track_mm','轮距','chassis',160,1,2000,'mm','左右轮接地点中心之间的距离，用于角速度换算。',false);
addParamMeta('max_wheel_rpm','最大轮速','chassis',1000,1,1000,'RPM','gugaPI 允许下发的最大轮速限幅。',false);
addParamMeta('motor_output_invert_flags','电机输出方向','motor',3,0,3,'bitmask','位 0 控制右轮 M1，位 1 控制左轮 M2。应与编码器方向保持闭环一致。',true,'flags');
addParamMeta('motor_encoder_invert_flags','编码器反馈方向','motor',1,0,3,'bitmask','位 0 控制右轮 M1，位 1 控制左轮 M2。错误方向可能形成正反馈。',true,'flags');

addParamMeta('speed_kp','速度环 Kp','speed',2,0,255,'Q4.4 raw','MotorDriver 速度 PID 比例增益。',true,'number','q44');
addParamMeta('speed_ki','速度环 Ki','speed',2,0,255,'Q4.4 raw','MotorDriver 速度 PID 积分增益。',true,'number','q44');
addParamMeta('speed_kd','速度环 Kd','speed',0,0,255,'Q4.4 raw','MotorDriver 速度 PID 微分增益。',true,'number','q44');
addParamMeta('speed_max_duty','速度环最大占空比','speed',60,0,100,'%','速度闭环输出上限，必须不小于最小占空比。',true);
addParamMeta('speed_min_duty','速度环最小占空比','speed',4,0,100,'%','克服静摩擦的最小输出，必须不大于最大占空比。',true);
addParamMeta('speed_accel_rpm_s','电机目标加速斜坡','speed',1500,0,65535,'RPM/s','MotorDriver 本地目标转速加速斜坡；0 表示立即跟随。',true);
addParamMeta('speed_decel_rpm_s','电机目标减速斜坡','speed',2000,0,65535,'RPM/s','MotorDriver 本地目标转速减速斜坡；0 表示立即跟随。',true);

addParamMeta('position_kp','位置环 Kp','position',15,0,255,'Q4.4 raw','MotorDriver 位置 PID 比例增益。',true,'number','q44');
addParamMeta('position_ki','位置环 Ki','position',0,0,255,'Q4.4 raw','MotorDriver 位置 PID 积分增益。',true,'number','q44');
addParamMeta('position_kd','位置环 Kd','position',0,0,255,'Q4.4 raw','MotorDriver 位置 PID 微分增益。',true,'number','q44');
addParamMeta('position_max_rpm','位置环最大转速','position',40,0,1000,'RPM','位置控制期间允许的最大轮速。',true);
addParamMeta('position_tolerance_counts','位置到位容差','position',3,0,65535,'counts','位置误差进入该范围时视为接近目标。',true);

addParamMeta('distance_speed_mode','距离速度模式','distance',1,0,1,'enum','0 为恒速，1 为梯形速度规划。',false);
addParamMeta('distance_accel_rpm_s','距离规划加速度','distance',600,1,5000,'RPM/s','距离动作上层基础转速的加速度。',false);
addParamMeta('distance_decel_rpm_s','距离规划减速度','distance',900,1,5000,'RPM/s','距离动作减速度和制动距离模型参数。',false);
addParamMeta('distance_creep_rpm','终点逼近转速','distance',15,1,500,'RPM','接近目标距离时的最低单方向逼近速度。',false);
addParamMeta('distance_stop_latency_ms','停车延迟补偿','distance',360,0,2000,'ms','MotorDriver 和调度链路的停车延迟补偿。',false,'number','ms');
addParamMeta('distance_brake_margin_mm','提前制动余量','distance',5,0,1000,'mm','在模型制动距离之外额外提前制动的距离。',false);
addParamMeta('distance_settle_rpm','停稳转速阈值','distance',3,0,100,'RPM','实际轮速低于该值时允许判定停稳。',false);
addParamMeta('distance_tolerance_mm','距离到位容差','distance',3,1,100,'mm','终点位置误差进入该范围时判定到位。',false);

['roll','pitch','yaw'].forEach(function(axis){
  addParamMeta('gy931_'+axis+'_zero_mdeg','GY931 '+axis.toUpperCase()+' 零点','gy931',0,-180000000,180000000,'mdeg','GY931 '+axis.toUpperCase()+' 方向零点偏置。',true,'number','mdeg');
});
['x','y','z'].forEach(function(axis){
  addParamMeta('imu_accel_bias_'+axis+'_mg','IMU 加速度 '+axis.toUpperCase()+' 偏置','imu',0,-200000,200000,'mg','从 ICM-45686 加速度测量中扣除的偏置。',false);
});
['x','y','z'].forEach(function(axis){
  addParamMeta('imu_gyro_bias_'+axis+'_mdps','IMU 陀螺仪 '+axis.toUpperCase()+' 偏置','imu',0,-2000000,2000000,'mdps','从 ICM-45686 角速度测量中扣除的偏置。',false,'number','mdps');
});

addParamMeta('heading_kp','航向修正增益','heading',1000,0,100000,'scaled','直行保持和转向使用的航向增益；1000 约等于每度误差修正 1 RPM。',false,'number','heading');
addParamMeta('heading_max_correction_rpm','最大航向修正','heading',30,0,500,'RPM','直行航向保持允许施加的最大左右差速。',false);
addParamMeta('heading_turn_max_rpm','转向最大差速修正','heading',60,0,1000,'RPM','原地转向的最大轮速；连续圆弧中也是转向强度达到100%的差速基准。外轮仍按基础速度加修正并受独立上限约束，内轮按强度插值到独立反转上限。',false);
addParamMeta('heading_turn_min_rpm','转向最小轮速','heading',20,0,500,'RPM','接近目标角度时用于克服静摩擦的最小轮速。',false);
addParamMeta('heading_tolerance_mdeg','转向角度容差','heading',3000,0,90000,'mdeg','航向误差小于该值时进入到位判定。',false,'number','mdeg');
addParamMeta('heading_settle_ms','航向稳定时间','heading',300,0,5000,'ms','航向持续处于容差内达到该时间后判定完成。',false,'number','ms');
addParamMeta('heading_turn_brake_ms','转向预测制动时间','heading',60,0,500,'ms','根据朝向目标的陀螺仪角速度预测惯性转角；数值越大，转向时越早停止驱动。修改后下一次转向立即生效。',false,'number','ms');
addParamMeta('heading_turn_brake_margin_mdeg','转向固定制动提前角','heading',500,0,30000,'mdeg','在预测惯性转角之外附加的固定提前制动角；数值越大，越不容易过冲。修改后下一次转向立即生效。',false,'number','mdeg');
addParamMeta('heading_turn_settle_rate_mdps','转向停稳角速度阈值','heading',1500,0,60000,'mdps','陀螺仪 Z 轴角速度绝对值不超过该阈值时，才允许累计转向稳定时间。',false,'number','mdps');
addParamMeta('heading_turn_settle_rpm','转向停稳轮速阈值','heading',3,0,100,'RPM','左右轮实际转速绝对值均不超过该阈值时，才允许累计转向稳定时间。',false);
addParamMeta('heading_lock_kp','静止锁向 Kp','heading',1500,0,100000,'scaled','静止锁向的角度误差比例增益；1500约等于每度误差修正1.5 RPM。',false,'number','heading');
addParamMeta('heading_lock_kd','静止锁向 Kd','heading',250,0,100000,'scaled','使用ICM Z轴角速度抑制回正过冲和来回摆动。',false);
addParamMeta('heading_lock_wake_mdeg','锁向唤醒角度','heading',2000,100,30000,'mdeg','偏离目标达到该角度后唤醒车轮执行原地回正。',false,'number','mdeg');
addParamMeta('heading_lock_settle_mdeg','锁向稳定角度','heading',800,50,29999,'mdeg','回正误差进入该范围后允许停止车轮并开始稳定判定；必须小于唤醒角度。',false,'number','mdeg');
addParamMeta('heading_lock_min_rpm','锁向最小纠偏轮速','heading',10,0,100,'RPM','纠偏方向与误差一致时，用于克服静摩擦的最小轮速。',false);
addParamMeta('heading_lock_max_rpm','锁向最大纠偏轮速','heading',30,1,200,'RPM','静止回正允许使用的最大左右轮转速，且不得超过底盘最大轮速。',false);
addParamMeta('heading_lock_settle_rate_mdps','锁向稳定角速度阈值','heading',1500,0,60000,'mdps','Z轴角速度绝对值低于该值时才允许进入稳定计时。',false,'number','mdps');
addParamMeta('heading_lock_settle_rpm','锁向稳定轮速阈值','heading',3,0,100,'RPM','两轮实际转速均不超过该值时才视为已经停稳。',false);
addParamMeta('heading_lock_settle_ms','锁向稳定时间','heading',250,50,5000,'ms','角度、角速度和轮速持续稳定达到该时间后重新进入锁定等待。',false,'number','ms');
addParamMeta('heading_lock_timeout_ms','锁向回正超时','heading',3000,500,10000,'ms','一次外力扰动回正超过该时间后停止电机并报告局部timeout。',false,'number','ms');

addParamMeta('ina_uv_trip_mv','欠压触发阈值','power',6000,1,25999,'mV','电源电压连续低于该值时触发欠压。',false);
addParamMeta('ina_uv_release_mv','欠压释放阈值','power',6500,2,26000,'mV','必须高于欠压触发阈值。',false);
addParamMeta('ina_oc_trip_ma','过流触发阈值','power',5000,1,6500,'mA','电流连续高于该值时触发过流。',false);
addParamMeta('ina_oc_release_ma','过流释放阈值','power',4500,0,6499,'mA','必须低于过流触发阈值。',false);
addParamMeta('ina_trip_samples','保护触发样本数','power',3,1,100,'samples','连续异常达到该次数后触发保护。',false);
addParamMeta('ina_release_samples','保护释放样本数','power',5,1,100,'samples','连续恢复达到该次数后解除非锁存状态。',false);
addParamMeta('ina_comm_fail_samples','INA219 通信失败次数','power',3,1,100,'samples','连续读取失败达到该次数后报告通信故障。',false);
addParamMeta('ina_latch_faults','锁存电源故障','power',0,0,1,'bool','开启后电源故障保持锁存，需要复位处理。',false,'bool');
addParamMeta('ina_motion_inhibit','电源异常禁止运动','power',0,0,1,'bool','开启后欠压、过流或通信异常会禁止运动。',false,'bool');

var DEFAULT_GRAY_WHITE=[3253,3217,3189,3316,3151,3011,2802,3188];
var DEFAULT_GRAY_BLACK=[1010,934,737,2010,1548,1347,753,1362];
for(var grayIndex=0;grayIndex<8;grayIndex++){
  addParamMeta('gray_white_'+grayIndex,'灰度 '+grayIndex+' 白色标定','gray_cal',DEFAULT_GRAY_WHITE[grayIndex],0,4095,'ADC','第 '+grayIndex+' 路白色表面的原始采样值；不得等于黑色标定值。',true);
}
for(var blackIndex=0;blackIndex<8;blackIndex++){
  addParamMeta('gray_black_'+blackIndex,'灰度 '+blackIndex+' 黑色标定','gray_cal',DEFAULT_GRAY_BLACK[blackIndex],0,4095,'ADC','第 '+blackIndex+' 路黑色表面的原始采样值；不得等于白色标定值。',true);
}
addParamMeta('gray_threshold','灰度判定阈值','gray_proc',500,1,999,'permille','归一化灰度的中心阈值。',true);
addParamMeta('gray_hysteresis','灰度判定回差','gray_proc',300,0,998,'permille','阈值回差，用于抑制边界抖动。',true);
addParamMeta('gray_position_floor','位置计算强度下限','gray_proc',100,0,999,'permille','参与线位置计算的通道最低强度。',true);
addParamMeta('gray_min_strength','最小赛道强度','gray_proc',600,1,8000,'sum','低于该强度时认为赛道信号不足。',true);
addParamMeta('gray_track_mask','循迹通道掩码','gray_proc',126,1,255,'bitmask','8 路灰度中参与循迹计算的通道位掩码。',true,'number','mask');

addParamMeta('lf_kp','循迹 Kp','linefollow',10000,0,1000000,'scaled','线位置误差的比例修正增益。',true);
addParamMeta('lf_kd','循迹 Kd','linefollow',0,0,1000000,'scaled','线位置误差变化率的微分修正增益。',true);
addParamMeta('lf_maxcorr','循迹最大差速修正','linefollow',30,0,500,'RPM','循迹控制允许施加的最大左右差速。',true);
addParamMeta('lf_lost_hold_ms','丢线保持时间','linefollow',150,0,10000,'ms','短时丢线时保持最近修正的时间。',true,'number','ms');
addParamMeta('lf_lost_stop_ms','丢线停车时间','linefollow',500,1,10000,'ms','持续丢线达到该时间后停车；必须不小于保持时间。',true,'number','ms');
addParamMeta('lf_slew_permille_s','循迹修正变化率','linefollow',25000,1,65535,'permille/s','限制左右差速修正的变化速度；数值越大响应越快。',true);
addParamMeta('road_align_distance_mm','路口对齐距离','linefollow',0,0,300,'mm','识别直角弯后继续按编码器前进的距离；该值直接改变实际转弯位置，0表示直接进入滚动圆弧转弯。',false);
addParamMeta('road_align_rpm','路口转弯基础速度','linefollow',30,1,300,'RPM','对齐、圆弧转弯和未确认线路时移动捕线的基础速度上限；实际不超过进入路口时的循迹基础速度。转弯末段会提前确认新线路，到达目标航向后尽快交还循迹并恢复原循迹速度。',false);
addParamMeta('road_turn_outer_max_rpm','路口外轮正转上限','linefollow',220,1,1000,'RPM','自动路口圆弧中外轮沿用基础速度加差速修正，并由该值封顶；增大内轮反转速度不会继续抬高外轮。',false);
addParamMeta('road_turn_inner_reverse_max_rpm','路口内轮最大反转','linefollow',120,0,1000,'RPM','自动路口圆弧满转向时内轮允许达到的反转速度；数值越大转弯半径越小，0表示内轮最多降到停止。',false);

addParamMeta('dm_position_kp_milli','达妙定位 Kp','dm',4000,0,10000,'milli','MIT 定位比例增益，4000 表示 4.000。',false);
addParamMeta('dm_position_kd_milli','达妙定位 Kd','dm',400,0,2000,'milli','MIT 定位微分增益，400 表示 0.400。',false);
addParamMeta('dm_speed_kd_milli','达妙定速 Kd','dm',500,0,2000,'milli','MIT 定速阻尼增益，500 表示 0.500。',false);
addParamMeta('dm_max_velocity_mrad_s','达妙最大轨迹速度','dm',2000,0,20000,'mrad/s','定位轨迹与定速动作允许的最大角速度；0 会禁止新的运动命令。',false);
addParamMeta('dm_max_tracking_error_mrad','达妙最大跟随误差','dm',250,1,250,'mrad','参考位置相对反馈位置的最大超前量。',false);
addParamMeta('dm_speed_slew_mrad_s2','达妙速度斜率','dm',2000,1,10000,'mrad/s²','定速启动和停止时的速度变化率。',false);
addParamMeta('dm_position_tolerance_mrad','达妙位置容差','dm',10,1,100,'mrad','定位完成所需的位置误差阈值。',false);
addParamMeta('dm_velocity_tolerance_mrad_s','达妙速度容差','dm',80,1,500,'mrad/s','定位和停止完成所需的速度阈值；DM-G6220 反馈约以 22 mrad/s 量化，80 可容纳零速附近的 55～77 mrad/s 抖动。',false);
addParamMeta('dm_settle_ms','达妙稳定时间','dm',200,50,1000,'ms','位置和速度持续满足容差后才判定完成。',false);
addParamMeta('dm_feedback_timeout_ms','达妙反馈超时','dm',100,50,500,'ms','活动控制期间反馈超过该时间触发全局 DM TIMEOUT。',false);

function paramHasNumbers(values,names){
  return names.every(function(name){return Number.isFinite(values[name])});
}

function paramCandidateWithValue(values,name,value){
  var candidate=Object.assign({},values);
  candidate[name]=value;
  // Match ConfigStore_Set's paired legacy/precise wheel-radius update.
  if(name==='wheel_radius_mm')candidate.wheel_radius_um=value*1000;
  else if(name==='wheel_radius_um')candidate.wheel_radius_mm=Math.floor(value/1000);
  return candidate;
}

function paramCandidateError(candidate,ranges){
  ranges=ranges||{};
  var names=Object.keys(candidate);
  for(var index=0;index<names.length;index++){
    var name=names[index],meta=PARAM_META[name],range=ranges[name]||meta;
    if(!meta)continue;
    if(!Number.isInteger(candidate[name])||candidate[name]<range.min||
       candidate[name]>range.max)return name+' 超出范围';
  }
  function invalid(names,test,message){
    return paramHasNumbers(candidate,names)&&test()?message:'';
  }
  var error=invalid(['speed_min_duty','speed_max_duty'],function(){return candidate.speed_min_duty>candidate.speed_max_duty},'速度环最小占空比不能大于最大占空比');
  if(error)return error;
  error=invalid(['heading_turn_min_rpm','heading_turn_max_rpm'],function(){return candidate.heading_turn_min_rpm>candidate.heading_turn_max_rpm},'转向最小轮速不能大于最大轮速');
  if(error)return error;
  error=invalid(['heading_lock_settle_mdeg','heading_lock_wake_mdeg'],function(){return candidate.heading_lock_settle_mdeg>=candidate.heading_lock_wake_mdeg},'锁向稳定角必须小于唤醒角');
  if(error)return error;
  error=invalid(['heading_lock_min_rpm','heading_lock_max_rpm'],function(){return candidate.heading_lock_min_rpm>candidate.heading_lock_max_rpm},'锁向最小轮速不能大于最大轮速');
  if(error)return error;
  error=invalid(['heading_lock_max_rpm','max_wheel_rpm'],function(){return candidate.heading_lock_max_rpm>candidate.max_wheel_rpm},'锁向最大轮速不能超过底盘最大轮速');
  if(error)return error;
  error=invalid(['wheel_radius_mm','wheel_radius_um'],function(){return candidate.wheel_radius_mm!==Math.floor(candidate.wheel_radius_um/1000)},'车轮半径毫米值与精确值不一致');
  if(error)return error;
  error=invalid(['distance_creep_rpm','max_wheel_rpm'],function(){return candidate.distance_creep_rpm>candidate.max_wheel_rpm},'终点逼近转速不能超过底盘最大轮速');
  if(error)return error;
  error=invalid(['distance_settle_rpm','distance_creep_rpm'],function(){return candidate.distance_settle_rpm>candidate.distance_creep_rpm},'停稳转速不能超过终点逼近转速');
  if(error)return error;
  error=invalid(['ina_uv_release_mv','ina_uv_trip_mv'],function(){return candidate.ina_uv_release_mv<=candidate.ina_uv_trip_mv},'欠压释放阈值必须高于触发阈值');
  if(error)return error;
  error=invalid(['ina_oc_release_ma','ina_oc_trip_ma'],function(){return candidate.ina_oc_release_ma>=candidate.ina_oc_trip_ma},'过流释放阈值必须低于触发阈值');
  if(error)return error;
  for(var gray=0;gray<8;gray++){
    var white='gray_white_'+gray,black='gray_black_'+gray;
    if(paramHasNumbers(candidate,[white,black])&&candidate[white]===candidate[black])return'灰度 '+gray+' 的黑白标定值不能相等';
  }
  error=invalid(['gray_threshold','gray_hysteresis'],function(){
    var lower=Math.floor(candidate.gray_hysteresis/2),upper=Math.floor((candidate.gray_hysteresis+1)/2);
    return candidate.gray_threshold<=lower||candidate.gray_threshold+upper>=1000;
  },'灰度阈值与回差组合无效');
  if(error)return error;
  return invalid(['lf_lost_stop_ms','lf_lost_hold_ms'],function(){return candidate.lf_lost_stop_ms<candidate.lf_lost_hold_ms},'丢线停车时间不能小于保持时间');
}

function paramPlanImport(currentValues,targetValues,ranges){
  var desired=Object.assign({},currentValues,targetValues);
  var hasMm=Object.prototype.hasOwnProperty.call(targetValues,'wheel_radius_mm');
  var hasUm=Object.prototype.hasOwnProperty.call(targetValues,'wheel_radius_um');
  if(hasMm&&!hasUm)desired.wheel_radius_um=targetValues.wheel_radius_mm*1000;
  if(hasUm&&!hasMm)desired.wheel_radius_mm=Math.floor(targetValues.wheel_radius_um/1000);
  var finalError=paramCandidateError(desired,ranges);
  if(finalError)return{ok:false,error:'文件中的参数组合无效：'+finalError,steps:[]};
  var planned=Object.assign({},currentValues);
  var pending=Object.keys(targetValues).filter(function(name){return planned[name]!==desired[name]});
  var steps=[];
  while(pending.length){
    var selected=-1,next=null;
    for(var index=0;index<pending.length;index++){
      var name=pending[index],candidate=paramCandidateWithValue(planned,name,desired[name]);
      if(!paramCandidateError(candidate,ranges)){selected=index;next=candidate;break}
    }
    if(selected<0)return{ok:false,steps:steps,error:'无法生成安全写入顺序，关联参数：'+pending.slice(0,6).join(', ')};
    var selectedName=pending[selected];
    steps.push({name:selectedName,value:desired[selectedName]});
    planned=next;
    pending=pending.filter(function(name){return planned[name]!==desired[name]});
  }
  var mismatch=Object.keys(desired).find(function(name){return planned[name]!==desired[name]});
  return mismatch?{ok:false,steps:steps,error:'规划结果未达到目标参数：'+mismatch}:{ok:true,steps:steps,finalValues:planned};
}

var paramPageState={
  values:{},ranges:{},selected:null,group:'all',query:'',modifiedOnly:false,
  store:null,mode:'unknown',loaded:false,busy:false,connected:false,
  sessionChanged:{},restartPending:false,pollTimer:null,progress:''
};

function paramConnected(){return !!writer||simMode}
function paramEscape(text){return String(text).replace(/[&<>"']/g,function(c){return{'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;',"'":'&#39;'}[c]})}
function paramMeta(name){
  if(PARAM_META[name])return PARAM_META[name];
  return{name:name,label:name,group:'other',defaultValue:null,min:null,max:null,unit:'',description:'固件返回的新参数，前端尚未提供说明。',restart:true,kind:'number',preview:''};
}
function paramIsModified(name){
  var meta=paramMeta(name);
  return meta.defaultValue!==null&&paramPageState.values[name]!==meta.defaultValue;
}
function paramPreview(meta,value){
  if(!Number.isInteger(value))return'';
  if(meta.preview==='q44')return'实际系数 '+(value/16).toFixed(4);
  if(meta.preview==='mdeg')return(value/1000).toFixed(3)+'°';
  if(meta.preview==='mdps')return(value/1000).toFixed(3)+' °/s';
  if(meta.preview==='ms')return(value/1000).toFixed(3)+' s';
  if(meta.preview==='heading')return'约 '+(value/1000).toFixed(3)+' RPM/deg';
  if(meta.preview==='mask')return'0x'+value.toString(16).toUpperCase().padStart(2,'0');
  if(meta.kind==='flags'){
    var labels=[];
    if(value&1)labels.push('右轮 M1');
    if(value&2)labels.push('左轮 M2');
    return labels.length?labels.join('、')+' 反向':'均为正向';
  }
  return meta.unit?value+' '+meta.unit:'';
}
function paramToast(message,type){
  var toast=$('paramToast');
  toast.textContent=message;
  toast.className='show '+(type||'');
  clearTimeout(paramToast.timer);
  paramToast.timer=setTimeout(function(){toast.className=''},3200);
}
function paramParseValues(text){
  var values={},ranges={},re=/param ([a-zA-Z0-9_]+)=(-?\d+) range=(-?\d+)\.\.(-?\d+)/g,m;
  while((m=re.exec(text))!==null){
    values[m[1]]=Number(m[2]);
    ranges[m[1]]={min:Number(m[3]),max:Number(m[4])};
  }
  return{values:values,ranges:ranges};
}
function paramParseExportHeader(text){
  var m=text.match(/param export start=(\d+) count=(\d+) total=(\d+)/);
  return m?{start:Number(m[1]),count:Number(m[2]),total:Number(m[3])}:null;
}
function paramParseStatus(text){
  var m=text.match(/param loaded=(\d+) dirty=(\d+) len=(\d+) crc=(0x[0-9A-Fa-f]+) load=([a-z-]+) save=([a-z-]+)/);
  return m?{loaded:m[1]==='1',dirty:m[2]==='1',len:Number(m[3]),crc:m[4],load:m[5],save:m[6]}:null;
}
function paramParseMode(text){
  var m=text.match(/comp mode=([a-z-]+)/);
  return m?m[1]:'unknown';
}
function paramResponseOk(text,prefix){return text.indexOf(prefix+': ok')>=0}
async function paramRequireWritable(){
  var response=await send('comp status',{timeoutMs:2500});
  paramPageState.mode=paramParseMode(response);
  paramUpdateControls();
  if(paramPageState.mode==='running')throw new Error('任务正在 RUNNING，参数写入已锁定');
}

function paramVisibleNames(){
  var query=paramPageState.query.toLowerCase();
  return Object.keys(paramPageState.values).filter(function(name){
    var meta=paramMeta(name);
    if(paramPageState.group!=='all'&&meta.group!==paramPageState.group)return false;
    if(paramPageState.modifiedOnly&&!paramIsModified(name))return false;
    if(query&&[name,meta.label,meta.description,meta.unit].join(' ').toLowerCase().indexOf(query)<0)return false;
    return true;
  }).sort(function(a,b){
    var ai=PARAM_ORDER.indexOf(a),bi=PARAM_ORDER.indexOf(b);
    if(ai<0)ai=9999;if(bi<0)bi=9999;
    return ai-bi||a.localeCompare(b);
  });
}
function paramRenderGroups(){
  var counts={all:Object.keys(paramPageState.values).length};
  Object.keys(paramPageState.values).forEach(function(name){
    var group=paramMeta(name).group;
    counts[group]=(counts[group]||0)+1;
  });
  var html='',lastSection='';
  PARAM_GROUPS.forEach(function(group){
    if(group.id!=='all'&&!counts[group.id])return;
    if(group.section!==lastSection){html+='<div class="param-group-section">'+paramEscape(group.section)+'</div>';lastSection=group.section}
    html+='<button class="param-group'+(paramPageState.group===group.id?' active':'')+'" data-param-group="'+group.id+'" type="button"><span>'+paramEscape(group.label)+'</span><span class="count">'+(counts[group.id]||0)+'</span></button>';
  });
  $('paramGroups').innerHTML=html;
  document.querySelectorAll('[data-param-group]').forEach(function(button){
    button.onclick=function(){paramPageState.group=this.dataset.paramGroup;paramRenderGroups();paramRenderRows()};
  });
}
function paramRenderRows(){
  var names=paramVisibleNames(),html='';
  names.forEach(function(name){
    var meta=paramMeta(name),value=paramPageState.values[name];
    var badges='';
    if(paramIsModified(name))badges+='<span class="param-badge modified">非默认</span>';
    badges+='<span class="param-badge '+(meta.restart?'reboot':'live')+'">'+(meta.restart?'需重启':'实时')+'</span>';
    html+='<button class="param-row'+(paramPageState.selected===name?' active':'')+'" data-param-name="'+paramEscape(name)+'" type="button"><span class="param-row-name"><span class="param-row-label">'+paramEscape(meta.label)+'</span><span class="param-row-raw">'+paramEscape(name)+'</span></span><span class="param-row-value">'+value+'</span><span class="param-row-badges">'+badges+'</span></button>';
  });
  $('paramRows').innerHTML=html;
  $('paramEmpty').hidden=names.length!==0;
  document.querySelectorAll('[data-param-name]').forEach(function(button){
    button.onclick=function(){paramPageState.selected=this.dataset.paramName;paramRenderRows();paramRenderDetails()};
  });
  if(names.length&&(!paramPageState.selected||names.indexOf(paramPageState.selected)<0)){
    paramPageState.selected=names[0];
    paramRenderRows();
    return;
  }
  paramRenderDetails();
}
function paramRenderDetails(){
  var name=paramPageState.selected;
  if(!name||!Object.prototype.hasOwnProperty.call(paramPageState.values,name)){
    $('paramDetails').innerHTML='<div style="color:#758094;padding:30px 5px">选择一个参数查看详情</div>';
    return;
  }
  var meta=paramMeta(name),value=paramPageState.values[name],range=paramPageState.ranges[name]||{min:meta.min,max:meta.max};
  var editor='';
  if(meta.kind==='flags'){
    editor='<div class="param-switch-row"><span>右轮 M1 反向</span><input id="paramFlagM1" type="checkbox"'+((value&1)?' checked':'')+'></div>'+
      '<div class="param-switch-row"><span>左轮 M2 反向</span><input id="paramFlagM2" type="checkbox"'+((value&2)?' checked':'')+'></div>';
  }else if(meta.kind==='bool'){
    editor='<div class="param-switch-row"><span>启用</span><input id="paramBoolValue" type="checkbox"'+(value?' checked':'')+'></div>';
  }else{
    editor='<label class="edit-label" for="paramEditValue">原始整数值</label><input id="paramEditValue" type="number" step="1" min="'+range.min+'" max="'+range.max+'" value="'+value+'">';
  }
  var defaultText=meta.defaultValue===null?'未知':String(meta.defaultValue);
  $('paramDetails').innerHTML=
    '<div class="param-detail-kicker">'+paramEscape((PARAM_GROUPS.find(function(g){return g.id===meta.group})||{label:'其他'}).label)+'</div>'+
    '<h2 class="param-detail-title">'+paramEscape(meta.label)+'</h2><div class="param-detail-raw">'+paramEscape(name)+'</div>'+
    '<p class="param-detail-desc">'+paramEscape(meta.description)+'</p>'+
    '<div class="param-facts"><div class="param-fact"><span>当前值</span><strong>'+value+'</strong></div><div class="param-fact"><span>默认值</span><strong>'+defaultText+'</strong></div><div class="param-fact"><span>合法范围</span><strong>'+range.min+' .. '+range.max+'</strong></div><div class="param-fact"><span>单位 / 生效</span><strong>'+paramEscape(meta.unit||'—')+' · '+(meta.restart?'重启后':'实时')+'</strong></div></div>'+
    '<div class="param-edit-card">'+editor+'<div id="paramValuePreview" class="param-preview">'+paramEscape(paramPreview(meta,value))+'</div>'+
    '<div class="param-detail-actions"><button id="btnParamUseDefault" type="button"'+(meta.defaultValue===null?' disabled':'')+'>填入默认值</button><button id="btnParamApply" class="primary" type="button">应用到 RAM</button></div></div>';
  var numberInput=$('paramEditValue');
  if(numberInput)numberInput.oninput=function(){var n=Number(this.value);$('paramValuePreview').textContent=Number.isInteger(n)?paramPreview(meta,n):'请输入整数'};
  $('btnParamUseDefault').onclick=function(){
    if(meta.kind==='flags'){
      $('paramFlagM1').checked=!!(meta.defaultValue&1);$('paramFlagM2').checked=!!(meta.defaultValue&2);
    }else if(meta.kind==='bool')$('paramBoolValue').checked=!!meta.defaultValue;
    else{$('paramEditValue').value=meta.defaultValue;$('paramEditValue').dispatchEvent(new Event('input'))}
  };
  $('btnParamApply').onclick=paramApplySelected;
  paramUpdateControls();
}
function paramCurrentEditorValue(meta){
  if(meta.kind==='flags')return($('paramFlagM1').checked?1:0)|($('paramFlagM2').checked?2:0);
  if(meta.kind==='bool')return $('paramBoolValue').checked?1:0;
  return Number($('paramEditValue').value);
}
function paramUpdateControls(){
  paramPageState.connected=paramConnected();
  var running=paramPageState.mode==='running',locked=!paramPageState.connected||paramPageState.busy||running;
  ['btnParamImport','btnParamLoad','btnParamReset','btnParamSave','btnParamReboot'].forEach(function(id){$(id).disabled=locked});
  $('btnParamRefresh').disabled=!paramPageState.connected||paramPageState.busy;
  $('btnParamExport').disabled=!paramPageState.loaded||paramPageState.busy;
  ['btnParamApply','btnParamUseDefault','paramEditValue','paramFlagM1','paramFlagM2','paramBoolValue'].forEach(function(id){var el=$(id);if(el)el.disabled=locked||(id==='btnParamUseDefault'&&paramMeta(paramPageState.selected).defaultValue===null)});
  var dot=$('paramStatusDot'),text=$('paramStatusText');
  dot.className='param-status-dot';
  if(!paramPageState.connected){text.textContent='未连接';}
  else if(paramPageState.busy&&paramPageState.progress){dot.classList.add('warn');text.textContent=paramPageState.progress;}
  else if(!paramPageState.loaded){dot.classList.add('warn');text.textContent='已连接 · 未读取';}
  else{
    dot.classList.add(paramPageState.store&&paramPageState.store.dirty?'warn':'ok');
    text.textContent=paramPageState.mode+(paramPageState.store&&paramPageState.store.dirty?' · RAM 未保存':' · FRAM 已同步');
  }
  var notice=$('paramNotice');
  if(running){notice.textContent='任务正在 RUNNING：参数页已锁定，只允许查看。';notice.className='show'}
  else if(paramPageState.restartPending){notice.textContent='部分参数需要重启后生效。请先保存到 FRAM，再重启设备。';notice.className='show'}
  else{notice.textContent='';notice.className=''}
}
function paramRenderAll(){paramRenderGroups();paramRenderRows();paramUpdateControls()}

async function paramReadExportPages(){
  var values={},ranges={},start=0,total=null;
  while(total===null||start<total){
    paramPageState.progress='批量读取参数 '+start+(total===null?'':' / '+total);
    paramUpdateControls();
    var response=await send('param export '+start+' '+PARAM_EXPORT_BATCH_SIZE,{timeoutMs:2500});
    var header=paramParseExportHeader(response);
    if(!header){
      if(start===0)return null;
      throw new Error('批量参数响应中断');
    }
    if(header.start!==start||header.count>PARAM_EXPORT_BATCH_SIZE||header.total<header.start+header.count||(total!==null&&header.total!==total)||(header.count===0&&header.start<header.total)){
      throw new Error('批量参数响应无效');
    }
    var parsed=paramParseValues(response),names=Object.keys(parsed.values);
    if(names.length!==header.count)throw new Error('批量参数响应不完整');
    names.forEach(function(name){values[name]=parsed.values[name];ranges[name]=parsed.ranges[name]});
    total=header.total;
    if(header.count===0)break;
    start+=header.count;
  }
  return{values:values,ranges:ranges};
}

async function paramReadLegacy(){
  var values={},ranges={};
  for(var index=0;index<PARAM_ORDER.length;index++){
    var name=PARAM_ORDER[index];
    paramPageState.progress='兼容读取参数 '+(index+1)+' / '+PARAM_ORDER.length;
    paramUpdateControls();
    var response=await send('param get '+name,{timeoutMs:2500});
    var parsed=paramParseValues(response);
    if(Object.prototype.hasOwnProperty.call(parsed.values,name)){
      values[name]=parsed.values[name];
      ranges[name]=parsed.ranges[name];
    }
  }
  return{values:values,ranges:ranges};
}

async function paramRefresh(){
  if(!paramConnected()||paramPageState.busy)return;
  paramPageState.busy=true;paramPageState.progress='读取运行状态';paramUpdateControls();
  try{
    var modeText=await send('comp status',{timeoutMs:2500});
    paramPageState.mode=paramParseMode(modeText);
    paramPageState.progress='读取存储状态';paramUpdateControls();
    var statusText=await send('param status',{timeoutMs:2500});
    var store=paramParseStatus(statusText);
    var loaded=await paramReadExportPages();
    if(!loaded)loaded=await paramReadLegacy();
    var values=loaded.values,ranges=loaded.ranges;
    var missing=PARAM_ORDER.filter(function(name){return !Object.prototype.hasOwnProperty.call(values,name)});
    if(Object.keys(values).length===0)throw new Error('没有收到参数');
    paramPageState.values=values;
    paramPageState.ranges=ranges;
    paramPageState.store=store;
    paramPageState.loaded=true;
    if(!paramPageState.selected||!Object.prototype.hasOwnProperty.call(values,paramPageState.selected))paramPageState.selected=Object.keys(values)[0];
    paramRenderAll();
    if(missing.length)paramToast('已读取 '+Object.keys(values).length+' 项；当前固件缺少 '+missing.length+' 项','error');
  }catch(error){paramToast('参数刷新失败：'+error.message,'error')}
  finally{paramPageState.busy=false;paramPageState.progress='';paramUpdateControls()}
}
async function paramPollMode(){
  if(currentView!=='parameters'||!paramConnected()||paramPageState.busy)return;
  try{
    var response=await send('comp status',{timeoutMs:2200});
    paramPageState.mode=paramParseMode(response);
    paramUpdateControls();
  }catch(error){}
}
async function paramRefreshOne(name){
  var response=await send('param get '+name,{timeoutMs:2500});
  var parsed=paramParseValues(response);
  if(!Object.prototype.hasOwnProperty.call(parsed.values,name))throw new Error('未读回 '+name);
  paramPageState.values[name]=parsed.values[name];
  paramPageState.ranges[name]=parsed.ranges[name];
  var status=paramParseStatus(await send('param status',{timeoutMs:2500}));
  if(status)paramPageState.store=status;
}
async function paramApplySelected(){
  var name=paramPageState.selected,meta=paramMeta(name),range=paramPageState.ranges[name]||{min:meta.min,max:meta.max};
  if(paramPageState.mode==='running'){paramToast('RUNNING 状态禁止修改参数','error');return}
  var value=paramCurrentEditorValue(meta);
  if(!Number.isInteger(value)||value<range.min||value>range.max){paramToast('请输入范围 '+range.min+' .. '+range.max+' 内的整数','error');return}
  if(value===paramPageState.values[name]){paramToast('参数值没有变化');return}
  paramPageState.busy=true;paramUpdateControls();
  try{
    await paramRequireWritable();
    var response=await send('param set '+name+' '+value,{timeoutMs:2800});
    if(!paramResponseOk(response,'param set'))throw new Error('固件拒绝该值，请检查关联参数约束');
    await paramRefreshOne(name);
    paramPageState.sessionChanged[name]=true;
    if(meta.restart)paramPageState.restartPending=true;
    paramRenderAll();
    paramToast(meta.restart?'已写入 RAM，重启后生效':'已写入 RAM','ok');
  }catch(error){paramToast('应用失败：'+error.message,'error')}
  finally{paramPageState.busy=false;paramUpdateControls()}
}
async function paramSave(){
  paramPageState.busy=true;paramUpdateControls();
  try{
    await paramRequireWritable();
    var response=await send('param save',{timeoutMs:5000});
    if(!paramResponseOk(response,'param save'))throw new Error('FRAM 保存失败');
    var status=paramParseStatus(await send('param status',{timeoutMs:2500}));
    if(status)paramPageState.store=status;
    paramPageState.sessionChanged={};
    paramToast('参数已保存到 FRAM','ok');
  }catch(error){paramToast(error.message,'error')}
  finally{paramPageState.busy=false;paramUpdateControls()}
}
async function paramLoad(){
  var count=Object.keys(paramPageState.sessionChanged).length;
  if(!window.confirm('将从 FRAM 重新加载参数，并丢弃当前 RAM 修改。\\n本次会话已修改 '+count+' 项。\\n\\n确认继续？'))return;
  paramPageState.busy=true;paramUpdateControls();
  try{
    await paramRequireWritable();
    var response=await send('param load',{timeoutMs:5000});
    if(!paramResponseOk(response,'param load'))throw new Error('FRAM 重新加载失败');
    paramPageState.sessionChanged={};paramPageState.restartPending=true;
    paramPageState.busy=false;await paramRefresh();
    paramToast('已从 FRAM 重新加载；缓存型参数需重启','ok');
  }catch(error){paramToast(error.message,'error')}
  finally{paramPageState.busy=false;paramUpdateControls()}
}
async function paramReset(){
  var count=Object.keys(paramPageState.values).filter(paramIsModified).length;
  if(!window.confirm('将把 RAM 中的参数恢复为源码默认值。\\n预计改变 '+count+' 项，不会自动保存到 FRAM。\\n\\n确认继续？'))return;
  paramPageState.busy=true;paramUpdateControls();
  try{
    await paramRequireWritable();
    var response=await send('param reset',{timeoutMs:3000});
    if(!paramResponseOk(response,'param reset'))throw new Error('恢复默认失败');
    paramPageState.restartPending=true;
    Object.keys(paramPageState.values).forEach(function(name){paramPageState.sessionChanged[name]=true});
    paramPageState.busy=false;await paramRefresh();
    paramToast('已恢复默认值到 RAM，尚未保存','ok');
  }catch(error){paramToast(error.message,'error')}
  finally{paramPageState.busy=false;paramUpdateControls()}
}
function paramExport(){
  if(!paramPageState.loaded)return;
  var payload={format:'gugapi-parameters',version:1,exportedAt:new Date().toISOString(),parameters:{}};
  Object.keys(paramPageState.values).sort().forEach(function(name){payload.parameters[name]=paramPageState.values[name]});
  var blob=new Blob([JSON.stringify(payload,null,2)+'\n'],{type:'application/json'});
  var link=document.createElement('a');link.href=URL.createObjectURL(blob);
  link.download='gugapi-parameters-'+new Date().toISOString().replace(/[:.]/g,'-')+'.json';
  document.body.appendChild(link);link.click();link.remove();setTimeout(function(){URL.revokeObjectURL(link.href)},0);
  paramToast('参数文件已导出','ok');
}
async function paramImportFile(file){
  if(!file||paramPageState.mode==='running')return;
  try{
    var payload=JSON.parse(await file.text());
    if(!payload||payload.format!=='gugapi-parameters'||payload.version!==1||!payload.parameters||Array.isArray(payload.parameters))throw new Error('不是受支持的 gugaPI 参数文件');
    var targetValues={},unknown=[],invalid=[];
    Object.keys(payload.parameters).forEach(function(name){
      var value=payload.parameters[name];
      if(!Object.prototype.hasOwnProperty.call(paramPageState.values,name)){unknown.push(name);return}
      var range=paramPageState.ranges[name];
      if(!Number.isInteger(value)||value<range.min||value>range.max){invalid.push(name);return}
      targetValues[name]=value;
    });
    if(invalid.length)throw new Error('存在 '+invalid.length+' 个非整数或越界参数：'+invalid.slice(0,5).join(', '));
    var plan=paramPlanImport(paramPageState.values,targetValues,paramPageState.ranges);
    if(!plan.ok)throw new Error(plan.error);
    if(!plan.steps.length){paramToast('文件中没有需要修改的已知参数');return}
    if(!window.confirm('导入差异预览\\n将修改：'+plan.steps.length+' 项\\n未知并忽略：'+unknown.length+' 项\\n非法：0 项\\n\\n已自动安排关联参数的安全写入顺序。\\n修改只写入 RAM，不自动保存。确认继续？'))return;
    paramPageState.busy=true;paramUpdateControls();
    await paramRequireWritable();
    var applied=0,failed=null;
    for(var i=0;i<plan.steps.length;i++){
      var item=plan.steps[i],response=await send('param set '+item.name+' '+item.value,{timeoutMs:2800});
      if(!paramResponseOk(response,'param set')){failed=item.name;break}
      applied++;paramPageState.sessionChanged[item.name]=true;
      if(paramMeta(item.name).restart)paramPageState.restartPending=true;
    }
    paramPageState.busy=false;await paramRefresh();
    if(failed)paramToast('已应用 '+applied+' 项，固件在 '+failed+' 拒绝后停止导入','error');
    else paramToast('已导入 '+applied+' 项到 RAM，请检查后保存','ok');
  }catch(error){paramToast('导入失败：'+error.message,'error')}
  finally{paramPageState.busy=false;$('paramFileInput').value='';paramUpdateControls()}
}
async function paramReboot(){
  var dirty=paramPageState.store&&paramPageState.store.dirty;
  var message=dirty?'RAM 中仍有未保存修改，重启会丢失这些修改。\\n':'';
  message+='设备将立即复位，电机输出会停止。\\n\\n确认重启？';
  if(!window.confirm(message))return;
  paramPageState.busy=true;paramUpdateControls();
  try{
    await paramRequireWritable();
    await send('reset',{timeoutMs:900});
    await new Promise(function(resolve){setTimeout(resolve,1800)});
    paramPageState.loaded=false;paramPageState.restartPending=false;paramPageState.sessionChanged={};
    paramPageState.busy=false;await paramRefresh();
    paramToast('设备已重启并重新读取参数','ok');
  }catch(error){paramToast('重启后重新连接失败：'+error.message,'error')}
  finally{paramPageState.busy=false;paramUpdateControls()}
}

function ParamPage_OnShow(){
  if(paramPageState.pollTimer)clearInterval(paramPageState.pollTimer);
  paramPageState.pollTimer=setInterval(paramPollMode,2000);
  if(paramConnected()&&!paramPageState.loaded)paramRefresh();
  else paramUpdateControls();
}
function ParamPage_OnHide(){
  if(paramPageState.pollTimer){clearInterval(paramPageState.pollTimer);paramPageState.pollTimer=null}
}

$('paramSearch').oninput=function(){paramPageState.query=this.value.trim();paramRenderRows()};
$('paramModifiedOnly').onchange=function(){paramPageState.modifiedOnly=this.checked;paramRenderRows()};
$('btnParamRefresh').onclick=paramRefresh;
$('btnParamSave').onclick=paramSave;
$('btnParamLoad').onclick=paramLoad;
$('btnParamReset').onclick=paramReset;
$('btnParamExport').onclick=paramExport;
$('btnParamImport').onclick=function(){$('paramFileInput').click()};
$('paramFileInput').onchange=function(){paramImportFile(this.files[0])};
$('btnParamReboot').onclick=paramReboot;
$('btnRefresh').onclick=function(){if(currentView==='parameters')paramRefresh();else refreshSlots()};

var previousSerialStateHandler=onSerialStateChange;
onSerialStateChange=function(connected){
  if(typeof previousSerialStateHandler==='function')previousSerialStateHandler(connected);
  paramPageState.connected=connected;
  if(!connected){paramPageState.loaded=false;ParamPage_OnHide()}
  paramUpdateControls();
};

// Parameter simulation hooks into app1.js without changing the real shell path.
var simParamValues=null,simPersistedValues=null,simParamDirty=false;
function paramSimInit(){
  if(simParamValues)return;
  simParamValues={};
  PARAM_ORDER.forEach(function(name){simParamValues[name]=PARAM_META[name].defaultValue});
  simPersistedValues=Object.assign({},simParamValues);
}
function paramSimValid(candidate){
  return !paramCandidateError(candidate);
}
function paramSimCommand(cmd){
  paramSimInit();
  if(cmd==='comp status')return'comp mode=dev-running slot=0 valid=1 any_valid=1 count=5 step=0 result=none last=ok\r\n> ';
  if(cmd==='reset'){simParamValues=Object.assign({},simPersistedValues);simParamDirty=false;return'resetting...\r\n> '}
  if(cmd==='param status')return'param loaded=1 dirty='+(simParamDirty?1:0)+' len=243 crc=0x5C758F1C load=ok save=ok\r\n> ';
  if(cmd==='param export'||cmd.startsWith('param export ')){
    var exportParts=cmd.split(/\s+/),start=exportParts.length>=3?Number(exportParts[2]):0;
    var requested=exportParts.length>=4?Number(exportParts[3]):PARAM_EXPORT_BATCH_SIZE;
    if(!Number.isInteger(start)||!Number.isInteger(requested)||start<0||start>PARAM_ORDER.length||requested<1||requested>PARAM_EXPORT_BATCH_SIZE)return'usage: param export [start [count 1..16]]\r\n> ';
    var count=Math.min(requested,PARAM_ORDER.length-start),batch='param export start='+start+' count='+count+' total='+PARAM_ORDER.length+'\r\n';
    for(var exportIndex=start;exportIndex<start+count;exportIndex++){
      var exportName=PARAM_ORDER[exportIndex],exportMeta=PARAM_META[exportName];
      batch+='param '+exportName+'='+simParamValues[exportName]+' range='+exportMeta.min+'..'+exportMeta.max+'\r\n';
    }
    return batch+'> ';
  }
  if(cmd==='param get'||cmd==='param get '){
    var all='';
    PARAM_ORDER.forEach(function(name){var meta=PARAM_META[name];all+='param '+name+'='+simParamValues[name]+' range='+meta.min+'..'+meta.max+'\r\n'});
    return all+'> ';
  }
  if(cmd.startsWith('param get ')){
    var getName=cmd.slice(10).trim(),getMeta=PARAM_META[getName];
    return getMeta?'param '+getName+'='+simParamValues[getName]+' range='+getMeta.min+'..'+getMeta.max+'\r\n> ':'param: unknown\r\n> ';
  }
  if(cmd.startsWith('param set ')){
    var parts=cmd.split(/\s+/),name=parts[2],value=Number(parts[3]),meta=PARAM_META[name];
    if(!meta||!Number.isInteger(value)||value<meta.min||value>meta.max)return'param set: invalid-arg\r\n> ';
    var candidate=paramCandidateWithValue(simParamValues,name,value);
    if(!paramSimValid(candidate))return'param set: invalid-arg\r\n> ';
    simParamValues=candidate;simParamDirty=true;return'param set: ok\r\n> ';
  }
  if(cmd==='param save'){simPersistedValues=Object.assign({},simParamValues);simParamDirty=false;return'param save: ok\r\n> '}
  if(cmd==='param load'){simParamValues=Object.assign({},simPersistedValues);simParamDirty=false;return'param load: ok\r\n> '}
  if(cmd==='param reset'){
    PARAM_ORDER.forEach(function(name){simParamValues[name]=PARAM_META[name].defaultValue});
    simParamDirty=true;return'param reset: ok\r\n> ';
  }
  return'usage: param status|get|set|save|load|reset\r\n> ';
}

paramRenderAll();
