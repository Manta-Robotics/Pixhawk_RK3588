import test from 'node:test';
import assert from 'node:assert/strict';
import { UwbRawControl } from '../backend/uwb_raw_control.js';
import { UwbFollowController } from '../backend/uwb_follow.js';
import { IntermittentUwbFollow } from '../backend/uwb_target_prediction.js';
import { TraditionalManualPropulsion } from '../backend/traditional_manual_propulsion.js';
import { UwbAutoResume } from '../backend/uwb_auto_resume.js';

const config = { execution_mode:'relative_imu', bearing_sign:1, bearing_scale:1, bearing_offset_deg:0,
  following_distance_m:.5, safety_distance_m:.3, hard_stop_enabled:false, controller_profile:'traditional_fast',
  max_speed_mps:2, commissioning_mode:false, relative_throttle_limit_percent:70,
  relative_throttle_accel_percent_s:30, relative_throttle_decel_percent_s:45,
  steering_slew_deg_s:9, relative_nmpc:{enabled:true}, prediction:{enabled:true},
  auto_resume_after_uwb_timeout:true, uwb_timeout_ms:450 };
const sensor = (at,d=3,a=0,extra={}) => ({ receiverId:1,tagId:7,valid:true,
  rawDistanceM:d,rawAzimuthDeg:a,rawElevationDeg:0,receivedMeasurementAt:at,rawDistanceUpdatedAt:at,...extra });
const packet = (...sensors) => ({ online:false,fresh:false,measurementStatus:'suspect',sensors });
const ready = { flightMode:'MANUAL',armed:true,pilotSteering:{ready:true,type:0} };
function telemetry(now,u) { return {...ready, attitude:{yaw:0,yawRateDegS:0,updatedAt:now},
  gps:{fixType:0},velocity:{vx:null,vy:null},uwb:u}; }

test('raw control accepts a complete new coordinate even when diagnostic KF rejects its jump',()=>{
  const input=new UwbRawControl(), controller=new UwbFollowController(config);
  const session=new IntermittentUwbFollow(config.prediction,config);
  for(const [at,d,a] of [[1000,3,0],[1100,12,35],[1200,2,-35]]){
    const p=packet(sensor(at,d,a)), before=structuredClone(p), u=input.view(p,at);
    assert.equal(u.fresh,true);assert.equal(u.distanceM,d);assert.ok(Math.abs(u.azimuthDeg-a)<1e-9);
    assert.deepEqual(p,before);
    const result=session.update({controller,now:at,telemetry:telemetry(at,u),connected:true});
    assert.equal(result.safe,true);assert.equal(result.distanceM,d);
    assert.ok(Math.abs(result.controlBearingDeg-a)<1e-9);
    assert.notEqual(result.planner,'nmpc'); // no motion-fit warmup in raw mode
  }
});

test('partial, replayed, expired, invalid and disabled raw points cannot renew observation time',()=>{
  const input=new UwbRawControl({enabled_receiver_ids:[1],timeout_ms:450});
  const first=input.view(packet(sensor(1000)),1000);
  const partial=sensor(1000,12,70,{rawDistanceUpdatedAt:1100});
  assert.equal(input.view(packet(partial),1100).distanceM,3);
  assert.equal(input.view(packet(sensor(1000,15,80)),1200).distanceM,3);
  assert.equal(input.view(packet(sensor(900,18,90)),1300).measurementUpdatedAt,1000);
  assert.equal(first.measurementUpdatedAt,1000);
  assert.equal(input.view(packet(partial),1451).fresh,false);
  for(const extra of [{valid:false},{rawDistanceM:NaN},{rawAzimuthDeg:181},
    {rawDistanceM:0},{receivedMeasurementAt:3000},{receiverId:2},{rawRejectedAt:2000}])
    assert.equal(new UwbRawControl({enabled_receiver_ids:[1]}).view(packet(sensor(2000,3,0,extra)),2000).fresh,false);
  assert.equal(input.view(packet(sensor(2000)),2000,false).fresh,false);
});

test('coordinate steps and turn reversals keep each motor within acceleration/deceleration limits',()=>{
  const input=new UwbRawControl(), controller=new UwbFollowController(config);
  const session=new IntermittentUwbFollow(config.prediction,config);
  const motor=new TraditionalManualPropulsion({throttle_limit:70,steering_limit:18,
    throttle_accel_percent_s:30,throttle_decel_percent_s:45,steering_slew_deg_s:9});
  motor.reset(900);
  let prior={left:0,right:0},command=0;
  for(let now=1000;now<=9000;now+=100){
    const d=now<3000?3:now<5500?15:2;
    const a=now<3500?30:now<6500?-35:25;
    const t=telemetry(now,input.view(packet(sensor(now,d,a)),now));
    const result=session.update({controller,now,telemetry:t,connected:true});
    assert.equal(result.safe,true);
    const output=motor.update({now,result,telemetry:t,connected:true});
    for(const side of ['left','right']){
      const value=motor.limiter.motors[side], limit=value<prior[side]?4.5:3;
      assert.ok(Math.abs(value-prior[side])<=limit+1e-8,`${side}: ${prior[side]} -> ${value}`);
      assert.ok(value>=0&&value<=70);
    }
    assert.ok(Math.abs(motor.limiter.turnCommand-command)<=.9+1e-9);
    command=motor.limiter.turnCommand;prior={...motor.limiter.motors};
    assert.equal(output.safe,true);
  }
  const partial=telemetry(9050,input.view(packet(sensor(9000,.2,0,{rawDistanceUpdatedAt:9050})),9050));
  const blocked=session.update({controller,now:9050,telemetry:partial,connected:true});
  assert.equal(blocked.pausePropulsion,true);assert.equal(blocked.throttlePercent,0);
  // A safety stop bypasses comfort ramps and clears stored thrust immediately.
  const near=telemetry(9100,input.view(packet(sensor(9100,.2)),9100));
  const stopped=session.update({controller,now:9100,telemetry:near,connected:true});
  const zero=motor.update({now:9100,result:stopped,telemetry:near,connected:true});
  assert.equal(zero.throttle,0);assert.equal(zero.steering,0);
});

test('raw loss stops and one qualified frame restores only an existing follow request',()=>{
  const input=new UwbRawControl(), controller=new UwbFollowController(config);
  const session=new IntermittentUwbFollow(config.prediction,config), resume=new UwbAutoResume(config);
  const t=telemetry(1000,input.view(packet(sensor(1000)),1000));
  resume.started(t,1000);resume.stopped('uwb_stale');
  assert.equal(resume.observe(1100,telemetry(1100,input.view(packet(sensor(1100,9,75)),1100)),{safe:true}),true);
  const lost=telemetry(2000,input.view(packet(sensor(1100)),2000));
  const stopped=session.update({controller,now:2000,telemetry:lost,connected:true});
  assert.equal(stopped.safe,false);assert.equal(stopped.throttlePercent,0);
  const one=telemetry(2025,input.view(packet(sensor(2025,8,-35)),2025));
  assert.equal(session.update({controller,now:2025,telemetry:one,connected:true}).safe,true);
  resume.stopped('manual_stop');assert.equal(resume.observe(2025,one,{safe:true}),false);
  resume.search();resume.stopped('operator_disarm');assert.equal(resume.observe(2025,one,{safe:true}),false);
  one.armed=false;assert.equal(session.update({controller,now:2025,telemetry:one,connected:true}).safe,false);
});
