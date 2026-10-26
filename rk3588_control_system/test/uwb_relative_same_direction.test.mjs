import test from 'node:test';
import assert from 'node:assert/strict';
import fs from 'node:fs';
import vm from 'node:vm';
import {TraditionalManualPropulsion} from '../backend/traditional_manual_propulsion.js';
import {IntermittentUwbFollow} from '../backend/uwb_target_prediction.js';

function sender(manualLimit=100) {
  const source=fs.readFileSync(new URL('../backend/server.js',import.meta.url),'utf8');
  const functions=[
    source.slice(source.indexOf('function normalizeRoverControl('),source.indexOf('let lastRoverCommandAt')),
    source.slice(source.indexOf('function sendManualRoverOutput('),source.indexOf('function applyRoverControl(')),
    source.slice(source.indexOf('function sendUwbRelativeDrive('),source.indexOf('function holdOutputsAreNeutral('))
  ].join('\n');
  let now=1000;
  const calls=[],p=new TraditionalManualPropulsion({steering_slew_deg_s:9,
    throttle_accel_percent_s:30,throttle_decel_percent_s:45}),wrapper=new IntermittentUwbFollow();
  p.reset(900);
  const box={traditionalManualPropulsion:p,intermittentUwbFollow:wrapper,
    uwbFollowController:{relativeImuControl:{trackApplied(){}}},
    lastRoverCommandAt:0,manualRoverSource:'WEB',PWM_CENTER:1500,PWM_MIN:1000,PWM_MAX:2000,
    ROVER_THROTTLE_MIN:-100,ROVER_THROTTLE_MAX:100,ROVER_MANUAL_THROTTLE_LIMIT:manualLimit,
    ROVER_STEERING_MIN:-45,ROVER_STEERING_MAX:45,ROVER_MANUAL_STEERING_LIMIT:45,
    ROVER_CONTROL_PROTOCOL:'manual_control',ROVER_LEFT_CHANNEL:1,ROVER_RIGHT_CHANNEL:3,
    Date:{now:()=>now},
    systemState:{isConnected:true,telemetry:{flightMode:'MANUAL',armed:true,pilotSteering:{ready:true,type:0}},motorStatus:{}},io:{emit(){}},
    forceNeutralRoverControl(){},
    asFiniteNumber:(v,f)=>Number.isFinite(Number(v))?Number(v):f,
    clamp:(v,a,b)=>Math.max(a,Math.min(b,v)),toPwm:v=>Math.round(Math.max(1000,Math.min(2000,v))),
    sendMavlinkCommand:(command,params)=>calls.push({command,params})};
  const context=vm.createContext(box);vm.runInContext(functions,context);
  return {calls,advance(ms){now+=ms;},run(throttle,steering,extra={}) {
    box.input={relativeImuControl:true,safe:true,throttlePercent:throttle,steeringDegrees:steering,...extra};
    return vm.runInContext('sendUwbRelativeDrive(input)',context);
  }};
}
function forwardOnly(output) {
  const turn=output.steering/45*100;
  assert.ok(output.throttle>=0);
  assert.ok(output.throttle+turn>=-1e-10,JSON.stringify(output));
  assert.ok(output.throttle-turn>=-1e-10,JSON.stringify(output));
  assert.ok(output.leftPwm>=1500 && output.rightPwm>=1500);
}

test('production relative sender prevents the low-throttle/high-steering counter-rotation shown in the screenshot',()=>{
  for(const steering of [-18,18]) {
    const s=sender(),r=s.run(8,steering);forwardOnly(r);
    assert.equal(r.throttle,8,'limiting a turn must not increase mean thrust');
    assert.equal(r.sameDirectionLimited,true);
    assert.equal(r.sameDirectionSteeringLimit,8*.45*.8);
    assert.equal(s.calls[0].command,'ROVER_DRIVE');assert.equal(s.calls[0].params.motorTargetSemantics,true);
  }
});

test('forward, neutral, direction changes and braking never issue opposed motor targets',()=>{
  const s=sender();
  for(let i=0;i<1000;i++) {
    const throttle=i%23===0?0:(i%71)+.01,steering=Math.sin(i*.31)*45;
    const r=s.run(throttle,steering);forwardOnly(r);
    assert.ok(r.throttle<=throttle+1e-10);
    assert.ok(Math.abs(r.steering)<=Math.abs(steering)+1e-10);
  }
  const stopped=s.run(0,18);assert.equal(stopped.throttle,0);assert.equal(stopped.steering,0);
  const brake=s.run(.1,-18);forwardOnly(brake);assert.equal(brake.throttle,.1);
});

test('final guard follows the actual normalized manual limit and the recovery throttle cap',()=>{
  const s=sender(5),r=s.run(70,18);forwardOnly(r);assert.equal(r.throttle,5);
  for(const cap of [0,.1,1,5,15]) {
    const p=sender(),o=p.run(40,-18,{intermittentThrottleCap:cap,intermittentSteeringCap:18});
    forwardOnly(o);assert.ok(o.throttle<=cap);
  }
  for(const fault of [{pausePropulsion:true},{safe:false}]) {
    const o=sender().run(30,-18,fault);assert.equal(o.throttle,0);assert.equal(o.steering,0);
  }
});

test('the shared relative execution guard covers traditional/neural and GPS/no-GPS branches',()=>{
  for(const trackingMethod of ['traditional','neural'])for(const gpsSpeedControlActive of [false,true]) {
    const r=sender().run(5,-18,{trackingMethod,gpsSpeedControlActive});forwardOnly(r);
    assert.equal(r.propulsion,gpsSpeedControlActive?'uwb_gps_speed':'relative_rate_throttle');
  }
  const invalid=sender().run(-5,18);assert.equal(invalid.throttle,0);assert.equal(invalid.steering,0);
});


test('production raw relative sender ramps actual transmitted motor demands and stops immediately',()=>{
  const s=sender();let previous={left:0,right:0};
  for(let i=0;i<60;i++){
    const r=s.run(i<30?60:25,i<20?18:-18,{rawControl:true});forwardOnly(r);
    const turn=r.steering/45*100,next={left:r.throttle+turn,right:r.throttle-turn};
    for(const side of ['left','right']){
      const limit=next[side]<previous[side]?4.5:3;
      assert.ok(Math.abs(next[side]-previous[side])<=limit+1e-8,JSON.stringify({previous,next}));
    }
    assert.equal(s.calls.at(-1).command,'ROVER_DRIVE');
    previous=next;s.advance(100);
  }
  const stop=s.run(0,0,{rawControl:true,reason:'holding_distance'});
  assert.equal(stop.throttle,0);assert.equal(stop.steering,0);
});
