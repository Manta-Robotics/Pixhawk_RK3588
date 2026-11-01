import {UwbPathFollowController} from './uwb_path_follow.js';

const finite=Number.isFinite;
const wrap=v=>((v+180)%360+360)%360-180;
export const uwbPathIdentity=u=>JSON.stringify([u?.streamId??null,u?.activeReceiverIds||[],
  (u?.sensors||[]).map(s=>[s.receiverId,s.tagId])]);

// Only the public smoothed observation enters the route planner. Body geometry
// is rebuilt from one navigation pose; points from different boat poses are
// never plotted together as though they shared an origin.
export class SmoothedUwbPath {
  constructor(config={}) {this.config=config;this.planner=new UwbPathFollowController(config);this.identity=null;}
  update(now,t={}) {
    const u=t.uwb||{},c=this.config,p=this.planner,identity=uwbPathIdentity(u);
    if(identity!==this.identity){p.reset();this.identity=identity;}
    const base={enabled:p.options.enabled,coordinateSource:'smoothed',frame:'body_forward_right',
      evaluatedAt:now,inputUpdatedAt:u.measurementUpdatedAt??null,targetKey:identity,
      inputAgeMs:finite(u.measurementUpdatedAt)?Math.max(0,now-u.measurementUpdatedAt):null};
    const admitted=u.coordinateSource==='smoothed'&&u.smoothing?.ready===true
      &&u.online===true&&u.fresh===true&&u.measurementStatus==='trusted'
      &&finite(u.measurementUpdatedAt)&&u.measurementUpdatedAt>0&&u.measurementUpdatedAt<=now
      &&now-u.measurementUpdatedAt<=(c.uwbTimeoutMs??450)
      &&[u.relativeXM,u.relativeYM,u.distanceM,u.azimuthDeg].every(finite);
    if(!p.options.enabled||!admitted){p.reset();return {...base,active:false,reason:!p.options.enabled?'disabled':'smoothed_uwb_unavailable',pointCount:0,bodyPath:[],bodyObservations:[]};}
    const bearing=wrap(u.azimuthDeg*(c.bearingSign??1)*(c.bearingScale??1)+(c.bearingOffsetDeg??0));
    const result=p.update({now,telemetry:t,distanceM:u.distanceM,bearingDeg:bearing});
    if(!p.navigation||!finite(t.attitude?.yaw))return {...base,...result,bodyPath:[],bodyObservations:[]};
    const dt=(now-p.navigation.at)/1000;
    const boat={x:p.navigation.x+t.velocity.vx*dt,y:p.navigation.y+t.velocity.vy*dt};
    const yaw=(t.attitude.yaw+t.attitude.yawRateDegS*(now-t.attitude.updatedAt)/1000)*Math.PI/180;
    const body=a=>{const x=a.x-boat.x,y=a.y-boat.y;return {x:x*Math.cos(yaw)+y*Math.sin(yaw),y:-x*Math.sin(yaw)+y*Math.cos(yaw)};};
    const thin=arr=>arr.filter((_,i)=>i===arr.length-1||i%Math.max(1,Math.ceil(arr.length/63))===0).map(body);
    return {...base,...result,bodyPath:thin(p.curve),bodyObservations:thin(p.points),
      bodyLookahead:result.active?body(result.lookaheadPointNedM):null,
      bodyTarget:p.filtered?body(p.filtered):null};
  }
}

export function usableUwbPath(reference,telemetry,now,config={}) {
  const u=telemetry.uwb||{};
  const fresh=at=>finite(at)&&at>0&&at<=now&&now-at<=(config.navigationTimeoutMs??700);
  const navigation=telemetry.gps?.fixType>=(config.minimumGpsFixType??3)&&telemetry.ekf?.healthy===true
    &&[telemetry.gps?.updatedAt,telemetry.ekf?.updatedAt,telemetry.position?.updatedAt,
      telemetry.attitude?.updatedAt,telemetry.velocity?.updatedAt].every(fresh)
    &&[telemetry.velocity?.vx,telemetry.velocity?.vy,telemetry.attitude?.yaw].every(finite);
  return navigation&&reference?.active===true&&reference.coordinateSource==='smoothed'
    &&reference.targetKey===uwbPathIdentity(u)&&u.coordinateSource==='smoothed'
    &&u.smoothing?.ready===true&&u.fresh===true&&u.online===true&&u.measurementStatus==='trusted'
    &&finite(reference.evaluatedAt)&&reference.evaluatedAt<=now&&now-reference.evaluatedAt<=250
    &&finite(reference.inputUpdatedAt)&&reference.inputUpdatedAt<=u.measurementUpdatedAt
    &&now-reference.inputUpdatedAt<=450
    &&[reference.desiredSpeedMps,reference.headingErrorDeg,reference.yawRateDegS].every(finite)
    &&reference.desiredSpeedMps>=0;
}
