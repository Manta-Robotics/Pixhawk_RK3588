(function(root){
  'use strict';
  var frame=Object.freeze({width:240,height:220,cx:120,cy:110,radius:84});
  function finite(v){return typeof v==='number'&&Number.isFinite(v);}
  function project(forward,left,range){
    if(![forward,left,range].every(finite)||range<=0)return null;
    var distance=Math.hypot(forward,left),outside=distance>range;
    if(!finite(distance))return null;
    var factor=outside?range/distance:1;
    return {x:frame.cx-left/range*frame.radius*factor,
      y:frame.cy-forward/range*frame.radius*factor,distanceM:distance,
      azimuthDeg:Math.atan2(left,forward)*180/Math.PI,
      rotationDeg:-Math.atan2(left,forward)*180/Math.PI,outside:outside};
  }
  function ticks(range){return [range/3,range*2/3,range];}
  function meters(value){return value.toFixed(2).replace(/\.?0+$/,'')+' m';}
  function labelPosition(p,raw){
    return {x:Math.max(28,Math.min(212,p.x+(p.x>frame.cx?-9:9))),
      y:Math.max(24,Math.min(204,p.y+(raw||p.y<55?17:-9))),
      anchor:p.x>frame.cx?'end':'start'};
  }
  root.MantaUwbRadar={frame:frame,project:project,ticks:ticks,meters:meters,labelPosition:labelPosition};
})(typeof window!=='undefined'?window:globalThis);
