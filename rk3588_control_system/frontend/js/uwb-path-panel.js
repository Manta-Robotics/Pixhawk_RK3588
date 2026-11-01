(function(root){
  'use strict';
  var finite=Number.isFinite;
  function bodyMeasurement(u){
    var c=u.coordinateCalibration;
    if(c&&[u.distanceM,u.azimuthDeg,c.bearingSign,c.bearingScale,c.bearingOffsetDeg].every(finite)){
      var a=(u.azimuthDeg*c.bearingSign*c.bearingScale+c.bearingOffsetDeg)*Math.PI/180;
      return {x:u.distanceM*Math.cos(a),y:u.distanceM*Math.sin(a)};
    }
    return {x:u.relativeXM,y:-u.relativeYM};
  }
  function render(options){
    var u=options.uwb||{},p=options.path||{},follow=options.follow||{},now=options.now,range=options.range;
    var doc=options.document||root.document;
    function node(id){return doc.getElementById(id);}
    function text(id,value){var e=node(id);if(e)e.textContent=value;}
    function number(v,suffix){return finite(v)?v.toFixed(2)+(suffix||''):'--';}
    var age=finite(u.measurementUpdatedAt)?now-u.measurementUpdatedAt:Infinity;
    var fresh=u.fresh===true&&u.smoothing?.ready===true&&age>=0&&age<=500;
    text('uwbQualityRate',number(u.transport?.processedRateHz,' Hz'));
    text('uwbQualityAge',fresh?Math.round(age)+' ms':'等待定位');
    var current=p.enabled===true&&finite(p.evaluatedAt)&&now>=p.evaluatedAt&&now-p.evaluatedAt<=500&&fresh
      &&p.targetKey===JSON.stringify([u.streamId??null,u.activeReceiverIds||[],(u.sensors||[]).map(s=>[s.receiverId,s.tagId])]);
    var applied=current&&p.active&&p.controlEnabled!==false&&follow.active&&follow.pathFollowActive&&finite(follow.updatedAt)&&now-follow.updatedAt>=0&&now-follow.updatedAt<=500;
    var reasons={disabled:'未启用',smoothed_uwb_unavailable:'等待平滑定位',gps_fix_invalid:'等待 GPS 定位',gps_stale:'等待 GPS 更新',
      ekf_unavailable:'等待导航状态',position_invalid:'等待有效位置',imu_invalid:'等待航向',velocity_invalid:'等待有效速度',
      velocity_stale:'等待速度更新',gps_confirming:'确认导航稳定',path_warming_up:'正在积累轨迹',
      observation_unavailable:'目标暂不可跟随',gps_position_jump:'位置跳变，重新积累',target_position_jump:'目标跳变，重新积累',
      target_identity_changed:'信标已切换',observation_clock_gap:'定位间断，重新积累',clock_gap:'重新同步',
      path_rejoin_required:'等待重返路径',path_buffer_exhausted:'重建路径'};
    var status=applied?'优化跟随中':current&&p.active?(p.controlEnabled===false?'路径预览':'路径已就绪'):!p.enabled?'未启用':!fresh?'等待平滑定位':!current?'等待路径更新':reasons[p.reason]||'等待路径';
    text('uwbPathStatus',status);
    text('uwbPathHint',p.controlEnabled===false?'仅预览，船端推进跟随未启用':applied?'沿优化路径跟随':current&&p.active?'启动跟随后使用优化路径':current&&p.reason?.startsWith('gps_')?'GPS 就绪后可使用优化路径':'平滑定位 → 路径优化');
    var card=node('uwbPathCard');if(card)card.dataset.state=applied?'active':current&&p.active?'ready':'waiting';
    text('uwbPathPoints',current?String(p.pointCount||0):'--');
    text('uwbPathLength',current?number(p.pathLengthM,' m'):'--');
    text('uwbPathLookahead',current&&p.active?number(p.lookaheadM,' m'):'--');
    text('uwbPathSpeed',current&&p.active?number(p.desiredSpeedMps,' m/s'):'--');
    var show=current&&finite(range)&&range>0;
    function project(a){return [120+a.y/range*84,110-a.x/range*84];}
    for(var item of [['uwbObservedPath',p.bodyObservations],['uwbOptimizedPath',p.bodyPath]]){
      var e=node(item[0]);if(!e)continue;
      var points=show&&Array.isArray(item[1])?item[1].slice(0,64).filter(a=>[a.x,a.y].every(finite)).map(project):[];
      e.setAttribute('points',points.map(a=>a.map(v=>v.toFixed(2)).join(',')).join(' '));
      e.setAttribute('visibility',points.length>1?'visible':'hidden');
    }
    var goal=node('uwbPathLookaheadPoint'),a=p.bodyLookahead;
    if(goal){var visible=show&&p.active&&a&&[a.x,a.y].every(finite);goal.setAttribute('visibility',visible?'visible':'hidden');
      if(visible){var xy=project(a);goal.setAttribute('cx',xy[0]);goal.setAttribute('cy',xy[1]);}}
  }
  root.MantaUwbPathPanel={render:render,bodyMeasurement:bodyMeasurement};
})(typeof window!=='undefined'?window:globalThis);
