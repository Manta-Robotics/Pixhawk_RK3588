// These adapters run only at the existing source/command callbacks; dashboard
// polling never changes measurement age or writes an observation.
const wrap = (n) => ((((n + 180) % 360) + 360) % 360) - 180;
export function acceptUwb(live, packet, calibration, installation) {
  const stamp = packet?.experiment_stamp;
  if (!stamp?.sample_id || !stamp.obs_ns || !packet.fresh || packet.measurementStatus === 'suspect') return;
  const raw = packet.rawAzimuthDeg,
    horizontal = packet.azimuthDeg;
  const calibrated = [
    calibration?.bearingSign,
    calibration?.bearingScale,
    calibration?.bearingOffsetDeg
  ].every(Number.isFinite);
  const bearing =
    calibrated && Number.isFinite(horizontal)
      ? wrap(horizontal * calibration.bearingSign * calibration.bearingScale + calibration.bearingOffsetDeg)
      : null;
  const elevation =
    packet.elevationValid === true || packet.sensors?.some((s) => s.valid && s.elevationValid === true);
  live.ingest({
    source: 'uwb',
    ...stamp,
    time_kind: 'host_receive',
    quality: 'measured',
    values: {
      uwb_range_m: packet.distanceM,
      uwb_bearing_deg: bearing,
      uwb_elevation_deg: elevation ? packet.elevationDeg : null
    },
    meta: {
      raw_azimuth_deg: raw ?? null,
      elevation_valid: !!elevation,
      calibration_id: installation.uwb_calibration_id ?? null,
      sign: calibration?.bearingSign,
      scale: calibration?.bearingScale,
      offset_deg: calibration?.bearingOffsetDeg
    }
  });
}
export function acceptGimbal(live, feedback, siyi, installation, sampleId) {
  if (!feedback || !feedback.checksumValid || (siyi && feedback.attitudeValid !== true)) return;
  const verified = siyi || installation.gimbal_pitch_down_positive === true;
  live.local(
    'gimbal',
    {
      gimbal_yaw_deg: verified ? feedback.yawDeg : null,
      gimbal_pitch_deg: verified ? -feedback.pitchDeg : null
    },
    {
      protocol: siyi ? 'siyi' : 'serial',
      feedback_valid: true,
      calibration_id: installation.gimbal_calibration_id ?? null,
      mount: feedback.mounting ?? null,
      raw_yaw_deg: feedback.yawDeg,
      raw_pitch_deg: feedback.pitchDeg
    },
    sampleId ? { sample_id: sampleId } : {}
  );
}
const routeModes = {
  '/uwb-follow/mode': ['follow_side', (body) => body.mode],
  '/uwb-follow/method': ['follow_method', (body) => body.method],
  '/uwb-follow/start': ['follow_active', () => true],
  '/uwb-follow/stop': ['follow_active', () => false],
  '/gimbal/track/start': ['gimbal_mode', (body) => body.mode ?? 'face'],
  '/gimbal/track/stop': ['gimbal_mode', () => 'idle']
};
export function modeMiddleware(live, refresh) {
  return (req, res, next) => {
    const route = routeModes[req.path];
    if (req.method !== 'POST' || !route) return next();
    const [axis, read] = route,
      requested = read(req.body ?? {}),
      id = live.modeRequest(axis, requested, 'web');
    const original = res.json;
    res.json = function (body) {
      refresh();
      if (res.statusCode >= 400 || body?.success === false)
        live.modeOutcome(axis, id, 'rejected', body?.message ?? '请求被拒绝');
      else if (live.actual(axis) === requested) live.modeOutcome(axis, id, 'applied', null);
      return original.call(this, body);
    };
    next();
  };
}
