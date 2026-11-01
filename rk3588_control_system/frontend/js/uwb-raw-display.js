(function (root) {
  'use strict';
  function finite(value) { return typeof value === 'number' && Number.isFinite(value); }
  // Diagnostic view only: never supplies observations to fusion or control.
  function RawDisplay() { this.samples = new Map(); }
  RawDisplay.prototype.reset = function () { this.samples.clear(); };
  RawDisplay.prototype.view = function (packet, now, connected) {
    if (!connected) { this.reset(); return { valid: false }; }
    var next = new Map(), result = null;
    (packet.sensors || []).forEach(function (sensor) {
      var key = JSON.stringify([sensor.receiverId, sensor.tagId]);
      var clock = sensor.receivedMeasurementAt, previous = this.samples.get(key);
      var observation = sensor.rawObservation;
      if (observation && (observation.at !== clock || observation.tagId !== sensor.tagId)) return;
      if (sensor.valid === false || !finite(clock) || clock > now || now - clock > 700) return;
      var sample = previous;
      // DIST precedes AZ on the forwarding link. Never combine the next DIST
      // with an earlier complete frame's AZ, or retarget on metadata/repaints.
      var complete = observation || (finite(sensor.rawDistanceUpdatedAt) && sensor.rawDistanceUpdatedAt <= clock);
      if ((!previous || previous.at !== clock) && complete) {
        var distance = observation ? observation.distanceM : sensor.rawDistanceM;
        var azimuth = observation ? observation.azimuthDeg : sensor.rawAzimuthDeg;
        if (!finite(distance) || distance <= 0 || !finite(azimuth) || Math.abs(azimuth) > 180) return;
        var angle = (azimuth + (finite(sensor.mountYawDeg) ? sensor.mountYawDeg : 0)) * Math.PI / 180;
        sample = { valid: true, at: clock, receiverId: sensor.receiverId, tagId: sensor.tagId,
          distanceM: distance, azimuthDeg: azimuth,
          forward: (finite(sensor.positionXM) ? sensor.positionXM : 0) + distance * Math.cos(angle),
          left: (finite(sensor.positionYM) ? sensor.positionYM : 0) + distance * Math.sin(angle) };
      }
      if (!sample || sample.at > now || now - sample.at > 700) return;
      next.set(key, sample);
      // A raw point represents one receiver, never an invented raw fusion.
      if (!result || sample.at > result.at) result = sample;
    }, this);
    this.samples = next;
    if (result && packet.coordinateCalibration) {
      // Keep cached module geometry and numeric diagnostics unchanged. Only
      // the plotted point uses the same body calibration as measured/predicted.
      var body = root.MantaUwbCoordinates.bodyPosition(Math.hypot(result.forward, result.left),
        Math.atan2(result.left, result.forward) * 180 / Math.PI, packet.coordinateCalibration);
      return body ? Object.assign({}, result, { forward: body.x, left: -body.y }) : { valid: false };
    }
    return result || { valid: false };
  };
  root.MantaUwbRawDisplay = RawDisplay;
})(typeof window !== 'undefined' ? window : globalThis);
