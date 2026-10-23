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
      if (sensor.valid === false || !finite(clock) || clock > now || now - clock > 700) return;
      var sample = previous;
      // DIST precedes AZ on the forwarding link. Never combine the next DIST
      // with an earlier complete frame's AZ, or retarget on metadata/repaints.
      var complete = finite(sensor.rawDistanceUpdatedAt) && sensor.rawDistanceUpdatedAt <= clock;
      if ((!previous || previous.at !== clock) && complete) {
        if (!finite(sensor.rawDistanceM) || sensor.rawDistanceM <= 0
          || !finite(sensor.rawAzimuthDeg) || Math.abs(sensor.rawAzimuthDeg) > 180) return;
        var angle = (sensor.rawAzimuthDeg + (finite(sensor.mountYawDeg) ? sensor.mountYawDeg : 0)) * Math.PI / 180;
        sample = { valid: true, at: clock, receiverId: sensor.receiverId, tagId: sensor.tagId,
          distanceM: sensor.rawDistanceM, azimuthDeg: sensor.rawAzimuthDeg,
          forward: (finite(sensor.positionXM) ? sensor.positionXM : 0) + sensor.rawDistanceM * Math.cos(angle),
          left: (finite(sensor.positionYM) ? sensor.positionYM : 0) + sensor.rawDistanceM * Math.sin(angle) };
      }
      if (!sample || sample.at > now || now - sample.at > 700) return;
      next.set(key, sample);
      // A raw point represents one receiver, never an invented raw fusion.
      if (!result || sample.at > result.at) result = sample;
    }, this);
    this.samples = next;
    return result || { valid: false };
  };
  root.MantaUwbRawDisplay = RawDisplay;
})(typeof window !== 'undefined' ? window : globalThis);
