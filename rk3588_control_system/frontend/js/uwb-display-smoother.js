(function (root) {
  'use strict';
  function wrap(value) { return ((value + 180) % 360 + 360) % 360 - 180; }
  function finite(value) { return typeof value === 'number' && Number.isFinite(value); }
  function UwbDisplaySmoother(tauMs) { this.tauMs = tauMs || 120; this.reset(); }
  UwbDisplaySmoother.prototype.reset = function () {
    this.angle = this.target = this.at = this.clock = null; this.key = null;
  };
  UwbDisplaySmoother.prototype.view = function (packet, now) {
    packet = packet || {};
    var clock = packet.measurementUpdatedAt;
    var valid = packet.online === true && packet.fresh === true
      && finite(packet.distanceM) && packet.distanceM > 0 && finite(packet.azimuthDeg);
    if (finite(clock) && (now - clock > 700 || now < clock)) valid = false;
    if (!valid) { this.reset(); return { valid: false }; }
    // Legacy/demo packets without a measurement clock do not earn synthetic
    // observations from render calls. Display their original values directly.
    if (!finite(clock)) { this.reset(); return { valid: true, distanceM: packet.distanceM, azimuthDeg: packet.azimuthDeg }; }
    var key = JSON.stringify([packet.activeReceiverIds || [], (packet.sensors || []).map(function (s) { return [s.receiverId, s.tagId]; })]);
    if (this.angle === null || this.key !== key || clock < this.clock || now < this.at || now - this.at > 250) {
      this.angle = this.target = packet.azimuthDeg; this.at = now; this.clock = clock; this.key = key;
    }
    // Advance toward the previous target before admitting this render's new
    // target. Duplicate full-telemetry packets cannot change the source clock.
    this.angle = wrap(this.angle + (1 - Math.exp(-(now - this.at) / this.tauMs)) * wrap(this.target - this.angle));
    this.at = now;
    if (clock !== this.clock) { this.target = packet.azimuthDeg; this.clock = clock; }
    return { valid: true, distanceM: packet.distanceM, azimuthDeg: this.angle };
  };
  root.MantaUwbDisplaySmoother = UwbDisplaySmoother;
})(typeof window !== 'undefined' ? window : globalThis);
