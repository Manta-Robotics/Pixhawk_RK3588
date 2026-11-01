(function (root) {
  'use strict';
  function finite(value) { return typeof value === 'number' && Number.isFinite(value); }
  // Retain the existing public API, but all filtering now belongs to Jetson.
  // Repainting may validate an observation; it must never synthesize motion.
  function UwbDisplaySmoother() {}
  UwbDisplaySmoother.prototype.reset = function () {};
  UwbDisplaySmoother.prototype.view = function (packet, now) {
    packet = packet || {};
    var clock = packet.measurementUpdatedAt;
    var valid = packet.online === true && packet.fresh === true
      && (!packet.measurementStatus || packet.measurementStatus === 'trusted')
      && finite(packet.distanceM) && packet.distanceM > 0 && finite(packet.azimuthDeg);
    if (finite(clock) && (now - clock > 700 || now < clock)) valid = false;
    if (!valid) { this.reset(); return { valid: false }; }
    // Legacy/demo packets without a measurement clock do not earn synthetic
    // observations from render calls. Display their original values directly.
    if (!finite(clock)) { this.reset(); return { valid: true, distanceM: packet.distanceM, azimuthDeg: packet.azimuthDeg }; }
    return { valid: true, distanceM: packet.distanceM, azimuthDeg: packet.azimuthDeg };
  };
  // A moving scale makes a stationary target appear to jump. Range changes
  // only when the operator changes the radar's range selector.
  function UwbPlotScale() { this.reset(); }
  UwbPlotScale.prototype.reset = function () { this.scale = 2; };
  UwbPlotScale.prototype.setRange = function (range) {
    if ([1,2,5,10,20,50].indexOf(range) !== -1) this.scale = range;
    return this.scale;
  };
  UwbPlotScale.prototype.view = function () { return this.scale; };
  // Presentation-only memory. A retained point never becomes an admitted
  // observation, and is cleared across disconnects or receiver/tag changes.
  function UwbDisplayHold() { this.reset(); }
  UwbDisplayHold.prototype.reset = function () { this.last = null; this.key = null; };
  UwbDisplayHold.prototype.view = function (sample, connected, key) {
    if (!connected) { this.reset(); return { valid: false, stale: false }; }
    if (key !== this.key) { this.last = null; this.key = key; }
    if (sample && sample.valid) {
      this.last = Object.assign({}, sample);
      return Object.assign({}, sample, { stale: false });
    }
    return this.last ? Object.assign({}, this.last, { stale: true }) : { valid: false, stale: false };
  };
  root.MantaUwbDisplayHold = UwbDisplayHold;
  root.MantaUwbDisplaySmoother = UwbDisplaySmoother;
  root.MantaUwbPlotScale = UwbPlotScale;
})(typeof window !== 'undefined' ? window : globalThis);
