(function (root) {
  'use strict';
  function wrap(value) { return ((value + 180) % 360 + 360) % 360 - 180; }
  function finite(value) { return typeof value === 'number' && Number.isFinite(value); }
  function moving(packet) {
    var selected = packet.activeReceiverIds || [];
    return (packet.sensors || []).some(function (s) {
      return selected.includes(s.receiverId) && s.angularFilterMode === 'moving';
    });
  }
  function UwbDisplaySmoother(tauMs) { this.tauMs = tauMs || 120; this.reset(); }
  UwbDisplaySmoother.prototype.reset = function () {
    this.angle = this.target = this.at = this.clock = null; this.key = null; this.moving = false;
  };
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
    var key = JSON.stringify([packet.activeReceiverIds || [], (packet.sensors || []).map(function (s) { return [s.receiverId, s.tagId]; })]);
    if (this.angle === null || this.key !== key || clock < this.clock || now < this.at || now - this.at > 250) {
      this.angle = this.target = packet.azimuthDeg; this.at = now; this.clock = clock; this.key = key;
      this.moving = moving(packet);
    }
    // Advance toward the previous target before admitting this render's new
    // target. Duplicate full-telemetry packets cannot change the source clock.
    this.angle = wrap(this.angle + (1 - Math.exp(-(now - this.at) / this.tauMs)) * wrap(this.target - this.angle));
    this.at = now;
    if (clock !== this.clock) {
      this.target = packet.azimuthDeg; this.clock = clock; this.moving = moving(packet);
    }
    // Moving data already passed the position filter and angular admission.
    // Follow its newest admitted posterior rather than add another 120 ms EMA.
    // Latch both target and mode only on a new measurement clock.
    if (this.moving) this.angle = this.target;
    return { valid: true, distanceM: packet.distanceM, azimuthDeg: this.angle };
  };
  // Chart range is presentation state, independent of measurement admission.
  // Expand only when the current range is exceeded; shrink after a sustained
  // smaller range, so noise near an automatic zoom threshold cannot bounce it.
  function UwbPlotScale() { this.reset(); }
  UwbPlotScale.prototype.reset = function () {
    this.scale = this.smaller = this.since = this.at = null;
  };
  UwbPlotScale.prototype.view = function (magnitude, now) {
    if (!finite(magnitude) || magnitude < 0 || !finite(now)) return this.scale || 5;
    if (this.at !== null && now < this.at) this.reset();
    this.at = now;
    var ranges = [1, 2, 5, 10, 20, 50], wanted = Math.ceil(magnitude / .9);
    for (var i = 0; i < ranges.length; i++) {
      if (magnitude <= ranges[i] * .9) { wanted = ranges[i]; break; }
    }
    wanted = Math.max(1, wanted);
    var smaller = Math.max(1, Math.ceil(magnitude / .65));
    for (var j = 0; j < ranges.length; j++) {
      if (magnitude <= ranges[j] * .65) { smaller = ranges[j]; break; }
    }
    if (this.scale === null || magnitude > this.scale) {
      this.scale = wanted; this.smaller = this.since = null;
    } else if (smaller < this.scale) {
      if (this.smaller !== smaller) { this.smaller = smaller; this.since = now; }
      if (now - this.since >= 2000) { this.scale = smaller; this.smaller = this.since = null; }
    } else {
      this.smaller = this.since = null;
    }
    return this.scale;
  };
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
