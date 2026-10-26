const finite = Number.isFinite;

// Select a complete raw observation without changing the display posterior.
// Repaints, diagnostics and the next partial DIST cannot renew its clock.
export class UwbRawControl {
  constructor(config = {}) {
    this.timeout = config.timeout_ms ?? 450;
    this.minimum = config.minimum_distance_m ?? .15;
    this.maximum = config.maximum_distance_m ?? 20;
    this.enabled = config.enabled_receiver_ids ?? [1, 2];
    this.reset();
  }
  reset() { this.samples = new Map(); this.selected = null; }
  view(packet = {}, now, connected = true) {
    if (!connected) this.reset();
    const next = new Map();
    for (const sensor of connected ? packet.sensors || [] : []) {
      if (!this.enabled.includes(sensor.receiverId)) continue;
      const key = JSON.stringify([sensor.receiverId, sensor.tagId]);
      const clock = sensor.receivedMeasurementAt, previous = this.samples.get(key);
      if (sensor.valid === false || !finite(clock) || clock > now || now - clock > this.timeout
        || (finite(sensor.rawRejectedAt) && sensor.rawRejectedAt >= clock)) continue;
      let sample = previous;
      if ((!previous || clock > previous.at) && finite(sensor.rawDistanceUpdatedAt)
        && sensor.rawDistanceUpdatedAt <= clock) {
        const d = sensor.rawDistanceM, a = sensor.rawAzimuthDeg;
        if (!finite(d) || d < this.minimum || d > this.maximum || !finite(a) || Math.abs(a) > 180) continue;
        const angle = (a + (sensor.mountYawDeg || 0)) * Math.PI / 180;
        const x = (sensor.positionXM || 0) + d * Math.cos(angle);
        const y = (sensor.positionYM || 0) + d * Math.sin(angle);
        if (!finite(x) || !finite(y)) continue;
        sample = { at: clock, distanceM: Math.hypot(x, y), azimuthDeg: Math.atan2(y, x) * 180 / Math.PI,
          elevationDeg: sensor.rawElevationDeg, receiverId: sensor.receiverId, tagId: sensor.tagId };
      }
      if (sample && sample.at <= now && now - sample.at <= this.timeout) next.set(key, sample);
    }
    this.samples = next;
    // Retain a fresh receiver instead of alternating sides on every repaint.
    let selected = next.get(this.selected);
    if (!selected) for (const [key, sample] of next) {
      if (!selected || sample.at > selected.at) { selected = sample; this.selected = key; }
    }
    const fresh = Boolean(selected);
    return { ...packet, controlSource: 'raw', online: fresh, fresh,
      distanceM: selected?.distanceM ?? null, azimuthDeg: selected?.azimuthDeg ?? null,
      elevationDeg: selected?.elevationDeg ?? null,
      measurementUpdatedAt: selected?.at ?? null,
      activeReceiverIds: selected ? [selected.receiverId] : [], tagId: selected?.tagId ?? null,
      measurementStatus: fresh ? 'trusted' : 'unavailable',
      rejectionReason: fresh ? null : 'uwb_stale' };
  }
}
