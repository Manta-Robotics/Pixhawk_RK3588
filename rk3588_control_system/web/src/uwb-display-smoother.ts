import {finite, type UwbPacket, type DisplayPoint} from './uwb-types';

const wrap = (value: number) => ((value + 180) % 360 + 360) % 360 - 180;
const moving = (packet: UwbPacket) => (packet.sensors || []).some(sensor =>
  sensor.receiverId !== undefined && (packet.activeReceiverIds || []).includes(sensor.receiverId)
  && sensor.angularFilterMode === 'moving');

/** Display-only smoothing; never changes an observation or its source clock. */
export class UwbDisplaySmoother {
  readonly tauMs: number;
  angle: number | null = null;
  target: number | null = null;
  at: number | null = null;
  clock: number | null = null;
  key: string | null = null;
  moving = false;
  constructor(tauMs?: number) { this.tauMs = tauMs || 120; }
  reset(): void {
    this.angle = this.target = this.at = this.clock = null;
    this.key = null;
    this.moving = false;
  }
  view(packet: UwbPacket | null | undefined, now: number): DisplayPoint {
    packet = packet || {};
    const {measurementUpdatedAt: clock, distanceM, azimuthDeg} = packet;
    if (packet.online !== true || packet.fresh !== true
      || (packet.measurementStatus && packet.measurementStatus !== 'trusted')
      || !finite(distanceM) || distanceM <= 0 || !finite(azimuthDeg)
      || (finite(clock) && (now - clock > 700 || now < clock))) {
      this.reset();
      return {valid: false};
    }
    // Demo packets without a clock do not earn synthetic observations on repaint.
    if (!finite(clock)) { this.reset(); return {valid: true, distanceM, azimuthDeg}; }
    const key = JSON.stringify([packet.activeReceiverIds || [],
      (packet.sensors || []).map(sensor => [sensor.receiverId, sensor.tagId])]);
    if (this.angle === null || this.target === null || this.at === null || this.clock === null
      || this.key !== key || clock < this.clock || now < this.at || now - this.at > 250) {
      this.angle = this.target = azimuthDeg;
      this.at = now; this.clock = clock; this.key = key;
      this.moving = moving(packet);
    }
    this.angle = wrap(this.angle + (1 - Math.exp(-(now - this.at) / this.tauMs)) * wrap(this.target - this.angle));
    this.at = now;
    if (clock !== this.clock) {
      this.target = azimuthDeg; this.clock = clock; this.moving = moving(packet);
    }
    // Moving posteriors already passed admission; avoid adding a second delay.
    if (this.moving) this.angle = this.target;
    return {valid: true, distanceM, azimuthDeg: this.angle};
  }
}
if (typeof window !== 'undefined') window.MantaUwbDisplaySmoother = UwbDisplaySmoother;
