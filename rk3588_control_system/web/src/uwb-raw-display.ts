import {finite, type UwbPacket, type UwbSensor} from './uwb-types';

interface RawPoint {
  valid: true;
  at: number;
  receiverId: UwbSensor['receiverId'];
  tagId: UwbSensor['tagId'];
  distanceM: number;
  azimuthDeg: number;
  forward: number;
  left: number;
}
/** Diagnostic raw point only: never feeds fusion or control. */
export class RawDisplay {
  samples = new Map<string, RawPoint>();
  reset(): void { this.samples.clear(); }
  view(packet: UwbPacket, now: number, connected: boolean): RawPoint | {valid: false} {
    if (!connected) { this.reset(); return {valid: false}; }
    const next = new Map<string, RawPoint>();
    let result: RawPoint | undefined;
    for (const sensor of packet.sensors || []) {
      const key = JSON.stringify([sensor.receiverId, sensor.tagId]);
      const clock = sensor.receivedMeasurementAt;
      let sample = this.samples.get(key);
      if (sensor.valid === false || !finite(clock) || clock > now || now - clock > 700) continue;
      // DIST precedes AZ. A partial frame must not retarget or freshen a point.
      const complete = finite(sensor.rawDistanceUpdatedAt) && sensor.rawDistanceUpdatedAt <= clock;
      if ((!sample || sample.at !== clock) && complete) {
        const {rawDistanceM: distanceM, rawAzimuthDeg: azimuthDeg} = sensor;
        if (!finite(distanceM) || distanceM <= 0 || !finite(azimuthDeg) || Math.abs(azimuthDeg) > 180) continue;
        const angle = (azimuthDeg + (finite(sensor.mountYawDeg) ? sensor.mountYawDeg : 0)) * Math.PI / 180;
        sample = {valid: true, at: clock, receiverId: sensor.receiverId, tagId: sensor.tagId,
          distanceM, azimuthDeg,
          forward: (finite(sensor.positionXM) ? sensor.positionXM : 0) + distanceM * Math.cos(angle),
          left: (finite(sensor.positionYM) ? sensor.positionYM : 0) + distanceM * Math.sin(angle)};
      }
      if (!sample || sample.at > now || now - sample.at > 700) continue;
      next.set(key, sample);
      if (!result || sample.at > result.at) result = sample;
    }
    this.samples = next;
    return result || {valid: false};
  }
}
if (typeof window !== 'undefined') window.MantaUwbRawDisplay = RawDisplay;
