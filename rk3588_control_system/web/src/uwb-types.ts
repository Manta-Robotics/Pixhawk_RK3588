/** Incoming telemetry may omit fields while a receiver is starting or offline. */
export interface UwbSensor {
  receiverId?: number;
  tagId?: number | string | null;
  valid?: boolean;
  angularFilterMode?: string;
  receivedMeasurementAt?: number | null;
  rawDistanceUpdatedAt?: number | null;
  rawDistanceM?: number | null;
  rawAzimuthDeg?: number | null;
  mountYawDeg?: number | null;
  positionXM?: number | null;
  positionYM?: number | null;
}
export interface UwbPacket {
  online?: boolean;
  fresh?: boolean;
  measurementStatus?: string;
  controlSource?: 'raw' | 'filtered';
  trustChecksEnabled?: boolean;
  measurementUpdatedAt?: number | null;
  distanceM?: number | null;
  azimuthDeg?: number | null;
  activeReceiverIds?: number[];
  sensors?: UwbSensor[];
}
export type DisplayPoint = {valid: false} | {valid: true; distanceM: number; azimuthDeg: number};
export function finite(value: unknown): value is number {
  return typeof value === 'number' && Number.isFinite(value);
}
