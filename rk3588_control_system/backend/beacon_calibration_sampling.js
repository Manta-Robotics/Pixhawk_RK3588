import { normalizeGeometryAngleDeg as wrap } from './gimbal_beacon_geometry.js';

const median = values => {
  const v = [...values].sort((a,b) => a-b), mid = Math.floor(v.length/2);
  return v.length % 2 ? v[mid] : (v[mid-1]+v[mid])/2;
};
const mean = values => values.reduce((s,v) => s+v,0)/values.length;
const circularMean = values => Math.atan2(mean(values.map(v => Math.sin(v*Math.PI/180))),mean(values.map(v => Math.cos(v*Math.PI/180))))*180/Math.PI;
const circularMedian = values => values.reduce((best,v) => {
  const score = values.reduce((s,x) => s+Math.abs(wrap(x-v)),0);
  return score < best.score ? {value:v,score} : best;
}, {value:values[0],score:Infinity}).value;
const rms = values => Math.sqrt(mean(values.map(v => v*v)));

export function summarizeBeaconCalibrationReadings(readings, { minSamples = 24 } = {}) {
  if (!Array.isArray(readings) || readings.length < minSamples) throw new Error('Not enough distinct stable UWB observations; hold still and retry this point');
  const keys = ['distanceM','uwbAzimuthDeg','uwbElevationDeg','gimbalYawDeg','gimbalPitchDeg','at'];
  if (readings.some(s => keys.some(k => !Number.isFinite(s[k])) || s.distanceM <= 0 || Math.abs(s.uwbAzimuthDeg)>180 || Math.abs(s.uwbElevationDeg)>90)) throw new Error('Invalid calibration observation');
  if (new Set(readings.map(s=>s.at)).size !== readings.length || new Set(readings.map(s=>s.key)).size !== 1) throw new Error('Calibration observations must use distinct frames from the same receiver and tag');
  const yaw0 = circularMedian(readings.map(s=>s.uwbAzimuthDeg));
  const pitch0 = median(readings.map(s=>s.uwbElevationDeg));
  const distance0 = median(readings.map(s=>s.distanceM));
  const inliers = readings.filter(s => Math.abs(wrap(s.uwbAzimuthDeg-yaw0)) <= 4
    && Math.abs(s.uwbElevationDeg-pitch0) <= 4 && Math.abs(s.distanceM-distance0) <= Math.max(.08,distance0*.05));
  if (inliers.length < minSamples || inliers.length < readings.length*.8) throw new Error('Too many UWB outliers; hold the beacon still at image center and retry this point');
  if (Math.max(...inliers.map(s=>s.at))-Math.min(...inliers.map(s=>s.at)) < 1000) throw new Error('Hold the beacon still for at least one second');
  const uwbAzimuthDeg = circularMean(inliers.map(s=>s.uwbAzimuthDeg));
  const uwbElevationDeg = mean(inliers.map(s=>s.uwbElevationDeg));
  const yawRmsDeg = rms(inliers.map(s=>wrap(s.uwbAzimuthDeg-uwbAzimuthDeg)));
  const pitchRmsDeg = rms(inliers.map(s=>s.uwbElevationDeg-uwbElevationDeg));
  if (yawRmsDeg > 1.5 || pitchRmsDeg > 1.5) throw new Error('UWB angles are not stable enough for precision calibration; hold still and retry this point');
  const gimbalYawDeg = circularMean(inliers.map(s=>s.gimbalYawDeg));
  const gimbalPitchDeg = mean(inliers.map(s=>s.gimbalPitchDeg));
  const gimbalYawRmsDeg = rms(inliers.map(s=>wrap(s.gimbalYawDeg-gimbalYawDeg)));
  const gimbalPitchRmsDeg = rms(inliers.map(s=>s.gimbalPitchDeg-gimbalPitchDeg));
  if (gimbalYawRmsDeg>.35 || gimbalPitchRmsDeg>.35
    || inliers.some(s=>Math.abs(wrap(s.gimbalYawDeg-gimbalYawDeg))>.8 || Math.abs(s.gimbalPitchDeg-gimbalPitchDeg)>.8)) {
    throw new Error('Gimbal is still moving; wait for the measured pose to settle and retry this point');
  }
  return {
    distanceM: median(inliers.map(s=>s.distanceM)), uwbAzimuthDeg, uwbElevationDeg,
    gimbalYawDeg, gimbalPitchDeg,
    feedbackVerified: inliers.every(s=>s.feedbackVerified), sampleCount:inliers.length,
    receiverIdentity:inliers[0].key,
    quality:{receivedCount:readings.length,rejectedCount:readings.length-inliers.length,
      durationMs:Math.max(...inliers.map(s=>s.at))-Math.min(...inliers.map(s=>s.at)),yawRmsDeg,pitchRmsDeg,gimbalYawRmsDeg,gimbalPitchRmsDeg,
      distanceRmsM:rms(inliers.map(s=>s.distanceM-distance0))}
  };
}
