import type {io} from 'socket.io-client';
import type {UwbDisplaySmoother} from './uwb-display-smoother';
import type {RawDisplay} from './uwb-raw-display';
declare global {
  interface Window {
    io:typeof io;
    MantaAppTransport:any;
    MantaOfflineSatelliteMap:any;
    MantaUwbDisplaySmoother:typeof UwbDisplaySmoother;
    MantaUwbRawDisplay:typeof RawDisplay;
    MantaPreview:any;
    resetMantaJoystick?:()=>void;
    openMantaImuCalibration?:()=>void;
  }
  const MediaMTXWebRTCReader:any;
}
export {};
