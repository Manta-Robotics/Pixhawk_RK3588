import { parentPort,workerData } from 'node:worker_threads';
import { runUwbWorker } from './uwb_usb_worker_runtime.js';
runUwbWorker(parentPort,workerData);
