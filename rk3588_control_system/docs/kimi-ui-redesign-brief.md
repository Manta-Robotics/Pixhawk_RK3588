# MANTA App UI redesign brief for Kimi

## Deliverable

Create a completely new, high-fidelity, interactive landscape prototype for iOS/iPadOS. Build the actual preview, not a written analysis, moodboard, tutorial, or wireframe. One prototype must cover iPad 4:3 landscape first and adapt cleanly to phone landscape. Portrait is unsupported.

Use the attached current frontend only as the functional/interface reference. The appearance may be replaced completely. Do not change backend, motor, gimbal, transport behavior, endpoint names, payloads, or safety semantics.

## Product and visual direction

- MANTA is an autonomous surface photography robot: a DJI Pocket 3 capable of moving on water.
- Audience: open-water swimmers and professional wild-swimming athletes.
- Character: professional, minimal, outdoor. Reference DJI Mimo and DJI Fly; learn composition, hierarchy, color, and micro-motion from strong Kimi frontend work.
- Avoid AI-template styling, cinematic-poster styling, industrial control-panel styling, dense HUD decoration, neon overload, and repeated marketing copy.
- Palette: black/graphite/ice blue; green for healthy/connected, orange for warning, red for danger/stop. Use restrained soft glass, generous whitespace, and very few precise lines.
- Use the attached white MANTA product image and white MANTA logo language. On the landing screen, show the brand and “Your AI Camera on Water” only once. Do not add `SURFACE` or duplicate the slogan.
- Default Simplified Chinese. English mode must translate every visible label and state.

## Required screens and states

1. Landing/device discovery: product image, nearby MANTA ONE card, model `MNTA`, serial `2407`, BLE discovery, internet-preserved note, connect button, first-pair six-digit PIN.
2. Connected control: four functional areas. Left top IMU, left bottom 360-degree joystick plus adjacent speed slider, right top uncropped gimbal feed, right bottom live logs. Provide one control to swap left/right groups.
3. Map: MANTA-logo marker; bottom-left GPS connected/waiting/degraded status, satellites/HDOP, and map provider.
4. Recordings: DJI-style large thumbnails; play/pause, draggable timeline, elapsed/duration, 0.5/1/1.5/2x playback. V1 supports download only. While downloading, block page exit; save in app first. Do not expose rename/delete/export-to-Photos as working V1 actions.
5. Fold-down utilities: settings, complete logs, help/safety. Main navigation contains only Device, Map, Recordings.

## Connected-control behavior

- Always visible: connection state, signal latency, current speed, flight-controller temperature.
- IMU attitude display is 1.5x the former size. Six-position calibration is a guided sequence: level, left down, right down, nose down, nose up, back down. Lock thrust during calibration.
- 360-degree joystick keeps existing vector logic. Speed slider sits beside it. Emergency stop is a one-swipe action and immediately outputs zero motion.
- Gimbal video must never crop its top edge. Keep existing gimbal modules: click-to-center, face tracking, swimmer tracking, home, stop, record, OSD toggle, more/settings. Recording shows `REC mm:ss` in the video top-right.
- Gimbal motor jam/fault immediately disables every gimbal action, stops tracking/motor/recording, shows the fault code and diagnostic report, but leaves normal vehicle movement available with a warning.
- Logs distinguish command, upper computer, lower computer, motor, gimbal, Pixhawk, link, and safety status.
- Prototype state simulator: connected, video lost, gimbal fault, high temperature, emergency stop, offline. Simulate everything except battery.

## Existing DOM/interaction hooks to preserve

Keep these IDs/data hooks or provide a thin compatibility adapter: `appShell`, `connectButton`, `pairingModal`, `pairButton`, `disconnectButton`, `layoutToggle`, `toolDrawer`, `transportToggle`, `languageSelect`, `themeSelect`, `attitudeOrb`, `rollValue`, `pitchValue`, `yawValue`, `imuCalibrate`, `imuCalibrationModal`, `joystick`, `joyStick`, `driveOutput`, `leftMotor`, `rightMotor`, `estopSlider`, `estopThumb`, `gimbalVideo`, `liveGimbalFeed`, `recordButton`, `recordTimer`, `gimbalOsd`, `gimbalFault`, `openReport`, `logStream`, `gpsStatus`, `gpsDetail`, `mapSource`, `mediaPlayPause`, `mediaScrubber`, `mediaPlaybackRate`, `downloadModal`, `data-nav`, `data-area`, `data-gimbal-mode`, `data-gimbal-action`, `data-demo-state`.

## Current frontend transport contract

Discovery and state:

- `GET /health` discovers the board.
- `GET /api/status`, no-store; poll fallback every 2 seconds.
- Status supplies `data.telemetry` and `data.gimbal`.
- Telemetry fields consumed by UI: `velocity.vx/vy`, `temperature.flightController|hostBoard`, `attitude.roll/pitch/yaw`, `battery`, `imuCalibration`, `gps`.

Realtime Socket.IO:

- Outbound `rover_drive`: `{ throttle: -100..100, steering: -45..45 }`, throttled to at most 20 Hz; zero command sends immediately.
- Inbound: `telemetry_update`, `system_state`, `rover_drive_ack`, `gimbal_state`, `log_entry`, plus `connect`/`disconnect`.
- HTTP fallback: `POST /api/control/rover` with the same payload.

Safety and gimbal POST endpoints:

- `/api/emergency/stop` `{}`
- `/api/gimbal/home` `{ preserveTracking: true }`
- `/api/gimbal/stop` `{}`
- `/api/gimbal/recording/start` `{}`
- `/api/gimbal/recording/stop` `{}`
- `/api/gimbal/track/start` `{ mode: "face" | "swimmer" }`
- `/api/gimbal/track/stop` `{}` for click-to-center mode
- `/api/gimbal/osd` `{ mode: 2 | 0 }`

IMU calibration:

- `POST /api/calibration/imu/start` `{ type: "ACCEL" }`
- `POST /api/calibration/imu/confirm` `{ positionCode: 1..6 }`
- HTTP success only confirms command delivery; the UI must wait for FCU telemetry state before marking a face complete.

Recordings:

- `GET /api/gimbal/recordings` returns `{ success, recordings[] }`.
- Each recording may provide `name`, `title`, `modifiedAt`, `size`, `relativeUrl|url`, `relativeDownloadUrl|downloadUrl`.

## Safety rules

- Lost/degraded link locks local controls and resets joystick output.
- Emergency stop sets motion to zero immediately.
- During IMU calibration, non-zero drive commands are blocked.
- Download navigation remains locked until completion.
- Gimbal stall monitor faults after three consecutive 1.8-second windows with a fresh non-zero target but angle delta under 0.8 degrees and gyro under 1 dps; feedback timeout is 5 seconds. Fault action is `STOP_GIMBAL`.

## Output requirement

Return one polished, working prototype with all four views and the key dialogs/states above. Prioritize visual composition and realistic micro-interactions. Do not spend tokens explaining decisions before the prototype.
