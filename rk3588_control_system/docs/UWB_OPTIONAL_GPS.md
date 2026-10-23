# UWB following with optional GPS speed feedback

Both branches use `MANUAL` propulsion and UWB bearing/IMU yaw-rate guidance.
There is no automatic flight-mode switch when GPS appears or disappears.

- `execution_mode: relative_imu` preserves the existing relative range-rate and
  cruise-thrust controller when GPS is unavailable.
- `gps_speed_closed_loop.enabled: true` supplements surge control with GPS/EKF
  velocity feedback after 2000 ms of continuously valid navigation.
- Valid navigation needs a sufficient GPS fix, fresh GPS/position/EKF/heading,
  healthy EKF, finite velocity, and speed within the configured observation bound.
- GPS loss, stale navigation or invalid velocity immediately returns to relative
  control. Reacquisition repeats the confirmation delay. Actuator slew, braking,
  close-range protection, UWB prediction limits and IMU freshness still apply.
- A filtered target radial speed combines observed UWB range rate and own
  GPS/EKF velocity projected along the UWB line of sight. A PI controller tracks
  the resulting bounded forward speed. Steering continues to use UWB bearing;
  GPS position is not a steering target or a replacement for the beacon.
- GPS acquisition seeds the PI output from applied throttle; saturation and
  applied-output tracking limit integral windup. No control command or arming
  request is sent by the GPS controller itself.

Diagnostics expose `speedControlMode`, `gpsSpeedControlActive`,
`gpsSpeedControlReason`, `measuredForwardSpeedMps` and `desiredSpeedMps`.
Without GPS, absolute speed fields remain unavailable. The neural policy keeps
its explicit missing-absolute-velocity observation adaptation; the optional GPS
loop governs its nominal speed without claiming the actor received GPS inputs.

Regression tests: `node --test test/uwb_optional_gps.test.mjs` plus the existing
relative IMU, prediction, braking, propulsion and follow-session tests.
The live verification replays read-only telemetry into isolated controller
instances; simulated arming/GPS values never reach the vessel or command path.
Actual follow intent remains cleared during deployment and verification.
