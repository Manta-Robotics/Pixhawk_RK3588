# UWB vessel release — 2026-10-19

This release activates the latest vessel UWB/GPS/IMU control implementation, including Reject NMPC, FOV-aware turning, independent measurement recovery, automatic rearm after UWB timeout, and GPS drift assistance. It also includes the runtime modules and dashboard integration required by the current vessel application.

- Safety separation remains 0.3 m; the target-loss deadline remains 5000 ms.
- A rejected or missing observation enters bounded NMPC on the next control cycle. Invalid radio data never updates the accepted target state. A feasible plan checks uncertainty, braking clearance and the ±45° UWB cone; an infeasible or over-budget solve falls back to GPS drift assistance.
- Fresh measurements recover through the measurement admission path. A timeout still disarms; automatic rearm requires fresh, consistent observations and preflight checks. Operator stop/override cancels recovery intent.
- Hardware transport and device credentials are preserved on deployment. The repository retains its existing credential defaults. Deployment does not initiate following or arm the vessel.

## Validation and limits

The staged application passed 268 JavaScript tests, 58 Python tests, configuration validation and static source checks on RK3588 before activation. Backend and MAVLink bridge services were restarted while the vessel was disarmed and following was stopped. The activated source files were verified against the tested candidate hashes.

The previous complete 1435-row, fixed-path, zero-wind / original-current replay did **not** meet full-following performance criteria. The adaptive NMPC version reached 42.518 m maximum separation, 35.797 m median separation over the final 20 seconds, and 51.37° maximum actual bearing. Passing software tests does not establish complete following or continuous physical FOV retention. No new closed-loop replay or water trial was performed as part of this deployment.

The hull/thrust model, latency model and uncertainty bounds still require physical calibration. NMPC uses bounded approximate coordinate search and a 20 ms solve budget; its predicted constraints are not a guarantee on the real vessel trajectory.
