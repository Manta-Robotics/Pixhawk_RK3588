"""Quarantine discontinuous module output before Kalman correction/recovery.

This is a continuity check, not a radio NLOS detector: the serial protocol has
no channel-quality information. A smooth, biased path can remain undetectable.
"""
import math


class UwbOcclusionGuard:
    def __init__(self, speed_limit, distance_noise, angle_noise, recovery_ms=250):
        self.speed_limit = speed_limit
        self.step_margin = max(.24, 2 * distance_noise)
        self.angle_margin = max(12., 3 * angle_noise)
        self.recovery_ms = recovery_ms
        self.reset()

    def reset(self):
        self.anchor = None
        self.history = []
        self.velocity = (0., 0.)
        self.reference_position = None
        self.bad_count = 0
        self.interrupt_confirmation()
        self.diagnostics = {'state': 'clear', 'reason': None}

    def confirm_new_track(self, position, at):
        # Bound motion-model residuals, not displacement from a fixed cluster.
        # Validate even packets between the retained 25 ms evidence samples.
        consistent = (self.candidate_last_at is not None
                      and 0 <= at - self.candidate_last_at <= 150)
        if consistent:
            prior = self.candidate_last_position
            budget = self.speed_limit * (at - self.candidate_last_at) / 1000
            angle = abs(math.degrees(math.atan2(
                position[1] * prior[0] - position[0] * prior[1],
                position[0] * prior[0] + position[1] * prior[1])))
            consistent = (math.dist(position, prior) <= self.step_margin + budget
                          and angle <= self.angle_margin + math.degrees(
                              budget / max(.15, min(math.hypot(*position), math.hypot(*prior)))))
        trial = list(self.reacquisition)
        if trial and trial[-1][0] == at:
            trial[-1] = (at, position)
        else:
            trial.append((at, position))
        velocity, residual = None, 0.
        if consistent and len(trial) >= 4 and at - trial[0][0] >= 100:
            ts = [(t-at)/1000 for t, _ in trial]
            mt = sum(ts)/len(ts)
            denominator = sum((t-mt)**2 for t in ts)
            means = tuple(sum(z[k] for _, z in trial)/len(trial) for k in (0, 1))
            velocity = tuple(sum((t-mt)*(z[k]-means[k]) for t, (_, z) in zip(ts, trial))
                             / denominator for k in (0, 1))
            fitted = tuple(m-v*mt for m, v in zip(means, velocity))
            consistent = math.hypot(*velocity) <= self.speed_limit
            for t, z in trial:
                expected = tuple(p+v*(t-at)/1000 for p, v in zip(fitted, velocity))
                residual = max(residual, math.dist(z, expected))
                angle = abs(math.degrees(math.atan2(z[1]*expected[0]-z[0]*expected[1],
                                                   z[0]*expected[0]+z[1]*expected[1])))
                consistent = consistent and residual <= self.step_margin and angle <= self.angle_margin
        if not consistent:
            self.reacquisition = []
            velocity = None
        self.candidate_last_at = at
        self.candidate_last_position = position
        self.candidate_velocity = velocity or (0., 0.)
        if not self.reacquisition or at - self.reacquisition[-1][0] >= 25:
            self.reacquisition.append((at, position))
        self.reacquisition = [(t, z) for t, z in self.reacquisition if 0 <= at - t <= 3200][-128:]
        span = at - self.reacquisition[0][0]
        self.diagnostics.update(candidate_hits=len(self.reacquisition), candidate_ms=span,
                                candidate_required_ms=1500,
                                candidate_speed_mps=math.hypot(*velocity) if velocity else None,
                                candidate_residual_m=residual if velocity else None)
        ready = span >= 1500 and len(self.reacquisition) >= 16
        if ready:
            self.diagnostics.update(state='recovering', reason='stable_new_track_confirmed')
        return ready

    def check(self, position, at):
        if self.anchor is None:
            return True
        anchor_at, anchor = self.anchor
        dt = (at - anchor_at) / 1000
        distance = math.hypot(*position)
        previous_distance = math.hypot(*anchor)
        step = math.dist(position, anchor)
        angle = abs(math.degrees(math.atan2(
            position[1] * anchor[0] - position[0] * anchor[1],
            position[0] * anchor[0] + position[1] * anchor[1])))
        # Only a short gap provides evidence of an impossible instantaneous
        # jump. Long radio gaps still go through the original Kalman gates.
        motion_budget = self.speed_limit * max(0., dt)
        angle_budget = math.degrees(motion_budget / max(.15, min(distance, previous_distance)))
        jump = (0 < dt <= .3 and (step > self.step_margin + motion_budget
                or angle > self.angle_margin + angle_budget))
        if not self.bad_count and not jump:
            self.diagnostics = {'state': 'clear', 'reason': None}
            return True
        # Freeze the old return corridor. A displaced cluster uses a separate,
        # longer confirmation window rather than widening this corridor.
        horizon = min(max(dt, 0.), .25)
        expected = tuple(z + v * horizon for z, v in zip(self.reference_position, self.velocity))
        return_error = math.dist(position, expected)
        return_angle = abs(math.degrees(math.atan2(
            position[1] * expected[0] - position[0] * expected[1],
            position[0] * expected[0] + position[1] * expected[1])))
        corridor = max(.30, self.step_margin)
        returning = return_error <= corridor and return_angle <= self.angle_margin and not jump
        if not returning:
            self.bad_count += 1
            self.return_at = self.return_last_at = None
            self.return_hits = 0
            self.diagnostics = {'state': 'quarantined' if self.bad_count >= 2 else 'suspect',
                                'reason': 'raw_position_jump', 'step_m': step,
                                'return_error_m': return_error, 'return_angle_error_deg': return_angle,
                                'bad_count': self.bad_count}
            return self.confirm_new_track(position, at)
        self.reacquisition = []
        self.candidate_last_at = None
        self.candidate_velocity = (0., 0.)
        # One isolated spike may return immediately to the existing track.
        # A burst requires uninterrupted fresh observations for recovery_ms.
        if self.bad_count < 2:
            self.diagnostics = {'state': 'clear', 'reason': 'isolated_spike_returned'}
            return True
        if self.return_last_at is None or at - self.return_last_at > 150:
            self.return_at, self.return_hits = at, 0
        self.return_last_at = at
        self.return_hits += 1
        ready = at - self.return_at >= self.recovery_ms and self.return_hits >= 3
        self.diagnostics = {'state': 'recovering', 'reason': 'confirming_original_track',
                            'return_error_m': return_error, 'return_angle_error_deg': return_angle,
                            'hits': self.return_hits,
                            'confirmed_ms': at - self.return_at, 'required_ms': self.recovery_ms}
        return ready

    def interrupt_confirmation(self):
        self.return_at = self.return_last_at = None
        self.return_hits = 0
        self.reacquisition = []
        self.candidate_last_at = None
        self.candidate_velocity = (0., 0.)

    def radio_loss(self):
        if self.anchor is not None:
            self.bad_count += 1
            self.interrupt_confirmation()
            self.diagnostics = {'state': 'quarantined' if self.bad_count >= 2 else 'suspect',
                                'reason': 'radio_loss', 'bad_count': self.bad_count}

    def accept(self, position, at):
        new_track = self.diagnostics.get('reason') == 'stable_new_track_confirmed'
        recovering = self.bad_count >= 2
        if recovering:
            self.history = []
        self.history = [(t, z) for t, z in self.history if 0 <= at - t <= 250]
        self.history.append((at, position))
        self.history = self.history[-32:]
        self.velocity = (0., 0.)
        self.reference_position = position
        if len(self.history) >= 3 and at - self.history[0][0] >= 100:
            ts = [(t - at) / 1000 for t, _ in self.history]
            mt = sum(ts) / len(ts)
            denominator = sum((t - mt) ** 2 for t in ts)
            means = tuple(sum(z[k] for _, z in self.history) / len(ts) for k in (0, 1))
            velocity = tuple(sum((t - mt) * (z[k] - means[k])
                for t, (_, z) in zip(ts, self.history)) / denominator for k in (0, 1))
            self.reference_position = tuple(m - v * mt for m, v in zip(means, velocity))
            speed = math.hypot(*velocity)
            # Do not extrapolate a stationary noise slope into a displaced
            # recovery corridor. Short genuine motion still fits its margin.
            if speed > 2.5 * (self.step_margin / 2) / math.sqrt(denominator):
                self.velocity = tuple(v * min(1., self.speed_limit / max(speed, 1e-9)) for v in velocity)
        self.anchor = (at, position)
        self.bad_count = 0
        self.interrupt_confirmation()
        self.diagnostics = {'state': 'clear', 'reason': 'stable_new_track_confirmed' if new_track
                            else 'original_track_confirmed' if recovering else None}
