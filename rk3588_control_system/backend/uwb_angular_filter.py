"""Smooth admitted world bearings; admission and range stay with the caller."""
import math
import statistics


def wrap(angle):
    return (angle + 180.0) % 360.0 - 180.0


class AdaptiveAngularFilter:
    def __init__(self, stationary_tau=.4, moving_tau=.06):
        self.stationary_tau, self.moving_tau = stationary_tau, moving_tau
        self.reset()

    def reset(self):
        self.angle = self.at = None
        self.points = []
        self.mode, self.rate = 'acquiring', 0.0
        self.stable_since = None

    def observe(self, raw_angle, posterior_angle, at, noise_deg):
        if self.at is None or at <= self.at or at - self.at > 250:
            self.reset()
            self.angle, self.at = posterior_angle, at
        previous_at = self.at
        unwrapped = raw_angle
        if self.points:
            unwrapped = self.points[-1][1] + wrap(raw_angle - self.points[-1][1])
        self.points = [(t, angle) for t, angle in self.points if 0 <= at - t <= 500]
        self.points.append((at, unwrapped))
        self.points = self.points[-32:]
        moving = False
        if len(self.points) >= 4 and at - self.points[0][0] >= 100:
            ts = [(t - at) / 1000 for t, _ in self.points]
            angles = [angle for _, angle in self.points]
            mt, ma = statistics.mean(ts), statistics.mean(angles)
            denominator = sum((t - mt) ** 2 for t in ts)
            self.rate = sum((t - mt) * (angle - ma) for t, angle in zip(ts, angles)) / denominator
            errors = [angle - ma - self.rate * (t - mt) for t, angle in zip(ts, angles)]
            residual = math.sqrt(sum(error ** 2 for error in errors) / max(1, len(errors) - 2))
            rate_error = max(noise_deg * .5, residual) / math.sqrt(denominator)
            moving = abs(self.rate) > max(4.0, 2.5 * rate_error)
            # Release promptly for persistent displacement, not one noisy point.
            recent = self.points[-3:]
            departures = [wrap(angle - self.angle) for _, angle in recent]
            if (at - recent[0][0] >= 30
                    and min(abs(value) for value in departures) > max(3.0, 2.5 * noise_deg)
                    and (all(value > 0 for value in departures) or all(value < 0 for value in departures))):
                moving = True
        if moving:
            self.mode, self.stable_since = 'moving', None
        else:
            if self.stable_since is None:
                self.stable_since = at
            if at - self.stable_since >= 600:
                self.mode = 'stationary'
        tau = self.stationary_tau if self.mode == 'stationary' else self.moving_tau
        dt = max(0, (at - previous_at) / 1000)
        delta = wrap(posterior_angle - self.angle)
        if self.mode != 'stationary' or abs(delta) > .5 * noise_deg:
            self.angle = wrap(self.angle + (1 - math.exp(-dt / tau)) * delta)
        self.at = at
        return self.angle
