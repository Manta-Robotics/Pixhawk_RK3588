"""Gate raw complete UWB observations before correcting a CV Kalman filter.

Two independent position/velocity axes form a conservative Cartesian model.
Azimuth calibration and fresh vessel heading remove angular wrap and rotation.
Without reliable heading, a frame is rejected instead of inventing a pose.
"""
import math
import itertools
from uwb_angular_filter import AdaptiveAngularFilter

DEFAULTS = {
    'enabled': True, 'distance_std_m': 0.12, 'angle_std_deg': 2.0,
    'acceleration_std_mps2': 2.0, 'initial_velocity_std_mps': 2.0,
    'nis_threshold': 9.21, 'absolute_gate_m': 0.6,
    'max_relative_speed_mps': 6.0, 'rejection_covariance_scale': 1.25,
    'adaptive_enabled': True, 'stationary_time_constant_s': .4, 'moving_time_constant_s': .06,
}


def wrap(degrees):
    return (degrees + 180.0) % 360.0 - 180.0


def finite(value):
    return value is not None and isinstance(value, (int, float)) and math.isfinite(value)


class Axis:
    def __init__(self, position, position_variance, velocity_variance):
        self.position, self.velocity = position, 0.0
        self.p00, self.p01, self.p11 = position_variance, 0.0, velocity_variance

    def predicted(self, dt, q):
        # Integrate continuous acceleration noise over elapsed time. Keep the
        # existing acceleration-variance tuning anchored to a 100 ms interval;
        # splitting that interval into four 25 ms polls must not reduce Q.
        density = q * 0.1
        return (self.position + self.velocity * dt,
                self.p00 + 2 * dt * self.p01 + dt * dt * self.p11 + density * dt ** 3 / 3,
                self.p01 + dt * self.p11 + density * dt * dt / 2,
                self.p11 + density * dt)

    def predict(self, dt, q):
        self.position, self.p00, self.p01, self.p11 = self.predicted(dt, q)

    def correct(self, measurement, variance):
        p00, p01, p11 = self.p00, self.p01, self.p11
        s = p00 + variance
        k0, k1 = p00 / s, p01 / s
        residual = measurement - self.position
        self.position += k0 * residual
        self.velocity += k1 * residual
        # Joseph form preserves positive covariance after accepted updates.
        self.p00 = max(1e-9, (1 - k0) ** 2 * p00 + k0 * k0 * variance)
        self.p01 = (1 - k0) * (p01 - k1 * p00) + k0 * k1 * variance
        self.p11 = max(1e-9, p11 - 2 * k1 * p01 + k1 * k1 * (p00 + variance))
        limit = math.sqrt(self.p00 * self.p11)
        self.p01 = max(-limit, min(limit, self.p01))

    def inflate(self, scale):
        # Keep rejected observations from reducing covariance, with a bounded
        # scale to avoid numeric overflow during extended radio outages.
        factor = min(scale, 1e6 / max(self.p00, self.p11, 1e-9))
        factor = max(1.0, factor)
        self.p00 *= factor; self.p01 *= factor; self.p11 *= factor


class GatedUwbKalman:
    def __init__(self, config=None, minimum_distance_m=0.15, maximum_distance_m=20.0,
                 bearing_sign=1.0, bearing_scale=1.0, bearing_offset_deg=0.0, mount_yaw_deg=0.0):
        self.config = {**DEFAULTS, **(config or {})}
        for key in DEFAULTS:
            if key not in ('enabled', 'adaptive_enabled') and (not finite(self.config[key]) or self.config[key] <= 0):
                raise ValueError('invalid UWB measurement filter parameter: ' + key)
        if not isinstance(self.config['adaptive_enabled'], bool):
            raise ValueError('adaptive_enabled must be boolean')
        if not .01 <= self.config['moving_time_constant_s'] <= self.config['stationary_time_constant_s'] <= 2:
            raise ValueError('invalid adaptive angular time constants')
        if self.config['rejection_covariance_scale'] < 1:
            raise ValueError('rejection covariance scale must not reduce uncertainty')
        if not finite(bearing_scale) or bearing_scale <= 0 or bearing_sign not in (-1, 1):
            raise ValueError('invalid bearing calibration')
        self.minimum_distance_m, self.maximum_distance_m = minimum_distance_m, maximum_distance_m
        self.sign, self.scale, self.offset, self.mount = bearing_sign, bearing_scale, bearing_offset_deg, mount_yaw_deg
        self.axes = None
        self.at = None
        self.last_accepted_at = None
        self.last_received_at = None
        self.last_sequence = None
        self.tag_id = None
        self.accepted_count, self.rejected_count = 0, 0
        self.status, self.reason = 'unavailable', 'not_initialized'
        self.nis, self.innovation_m = None, None
        # Vessel recovery: independent of innovation acceptance. Raw
        # observations retain their own clock and never refresh an accepted one.
        self.raw_window = []
        self.recovery_count = 0
        self.recovery_diagnostics = {'reason': 'warming_up', 'hits': 0}
        self.angular_filter = AdaptiveAngularFilter(self.config['stationary_time_constant_s'], self.config['moving_time_constant_s'])

    def _predict(self, now_ms):
        if self.axes is None: return
        dt = max(0.0, (now_ms - self.at) / 1000.0)
        # Bound process calculations after long gaps. The full age remains in
        # the accepted timestamp, so control cannot revive an expired sample.
        dt = min(dt, 10.0)
        for axis in self.axes: axis.predict(dt, self.config['acceleration_std_mps2'] ** 2)
        self.at = now_ms

    def reject(self, now_ms, reason, heading_deg=None):
        self._predict(now_ms)
        # Time propagation already grows covariance during loss. Rejection
        # frequency must not itself make an arbitrary new cluster trusted.
        self.rejected_count += 1
        self.status, self.reason = 'suspect', reason
        return self.result(now_ms, heading_deg)

    def raw_candidate(self, measurement, variance, at):
        self.raw_window = [p for p in self.raw_window if 0 <= at-p['at'] <= 500]
        point = dict(at=at, z=measurement, variance=variance)
        # At 40 Hz, the last five consecutive samples span only 100 ms and
        # can never meet the 180 ms consensus requirement below. Keep the
        # newest sample in each 75 ms time bucket instead. Five buckets bound
        # fit cost while retaining enough elapsed-time evidence at any rate.
        if self.raw_window and self.raw_window[-1]['at'] // 75 == at // 75:
            self.raw_window[-1] = point
        else:
            self.raw_window.append(point)
        self.raw_window = self.raw_window[-5:]
        self.recovery_diagnostics = {'reason': 'insufficient_hits', 'hits': len(self.raw_window)}
        if len(self.raw_window) < 3: return None
        def fit(points):
            ts=[(p['at']-at)/1000 for p in points]
            mt=sum(ts)/len(ts); denominator=sum((t-mt)**2 for t in ts)
            if denominator <= 0: return None
            means=[sum(p['z'][k] for p in points)/len(points) for k in (0,1)]
            velocity=[sum((t-mt)*(p['z'][k]-means[k]) for p,t in zip(points,ts))/denominator for k in (0,1)]
            position=[means[k]-velocity[k]*mt for k in (0,1)]
            return position,velocity,denominator
        candidates=[]
        for group in itertools.combinations(self.raw_window,3):
            model=fit(group)
            if not model: continue
            position,velocity,_=model
            if math.hypot(*velocity)>self.config['max_relative_speed_mps']: continue
            inliers=[p for p in self.raw_window if math.hypot(*(p['z'][k]-position[k]-velocity[k]*(p['at']-at)/1000 for k in (0,1)))<=.24]
            if len(inliers)<3 or inliers[-1]['at']!=at or inliers[-1]['at']-inliers[0]['at']<180: continue
            if any(b['at']-a['at']>450 for a,b in zip(inliers,inliers[1:])): continue
            position,velocity,denominator=fit(inliers)
            residual=max(math.hypot(*(p['z'][k]-position[k]-velocity[k]*(p['at']-at)/1000 for k in (0,1))) for p in inliers)
            if residual>.24 or math.hypot(*velocity)>self.config['max_relative_speed_mps']: continue
            mean_variance=sum(p['variance'] for p in inliers)/len(inliers)
            # Covariance at the newest observation, including fit extrapolation.
            mt=sum((p['at']-at)/1000 for p in inliers)/len(inliers)
            p00=mean_variance*(1/len(inliers)+mt*mt/denominator)+residual**2
            uncertainty=2*math.sqrt(2*p00)
            candidates.append((len(inliers),-residual,position,velocity,p00,mean_variance/denominator,uncertainty))
        if not candidates:
            self.recovery_diagnostics={'reason':'inconsistent_window','hits':0}
            return None
        best=max(candidates,key=lambda p:p[:2])
        self.recovery_diagnostics={'reason':'candidate_ready' if best[-1]<=.5 else 'candidate_uncertain',
            'hits':best[0],'residual_m':-best[1],'uncertainty_m':best[-1]}
        # Admission and raw recovery are owned here. The upper navigation layer
        # consumes the versioned accepted posterior and preserves its covariance
        # for clearance/braking; it must not repeat a conflicting raw-window fit.
        # The 0.5 m uncertainty budget applies to ordinary blind prediction,
        # not fresh admitted positions. Reset may therefore remain uncertain.
        return best

    def recover_candidate(self, candidate, now_ms, heading_deg):
        _,_,position,velocity,p00,p11,_=candidate
        self.axes=[Axis(z,p00,p11) for z in position]
        for axis,v in zip(self.axes,velocity): axis.velocity=v
        self.at=self.last_accepted_at=now_ms
        self.accepted_count+=1;self.recovery_count+=1
        self.status,self.reason='trusted','raw_window_reacquired'
        self.angular_filter.reset()
        self._update_angular_filter(position, now_ms)
        return {**self.result(now_ms,heading_deg),'accepted':True,'duplicate':False}

    def _update_angular_filter(self, measurement, now_ms):
        if self.config['adaptive_enabled']:
            posterior = math.degrees(math.atan2(self.axes[1].position, self.axes[0].position))
            raw = math.degrees(math.atan2(measurement[1], measurement[0]))
            self.angular_filter.observe(raw, posterior, now_ms, self.config['angle_std_deg'] * self.scale)

    def update(self, distance_m, azimuth_deg, now_ms, heading_deg, *, elevation_deg=None,
               valid=True, sequence_id=None, tag_id=None):
        self.nis, self.innovation_m = None, None
        if not finite(now_ms) or now_ms <= 0:
            raise ValueError('measurement time must be a positive number')
        if self.at is not None and now_ms < self.at:
            self.axes = None; self.at = None; self.last_accepted_at = None
            self.raw_window=[]
            self.angular_filter.reset()
            return self.reject(now_ms, 'measurement_clock_reset', heading_deg)
        if not finite(distance_m) or not self.minimum_distance_m <= distance_m <= self.maximum_distance_m:
            return self.reject(now_ms, 'distance_out_of_range', heading_deg)
        if not finite(azimuth_deg) or abs(azimuth_deg) > 180:
            return self.reject(now_ms, 'azimuth_out_of_range', heading_deg)
        if elevation_deg is not None and (not finite(elevation_deg) or abs(elevation_deg) > 90):
            return self.reject(now_ms, 'elevation_out_of_range', heading_deg)
        if valid is False: return self.reject(now_ms, 'radio_invalid', heading_deg)
        if not finite(heading_deg): return self.reject(now_ms, 'heading_unavailable', heading_deg)
        if (sequence_id is not None and sequence_id == self.last_sequence) or now_ms == self.last_received_at:
            # Replays do not update time, covariance, confidence or counters.
            return {**self.result(now_ms, heading_deg), 'accepted': False, 'duplicate': True}
        self.last_received_at, self.last_sequence = now_ms, sequence_id
        if tag_id and self.tag_id and tag_id != self.tag_id:
            self.axes = None; self.at = None; self.last_accepted_at = None; self.tag_id = tag_id
            self.raw_window=[]
            self.angular_filter.reset()
            return self.reject(now_ms, 'target_identity_changed', heading_deg)
        if tag_id: self.tag_id = tag_id
        angle = math.radians(wrap((azimuth_deg + self.mount) * self.sign * self.scale + self.offset + heading_deg))
        measurement = (distance_m * math.cos(angle), distance_m * math.sin(angle))
        # Isotropic R conservatively covers radial and transverse sensor noise.
        variance = self.config['distance_std_m'] ** 2 + (distance_m * math.radians(self.config['angle_std_deg'])) ** 2
        candidate=self.raw_candidate(measurement,variance,now_ms)
        if self.axes is None:
            self.axes = [Axis(z, variance, self.config['initial_velocity_std_mps'] ** 2) for z in measurement]
            self.at = now_ms
        else:
            self._predict(now_ms)
            residual = [z - axis.position for z, axis in zip(measurement, self.axes)]
            self.nis = sum(r * r / (axis.p00 + variance) for r, axis in zip(residual, self.axes))
            self.innovation_m = math.hypot(*residual)
            age = max(0, (now_ms - self.last_accepted_at) / 1000.0)
            absolute_limit = self.config['absolute_gate_m'] + self.config['max_relative_speed_mps'] * age
            if self.nis > self.config['nis_threshold'] or self.innovation_m > absolute_limit:
                if candidate and now_ms-self.last_accepted_at>=200:
                    return self.recover_candidate(candidate,now_ms,heading_deg)
                return self.reject(now_ms, 'innovation_outlier', heading_deg)
            for axis, z in zip(self.axes, measurement): axis.correct(z, variance)
        speed = math.hypot(*(axis.velocity for axis in self.axes))
        if speed > self.config['max_relative_speed_mps']:
            for axis in self.axes: axis.velocity *= self.config['max_relative_speed_mps'] / speed
        self.last_accepted_at = now_ms
        self.accepted_count += 1
        self.status, self.reason = 'trusted', 'kalman_updated'
        self._update_angular_filter(measurement, now_ms)
        return {**self.result(now_ms, heading_deg), 'accepted': True, 'duplicate': False}

    def result(self, now_ms, heading_deg=None):
        output = {'admission_version': 1, 'accepted': False, 'duplicate': False, 'status': self.status, 'reason': self.reason,
                  'accepted_at': self.last_accepted_at, 'accepted_count': self.accepted_count,
                  'rejected_count': self.rejected_count, 'nis': self.nis, 'innovation_m': self.innovation_m,
                  'uncertainty_m': None, 'distance_m': None, 'azimuth_deg': None}
        output.update(recovery_count=self.recovery_count,recovery_diagnostics=self.recovery_diagnostics.copy())
        if not self.axes: return output
        dt = min(10.0, max(0, (now_ms - self.at) / 1000.0))
        predicted = [axis.predicted(dt, self.config['acceleration_std_mps2'] ** 2) for axis in self.axes]
        output['uncertainty_m'] = 2 * math.sqrt(max(0, predicted[0][1] + predicted[1][1]))
        x, y = predicted[0][0], predicted[1][0]
        distance = math.hypot(x, y)
        world_angle = math.degrees(math.atan2(y, x))
        smooth_angle = self.angular_filter.angle if self.config['adaptive_enabled'] and self.angular_filter.angle is not None else world_angle
        displacement = 2 * distance * abs(math.sin(math.radians(wrap(smooth_angle - world_angle)) / 2))
        output['base_uncertainty_m'] = output['uncertainty_m']
        output['uncertainty_m'] += displacement
        output.update(angular_filter_mode=self.angular_filter.mode if self.config['adaptive_enabled'] else 'disabled',
                      angular_filter_rate_deg_s=self.angular_filter.rate / self.scale,
                      smoothing_displacement_m=displacement)
        if finite(heading_deg):
            output['distance_m'] = distance
            original_angle = wrap(world_angle - heading_deg - self.offset)
            output['unsmoothed_azimuth_deg'] = wrap(original_angle / (self.sign * self.scale) - self.mount)
            physical_angle = wrap(smooth_angle - heading_deg - self.offset)
            output['azimuth_deg'] = wrap(physical_angle / (self.sign * self.scale) - self.mount)
        return output
