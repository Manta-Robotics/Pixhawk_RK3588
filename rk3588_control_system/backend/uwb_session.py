"""Reference implementation of the native UWB session fence and stop latch."""
import math
import secrets


def delta32(value, origin):
    return ((int(value) - int(origin) + 0x80000000) & 0xFFFFFFFF) - 0x80000000


class UwbSession:
    def __init__(self):
        self.nonce = secrets.randbits(62)
        self.reset()

    def reset(self):
        self.ready = self.pending = False
        self.sent = -1e12
        self.boot = self.floor = self.at = self.rtt = 0
        self.started = set()

    def request(self, now):
        if now - self.sent < 1000:
            return None
        self.nonce += 1
        self.sent, self.pending = now, True
        return self.nonce

    def reply(self, tc1, ts1, now):
        if not (self.pending and tc1 > 0 and ts1 == self.nonce and 0 <= now - self.sent <= 500):
            return None
        boot = (tc1 // 1_000_000) & 0xFFFFFFFF
        reset = not self.ready or abs(delta32(boot, self.boot) - (now - self.at)) > 1500
        if reset:
            self.floor = boot
            self.started.clear()
        elif delta32(boot, self.floor) > 500:
            self.floor = (boot - 500) & 0xFFFFFFFF
        self.boot, self.at, self.rtt = boot, now, now - self.sent
        self.ready, self.pending = True, False
        return reset

    def admit(self, source, parsed, stamp, now):
        if not self.ready or parsed is None or stamp is None:
            return False
        age = delta32(stamp, self.boot)
        if delta32(stamp, self.floor) <= 0 or not now - self.at - 500 <= age <= now - self.at + self.rtt + 250:
            return False
        key = (*source, parsed[0])
        if parsed[1] == 'DIST':
            self.started.add(key)
        elif parsed[1] in ('EL', 'AZ') and key not in self.started:
            return False
        return True


class UwbClearance:
    def __init__(self, config=None):
        config = config or {}
        self.stop = max(.1, config.get('safety_distance_m', .45))
        self.resume = max(self.stop + .05, config.get('hard_stop_resume_distance_m', self.stop + .1))
        self.states = {}

    def acknowledge(self, now):
        for state in self.states.values():
            if state['ack'] is None:
                state['ack'] = now

    def update(self, receiver, field, value, now, gated, enabled):
        key = receiver.receiver_id
        if key not in enabled:
            return
        distance = field == 'DIST' or (field == 'REJECTED' and receiver.invalid_reason == 'distance_out_of_range')
        state = self.states.get(key)
        if distance and math.isfinite(value) and 0 <= value <= self.stop:
            self.states[key] = dict(distance=min(state['distance'], value) if state else value,
                                    at=now, ack=None, since=None, last=None, hits=0)
            return
        if state is None:
            return
        if field == 'REJECTED' or (field == 'VALID' and receiver.valid is False):
            state.update(since=None, last=None, hits=0)
        if not receiver.measurement_completed:
            return
        safe = (receiver.valid is not False and (not gated or receiver.measurement_status == 'trusted')
                and receiver.raw_distance_m is not None and receiver.raw_distance_m >= self.resume
                and receiver.distance_m is not None and receiver.distance_m >= self.resume)
        if not safe or state['ack'] is None or now <= state['ack']:
            state.update(since=None, last=None, hits=0)
            return
        if state['last'] is None or not 0 <= now - state['last'] <= 150:
            state.update(since=now, hits=0)
        if state['last'] is None or now > state['last']:
            state['hits'] += 1
        state['last'] = now
        if now - state['since'] >= 500 and state['hits'] >= 3:
            del self.states[key]

    def payload(self):
        return dict(clearanceStop=bool(self.states),
                    clearanceDistanceM=min((s['distance'] for s in self.states.values()), default=None),
                    clearanceObservedAt=max((s['at'] for s in self.states.values()), default=None))
