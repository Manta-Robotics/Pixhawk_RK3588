"""Conservative motor labels; no inference that neutral PWM proves a stationary hull."""
import math


def number(value):
    if value is None or isinstance(value, bool):
        return None
    try:
        value = float(value)
        return value if math.isfinite(value) else None
    except (TypeError, ValueError):
        return None


class MotorLabels:
    def __init__(self, center=1500, deadband=25, settle_s=2.0, stale_s=0.7):
        self.center = center
        self.deadband = deadband
        self.settle_s = settle_s
        self.stale_s = stale_s
        self.neutral_since = None

    def classify(self, outputs, source_mono_ns, now_ns):
        values = [number(v) for v in outputs]
        age_s = (now_ns - source_mono_ns) / 1e9 if source_mono_ns is not None else None
        if (len(values) != 2 or any(v is None or not 800 <= v <= 2200 for v in values)
                or age_s is None or not 0 <= age_s <= self.stale_s):
            self.neutral_since = None
            return 'unknown', None
        if any(abs(v - self.center) > self.deadband for v in values):
            self.neutral_since = None
            return 'driven', 0.0
        if self.neutral_since is None:
            self.neutral_since = now_ns
        duration = (now_ns - self.neutral_since) / 1e9
        return ('neutral' if duration >= self.settle_s else 'neutral_settling'), duration


def esc_power(message, channel):
    """MAVLink ESC_TELEMETRY voltage/current are centivolts/centiamps."""
    name = message.get('mavpackettype', '')
    try:
        first = int(name.split('_')[2])
        index = channel - first
        if not 0 <= index < 4:
            return None
        voltage = number(message['voltage'][index])
        current = number(message['current'][index])
        if voltage is None or current is None or not 0 < voltage < 65535 or not 0 <= current < 65535:
            return None
        return voltage * current / 10000.0
    except (ValueError, KeyError, IndexError, TypeError):
        return None
