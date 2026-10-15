"""Opt-in, sampled acceleration guards; not a hardware safety interlock."""
import math
import time


class SensorGuard:
    def __init__(self):
        self.options = dict(impact_stop=False, impact_flash=False, z_stop=False,
                            z_color=False, swipe_colors=False,
                            impact_g=1.1, z_g=0.3, z_persistence_s=0.15)
        self.previous = None
        self.z_since = None
        self.flash_until = 0.0
        self.fault = None
        self.delta_g = None

    def configure(self, values):
        options = dict(self.options)
        for key, value in values.items():
            if key not in options:
                raise ValueError('unknown sensor guard option')
            if isinstance(options[key], bool):
                if type(value) is not bool:
                    raise ValueError(key + ' must be true or false')
            else:
                value = float(value)
                lo, hi = {'impact_g': (0.1, 8), 'z_g': (0.05, 2),
                          'z_persistence_s': (0.05, 2)}[key]
                if not math.isfinite(value) or not lo <= value <= hi:
                    raise ValueError(key + ' outside allowed range')
            options[key] = value
        self.options = options
        self.previous = None
        self.z_since = None
        self.flash_until = 0.0

    def acknowledge(self):
        self.fault = None
        self.z_since = None
        self.previous = None

    def update(self, accel, healthy=True, now=None):
        now = time.monotonic() if now is None else now
        if not healthy or len(accel) != 3 or not all(math.isfinite(v) for v in accel):
            self.previous = None
            self.z_since = None
            self.delta_g = None
            return None
        self.delta_g = None if self.previous is None else math.sqrt(sum(
            (a-b)**2 for a, b in zip(accel, self.previous)))
        self.previous = list(accel)
        impact = self.delta_g is not None and self.delta_g >= self.options['impact_g']
        if impact and self.options['impact_flash']:
            self.flash_until = now + 1.5
        outside = abs(accel[2]) >= self.options['z_g']
        if not outside:
            self.z_since = None
        elif self.z_since is None:
            self.z_since = now
        reason = None
        if impact and self.options['impact_stop']:
            reason = 'impact: sample change %.2f g' % self.delta_g
        elif (outside and self.options['z_stop'] and self.z_since is not None
              and now-self.z_since >= self.options['z_persistence_s']):
            reason = '|Z| %.2f g exceeded %.2f g' % (abs(accel[2]), self.options['z_g'])
        if reason and self.fault is None:
            self.fault = reason
            return reason
        return None

    def led(self, accel, now=None):
        now = time.monotonic() if now is None else now
        if now < self.flash_until:
            return 'red' if int(now * 10) % 2 == 0 else 'off'
        if self.options['z_color'] and accel and abs(accel[2]) >= self.options['z_g']:
            return 'yellow'
        return None

    def snapshot(self):
        return dict(self.options, fault=self.fault, delta_g=self.delta_g)
