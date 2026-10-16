"""Opt-in, sampled acceleration guards; not a hardware safety interlock."""
import math
import time


class SensorGuard:
    def __init__(self):
        self.options = dict(impact_stop=False, impact_flash=False, z_stop=False,
                            z_color=False, swipe_colors=False,
                            burst_stop=False, burst_g=0.65, burst_count=3,
                            burst_window_s=0.6,
                            impact_g=1.1, z_g=0.3, z_persistence_s=0.15)
        self.previous = None
        self.z_since = None
        self.flash_until = 0.0
        self.fault = None
        self.delta_g = None
        self._reset_burst()

    def _reset_burst(self):
        self._baseline = None
        self._burst_at = None
        self._peak_direction = None
        self._peak_rearmed = True
        self._peaks = []

    def _update_burst(self, accel, now):
        # Remove slow gravity/orientation changes, but retain raw vibration.
        # Opposite lobes count separately; one sustained excursion counts once.
        if self._baseline is None or now <= self._burst_at or now-self._burst_at > .1:
            self._reset_burst()
            self._baseline = list(accel)
            self._burst_at = now
            return None
        dt = now-self._burst_at
        self._burst_at = now
        residual = [v-b for v, b in zip(accel, self._baseline)]
        magnitude = math.sqrt(sum(v*v for v in residual))
        alpha = 1-math.exp(-dt/.2)
        self._baseline = [b+alpha*r for b, r in zip(self._baseline, residual)]
        self._peaks = [t for t in self._peaks if now-t <= self.options['burst_window_s']]
        if self._peaks and now-self._peaks[-1] > .3:
            self._peaks = []
        if magnitude < self.options['burst_g']*.4:
            self._peak_rearmed = True
        direction = [v/magnitude for v in residual] if magnitude else [0, 0, 0]
        reversed_peak = (self._peak_direction is not None and
                         sum(a*b for a, b in zip(direction, self._peak_direction)) < -.5)
        if (magnitude >= self.options['burst_g'] and
                (self._peak_rearmed or reversed_peak) and
                (not self._peaks or now-self._peaks[-1] >= .03)):
            self._peaks.append(now)
            self._peak_direction = direction
            self._peak_rearmed = False
        if self.options['burst_stop'] and len(self._peaks) >= self.options['burst_count']:
            return 'vibration burst: %d large spikes in %.2fs (suspected skipping)' % (
                len(self._peaks), self._peaks[-1]-self._peaks[0])
        return None

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
                          'burst_g': (.1, 8), 'burst_count': (2, 12),
                          'burst_window_s': (.1, 2),
                          'z_persistence_s': (0.05, 2)}[key]
                if not math.isfinite(value) or not lo <= value <= hi:
                    raise ValueError(key + ' outside allowed range')
                if key == 'burst_count' and value != int(value):
                    raise ValueError('burst count must be a whole number')
            options[key] = value
        self.options = options
        self.previous = None
        self.z_since = None
        self.flash_until = 0.0
        self._reset_burst()

    def acknowledge(self):
        self.fault = None
        self.z_since = None
        self.previous = None
        self._reset_burst()

    def update(self, accel, healthy=True, now=None):
        now = time.monotonic() if now is None else now
        if not healthy or len(accel) != 3 or not all(math.isfinite(v) for v in accel):
            self.previous = None
            self.z_since = None
            self.delta_g = None
            self._reset_burst()
            return None
        burst_reason = self._update_burst(accel, now)
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
        reason = burst_reason
        if reason:
            self.flash_until = now + 1.5
        elif impact and self.options['impact_stop']:
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
        return dict(self.options, fault=self.fault, delta_g=self.delta_g,
                    burst_spikes=len(self._peaks))
