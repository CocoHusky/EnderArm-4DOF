"""Calibration and fault logic for FRDM-KL02Z accelerometer data."""
import math
import time


def _unit(values):
    vals=[float(v) for v in values]
    mag=math.sqrt(sum(v*v for v in vals))
    if not math.isfinite(mag) or mag<1e-9:
        raise ValueError("acceleration vector is invalid")
    return [v/mag for v in vals],mag


def tilt_degrees(accel_g, reference):
    a,_=_unit(accel_g)
    b,_=_unit(reference)
    dot=max(-1.0,min(1.0,sum(x*y for x,y in zip(a,b))))
    return math.degrees(math.acos(dot))


class TiltMonitor:
    def __init__(self, allowed_deg=15.0, persistence_s=0.75):
        self.allowed_deg=float(allowed_deg)
        self.persistence_s=float(persistence_s)
        self.reference=None
        self.enabled=False
        self.kill_enabled=False
        self.fault_latched=False
        self.fault_reason=None
        self._outside_since=None
        self.last_tilt_deg=None

    def configure(self, allowed_deg=None, persistence_s=None):
        if allowed_deg is not None:
            value=float(allowed_deg)
            if not 1.0<=value<=60.0:
                raise ValueError("allowed tilt must be 1–60 degrees")
            self.allowed_deg=value
        if persistence_s is not None:
            value=float(persistence_s)
            if not 0.05<=value<=10.0:
                raise ValueError("persistence must be 0.05–10 seconds")
            self.persistence_s=value

    def calibrate(self, accel_g):
        ref,mag=_unit(accel_g)
        if not 0.6<=mag<=1.4:
            raise ValueError("calibrate only while board is stationary near 1 g")
        self.reference=ref
        self.fault_latched=False
        self.fault_reason=None
        self._outside_since=None
        self.last_tilt_deg=0.0
        return list(ref)

    def acknowledge(self):
        self.fault_latched=False
        self.fault_reason=None
        self._outside_since=None

    def update(self, accel_g, healthy=True, now=None):
        now=time.monotonic() if now is None else float(now)
        if self.reference is None or not healthy:
            self.last_tilt_deg=None
            self._outside_since=None
            return False
        angle=tilt_degrees(accel_g,self.reference)
        self.last_tilt_deg=angle
        if not self.enabled or self.fault_latched:
            self._outside_since=None
            return False
        if angle<=self.allowed_deg:
            self._outside_since=None
            return False
        if self._outside_since is None:
            self._outside_since=now
            return False
        if now-self._outside_since<self.persistence_s:
            return False
        self.fault_latched=True
        self.fault_reason=f"tilt {angle:.1f}° exceeded {self.allowed_deg:.1f}° for {self.persistence_s:.2f}s"
        return True

    def snapshot(self):
        return {
            "calibrated":self.reference is not None,
            "reference":None if self.reference is None else list(self.reference),
            "enabled":self.enabled,
            "kill_enabled":self.kill_enabled,
            "allowed_deg":self.allowed_deg,
            "persistence_s":self.persistence_s,
            "tilt_deg":self.last_tilt_deg,
            "fault_latched":self.fault_latched,
            "fault_reason":self.fault_reason,
        }
