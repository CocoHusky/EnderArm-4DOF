"""Optional FRDM-KL02Z host service.

Owns serial health, real-sample calibration, tilt latching, and disabled-by-default
swipe automation. It has no dependency on robot motion implementation; all robot
actions arrive through explicit callbacks.
"""
import json
import math
import os
import threading
import time

from sensor_monitor import TiltMonitor
from sensor_transport import SensorSerial
from sensor_guard import SensorGuard


LED_COLORS = ("off", "red", "green", "blue", "yellow", "cyan", "magenta", "white")
SENSOR_MODES = ("manual", "acceleration", "orientation", "slider", "tap", "motion")
DEFAULT_TAPS = [
    {"program": "", "color": "red"},
    {"program": "", "color": "green"},
    {"program": "", "color": "blue"},
]
GESTURES = ('press_1', 'press_2', 'press_3', 'swipe_left', 'swipe_right')
GESTURE_LABELS = ('Press 1 · left', 'Press 2 · center', 'Press 3 · right',
                  'Swipe left', 'Swipe right')


class SensorService:
    def __init__(
        self,
        config_path,
        port,
        swipe_feature=False,
        on_kill=None,
        can_run_program=None,
        run_program=None,
    ):
        self.config_path = str(config_path)
        self.transport = SensorSerial(port, 115200, stale_after=0.75)
        self.swipe_feature = bool(swipe_feature)
        self.on_kill = on_kill
        self.can_run_program = can_run_program or (lambda: (False, "robot callback unavailable"))
        self.run_program = run_program or (lambda name: None)
        self.monitor = TiltMonitor()
        self.guard = SensorGuard()
        self._guard_led = None
        self._feedback_base = 'off'
        self._lock = threading.RLock()
        self._stop = threading.Event()
        self._thread = None
        self._last_seq = None
        self._ever_connected = False
        self._last_reconnects = 0
        self.swipe_enabled = False
        self.selected_tap = 0
        self.countdown_deadline = None
        self.countdown_program = None
        self.countdown_error = None
        self.last_swipe = "none"
        self.last_swipe_at = None
        self.taps = [dict(x) for x in DEFAULT_TAPS]
        self.gestures = {key: {'program': '', 'color': color} for key, color in
                         zip(GESTURES, ('red', 'green', 'blue', 'cyan', 'magenta'))}
        self.last_gesture = None
        self._last_gesture_id = None
        self._load_config()

    def _load_config(self):
        try:
            with open(self.config_path) as stream:
                raw = json.load(stream)
        except (FileNotFoundError, json.JSONDecodeError, OSError):
            return
        if not isinstance(raw, dict):
            return
        try:
            self.monitor.configure(raw.get("allowed_deg"), raw.get("persistence_s"))
        except (TypeError, ValueError):
            pass
        ref = raw.get("reference")
        if isinstance(ref, list) and len(ref) == 3:
            try:
                values = [float(v) for v in ref]
                mag = math.sqrt(sum(v*v for v in values))
                if 0.9 <= mag <= 1.1:
                    self.monitor.reference = [v/mag for v in values]
            except (TypeError, ValueError):
                pass
        taps = raw.get("taps")
        if isinstance(taps, list) and taps:
            clean = []
            for item in taps[:8]:
                if not isinstance(item, dict):
                    continue
                color = str(item.get("color", "off")).lower()
                if color not in LED_COLORS:
                    color = "off"
                clean.append({"program": str(item.get("program", ""))[:80], "color": color})
            if clean:
                self.taps = clean
                for key, tap in zip(GESTURES[:3], clean):
                    self.gestures[key] = dict(tap)
        mappings = raw.get('gestures')
        if isinstance(mappings, dict):
            for key in GESTURES:
                item = mappings.get(key)
                if isinstance(item, dict) and item.get('color') in LED_COLORS:
                    self.gestures[key] = {'program': str(item.get('program', ''))[:80],
                                          'color': item['color']}
        try:
            self.selected_tap = max(0, min(len(self.taps)-1, int(raw.get("selected_tap", 0))))
        except (TypeError, ValueError):
            self.selected_tap = 0

    def _save_config(self):
        os.makedirs(os.path.dirname(self.config_path), exist_ok=True)
        payload = {
            "allowed_deg": self.monitor.allowed_deg,
            "persistence_s": self.monitor.persistence_s,
            "reference": self.monitor.reference,
            "taps": self.taps,
            "selected_tap": self.selected_tap,
            "gestures": self.gestures,
        }
        tmp = self.config_path + ".tmp"
        with open(tmp, "w") as stream:
            json.dump(payload, stream, separators=(",", ":"))
        os.replace(tmp, self.config_path)

    def start(self):
        if self._thread and self._thread.is_alive():
            return
        self._stop.clear()
        self.transport.start()
        self._thread = threading.Thread(target=self._run, name="sensor-service", daemon=True)
        self._thread.start()

    def stop(self):
        self._stop.set()
        self.transport.stop()
        if self._thread and self._thread.is_alive():
            self._thread.join(timeout=1.5)

    @staticmethod
    def _frame_healthy(link):
        frame = link.get("frame")
        if not link.get("fresh") or not isinstance(frame, dict):
            return False
        health = frame.get("health")
        return bool(isinstance(health, dict) and health.get("accel") and health.get("slider"))

    def _cancel_swipe(self, reason=None, disable=False):
        with self._lock:
            self.countdown_deadline = None
            self.countdown_program = None
            if reason:
                self.countdown_error = str(reason)
            if disable:
                self.swipe_enabled = False

    def _handle_swipe(self, direction):
        if direction not in ("left", "right"):
            return
        with self._lock:
            if not self.swipe_feature or not self.swipe_enabled or not self.taps:
                return
            step = -1 if direction == "left" else 1
            self.selected_tap = (self.selected_tap + step) % len(self.taps)
            tap = self.taps[self.selected_tap]
            self.last_swipe = direction
            self.last_swipe_at = time.time()
            self.countdown_deadline = time.monotonic() + 2.0
            self.countdown_program = tap.get("program") or None
            self.countdown_error = None
            color = tap.get("color", "off")
        try:
            self.transport.send_command("LED " + color.upper())
        except Exception as exc:
            self._cancel_swipe(f"LED command failed: {exc}")

    def _handle_gesture(self, key):
        if key not in self.gestures:
            return
        with self._lock:
            self.last_gesture = key
            mapping = self.gestures[key]
            self._feedback_base = mapping['color']
            armed = self.swipe_feature and self.swipe_enabled
            self._cancel_swipe()
            if armed and not self.guard.fault and not self.monitor.fault_latched:
                self.countdown_program = mapping['program'] or None
                self.countdown_deadline = time.monotonic() + 2.0 if self.countdown_program else None
                self.countdown_error = None if self.countdown_program else 'Gesture has no saved program'
            feedback = armed or self.guard.options['swipe_colors']
        if feedback and self._guard_led is None:
            try:
                self.transport.send_command('LED ' + mapping['color'].upper())
            except Exception as exc:
                self._cancel_swipe('LED command failed: ' + str(exc))

    def _process_gesture(self, slider, healthy):
        """Consume persistent event IDs once; never replay an event on reconnect."""
        event_id = slider.get('gesture_id')
        if not healthy:
            self._last_gesture_id = None
            return
        if slider.get('active'):
            self._cancel_swipe()
        if type(event_id) is int:
            previous = self._last_gesture_id
            self._last_gesture_id = event_id
            if previous is not None and event_id > previous:
                self._handle_gesture(slider.get('gesture'))
        elif slider.get('swipe') in ('left', 'right'):
            self._handle_swipe(slider['swipe'])

    def _countdown_tick(self, link):
        with self._lock:
            deadline = self.countdown_deadline
            program = self.countdown_program
            enabled = self.swipe_enabled
        if deadline is None or time.monotonic() < deadline:
            return
        self._cancel_swipe()
        if not enabled:
            return
        if self.guard.fault or self.monitor.fault_latched:
            self._cancel_swipe('sensor fault requires acknowledgement', disable=True)
            return
        if not self._frame_healthy(link):
            self._cancel_swipe("sensor health invalid at countdown expiry", disable=True)
            return
        if not program:
            self._cancel_swipe("selected tap has no saved program")
            return
        ok, reason = self.can_run_program()
        if not ok:
            self._cancel_swipe(reason or "robot is not ready")
            return
        try:
            self.run_program(program)
        except Exception as exc:
            self._cancel_swipe(str(exc))

    def _run(self):
        while not self._stop.wait(0.025):
            link = self.transport.snapshot()
            self._update_guard_led(link)
            reconnects = int(link.get("reconnects") or 0)
            connected = bool(link.get("connected"))
            if connected:
                self._ever_connected = True
            if self._ever_connected and (not connected or reconnects > self._last_reconnects + (1 if self._last_reconnects == 0 else 0)):
                self._last_gesture_id = None
                self._cancel_swipe("sensor disconnected/reconnected; swipe automation requires explicit re-enable", disable=True)
            self._last_reconnects = max(self._last_reconnects, reconnects)

            frame = link.get("frame")
            if not link.get("fresh") or not isinstance(frame, dict):
                self._last_gesture_id = None
                self.monitor.update([0, 0, 1], healthy=False)
                self._countdown_tick(link)
                continue
            seq = frame.get("seq")
            if seq == self._last_seq:
                self._countdown_tick(link)
                continue
            self._last_seq = seq

            accel = frame.get("accel_mg") or []
            try:
                accel_g = [float(v) / 1000.0 for v in accel]
            except (TypeError, ValueError):
                accel_g = []
            healthy = self._frame_healthy(link) and len(accel_g) == 3
            accel_healthy = bool(link.get('fresh') and frame.get('health', {}).get('accel') and len(accel_g) == 3)
            reason = self.guard.update(accel_g, healthy=accel_healthy)
            if reason:
                self._cancel_swipe(reason, disable=True)
                if self.on_kill:
                    try:
                        self.on_kill(reason)
                    except Exception as exc:
                        self.countdown_error = 'KILL request failed: ' + str(exc)
            triggered = self.monitor.update(accel_g if len(accel_g) == 3 else [0, 0, 1], healthy=healthy)
            if triggered and self.monitor.kill_enabled and self.on_kill:
                try:
                    self.on_kill(self.monitor.fault_reason)
                except Exception:
                    pass

            slider = frame.get("slider") or {}
            self._process_gesture(slider, healthy)
            self._countdown_tick(link)

    def calibrate(self, sample_count=20, timeout_s=2.0):
        samples = []
        seen = set()
        deadline = time.monotonic() + float(timeout_s)
        while len(samples) < int(sample_count) and time.monotonic() < deadline:
            link = self.transport.snapshot()
            frame = link.get("frame")
            if self._frame_healthy(link) and isinstance(frame, dict):
                seq = frame.get("seq")
                if seq not in seen:
                    seen.add(seq)
                    a = frame.get("accel_mg")
                    if isinstance(a, list) and len(a) == 3:
                        samples.append([float(v)/1000.0 for v in a])
            time.sleep(0.025)
        if len(samples) < max(5, int(sample_count)//2):
            raise RuntimeError("not enough fresh accelerometer samples for calibration")
        avg = [sum(v[i] for v in samples)/len(samples) for i in range(3)]
        spread = max(
            math.sqrt(sum((v[i]-avg[i])**2 for i in range(3)))
            for v in samples
        )
        if spread > 0.12:
            raise RuntimeError("board moved during calibration; keep the robot stationary")
        ref = self.monitor.calibrate(avg)
        self._save_config()
        return {"reference": ref, "samples": len(samples), "max_deviation_g": spread}

    def configure_monitor(self, allowed_deg=None, persistence_s=None, enabled=None, kill_enabled=None):
        with self._lock:
            self.monitor.configure(allowed_deg, persistence_s)
            if enabled is not None:
                self.monitor.enabled = bool(enabled)
            if kill_enabled is not None:
                if kill_enabled and not self.monitor.enabled:
                    raise ValueError("enable tilt monitoring before enabling KILL")
                self.monitor.kill_enabled = bool(kill_enabled)
            self._save_config()
        return self.snapshot()

    def acknowledge_fault(self):
        self.monitor.acknowledge()
        self.guard.acknowledge()
        return self.snapshot()

    def configure_guard(self, options):
        if not isinstance(options, dict):
            raise ValueError('guard options must be an object')
        with self._lock:
            self.guard.configure(options)
        return self.snapshot()

    def _update_guard_led(self, link):
        frame = link.get('frame') or {}
        if not link.get('fresh'):
            self.guard.update([], healthy=False)
            return
        accel = [float(v)/1000 for v in frame.get('accel_mg', [])]
        color = self.guard.led(accel) if frame.get('health', {}).get('accel') else None
        if color == self._guard_led:
            return
        if self._guard_led is None:
            self._feedback_base = frame.get('led', 'off')
        try:
            self.transport.send_command('LED ' + (color or self._feedback_base).upper())
            self._guard_led = color
        except Exception:
            pass

    def set_led(self, color):
        color = str(color).lower()
        if color not in LED_COLORS:
            raise ValueError("unsupported LED color")
        self.transport.send_command("LED " + color.upper())
        return self.snapshot()

    def set_mode(self, mode):
        mode = str(mode).lower()
        if mode not in SENSOR_MODES:
            raise ValueError("unsupported sensor feedback mode")
        if not self.transport.snapshot().get('fresh'):
            raise RuntimeError('Wait for live sensor readings before changing mode')
        self.transport.send_command('MODE ' + mode.upper())
        return self.snapshot()

    def set_taps(self, taps):
        if not isinstance(taps, list) or not 1 <= len(taps) <= 8:
            raise ValueError("taps must contain 1–8 entries")
        clean = []
        for item in taps:
            if not isinstance(item, dict):
                raise ValueError("each tap must be an object")
            color = str(item.get("color", "off")).lower()
            if color not in LED_COLORS:
                raise ValueError("unsupported tap LED color")
            clean.append({"program": str(item.get("program", ""))[:80], "color": color})
        with self._lock:
            self.taps = clean
            self.selected_tap = min(self.selected_tap, len(clean)-1)
            self._save_config()
        return self.snapshot()

    def set_gestures(self, mappings):
        if not isinstance(mappings, dict) or set(mappings) != set(GESTURES):
            raise ValueError('provide all five gesture mappings')
        clean = {}
        for key, item in mappings.items():
            if not isinstance(item, dict) or item.get('color') not in LED_COLORS:
                raise ValueError('each gesture needs a valid LED color')
            clean[key] = {'program': str(item.get('program', ''))[:80], 'color': item['color']}
        with self._lock:
            self._cancel_swipe('Mappings changed; enable gestures again', disable=True)
            self.gestures = clean
            self._save_config()
        return self.snapshot()

    def set_swipe_enabled(self, enabled):
        if enabled and (self.guard.fault or self.monitor.fault_latched):
            raise RuntimeError('acknowledge sensor fault before enabling swipes')
        if enabled and not self.swipe_feature:
            raise RuntimeError("sensor swipe programs are disabled by features.json")
        if enabled:
            link = self.transport.snapshot()
            if not self._frame_healthy(link):
                raise RuntimeError("sensor must be connected and healthy before enabling swipe programs")
        with self._lock:
            self.swipe_enabled = bool(enabled)
            self.countdown_deadline = None
            self.countdown_program = None
            self.countdown_error = None
        return self.snapshot()

    def snapshot(self):
        link = self.transport.snapshot()
        frame = link.get("frame")
        fresh = self._frame_healthy(link)
        accel_g = None
        if fresh and isinstance(frame, dict):
            try:
                accel_g = [round(float(v)/1000.0, 5) for v in frame["accel_mg"]]
            except Exception:
                accel_g = None
        mon = self.monitor.snapshot()
        if accel_g is None:
            mon["tilt_deg"] = None
        with self._lock:
            deadline = self.countdown_deadline
            remaining = None if deadline is None else max(0.0, deadline-time.monotonic())
            taps = [dict(x) for x in self.taps]
            selected = self.selected_tap
            return {
                "enabled": True,
                "port": self.transport.port,
                "connected": bool(link.get("connected")),
                "fresh": bool(fresh),
                "sample_age_s": link.get("age_s"),
                "error": link.get("error"),
                "malformed_frames": link.get("malformed_frames"),
                "reconnects": link.get("reconnects"),
                "frame": frame if fresh else None,
                "accel_g": accel_g,
                "monitor": mon,
                "guard": self.guard.snapshot(),
                "gestures": {key:dict(value, label=GESTURE_LABELS[GESTURES.index(key)])
                             for key,value in self.gestures.items()},
                "last_gesture": self.last_gesture,
                "swipe": {
                    "feature_enabled": self.swipe_feature,
                    "enabled": self.swipe_enabled,
                    "taps": taps,
                    "selected_tap": selected,
                    "countdown_s": remaining,
                    "countdown_program": self.countdown_program,
                    "error": self.countdown_error,
                    "last_swipe": self.last_swipe,
                    "last_swipe_at": self.last_swipe_at,
                },
            }
