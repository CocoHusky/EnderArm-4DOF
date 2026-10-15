"""Optional FRDM-KL02Z serial transport. No device is opened at import time."""
import json
import math
import threading
import time


class SensorProtocolError(ValueError):
    pass


def parse_sensor_line(line):
    if isinstance(line, bytes):
        line=line.decode("utf-8","strict")
    try:
        obj=json.loads(line)
    except Exception as exc:
        raise SensorProtocolError(f"invalid JSON: {exc}") from exc
    if not isinstance(obj,dict):
        raise SensorProtocolError("sensor frame must be a JSON object")
    required=("v","device","seq","ms","accel_mg","slider","led","health")
    missing=[key for key in required if key not in obj]
    if missing:
        raise SensorProtocolError("missing fields: "+", ".join(missing))
    if obj["device"]!="FRDM-KL02Z":
        raise SensorProtocolError(f"unexpected device {obj['device']!r}")
    if not isinstance(obj["accel_mg"],list) or len(obj["accel_mg"])!=3:
        raise SensorProtocolError("accel_mg must contain X/Y/Z")
    for value in obj["accel_mg"]:
        if type(value) not in (int,float) or not math.isfinite(value):
            raise SensorProtocolError("accel_mg values must be numeric")
    slider=obj["slider"]
    if not isinstance(slider,dict):
        raise SensorProtocolError("slider must be an object")
    for key in ("raw_left","raw_right","position","active","swipe"):
        if key not in slider:
            raise SensorProtocolError(f"slider missing {key}")
    if slider["swipe"] not in ("none","left","right"):
        raise SensorProtocolError("slider swipe must be none/left/right")
    if 'gesture_id' in slider or 'gesture' in slider:
        if (type(slider.get('gesture_id')) is not int or slider['gesture_id'] < 0
                or slider.get('gesture') not in ('none', 'press_1', 'press_2', 'press_3', 'swipe_left', 'swipe_right')):
            raise SensorProtocolError('invalid touch gesture event')
    if not isinstance(obj["led"],str):
        raise SensorProtocolError("led must be a string")
    if not isinstance(obj["health"],dict):
        raise SensorProtocolError("health must be an object")
    return obj


class SensorSerial:
    """Reconnect-capable pyserial reader with explicit stale/error state."""
    def __init__(self, port, baud=115200, stale_after=0.75):
        self.port=port
        self.baud=int(baud)
        self.stale_after=float(stale_after)
        self._lock=threading.Lock()
        self._stop=threading.Event()
        self._thread=None
        self._serial=None
        self._frame=None
        self._last_rx_monotonic=None
        self._connected=False
        self._error=None
        self._malformed=0
        self._reconnects=0

    def start(self):
        if self._thread and self._thread.is_alive():
            return
        self._stop.clear()
        self._thread=threading.Thread(target=self._run,name="frdm-kl02z",daemon=True)
        self._thread.start()

    def stop(self):
        self._stop.set()
        serial_obj=self._serial
        if serial_obj:
            try: serial_obj.close()
            except Exception: pass
        thread=self._thread
        if thread and thread.is_alive():
            thread.join(timeout=1.5)

    def _set_link(self, connected, error=None):
        with self._lock:
            self._connected=bool(connected)
            self._error=None if error is None else str(error)

    def _run(self):
        while not self._stop.is_set():
            try:
                import serial
                with serial.Serial(self.port,self.baud,timeout=0.25,write_timeout=0.5) as stream:
                    self._serial=stream
                    self._reconnects+=1
                    self._set_link(True)
                    stream.reset_input_buffer()
                    while not self._stop.is_set():
                        raw=stream.readline()
                        if not raw:
                            continue
                        try:
                            frame=parse_sensor_line(raw)
                        except Exception as exc:
                            with self._lock:
                                self._malformed+=1
                                self._error=str(exc)
                            continue
                        now=time.monotonic()
                        with self._lock:
                            self._frame=frame
                            self._last_rx_monotonic=now
                            self._connected=True
                            self._error=None
            except Exception as exc:
                self._set_link(False,exc)
                if self._stop.wait(1.0):
                    break
            finally:
                self._serial=None
        self._set_link(False,self._error)

    def send_command(self, command):
        stream=self._serial
        if stream is None:
            raise RuntimeError("sensor is not connected")
        payload=(command.strip()+"\n").encode("ascii","strict")
        stream.write(payload)
        stream.flush()

    def snapshot(self):
        now=time.monotonic()
        with self._lock:
            frame=None if self._frame is None else json.loads(json.dumps(self._frame))
            age=None if self._last_rx_monotonic is None else max(0.0,now-self._last_rx_monotonic)
            connected=self._connected
            error=self._error
            malformed=self._malformed
            reconnects=self._reconnects
        fresh=bool(connected and age is not None and age<=self.stale_after)
        return {
            "connected":connected,
            "fresh":fresh,
            "age_s":age,
            "error":error,
            "malformed_frames":malformed,
            "reconnects":reconnects,
            "frame":frame,
        }
