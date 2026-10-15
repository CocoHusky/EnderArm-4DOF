import json
import unittest

from sensor_monitor import TiltMonitor, tilt_degrees
from sensor_transport import SensorProtocolError, parse_sensor_line


class SensorProtocolTests(unittest.TestCase):
    def frame(self):
        return {
            "v":1,"device":"FRDM-KL02Z","fw":"0.1.0","seq":12,"ms":3456,
            "accel_mg":[0,0,1000],
            "slider":{"raw_left":10,"raw_right":11,"position":0.5,"active":False,"swipe":"none"},
            "led":"off","health":{"accel":True,"slider":True}
        }

    def test_accepts_real_frame_shape(self):
        frame=parse_sensor_line(json.dumps(self.frame()))
        self.assertEqual(frame["device"],"FRDM-KL02Z")
        self.assertEqual(frame["accel_mg"][2],1000)

    def test_rejects_wrong_device(self):
        frame=self.frame(); frame["device"]="FRDM-KL25Z"
        with self.assertRaises(SensorProtocolError):
            parse_sensor_line(json.dumps(frame))

    def test_rejects_missing_acceleration(self):
        frame=self.frame(); del frame["accel_mg"]
        with self.assertRaises(SensorProtocolError):
            parse_sensor_line(json.dumps(frame))


class TiltTests(unittest.TestCase):
    def test_tilt_geometry(self):
        self.assertAlmostEqual(tilt_degrees([0,0,1],[0,0,1]),0.0,places=6)
        self.assertAlmostEqual(tilt_degrees([1,0,0],[0,0,1]),90.0,places=6)

    def test_persistence_latches_once(self):
        monitor=TiltMonitor(allowed_deg=10,persistence_s=0.5)
        monitor.calibrate([0,0,1])
        monitor.enabled=True
        self.assertFalse(monitor.update([0.5,0,0.866],now=1.0))
        self.assertFalse(monitor.update([0.5,0,0.866],now=1.4))
        self.assertTrue(monitor.update([0.5,0,0.866],now=1.6))
        self.assertTrue(monitor.fault_latched)
        self.assertFalse(monitor.update([0,0,1],now=2.0))
        monitor.acknowledge()
        self.assertFalse(monitor.fault_latched)

    def test_bad_health_cannot_fault(self):
        monitor=TiltMonitor(allowed_deg=5,persistence_s=0.1)
        monitor.calibrate([0,0,1])
        monitor.enabled=True
        monitor.update([1,0,0],healthy=False,now=1)
        self.assertFalse(monitor.update([1,0,0],healthy=False,now=2))
        self.assertFalse(monitor.fault_latched)


if __name__=="__main__":
    unittest.main()
