import json
from pathlib import Path
import tempfile
import unittest
from features import load_features, DEFAULTS


class FeatureTests(unittest.TestCase):
    def test_missing_configuration_needs_no_camera_or_vpn(self):
        with tempfile.TemporaryDirectory() as directory:
            flags = load_features(Path(directory) / 'missing.json')
        self.assertFalse(flags['camera'])
        self.assertFalse(flags['remote_access'])
        self.assertFalse(flags['sensors'])
        self.assertFalse(flags['sensor_swipe_programs'])

    def test_sensor_import_does_not_require_serial_or_open_hardware(self):
        from sensor_transport import SensorSerial
        sensor = SensorSerial('not-a-device')
        self.assertIsNone(sensor._serial)
        self.assertIsNone(sensor._thread)

    def test_independent_optional_features(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'features.json'
            path.write_text(json.dumps({'camera': True, 'rail': False}))
            flags = load_features(path)
        self.assertTrue(flags['camera'])
        self.assertFalse(flags['rail'])
        self.assertFalse(flags['remote_access'])
        self.assertTrue(DEFAULTS['rail'])

    def test_rejects_mistyped_or_unknown_settings(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'features.json'
            for raw in ({'camera': 'false'}, {'rail': 1}, {'typo': True}, []):
                path.write_text(json.dumps(raw))
                with self.assertRaises(ValueError):
                    load_features(path)

    def test_camera_import_does_not_open_hardware(self):
        from arm_camera import ArmCamera
        camera = ArmCamera()
        self.assertIsNone(camera.worker)
        self.assertEqual(camera.viewers, 0)
