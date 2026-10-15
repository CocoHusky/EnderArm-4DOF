import unittest

from sensor_guard import SensorGuard


class GuardTests(unittest.TestCase):
    def test_off_by_default_and_first_sample_not_an_impact(self):
        guard = SensorGuard()
        self.assertIsNone(guard.update([0, 1, 0], now=0))
        self.assertIsNone(guard.update([0, 3, 1], now=1))
        self.assertIsNone(guard.led([0, 3, 1], now=1))
        guard.configure({'impact_stop': True})
        self.assertIsNone(guard.update([0, 3, 1], now=2))

    def test_impact_latches_once_and_flashes_red(self):
        guard = SensorGuard()
        guard.configure({'impact_stop': True, 'impact_flash': True})
        guard.update([0, 1, 0], now=0)
        self.assertIn('impact', guard.update([0, 3, 0], now=1))
        self.assertEqual(guard.led([0, 3, 0], now=1), 'red')
        self.assertEqual(guard.led([0, 3, 0], now=1.1), 'off')
        self.assertIsNone(guard.update([0, 1, 0], now=1.2))
        guard.acknowledge()
        self.assertIsNone(guard.fault)

    def test_absolute_z_persistence_and_feedback(self):
        guard = SensorGuard()
        guard.configure({'z_stop': True, 'z_color': True})
        self.assertIsNone(guard.update([0, 1, -0.4], now=1))
        self.assertIsNone(guard.update([0, 1, -0.4], now=1.1))
        self.assertIn('|Z|', guard.update([0, 1, -0.4], now=1.2))
        self.assertEqual(guard.led([0, 1, -0.4], now=1.2), 'yellow')
        self.assertIsNone(guard.led([0, 1, 0], now=1.3))

    def test_unhealthy_resets_history(self):
        guard = SensorGuard()
        guard.configure({'impact_stop': True, 'z_stop': True})
        guard.update([0, 1, 0.5], now=0)
        guard.update([], healthy=False, now=1)
        self.assertIsNone(guard.update([0, 3, 0.5], now=2))
        self.assertIsNone(guard.update([float('nan'), 0, 0], now=3))

    def test_invalid_options_rejected_atomically(self):
        guard = SensorGuard()
        for value in (float('nan'), 0, 9):
            with self.assertRaises(ValueError):
                guard.configure({'impact_stop': True, 'impact_g': value})
        self.assertFalse(guard.options['impact_stop'])


if __name__ == '__main__':
    unittest.main()
