import unittest

from sensor_guard import SensorGuard


class GuardTests(unittest.TestCase):
    def test_burst_kills_on_third_or_sixth_distinct_lobe_on_any_axis(self):
        for axis in range(3):
            for count in (3, 6):
                guard = SensorGuard()
                guard.configure({'burst_stop': True, 'burst_count': count})
                guard.update([0, 1, 0], now=0)
                for n in range(count):
                    sample = [0, 1, 0]
                    sample[axis] += 1.5 if n % 2 == 0 else -1.5
                    reason = guard.update(sample, now=.05*(n+1))
                    if n < count-1:
                        self.assertIsNone(reason)
                    else:
                        self.assertIn('suspected skipping', reason)
                self.assertIsNone(guard.update([0, 1, 0], now=.05*(count+1)))
                self.assertIsNotNone(guard.fault)

    def test_sustained_excursion_noise_and_slow_tilt_do_not_count_as_burst(self):
        import math
        for mode in ('step', 'noise', 'tilt'):
            guard = SensorGuard()
            guard.configure({'burst_stop': True})
            guard.update([0, 1, 0], now=0)
            for n in range(1, 301):
                t = n/75
                sample = {'step': [2, 1, 0],
                          'noise': [.05*math.sin(n), 1, .05*math.cos(n)],
                          'tilt': [0, math.cos(t*.4), math.sin(t*.4)]}[mode]
                self.assertIsNone(guard.update(sample, now=t), mode)

    def test_burst_history_does_not_cross_data_gaps_or_acknowledgement(self):
        guard = SensorGuard()
        guard.configure({'burst_stop': True})
        guard.update([0, 1, 0], now=0)
        guard.update([2, 1, 0], now=.05)
        guard.update([-2, 1, 0], now=.1)
        self.assertEqual(guard.snapshot()['burst_spikes'], 2)
        guard.update([0, 1, 0], now=.5)
        self.assertEqual(guard.snapshot()['burst_spikes'], 0)
        guard.update([2, 1, 0], now=.55)
        guard.update([], healthy=False, now=.56)
        self.assertEqual(guard.snapshot()['burst_spikes'], 0)
        guard.acknowledge()
        self.assertIsNone(guard.update([2, 1, 0], now=.6))

    def test_burst_is_off_by_default_and_count_is_validated(self):
        guard = SensorGuard()
        guard.update([0, 1, 0], now=0)
        for n in range(1, 9):
            self.assertIsNone(guard.update([2*(-1)**n, 1, 0], now=n*.05))
        self.assertFalse(guard.options['burst_stop'])
        for options in ({'burst_count': 3.5}, {'burst_g': float('nan')},
                        {'burst_window_s': 0}, {'burst_stop': 1}):
            with self.assertRaises(ValueError):
                guard.configure(options)

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
