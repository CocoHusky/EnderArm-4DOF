import tempfile
from types import SimpleNamespace
import unittest

from sensor_service import SensorService
from sensor_transport import SensorSerial, SensorProtocolError, parse_sensor_line


class SensorStreamTests(unittest.TestCase):
    def test_compact_sample_preserves_metadata_and_updates_health(self):
        previous = {'v':1,'device':'FRDM-KL02Z','fw':'0.4.0','seq':1,'ms':50,
                    'accel_mg':[0,0,1000],'slider':{'raw_left':1,'raw_right':2,
                    'position':500,'active':False,'swipe':'none'},
                    'led':'red','health':{'accel':True,'slider':True}}
        sample = parse_sensor_line('{"a":[10,20,990],"s":2,"t":60,"h":0}', previous)
        self.assertEqual(sample['seq'], 2)
        self.assertEqual(sample['accel_mg'], [10,20,990])
        self.assertEqual(sample['slider'], previous['slider'])
        self.assertFalse(sample['health']['accel'])
        self.assertTrue(previous['health']['accel'])
        with self.assertRaises(SensorProtocolError):
            parse_sensor_line('{"a":[1,2,3],"s":2,"t":60,"h":1}')
        with self.assertRaises(SensorProtocolError):
            parse_sensor_line('{"a":[1,2],"s":2,"t":60,"h":1}', previous)

    def test_stream_keeps_every_frame_and_replays_after_cursor(self):
        transport = SensorSerial('fake')
        for i in range(10):
            transport._record_frame({'seq':i, 'ms':i*50, 'accel_mg':[i, 0, 1000]}, i*.05)
        packet = transport.samples_since()
        self.assertEqual([s['seq'] for s in packet['samples']], list(range(10)))
        self.assertEqual(packet['source_hz'], 20)
        self.assertEqual(packet['samples'][5]['accel_g'], [.005, 0, 1])
        self.assertEqual([s['id'] for s in transport.samples_since(7)['samples']], [8, 9, 10])

    def test_stream_reports_overflow_and_recovers_after_host_restart(self):
        transport = SensorSerial('fake')
        for i in range(4098):
            transport._record_frame({'seq':i, 'ms':i, 'accel_mg':[0, 0, 1000]}, i*.05)
        self.assertEqual(transport.samples_since(1)['dropped'], 1)
        packet = transport.samples_since(5000)
        self.assertTrue(packet['reset'])
        self.assertEqual(packet['cursor'], 4098)

    def make_service(self, directory):
        service = SensorService(directory+'/sensor.json', 'fake')
        self.link = {'fresh':True, 'connected':True, 'age_s':.01,
                     'frame':{'seq':1, 'accel_mg':[0, 0, 1000],
                              'health':{'accel':True, 'slider':True}}}
        service.transport = SimpleNamespace(port='fake', snapshot=lambda: self.link)
        service.monitor.reference = [0, 0, 1]
        service.monitor.last_tilt_deg = 10
        service.monitor.fault_latched = True
        service.monitor.fault_reason = 'prior correction fault'
        return service

    def test_good_correction_clears_tilt_and_rearms_without_manual_ack(self):
        with tempfile.TemporaryDirectory() as td:
            service = self.make_service(td)
            service.monitor.enabled = True
            self.assertTrue(service.clear_recovered_tilt())
            self.assertFalse(service.monitor.fault_latched)
            self.assertTrue(service.monitor.enabled)

    def test_hard_kill_requires_home_and_normal_live_feedback(self):
        with tempfile.TemporaryDirectory() as td:
            service = self.make_service(td)
            service.hard_kill_latched = True
            self.assertFalse(service.clear_recovered_tilt())
            service.monitor.last_tilt_deg = 42
            self.assertFalse(service.clear_recovered_tilt(after_home=True))
            service.monitor.last_tilt_deg = 10
            self.link['fresh'] = False
            self.assertFalse(service.clear_recovered_tilt(after_home=True))
            self.link['fresh'] = True
            self.assertTrue(service.clear_recovered_tilt(after_home=True))
            self.assertFalse(service.hard_kill_latched)

    def test_successful_home_rearms_vibration_but_correction_does_not(self):
        with tempfile.TemporaryDirectory() as td:
            service = self.make_service(td)
            service.guard.configure({'burst_stop': True})
            def burst(start):
                service.guard.update([0, 1, 0], now=start)
                reasons = [service.guard.update([1.5*(-1)**n, 1, 0],
                           now=start+.05*(n+1)) for n in range(3)]
                return [r for r in reasons if r]
            self.assertEqual(len(burst(0)), 1)
            service.clear_recovered_tilt()
            self.assertIsNotNone(service.guard.fault)
            self.link['fresh'] = False
            self.assertFalse(service.clear_recovered_tilt(after_home=True))
            self.assertIsNotNone(service.guard.fault)
            self.link['fresh'] = True
            self.assertTrue(service.clear_recovered_tilt(after_home=True))
            self.assertIsNone(service.guard.fault)
            self.assertEqual(len(burst(1)), 1)
