import json
import tempfile
import unittest
from unittest.mock import patch

from sensor_service import SensorService


class FakeTransport:
    def __init__(self, frame):
        self.port = "fake"
        self.frame = frame
        self.commands = []
        self.reconnects = 1

    def start(self): pass
    def stop(self): pass
    def send_command(self, command): self.commands.append(command)
    def snapshot(self):
        return {
            "connected": True, "fresh": True, "age_s": 0.01, "error": None,
            "malformed_frames": 0, "reconnects": self.reconnects, "frame": self.frame,
        }


def frame(seq=1, swipe="none"):
    return {
        "v":1,"device":"FRDM-KL02Z","fw":"test","seq":seq,"ms":100,
        "accel_mg":[0,0,1000],
        "slider":{"raw_left":5,"raw_right":5,"position":500,"active":False,"swipe":swipe},
        "led":"off","health":{"accel":True,"slider":True,"whoami":26},
    }


class SensorServiceTests(unittest.TestCase):
    def test_vibration_burst_kills_without_tilt_monitoring(self):
        with tempfile.TemporaryDirectory() as td:
            svc = self.make_service(td)
            svc.guard.configure({'burst_stop': True})
            calls = []
            svc.on_kill = calls.append
            samples = iter(([0, 0, 1000], [1500, 0, 1000],
                            [-1500, 0, 1000], [1500, 0, 1000], [0, 0, 1000]))
            clock = [0.0]
            class Stop:
                def wait(self, _timeout):
                    try:
                        accel = next(samples)
                    except StopIteration:
                        return True
                    clock[0] += .05
                    svc.transport.frame['seq'] += 1
                    svc.transport.frame['accel_mg'] = accel
                    return False
            svc._stop = Stop()
            with patch('sensor_guard.time.monotonic', lambda: clock[0]):
                svc._run()
            self.assertEqual(len(calls), 1)
            self.assertIn('suspected skipping', calls[0])
            self.assertFalse(svc.swipe_enabled)
            self.assertFalse(svc.monitor.enabled)

    def make_service(self, td, swipe=True):
        svc=SensorService(td+"/sensor_config.json","fake",swipe_feature=swipe)
        svc.transport=FakeTransport(frame())
        return svc

    def test_swipe_requires_explicit_enable(self):
        with tempfile.TemporaryDirectory() as td:
            svc=self.make_service(td)
            svc._handle_swipe("right")
            self.assertIsNone(svc.countdown_deadline)
            svc.set_swipe_enabled(True)
            svc._handle_swipe("right")
            self.assertIsNotNone(svc.countdown_deadline)
            self.assertEqual(svc.transport.commands[-1],"LED GREEN")

    def test_feedback_modes_are_sensor_commands_only(self):
        with tempfile.TemporaryDirectory() as td:
            svc=self.make_service(td)
            svc.set_mode('orientation')
            self.assertEqual(svc.transport.commands, ['MODE ORIENTATION'])
            with self.assertRaises(ValueError):
                svc.set_mode('unknown')

    def test_five_independent_gesture_mappings_and_persistence(self):
        with tempfile.TemporaryDirectory() as td:
            svc=self.make_service(td)
            mappings={key:dict(value,program=key) for key,value in svc.gestures.items()}
            svc.set_gestures(mappings)
            calls=[]
            svc.can_run_program=lambda:(True,'')
            svc.run_program=calls.append
            svc.set_swipe_enabled(True)
            for key in mappings:
                svc._handle_gesture(key)
                self.assertEqual(svc.countdown_program,key)
                svc.countdown_deadline=0
                svc._countdown_tick(svc.transport.snapshot())
            self.assertEqual(calls,list(mappings))
            other=SensorService(td+'/sensor_config.json','fake',swipe_feature=True)
            self.assertEqual(other.gestures,mappings)
            self.assertFalse(other.swipe_enabled)

    def test_gesture_event_consumed_once_and_not_replayed_after_reconnect(self):
        with tempfile.TemporaryDirectory() as td:
            svc=self.make_service(td)
            svc.guard.configure({'swipe_colors':True})
            event={'gesture_id':3,'gesture':'press_2'}
            svc._process_gesture(event,True)
            self.assertIsNone(svc.last_gesture)
            event['gesture_id']=4
            svc._process_gesture(event,True)
            svc._process_gesture(event,True)
            self.assertEqual(svc.transport.commands,['LED GREEN'])
            self.assertEqual(svc.last_gesture,'press_2')
            self.assertIsNone(svc.countdown_deadline)
            svc._process_gesture(event,False)
            event['gesture_id']=5
            svc._process_gesture(event,True)
            self.assertEqual(len(svc.transport.commands),1)

    def test_new_touch_and_mapping_change_cancel_pending_program(self):
        with tempfile.TemporaryDirectory() as td:
            svc=self.make_service(td)
            mappings={key:dict(value,program='saved') for key,value in svc.gestures.items()}
            svc.set_gestures(mappings)
            svc.set_swipe_enabled(True)
            svc._handle_gesture('press_1')
            self.assertIsNotNone(svc.countdown_deadline)
            svc._process_gesture({'gesture_id':0,'active':True},True)
            self.assertIsNone(svc.countdown_deadline)
            svc._handle_gesture('swipe_right')
            svc.set_gestures(mappings)
            self.assertFalse(svc.swipe_enabled)
            self.assertIsNone(svc.countdown_deadline)

    def test_legacy_taps_migrate_to_press_slots_without_assigning_swipes(self):
        with tempfile.TemporaryDirectory() as td:
            with open(td+'/sensor_config.json','w') as stream:
                json.dump({'taps':[{'program':'Left','color':'red'},
                                  {'program':'Center','color':'green'},
                                  {'program':'Right','color':'blue'}]},stream)
            svc=self.make_service(td)
            self.assertEqual(svc.gestures['press_2']['program'],'Center')
            self.assertEqual(svc.gestures['swipe_left']['program'],'')

    def test_invalid_gesture_mapping_is_atomic(self):
        with tempfile.TemporaryDirectory() as td:
            svc=self.make_service(td)
            before=svc.snapshot()['gestures']
            with self.assertRaises(ValueError):
                svc.set_gestures({'press_1':{'program':'X','color':'red'}})
            self.assertEqual(svc.snapshot()['gestures'],before)

    def test_unassigned_gesture_cancels_previous_start(self):
        with tempfile.TemporaryDirectory() as td:
            svc=self.make_service(td)
            svc.gestures['press_2']['program']='Center'
            svc.set_swipe_enabled(True)
            svc._handle_gesture('press_2')
            svc._handle_gesture('press_1')
            self.assertIsNone(svc.countdown_deadline)
            self.assertIsNone(svc.countdown_program)

    def test_swipe_direction_and_expiry_run_saved_mapping_once(self):
        with tempfile.TemporaryDirectory() as td:
            svc=self.make_service(td)
            svc.set_taps([{'program':'Center Beer','color':'green'},
                          {'program':'Right Beer','color':'blue'}])
            calls=[]
            svc.can_run_program=lambda:(True,'')
            svc.run_program=calls.append
            svc.set_swipe_enabled(True)
            svc._handle_swipe('right')
            self.assertEqual(svc.selected_tap,1)
            svc._countdown_tick(svc.transport.snapshot())
            self.assertEqual(calls,[])
            svc.countdown_deadline=0
            svc._countdown_tick(svc.transport.snapshot())
            svc._countdown_tick(svc.transport.snapshot())
            self.assertEqual(calls,['Right Beer'])
            svc._handle_swipe('left')
            self.assertEqual(svc.selected_tap,0)

    def test_disabled_feature_cannot_be_armed(self):
        with tempfile.TemporaryDirectory() as td:
            svc=self.make_service(td,swipe=False)
            with self.assertRaises(RuntimeError):
                svc.set_swipe_enabled(True)
            svc._handle_swipe('right')
            self.assertIsNone(svc.countdown_deadline)
            self.assertEqual(svc.transport.commands,[])

    def test_feedback_modes_reject_stale_sensor(self):
        with tempfile.TemporaryDirectory() as td:
            svc=self.make_service(td)
            svc.transport.snapshot=lambda:{'fresh': False}
            with self.assertRaises(RuntimeError):
                svc.set_mode('slider')
            self.assertEqual(svc.transport.commands, [])

    def test_disconnect_style_disable_cancels_countdown(self):
        with tempfile.TemporaryDirectory() as td:
            svc=self.make_service(td)
            svc.set_swipe_enabled(True)
            svc._handle_swipe("right")
            svc._cancel_swipe("link lost",disable=True)
            self.assertFalse(svc.swipe_enabled)
            self.assertIsNone(svc.countdown_deadline)

    def test_config_does_not_enable_automation_on_restart(self):
        with tempfile.TemporaryDirectory() as td:
            svc=self.make_service(td)
            svc.monitor.configure(12,0.4)
            svc.monitor.reference=[0,0,1]
            svc._save_config()
            other=SensorService(td+"/sensor_config.json","fake",swipe_feature=True)
            self.assertFalse(other.swipe_enabled)
            self.assertFalse(other.monitor.kill_enabled)
            self.assertAlmostEqual(other.monitor.allowed_deg,12)
            self.assertEqual(other.monitor.reference,[0.0,0.0,1.0])

    def test_recovery_route_persists_without_enabling_motion(self):
        with tempfile.TemporaryDirectory() as td:
            svc=self.make_service(td)
            svc.set_recovery_program('Unwind linkage')
            self.assertEqual(svc.snapshot()['recovery_program'], 'Unwind linkage')
            other=SensorService(td+'/sensor_config.json','fake',swipe_feature=True)
            self.assertEqual(other.recovery_program, 'Unwind linkage')
            self.assertFalse(other.monitor.kill_enabled)

    def test_tilt_correction_is_opt_in_and_excludes_kill(self):
        with tempfile.TemporaryDirectory() as td:
            svc=self.make_service(td)
            svc.configure_monitor(enabled=True, correct_enabled=True)
            self.assertTrue(svc.snapshot()['correct_enabled'])
            with self.assertRaises(ValueError):
                svc.configure_monitor(kill_enabled=True)
            other=SensorService(td+'/sensor_config.json','fake',swipe_feature=True)
            self.assertTrue(other.correct_enabled)
            self.assertFalse(other.monitor.kill_enabled)

    def test_snapshot_hides_sensor_values_when_stale(self):
        with tempfile.TemporaryDirectory() as td:
            svc=self.make_service(td)
            svc.transport.snapshot=lambda:{
                "connected":True,"fresh":False,"age_s":4.0,"error":None,
                "malformed_frames":0,"reconnects":1,"frame":frame(),
            }
            snap=svc.snapshot()
            self.assertIsNone(snap["frame"])
            self.assertIsNone(snap["accel_g"])

if __name__=="__main__":
    unittest.main()
