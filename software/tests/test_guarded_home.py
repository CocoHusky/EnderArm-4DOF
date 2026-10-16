"""Hardware-free ordering/reference checks; not physical commissioning."""
import ast
from pathlib import Path
from types import SimpleNamespace
import unittest

ROOT = Path(__file__).resolve().parents[2]


def extract(path, name, namespace):
    tree = ast.parse((ROOT / path).read_text())
    function = next(n for n in ast.walk(tree)
                    if isinstance(n, ast.FunctionDef) and n.name == name)
    exec(compile(ast.Module(body=[function], type_ignores=[]), '<guarded-home>', 'exec'), namespace)
    return namespace[name]


class GuardedHomeTests(unittest.TestCase):
    def test_host_pauses_corrects_and_restarts_switch_sequence(self):
        session = {'active': False, 'done': []}
        commands = []
        checks = [False, True, False, False, False, False]
        def command(text, **_kw):
            commands.append(text)
            if 'ACTION=BEGIN' in text:
                session.update(active=True, done=[])
            if 'ACTION=STEP' in text:
                session['done'].append(text.split('AXIS=')[1][0])
        def correct(_clear, _allow=True):
            self.assertTrue(ns['guarded_home_active'])
            return checks.pop(0)
        ns = {'state': {'homed': True}, 'SENSOR': SimpleNamespace(
                  guard=SimpleNamespace(fault=None), hard_kill_latched=False),
              '_arm_robot_status': lambda: {'guarded_home_version': 1, 'guarded_home': session},
              '_sync_state_from_arm_robot': lambda: None,
              'check_stop': lambda: None, 'time': SimpleNamespace(monotonic=lambda: 0),
              '_gcode': command, '_correct_before_home': correct,
              '_emergency_kill': lambda reason: self.fail(reason)}
        run = extract('software/host/robot_mapper_klipper.py', '_run_guarded_home', ns)
        run()
        self.assertEqual(sum('ACTION=BEGIN' in c for c in commands), 2)
        self.assertEqual(sum('AXIS=A' in c for c in commands), 2)
        self.assertIn('ACTION=FINISH', commands[-1])
        self.assertFalse(ns['guarded_home_active'])
        ns['state']['homed'] = False
        commands.clear()
        with self.assertRaises(RuntimeError):
            run()
        self.assertFalse(commands)
        ns['state']['homed'] = True
        kills = []
        ns['_emergency_kill'] = kills.append
        def lost_feedback(_clear, _allow=True):
            raise RuntimeError('feedback lost')
        ns['_correct_before_home'] = lost_feedback
        with self.assertRaises(RuntimeError):
            run()
        self.assertEqual(len(kills), 1)
        self.assertFalse(any('ACTION=STEP' in c or 'ACTION=FINISH' in c for c in commands))
        self.assertFalse(ns['guarded_home_active'])

    def test_native_seek_keeps_coordinates_until_real_touch_and_release(self):
        events = []
        class Joint:
            def __init__(self, value):
                self.commanded_pos = value
                self.hit = False
                self.seeks = 0
                self.rail = SimpleNamespace(get_endstops=lambda: [(SimpleNamespace(
                    query_endstop=lambda _time: self.hit), 'switch')])
            def do_set_position(self, value):
                events.append(('position', value))
                self.commanded_pos = value
            def do_homing_move(self, value, speed, accel, probe, trigger, check):
                events.append(('seek', value, check))
                self.commanded_pos = value
                self.seeks += 1
                self.hit = self.seeks == 2
            def do_move(self, value, *_args):
                events.append(('release', value))
                self.commanded_pos = value
                self.hit = False
        joints = [Joint(v) for v in (20, 30, 40, 0)]
        controller = SimpleNamespace(homed=True, dance_active=False, linear_enabled=False,
            x=joints[0], y=joints[1], z=joints[2], slide=joints[3],
            toolhead=SimpleNamespace(wait_moves=lambda: None, get_last_move_time=lambda: 0),
            _joint_position=lambda: tuple(j.commanded_pos for j in joints),
            _prepare_manual=lambda: None, _register_all=lambda: None,
            _check_guarded_home_segment=lambda *_args: None)
        class Command:
            def __init__(self, action): self.action = action
            def get(self, key, default=None):
                return {'ACTION': self.action, 'AXIS': 'C'}.get(key, default)
            def get_float(self, key, default, **limits): return default
            error = RuntimeError
        run = extract('software/klipper/arm_robot.py', 'cmd_ARM_HOME_GUARDED', {})
        run(controller, Command('BEGIN'))
        run(controller, Command('STEP'))
        self.assertEqual(joints[2].commanded_pos, 39.5)
        self.assertNotIn('C', controller.guarded_home['done'])
        run(controller, Command('STEP'))
        self.assertEqual(joints[2].commanded_pos, 39.)
        self.assertNotIn('C', controller.guarded_home['done'])
        for _ in range(15):
            run(controller, Command('STEP'))
        self.assertIn(('release', 46.5), events)
        self.assertEqual(joints[2].commanded_pos, 2.5)
        self.assertIn('C', controller.guarded_home['done'])
        self.assertTrue(all(not event[2] for event in events if event[0] == 'seek'))
        with self.assertRaises(RuntimeError):
            run(controller, Command('FINISH'))
        controller.guarded_home['done'] = ['A', 'B', 'C']
        run(controller, Command('FINISH'))
        self.assertFalse(controller.guarded_home['active'])
        controller.homed = False
        with self.assertRaises(RuntimeError):
            run(controller, Command('BEGIN'))
