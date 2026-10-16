"""Verify that tilt recovery cannot be cleared by automatic homing."""
import ast
from contextlib import nullcontext
from pathlib import Path
from types import SimpleNamespace
import unittest


ROOT = Path(__file__).resolve().parents[2]


def recovery_gate(state, sync, home):
    source = ast.parse((ROOT / 'software/host/robot_mapper_klipper.py').read_text())
    function = next(node for node in source.body
                    if isinstance(node, ast.FunctionDef)
                    and node.name == 'ensure_homed_for_motion')
    namespace = {'state': state, '_sync_state_from_arm_robot': sync, 'precise_home': home}
    exec(compile(ast.Module(body=[function], type_ignores=[]), '<recovery-gate>', 'exec'), namespace)
    return namespace['ensure_homed_for_motion']


def saved_recovery_runner(state, points, commands, routes):
    source = ast.parse((ROOT / 'software/host/robot_mapper_klipper.py').read_text())
    function = next(node for node in source.body
                    if isinstance(node, ast.FunctionDef)
                    and node.name == '_queue_saved_linkage_recovery')
    namespace = {
        'state': state,
        '_load_named_program': lambda _name: (points, {}),
        '_program_point': lambda point: point,
        'safe_joint_route': lambda start, end: (routes.append((start, end)) or [start, end]),
        'check_stop': lambda: None,
        '_gcode': lambda command, timeout: commands.append((command, timeout)),
    }
    exec(compile(ast.Module(body=[function], type_ignores=[]), '<saved-recovery>', 'exec'), namespace)
    return namespace['_queue_saved_linkage_recovery']


def tilt_recovery_worker(state, queue_saved):
    source = ast.parse((ROOT / 'software/host/robot_mapper_klipper.py').read_text())
    function = next(node for node in source.body
                    if isinstance(node, ast.FunctionDef)
                    and node.name == '_tilt_correction_worker')
    namespace = {
        'state': state,
        'lock': nullcontext(),
        '_sync_state_from_arm_robot': lambda: None,
        '_program_running': lambda: False,
        'SENSOR': SimpleNamespace(
            monitor=SimpleNamespace(kill_enabled=False), recovery_program='Known recovery'),
        '_start_tilt_warning_flash': lambda: (None, None),
        '_queue_saved_linkage_recovery': queue_saved,
        '_gcode': lambda *_args, **_kwargs: None,
        'MotionCancelled': RuntimeError,
        'stop_event': SimpleNamespace(clear=lambda: None),
        'correction_thread_lock': nullcontext(),
        'correction_thread': None,
    }
    exec(compile(ast.Module(body=[function], type_ignores=[]), '<tilt-recovery>', 'exec'), namespace)
    return namespace['_tilt_correction_worker']


class RecoveryHomeLockTests(unittest.TestCase):
    def test_recovery_lock_rejects_motion_without_automatic_home(self):
        state = {'home_required': True, 'homed': False}
        calls = []
        gate = recovery_gate(state, lambda: state.update(homed=True), lambda: calls.append('home'))

        with self.assertRaisesRegex(RuntimeError, 'press Home'):
            gate()
        self.assertEqual(calls, [])

    def test_unlocked_missing_reference_keeps_existing_automatic_home(self):
        state = {'home_required': False, 'homed': False}
        calls = []
        gate = recovery_gate(state, lambda: None, lambda: (calls.append('home'), state.update(homed=True)))

        gate()
        self.assertEqual(calls, ['home'])

    def test_saved_recovery_holds_live_base_and_rail(self):
        state = {'x': 10.0, 'y': 20.0, 'z': 77.0, 'linear_y': 4.0, 'message': ''}
        commands, routes = [], []
        run = saved_recovery_runner(state, [
            {'a': 30.0, 'b': 40.0, 'c': 12.0, 'd': 99.0},
            {'a': 50.0, 'b': 60.0, 'c': 2.0, 'd': 0.0},
        ], commands, routes)

        run('Counter clockwise recovery')
        self.assertEqual(routes, [((10.0, 20.0), (30.0, 40.0)),
                                  ((30.0, 40.0), (50.0, 60.0))])
        move_commands = [command for command, _timeout in commands if command.startswith('ARM_MOVE')]
        self.assertTrue(all('C=77.00000 D=4.00000' in command for command in move_commands))
        self.assertEqual((state['x'], state['y'], state['z'], state['linear_y']),
                         (50.0, 60.0, 77.0, 4.0))

    def test_completed_recovery_keeps_the_existing_home_reference(self):
        state = {
            'boundary_enabled': True, 'homed': True, 'home_required': False,
            'busy': False, 'arm_speed': 40, 'arm_accel': 200, 'arm_jerk': 6,
            'linear_speed': 30, 'linear_accel': 150, 'linear_jerk': 3,
        }
        worker = tilt_recovery_worker(state, lambda _name: None)

        worker('confirmed tilt')
        self.assertTrue(state['homed'])
        self.assertFalse(state['home_required'])

    def test_incomplete_recovery_requires_home(self):
        state = {
            'boundary_enabled': True, 'homed': True, 'home_required': False,
            'busy': False, 'arm_speed': 40, 'arm_accel': 200, 'arm_jerk': 6,
            'linear_speed': 30, 'linear_accel': 150, 'linear_jerk': 3,
        }
        worker = tilt_recovery_worker(state, lambda _name: (_ for _ in ()).throw(RuntimeError('stage failed')))

        worker('confirmed tilt')
        self.assertFalse(state['homed'])
        self.assertTrue(state['home_required'])


if __name__ == '__main__':
    unittest.main()
