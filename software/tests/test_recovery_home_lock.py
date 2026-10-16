"""Verify that tilt recovery cannot be cleared by automatic homing."""
import ast
from pathlib import Path
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


if __name__ == '__main__':
    unittest.main()
