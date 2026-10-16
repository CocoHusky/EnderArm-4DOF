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


if __name__ == '__main__':
    unittest.main()
