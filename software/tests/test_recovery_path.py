import math
import ast
from pathlib import Path
from types import SimpleNamespace
import tempfile
import unittest

from recovery_path import RECOVERY_ARC, TABLE_Z_MIN, recovery_steps
from sensor_service import SensorService


class RecoveryPathTests(unittest.TestCase):
    def test_feedback_stops_after_first_fixed_axis_step(self):
        path = Path(__file__).resolve().parents[1] / 'host/robot_mapper_klipper.py'
        source = ast.parse(path.read_text())
        function = next(n for n in source.body if isinstance(n, ast.FunctionDef)
                        and n.name == '_queue_incremental_recovery')
        readings = iter([40.0, 10.0])
        commands = []
        ns = {
            'math': math, 'time': SimpleNamespace(monotonic=lambda: 0),
            'SENSOR': SimpleNamespace(monitor=SimpleNamespace(allowed_deg=20)),
            '_settled_recovery_tilt': lambda: next(readings),
            'state': {'x': 10, 'y': 20, 'z': 77, 'linear_y': 4},
            'kinematic_state': lambda *_args: {'tool': {'x': 0, 'y': -235.9, 'z': -41.2}},
            'recovery_steps': lambda *_args: iter([(234, -42), (233, -43)]),
            'plan_world_target': lambda *_args, **_kwargs: {'A': 11, 'B': 21},
            'clip_xy_path': lambda *_args: (11, 21, False),
            'TABLE_Z_MIN': TABLE_Z_MIN, 'check_stop': lambda: None,
            '_gcode': lambda command, **_kwargs: commands.append(command),
            '_sync_state_from_arm_robot': lambda: None,
        }
        exec(compile(ast.Module(body=[function], type_ignores=[]), '<incremental>', 'exec'), ns)
        ns['_queue_incremental_recovery']()
        self.assertEqual(commands, ['ARM_MOVE_JOINT A=11.00000 B=21.00000 C=77.00000 D=4.00000', 'M400'])

    def test_green_direction_small_steps_and_table_floor(self):
        points = [RECOVERY_ARC[0], *recovery_steps(*RECOVERY_ARC[0])]
        self.assertEqual(points[-1], RECOVERY_ARC[-1])
        self.assertTrue(all(math.dist(a, b) <= 2.000001 for a, b in zip(points, points[1:])))
        self.assertTrue(all(b[0] <= a[0] for a, b in zip(points, points[1:])))
        self.assertGreaterEqual(min(p[1] for p in points), TABLE_Z_MIN)
        for point in RECOVERY_ARC:
            self.assertTrue(any(math.dist(point, p) < 1e-6 for p in points))

    def test_middle_entry_does_not_return_to_outer_edge(self):
        points = list(recovery_steps(*RECOVERY_ARC[4]))
        self.assertTrue(all(p[0] <= RECOVERY_ARC[4][0] for p in points))

    def test_unrecorded_entry_and_below_table_are_rejected(self):
        for point in [(300, 100), (179, -136)]:
            with self.assertRaises(ValueError):
                list(recovery_steps(*point))

    def test_hard_kill_overrides_latched_correction(self):
        with tempfile.TemporaryDirectory() as td:
            calls = []
            service = SensorService(td+'/sensor.json', 'fake', on_kill=calls.append)
            service.monitor.enabled = True
            service.correct_enabled = True
            service.monitor.fault_latched = True
            service.monitor.last_tilt_deg = 49.9
            self.assertFalse(service._check_hard_tilt())
            service.monitor.last_tilt_deg = 50
            self.assertTrue(service._check_hard_tilt())
            self.assertEqual(len(calls), 1)
            self.assertIn('KILL limit 50.0', calls[0])
            service._check_hard_tilt()
            self.assertEqual(len(calls), 1)

    def test_disabled_or_rejected_sample_does_not_hard_kill(self):
        with tempfile.TemporaryDirectory() as td:
            service = SensorService(td+'/sensor.json', 'fake')
            service.monitor.last_tilt_deg = 60
            self.assertFalse(service._check_hard_tilt())
            service.monitor.enabled = True
            service.monitor.sample_rejected = True
            self.assertFalse(service._check_hard_tilt())
