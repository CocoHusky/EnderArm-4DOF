import math
import ast
from pathlib import Path
from types import SimpleNamespace
import tempfile
import unittest

from recovery_path import RECOVERY_TOP_Z, TABLE_Z_MIN, recovery_steps
from sensor_service import SensorService


class RecoveryPathTests(unittest.TestCase):
    def test_full_scoop_continues_below_trigger_with_fixed_axes(self):
        path = Path(__file__).resolve().parents[1] / 'host/robot_mapper_klipper.py'
        source = ast.parse(path.read_text())
        function = next(n for n in source.body if isinstance(n, ast.FunctionDef)
                        and n.name == '_queue_incremental_recovery')
        readings = iter([40.0, 10.0])
        commands = []
        def steps(_r, _z, _samples, feasible):
            for point in [(234, -42), (233, -43)]:
                if feasible(*point):
                    yield point
        ns = {
            'math': math, 'time': SimpleNamespace(monotonic=lambda: 0),
            'SENSOR': SimpleNamespace(monitor=SimpleNamespace(allowed_deg=20),
                hard_kill_deg=50, snapshot=lambda: {'fresh': True,
                    'monitor': {'tilt_deg': 10, 'sample_rejected': False}}),
            '_PlannerPacer': lambda **_kwargs: SimpleNamespace(wait=lambda: None, add=lambda _duration: None),
            '_settled_recovery_tilt': lambda: next(readings),
            'state': {'x': 10, 'y': 20, 'z': 77, 'linear_y': 4},
            'kinematic_state': lambda *_args: {'tool': {'x': 0, 'y': -235.9, 'z': -41.2}},
            'recovery_steps': steps,
            'workspace_payload': lambda: {'side_samples': [(130, -41), (240, -41)]},
            'MotionCancelled': type('Cancelled', (Exception,), {}),
            'plan_world_target': lambda *_args, **_kwargs: {'A': 11, 'B': 21},
            'clip_xy_path': lambda *_args: (11, 21, False),
            'TABLE_Z_MIN': TABLE_Z_MIN, 'check_stop': lambda: None,
            '_gcode': lambda command, **_kwargs: commands.append(command),
            '_sync_state_from_arm_robot': lambda: None,
        }
        exec(compile(ast.Module(body=[function], type_ignores=[]), '<incremental>', 'exec'), ns)
        targets = []
        ns['plan_world_target'] = lambda *args, **_kwargs: (targets.append(args) or {'A': 11, 'B': 21})
        for yaw in (0, 90, 180, 270):
            with self.subTest(yaw=yaw):
                commands.clear()
                targets.clear()
                readings = iter([40.0, 10.0])
                angle = math.radians(yaw)
                tool = {'x': 235.9*math.sin(angle), 'y': 235.9*math.cos(angle), 'z': -41.2}
                ns['kinematic_state'] = lambda *_args: {'tool': tool}
                ns['state']['z'] = 2.5+yaw
                ns['_queue_incremental_recovery']()
                moves = [c for c in commands if c.startswith('ARM_MOVE_JOINT')]
                self.assertEqual(len(moves), 2)
                self.assertEqual(commands[-1], 'M400')
                self.assertEqual(commands.count('M400'), 1)
                self.assertTrue(all(f'C={2.5+yaw:.5f} D=4.00000 TIME=' in c for c in moves))
                self.assertTrue(all(float(c.split('TIME=')[1]) >= .5 for c in moves))
                self.assertAlmostEqual(targets[0][0], 234*math.sin(angle))
                self.assertAlmostEqual(targets[0][1], 4+234*math.cos(angle))
        for sample in ({'fresh': False}, {'fresh': True, 'monitor': {'tilt_deg': 50}}):
            with self.subTest(sample=sample):
                readings = iter([40.0])
                commands.clear()
                ns['SENSOR'].snapshot = lambda: sample
                with self.assertRaises(RuntimeError):
                    ns['_queue_incremental_recovery']()
                self.assertFalse(commands)

    @staticmethod
    def envelope(z):
        return 130+.002*(z+60)**2, 240-.004*(z+40)**2

    def samples(self):
        return [(r, z) for z in range(-135, 41, 5) for r in self.envelope(z)]

    def feasible(self, r, z):
        inner, outer = self.envelope(z)
        return TABLE_Z_MIN <= z <= 40 and inner <= r <= outer

    def test_relative_direction_small_steps_and_table_floor(self):
        start = (235, -41)
        points = [start, *recovery_steps(*start, self.samples(), self.feasible)]
        self.assertAlmostEqual(points[-1][1], RECOVERY_TOP_Z, delta=.5)
        self.assertTrue(all(math.dist(a, b) <= 2.000001 for a, b in zip(points, points[1:])))
        self.assertGreaterEqual(min(p[1] for p in points), TABLE_Z_MIN)
        bottom = min(range(len(points)), key=lambda i: points[i][1])
        self.assertTrue(all(b[0] <= a[0] for a, b in zip(points[:bottom], points[1:bottom+1])))
        self.assertLess(min(p[0] for p in points), start[0]-50)
        self.assertTrue(all(self.feasible(*p) for p in points))

    def test_inner_start_goes_up_without_an_outer_loop(self):
        start = (136, -110)
        points = [start, *recovery_steps(*start, self.samples(), self.feasible)]
        self.assertTrue(all(b[1] >= a[1] for a, b in zip(points, points[1:])))

    def test_different_current_poses_produce_different_paths(self):
        first = list(recovery_steps(235, -41, self.samples(), self.feasible))
        second = list(recovery_steps(220, -75, self.samples(), self.feasible))
        self.assertNotEqual(first[0], second[0])
        self.assertTrue(all(math.dist((220, -75), p) <= 2.00001 for p in second[:1]))

    def test_below_table_and_missing_workspace_are_rejected(self):
        with self.assertRaises(ValueError):
            list(recovery_steps(179, -136, self.samples(), self.feasible))
        with self.assertRaises(ValueError):
            list(recovery_steps(179, -110, [], self.feasible))

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
