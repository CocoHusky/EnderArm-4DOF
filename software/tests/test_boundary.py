"""Exercise production boundary checks without importing controller I/O."""
import ast
import heapq
import math
from pathlib import Path
from types import SimpleNamespace
import unittest

import boundary_model

ROOT = Path(__file__).resolve().parents[2]


def checks(poly, yield_mm=5.0):
    ns = {'math': math, 'heapq': heapq, 'BOUNDARY_POLY': poly, 'BOUNDARY_YIELD': yield_mm,
          'state': {'home_x_counts': 3.5, 'home_y_counts': 3.5}}
    source = ast.parse((ROOT/'software/host/robot_mapper_klipper.py').read_text())
    names = {'_point_in_poly', '_seg_dist', 'boundary_score', 'clip_xy_path',
             '_joint_segment_is_safe', 'safe_joint_route'}
    for node in source.body:
        if isinstance(node, ast.FunctionDef) and node.name in names:
            exec(compile(ast.Module(body=[node], type_ignores=[]), '<host-boundary>', 'exec'), ns)
    return ns


class BoundaryTests(unittest.TestCase):
    def setUp(self):
        self.poly = boundary_model.build_boundary_poly()
        self.host = checks(self.poly)

    def test_confirmed_pose_has_existing_inward_margin(self):
        self.assertLess(checks(boundary_model.original_boundary_poly())['boundary_score'](180.5, 78.34), 0)
        self.assertAlmostEqual(self.host['boundary_score'](180.5, 78.34), 5.1, places=6)

    def test_upper_boundary_and_old_safe_region_preserved(self):
        original = boundary_model.original_boundary_poly()
        self.assertEqual(self.poly[:178], original[:178])
        old = checks(original)
        for a in range(0, 201, 5):
            for b in range(0, 161, 5):
                if old['boundary_score'](a, b) >= 5:
                    self.assertGreaterEqual(self.host['boundary_score'](a, b), 5-1e-9)

    def test_right_beer_segments_and_klipper_agree(self):
        source = ast.parse((ROOT/'software/klipper/arm_robot.py').read_text())
        cls = next(n for n in source.body if isinstance(n, ast.ClassDef) and n.name == 'ArmRobot')
        methods = [n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name in
                   {'_build_boundary_poly', '_point_in_poly', '_seg_dist', '_boundary_score', '_check_joint_path'}]
        ns = {'math': math, '_arm_boundary': boundary_model}
        bare = ast.ClassDef(name='Checks', bases=[], keywords=[], body=methods, decorator_list=[])
        tree = ast.fix_missing_locations(ast.Module(body=[bare], type_ignores=[]))
        exec(compile(tree, '<klipper-boundary>', 'exec'), ns)
        robot = ns['Checks']()
        robot.home_a = robot.home_b = 3.5
        robot.gcode = SimpleNamespace(error=RuntimeError)
        self.assertEqual(robot._build_boundary_poly(), self.poly)
        points = [(3.5,3.5),(78.94,69.42),(170.7497,119.3201),(168.43,111.63),
                  (171.7,96.56),(159.53,100.85),(172.13,80.44),
                  (180.5,78.34),(173.15,90.51),(170.84,102.44),
                  (166.6,98.06),(46.41,40.26)]
        for start, end in zip(points, points[1:]):
            _, _, clipped = self.host['clip_xy_path'](*start, *end)
            self.assertFalse(clipped, (start, end))
            robot._check_joint_path(*start, *end)

    def test_unrelated_outside_pose_still_rejected(self):
        self.assertLess(self.host['boundary_score'](200, 60), 0)
        self.assertTrue(self.host['clip_xy_path'](172.13,80.44,200,60)[2])

    def test_safe_corridor_routes_around_a_concave_boundary(self):
        # Both endpoints lie in this L-shaped region, while the direct
        # diagonal passes through its missing upper-right quadrant.
        host = checks([(0,0),(8,0),(8,3),(3,3),(3,8),(0,8),(0,0)], yield_mm=0)
        start=(1.0,7.0)
        end=(7.0,1.0)
        self.assertFalse(host['_joint_segment_is_safe'](start,end))
        route=host['safe_joint_route'](start,end,grid_step=1.0)
        self.assertGreater(len(route),2)
        self.assertEqual(route[0],start)
        self.assertEqual(route[-1],end)
        for left,right in zip(route,route[1:]):
            self.assertTrue(host['_joint_segment_is_safe'](left,right))


if __name__ == '__main__':
    unittest.main()
