import unittest

from kinematics import (
    LOWER_LINK_MM,
    TOOL_RADIAL_OFFSET_MM,
    UPPER_LINK_MM,
    base_step_travel_mm,
    full_linkage_from_motor_degrees,
    full_linkage_kinematics,
    forward_kinematics,
    home_linkage_pose,
    inverse_kinematics,
    member_angles,
    motor_motion_frames,
    link_step_travel_mm,
    side_joint_degrees_from_microsteps,
    side_motor_degrees_from_joint_degrees,
    side_motor_microsteps_from_joint_degrees,
    side_point_to_world,
    tested_output_motion_frames,
    tool_is_outwardmost,
)


class ForwardKinematicsTests(unittest.TestCase):
    def test_firmware_home_pose(self):
        points = forward_kinematics(0.0, 0.0, 90.0)

        self.assertEqual(points.shoulder.x, 0.0)
        self.assertEqual(points.elbow.y, 0.0)
        self.assertAlmostEqual(points.elbow.z, LOWER_LINK_MM)
        self.assertAlmostEqual(points.wrist.y, UPPER_LINK_MM)
        self.assertAlmostEqual(points.tool_center.y, UPPER_LINK_MM + TOOL_RADIAL_OFFSET_MM)
        self.assertAlmostEqual(points.tool_center.z, LOWER_LINK_MM)

    def test_base_rotation_uses_firmware_xy_convention(self):
        points = forward_kinematics(90.0, 0.0, 90.0)

        self.assertAlmostEqual(points.tool_center.x, 174.0)
        self.assertAlmostEqual(points.tool_center.y, 0.0, places=10)

    def test_one_degree_motion_is_a_chord(self):
        self.assertAlmostEqual(link_step_travel_mm(120.0, 1.0), 2.094369, places=6)
        self.assertAlmostEqual(base_step_travel_mm(174.0, 1.0), 3.036834, places=6)

    def test_side_motor_reduction(self):
        self.assertAlmostEqual(side_joint_degrees_from_microsteps(40.0), 1.0)
        self.assertAlmostEqual(side_motor_degrees_from_joint_degrees(1.0), 4.5)
        self.assertAlmostEqual(side_motor_microsteps_from_joint_degrees(1.0), 40.0)

    def test_motor_angle_wrapper_and_world_transform(self):
        points, tool = full_linkage_from_motor_degrees(99.0, -265.5, 90.0)

        expected = full_linkage_kinematics(22.0, -59.0).tool
        self.assertAlmostEqual(points.tool.y, expected.y)
        self.assertAlmostEqual(points.tool.z, expected.z)
        self.assertAlmostEqual(tool.x, expected.y)
        self.assertAlmostEqual(tool.y, 0.0, places=8)
        self.assertAlmostEqual(tool.z, expected.z)

        point = side_point_to_world(expected, 0.0)
        self.assertAlmostEqual(point.x, 0.0)
        self.assertAlmostEqual(point.y, expected.y)

    def test_motion_frame_generator_includes_all_pivots_and_tool(self):
        frames = list(motor_motion_frames(0.0, 90.0, 0.0, -90.0, steps=3))

        self.assertEqual(len(frames), 4)
        self.assertEqual(frames[0]["frame"], 0)
        self.assertEqual(frames[-1]["frame"], 3)
        self.assertAlmostEqual(frames[-1]["lower_motor_degrees"], 90.0)
        self.assertAlmostEqual(frames[-1]["crank_motor_degrees"], -90.0)
        self.assertTrue(hasattr(frames[0]["pivots"], "tool"))
        self.assertEqual(len(frames[0]["tool_xyz"].__dict__), 3)

    def test_tested_motion_frames_keep_x_as_lower_arm_and_y_as_crank(self):
        frames = list(tested_output_motion_frames())

        self.assertEqual(len(frames), 27)
        self.assertEqual(frames[0]["tested_position"], 1)
        self.assertAlmostEqual(frames[0]["x_lower_output_degrees"], 36.67)
        self.assertAlmostEqual(frames[0]["y_crank_output_degrees"], 14.44)
        self.assertAlmostEqual(frames[-1]["x_lower_output_degrees"], 40.89)
        self.assertAlmostEqual(frames[-1]["y_crank_output_degrees"], 30.67)
        self.assertTrue(hasattr(frames[0]["pivots"], "tool"))

    def test_inverse_kinematics_round_trips_firmware_home(self):
        solved = inverse_kinematics(0.0, 174.0, 120.0)

        self.assertAlmostEqual(solved.base_degrees, 0.0)
        self.assertAlmostEqual(solved.lower_degrees, 0.0)
        self.assertAlmostEqual(solved.upper_degrees, 90.0)

    def test_inverse_kinematics_round_trips_side_pose(self):
        for lower, upper in ((20.0, 110.0), (55.0, 90.0), (45.0, 85.0)):
            expected = forward_kinematics(0.0, lower, upper).tool_center
            solved = inverse_kinematics(expected.x, expected.y, expected.z)

            self.assertAlmostEqual(solved.lower_degrees, lower, places=8)
            self.assertAlmostEqual(solved.upper_degrees, upper, places=8)

    def test_inverse_kinematics_rejects_unreachable_target(self):
        with self.assertRaises(ValueError):
            inverse_kinematics(0.0, 400.0, 0.0)

    def test_named_member_angles_map_to_assembly1_links(self):
        points = full_linkage_kinematics(-70.0, -104.0)
        angles = member_angles(points)

        self.assertAlmostEqual(angles.ac_degrees, -70.0, places=8)
        self.assertAlmostEqual(angles.ab_degrees, -104.0, places=8)
        self.assertAlmostEqual(angles.bf_degrees, -70.0, places=8)
        self.assertAlmostEqual(angles.fch_degrees, 76.0, places=8)

    def test_home_pose_matches_assembly1_shape(self):
        points, angles = home_linkage_pose()

        self.assertAlmostEqual(angles.ac_degrees, -70.0, places=8)
        self.assertAlmostEqual(angles.ab_degrees, -104.0, places=8)
        self.assertEqual(points.a.x, 0.0)
        self.assertEqual(points.a.y, 0.0)
        self.assertEqual(points.a.z, 0.0)

        # Visual home contract: -70 deg from +Z is almost horizontal toward -Y,
        # only 20 deg above the -Y axis. For A-C = 120 mm this fixes C.
        self.assertAlmostEqual(points.c.y, -112.763114494309, places=8)
        self.assertAlmostEqual(points.c.z, 41.04241719908026, places=8)

        # Physical Assembly 1 topology from the reference drawing.
        self.assertLess(points.b.y, points.a.y)   # B left/down of A
        self.assertLess(points.c.y, points.a.y)   # C left/up of A
        self.assertLess(points.f.y, points.c.y)   # F farther left than C
        self.assertGreater(points.h.y, points.c.y) # H continues right from C
        self.assertGreater(points.tool.y, points.h.y) # T is right of H
        self.assertTrue(tool_is_outwardmost(points))

    def test_full_linkage_respects_all_dimension_constraints(self):
        points = full_linkage_kinematics(20.0, -135.0)

        def distance(first, second):
            return ((second.y - first.y) ** 2 + (second.z - first.z) ** 2) ** 0.5

        self.assertAlmostEqual(distance(points.a, points.b), 40.0, places=8)
        self.assertAlmostEqual(distance(points.a, points.c), 120.0, places=8)
        self.assertAlmostEqual(distance(points.b, points.f), 120.0, places=8)
        self.assertAlmostEqual(distance(points.c, points.f), 40.0, places=8)
        self.assertAlmostEqual(distance(points.c, points.d), 25.0, places=8)
        self.assertAlmostEqual(distance(points.c, points.e), 32.0, places=8)
        self.assertAlmostEqual(distance(points.d, points.e), 45.0, places=8)
        self.assertAlmostEqual(distance(points.a, points.g), 32.0, places=8)
        self.assertAlmostEqual(distance(points.g, points.e), 120.0, places=8)
        self.assertAlmostEqual(distance(points.c, points.h), 120.0, places=8)
        self.assertAlmostEqual(distance(points.d, points.i), 120.0, places=8)
        self.assertAlmostEqual(distance(points.h, points.i), 25.0, places=8)
        self.assertGreater(points.i.z, points.h.z)
        self.assertAlmostEqual(distance(points.h, points.tool), 27.0, places=8)
        self.assertAlmostEqual(points.d.z, points.e.z, places=8)
        self.assertAlmostEqual(points.tool.z, points.h.z, places=8)

if __name__ == "__main__":
    unittest.main()