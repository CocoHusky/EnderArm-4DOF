"""Authoritative kinematics for the Alex Arm robotic arm.

This module owns the robot's geometric definitions.  Renderers, mappers, and
motor-control code must consume these definitions instead of inventing their
own coordinate systems.

SIDE-PLANE CONTRACT
-------------------
A = (X=0, Y=0, Z=0) is the common center of the two 90-tooth side pivots.
The side linkage moves in the Y-Z plane, therefore X=0 for every side pivot.
+Y points outward from the base toward the tool.
+Z points upward.
qX is the absolute angle of link A-C (120 mm).
qY is the absolute angle of link A-B (40 mm).
qX=qY=0 degrees means the respective link points along +Z.
Positive q rotates from +Z toward +Y.

Physical homing does NOT redefine geometric zero.  Measured home/index
positions are converted to qX/qY with explicit calibration offsets and signs.

The long-term control path is:
Cartesian target -> inverse kinematics -> qX/qY -> calibrated 90T angles
-> motor shaft angles/microsteps -> hardware.
All lengths are millimetres and all public angles are degrees.
"""

from __future__ import annotations

from dataclasses import dataclass
from math import acos, atan2, cos, degrees, hypot, radians, sin


# --- Authoritative side-plane coordinate contract ---
# A is the common side-pivot origin.  All side-linkage pivots have X = 0.
SIDE_ORIGIN = (0.0, 0.0, 0.0)  # (X, Y, Z) mm
SIDE_PLANE = "YZ"
ANGLE_ZERO_AXIS = "+Z"
ANGLE_POSITIVE_TOWARD = "+Y"

LOWER_LINK_MM = 120.0       # A-C, the qX driven arm
CRANK_LINK_MM = 40.0        # A-B, the qY driven crank
UPPER_LINK_MM = 120.0
TOOL_RADIAL_OFFSET_MM = 54.0

# --- Motor / gearbox definition ---
# 20T motor pulley -> 90T driven/output pulley.
MOTOR_PULLEY_TEETH = 20.0
DRIVEN_PULLEY_TEETH = 90.0
OUTPUT_DEGREES_PER_MOTOR_DEGREE = MOTOR_PULLEY_TEETH / DRIVEN_PULLEY_TEETH
MICROSTEPS_PER_MOTOR_REVOLUTION = 16.0 * 200.0

# --- Measured physical HOME references ---
# Angle convention is now visually grounded:
#   0 deg   = straight up along +Z
#   +90 deg = horizontal toward +Y
#   -90 deg = horizontal toward -Y
# Therefore A-C = -70 deg is a strong backward tilt: only 20 deg above -Y.
#
# A-C is the directly measured main-arm home bearing.
HOME_AC_DEGREES = -70.0
#
# Current Assembly-1 home seed for the driven A-B crank.  Keep this separate
# from follower-link observations: B-F is NOT a motor coordinate.  Refine this
# value only from the physical A-B/home calibration.
HOME_QY_DEGREES = -104.0
#
# Independently observed home tool-tip reference from the previous mapper.
# This is a validation target; it is not used to redefine the measured angles.
HOME_TOOL_REFERENCE_X_MM = -3.0
HOME_TOOL_REFERENCE_Y_MM = 54.0
HOME_TOOL_REFERENCE_Z_MM = 130.0

# The closed four-bar has two mathematical F intersections.
# Assembly 1 = first returned circle intersection (f1): the physical
# upright/outward inverted-V branch. Assembly 2 = the mirror branch.
PHYSICAL_ASSEMBLY = 1

# User-measured, assembled-safe side-axis output positions.  The first value is
# X (the main 120 mm A-C arm); the second is Y (the 40 mm A-B crank).  These
# are angles of the 90-tooth driven gears, not motor-shaft angles.
TESTED_SIDE_OUTPUT_ANGLE_PAIRS_DEGREES: tuple[tuple[float, float], ...] = (
    (36.67, 14.44), (36.22, 30.44), (27.78, 30.44), (29.56, 8.89),
    (16.22, 17.78), (11.78, 13.33), (11.78, 13.33), (6.44, 8.89),
    (2.89, 5.78), (0.00, 2.22), (23.11, 2.22), (32.89, 13.33),
    (34.22, 13.33), (35.11, 13.78), (36.44, 14.67), (19.56, 20.89),
    (24.44, 26.22), (32.89, 32.44), (36.89, 31.56), (41.78, 24.89),
    (41.78, 20.00), (39.56, 17.33), (0.00, 0.00), (22.22, 0.00),
    (33.33, 11.11), (40.44, 17.78), (40.89, 30.67),
)


@dataclass(frozen=True)
class Point:
    """A Cartesian point in millimetres."""

    x: float
    y: float
    z: float


@dataclass(frozen=True)
class ArmPoints:
    """Centers of the kinematic pivots and the tool center point."""

    shoulder: Point
    elbow: Point
    wrist: Point
    tool_center: Point


@dataclass(frozen=True)
class JointAngles:
    """Side-plane output angles in the reference firmware convention."""

    base_degrees: float
    lower_degrees: float
    upper_degrees: float


@dataclass(frozen=True)
class LinkageDimensions:
    """Dimensions for the A-I side linkage drawn in the project sketch.

    The fixed ``A-G`` vector is chosen so A-G-E-C is a parallelogram and the
    D-E triplate edge is level in the side view. Replace its direction only
    after checking the physical assembly's neutral pose.
    """

    crank_ab_mm: float = 40.0
    lower_ac_mm: float = 120.0
    lower_blue_bf_mm: float = 120.0
    red_tail_cf_mm: float = 40.0
    upper_ch_mm: float = 120.0
    triplate_cd_mm: float = 25.0
    triplate_ce_mm: float = 32.0
    triplate_de_mm: float = 45.0
    green_ge_mm: float = 120.0
    upper_blue_di_mm: float = 120.0
    wrist_hi_mm: float = 25.0
    tool_nub_ht_mm: float = 27.0
    # A-G = C-E = 32 mm. This vector also sets the constant plate orientation.
    ground_g_y_mm: float = -26.933333333333334
    ground_g_z_mm: float = 17.27991769527724


@dataclass(frozen=True)
class LinkagePoints:
    """All labelled side-plane pivots in the dimension-driven A-I model."""

    a: Point
    b: Point
    c: Point
    d: Point
    e: Point
    f: Point
    g: Point
    h: Point
    i: Point
    tool: Point


@dataclass(frozen=True)
class MemberAngles:
    """Absolute side-plane member angles, measured from +Z toward +Y.

    ac_degrees is the main A-C driven arm and is exactly qX.
    ab_degrees is the short A-B crank and is exactly qY.
    bf_degrees is the blue B-F connecting link.
    fch_degrees is the red F-C-H member; F, C and H are collinear.
    """

    ac_degrees: float
    ab_degrees: float
    bf_degrees: float
    fch_degrees: float


def forward_kinematics(
    base_degrees: float,
    lower_degrees: float,
    upper_degrees: float,
) -> ArmPoints:
    """Return pivot centers for the firmware's nominal arm model.

    ``lower_degrees`` and ``upper_degrees`` are absolute link angles measured
    in the radial/Z plane from +Z toward the outward radial direction.  This
    is the convention returned by ``robotGeometry.cpp``, not a relative elbow
    angle.  The 54 mm tool offset stays radial in the firmware's model.
    """

    base = radians(base_degrees)
    lower = radians(lower_degrees)
    upper = radians(upper_degrees)

    radial_x = sin(base)
    radial_y = cos(base)

    elbow_radius = LOWER_LINK_MM * sin(lower)
    elbow = Point(
        elbow_radius * radial_x,
        elbow_radius * radial_y,
        LOWER_LINK_MM * cos(lower),
    )

    wrist_radius = elbow_radius + UPPER_LINK_MM * sin(upper)
    wrist = Point(
        wrist_radius * radial_x,
        wrist_radius * radial_y,
        elbow.z + UPPER_LINK_MM * cos(upper),
    )

    tool_radius = wrist_radius + TOOL_RADIAL_OFFSET_MM
    tool_center = Point(
        tool_radius * radial_x,
        tool_radius * radial_y,
        wrist.z,
    )

    return ArmPoints(Point(0.0, 0.0, 0.0), elbow, wrist, tool_center)


def inverse_kinematics(
    tool_x_mm: float,
    tool_y_mm: float,
    tool_z_mm: float,
) -> JointAngles:
    """Solve the nominal firmware model for a reachable tool-center target.

    The result follows ``robotGeometry.cpp``: ``upper_degrees`` is an absolute
    upper-link angle, not an elbow-relative angle. The returned branch matches
    the normal elbow-up configuration used by the supplied firmware. ``ValueError``
    means that the target cannot be reached by the two 120 mm links after the
    fixed 54 mm radial tool offset is removed.
    """

    tool_radius = hypot(tool_x_mm, tool_y_mm)
    if tool_radius == 0.0:
        raise ValueError("tool center must not lie on the yaw axis")

    wrist_radius = tool_radius - TOOL_RADIAL_OFFSET_MM
    if wrist_radius < 0.0:
        raise ValueError("tool center lies inside the fixed 54 mm radial offset")

    side_radius = hypot(wrist_radius, tool_z_mm)
    minimum_radius = abs(LOWER_LINK_MM - UPPER_LINK_MM)
    maximum_radius = LOWER_LINK_MM + UPPER_LINK_MM
    if not minimum_radius < side_radius <= maximum_radius:
        raise ValueError(
            f"target wrist radius {side_radius:.6f} mm is outside "
            f"the reachable range ({minimum_radius:.6f}, {maximum_radius:.6f}] mm"
        )

    lower_triangle_angle = acos(
        (LOWER_LINK_MM**2 + side_radius**2 - UPPER_LINK_MM**2)
        / (2.0 * LOWER_LINK_MM * side_radius)
    )
    link_internal_angle = acos(
        (LOWER_LINK_MM**2 + UPPER_LINK_MM**2 - side_radius**2)
        / (2.0 * LOWER_LINK_MM * UPPER_LINK_MM)
    )
    radial_direction = atan2(wrist_radius, tool_z_mm)
    lower = radial_direction - lower_triangle_angle
    upper = lower + (3.141592653589793 - link_internal_angle)
    base = atan2(tool_x_mm, tool_y_mm)

    return JointAngles(degrees(base), degrees(lower), degrees(upper))


def _side_distance(first: Point, second: Point) -> float:
    return hypot(second.y - first.y, second.z - first.z)


def _side_circle_intersections(
    first: Point,
    first_radius_mm: float,
    second: Point,
    second_radius_mm: float,
) -> tuple[Point, ...]:
    """Return the two intersections of circles in the fixed-yaw Y/Z plane."""

    delta_y = second.y - first.y
    delta_z = second.z - first.z
    center_distance = hypot(delta_y, delta_z)
    minimum_distance = abs(first_radius_mm - second_radius_mm)
    maximum_distance = first_radius_mm + second_radius_mm
    tolerance = 1e-9
    if (
        center_distance == 0.0
        or center_distance > maximum_distance + tolerance
        or center_distance < minimum_distance - tolerance
    ):
        return ()

    # Preserve valid tangent poses despite tiny trigonometric rounding errors.
    center_distance = min(max(center_distance, minimum_distance), maximum_distance)

    along = (
        first_radius_mm**2
        - second_radius_mm**2
        + center_distance**2
    ) / (2.0 * center_distance)
    across = max(0.0, first_radius_mm**2 - along**2) ** 0.5
    base_y = first.y + along * delta_y / center_distance
    base_z = first.z + along * delta_z / center_distance
    perpendicular_y = -delta_z * across / center_distance
    perpendicular_z = delta_y * across / center_distance
    return (
        Point(0.0, base_y + perpendicular_y, base_z + perpendicular_z),
        Point(0.0, base_y - perpendicular_y, base_z - perpendicular_z),
    )


def full_linkage_kinematics(
    lower_degrees: float,
    crank_degrees: float,
    dimensions: LinkageDimensions = LinkageDimensions(),
    *,
    assembly: int = PHYSICAL_ASSEMBLY,
) -> LinkagePoints:
    """Solve the complete labelled A-I sketch as a planar closed linkage.

    ``lower_degrees`` is q_lower = A->C and ``crank_degrees`` is
    q_crank = A->B. Both are 90T output-joint angles measured from +Z toward
    +Y. B-F is a follower link, never an independent motor coordinate.

    Assembly 1 is the physical first circle-intersection branch (f1), the
    upright/outward inverted-V configuration. Assembly 2 is the mirror branch.
    """

    lower = radians(lower_degrees)
    crank = radians(crank_degrees)
    a = Point(0.0, 0.0, 0.0)
    g = Point(0.0, dimensions.ground_g_y_mm, dimensions.ground_g_z_mm)
    b = Point(
        0.0,
        dimensions.crank_ab_mm * sin(crank),
        dimensions.crank_ab_mm * cos(crank),
    )
    c = Point(
        0.0,
        dimensions.lower_ac_mm * sin(lower),
        dimensions.lower_ac_mm * cos(lower),
    )

    f_points = _side_circle_intersections(
        b, dimensions.lower_blue_bf_mm, c, dimensions.red_tail_cf_mm
    )
    if not f_points:
        raise ValueError("motor angles reach a lower crank singularity")

    if assembly not in (1, 2):
        raise ValueError("assembly must be 1 or 2")
    # f1 (index 0) is physical Assembly 1; f2 (index 1) is the mirror branch.
    f = f_points[assembly - 1]

    tail_y = c.y - f.y
    tail_z = c.z - f.z
    h = Point(
        0.0,
        c.y + tail_y * dimensions.upper_ch_mm / dimensions.red_tail_cf_mm,
        c.z + tail_z * dimensions.upper_ch_mm / dimensions.red_tail_cf_mm,
    )
    # A-G-E-C is a 32 by 120 parallelogram.
    e = Point(0.0, c.y + g.y, c.z + g.z)
    # The 25/32/45 triplate has its D-E edge along +Y.
    d = Point(0.0, e.y + dimensions.triplate_de_mm, e.z)
    # C-D-I-H is a 25 by 120 parallelogram.
    i = Point(0.0, h.y + (d.y - c.y), h.z + (d.z - c.z))
    # Rigid H-I-T wrist plate; H-T is always parallel to +Y.
    tool = Point(0.0, h.y + dimensions.tool_nub_ht_mm, h.z)
    return LinkagePoints(a, b, c, d, e, f, g, h, i, tool)


def _member_angle_degrees(start: Point, end: Point) -> float:
    """Return an absolute side-plane angle measured from +Z toward +Y."""

    return degrees(atan2(end.y - start.y, end.z - start.z))


def member_angles(points: LinkagePoints) -> MemberAngles:
    """Return named physical member angles for a solved linkage pose."""

    return MemberAngles(
        ac_degrees=_member_angle_degrees(points.a, points.c),
        ab_degrees=_member_angle_degrees(points.a, points.b),
        bf_degrees=_member_angle_degrees(points.b, points.f),
        fch_degrees=_member_angle_degrees(points.f, points.h),
    )


def inverse_side(
    target_y: float,
    target_z: float,
    seed_lower: float,
    seed_crank: float,
    *,
    assembly: int = PHYSICAL_ASSEMBLY,
    dimensions: LinkageDimensions = LinkageDimensions(),
) -> tuple[float, float]:
    """Solve Assembly-1 closed-linkage IK for side-plane tool Y/Z.

    The seed selects the nearby physical solution and helps keep the solve on
    the intended assembly branch. Returned values are 90T output angles:
    q_lower=A->C and q_crank=A->B, in degrees.
    """
    q_lower = float(seed_lower)
    q_crank = float(seed_crank)
    epsilon_deg = 0.01

    for _ in range(80):
        p0 = full_linkage_kinematics(
            q_lower, q_crank, dimensions, assembly=assembly
        ).tool
        err_y = p0.y - float(target_y)
        err_z = p0.z - float(target_z)
        if hypot(err_y, err_z) < 0.01:
            return q_lower, q_crank

        pl = full_linkage_kinematics(
            q_lower + epsilon_deg, q_crank, dimensions, assembly=assembly
        ).tool
        pc = full_linkage_kinematics(
            q_lower, q_crank + epsilon_deg, dimensions, assembly=assembly
        ).tool

        a = (pl.y - p0.y) / epsilon_deg
        c = (pl.z - p0.z) / epsilon_deg
        b = (pc.y - p0.y) / epsilon_deg
        d = (pc.z - p0.z) / epsilon_deg
        determinant = a * d - b * c
        if abs(determinant) < 1e-8:
            raise ValueError("Near linkage singularity")

        delta_lower = (d * err_y - b * err_z) / determinant
        delta_crank = (-c * err_y + a * err_z) / determinant
        q_lower -= delta_lower
        q_crank -= delta_crank

    raise ValueError("Target is unreachable or seed is on the wrong branch")


def tool_is_outwardmost(points: LinkagePoints, tolerance_mm: float = 1e-9) -> bool:
    """Return True only when T is the most-outward (+Y) point of the arm.

    This is a physical assembly invariant supplied from the real robot:
    throughout the valid workspace, no labelled linkage pivot may extend
    farther in +Y than the tool point T.
    """

    labelled = (
        points.a, points.b, points.c, points.d, points.e,
        points.f, points.g, points.h, points.i,
    )
    return all(points.tool.y >= point.y - tolerance_mm for point in labelled)


def home_linkage_pose(
    dimensions: LinkageDimensions = LinkageDimensions(),
) -> tuple[LinkagePoints, MemberAngles]:
    """Return the current measured physical home pose for animation/grounding.

    The independent driven coordinates are qX=A-C and qY=A-B.  qX is measured
    directly at about -70 deg.  qY is currently inferred as about +122 deg
    because that closed-linkage solution reproduces the measured F-C-H ~= -76
    deg and B-F ~= -64 deg on the physical assembly branch.
    """

    points = full_linkage_kinematics(HOME_AC_DEGREES, HOME_QY_DEGREES, dimensions)
    return points, member_angles(points)


def side_point_to_world(point: Point, base_degrees: float) -> Point:
    """Map a fixed-yaw side-plane point into the arm's world XYZ frame.

    At zero yaw, +Y is outward toward the tool and +Z is upward. Positive yaw
    rotates +Y toward +X, matching the reference firmware convention.
    """

    base = radians(base_degrees)
    return Point(
        point.y * sin(base),
        point.y * cos(base),
        point.z,
    )


def full_linkage_from_motor_degrees(
    lower_motor_degrees: float,
    crank_motor_degrees: float,
    base_degrees: float = 0.0,
    dimensions: LinkageDimensions = LinkageDimensions(),
) -> tuple[LinkagePoints, Point]:
    """Return all side pivots and the world-space T point from motor angles.

    Both side axes use a 20-tooth motor pulley driving a 90-tooth output, so
    output degrees equal motor degrees multiplied by 20/90. Mechanical home
    offsets must be applied by the caller before using physical hardware.
    """

    output_per_motor = MOTOR_PULLEY_TEETH / DRIVEN_PULLEY_TEETH
    points = full_linkage_kinematics(
        lower_motor_degrees * output_per_motor,
        crank_motor_degrees * output_per_motor,
        dimensions,
    )
    return points, side_point_to_world(points.tool, base_degrees)


def tested_output_motion_frames(
    base_degrees: float = 0.0,
    dimensions: LinkageDimensions = LinkageDimensions(),
):
    """Yield only the user-measured X/Y side-axis output poses.

    This is a conservative command source: it emits the 27 manually tested
    90-tooth output-gear pairs and deliberately does not certify the straight
    line between two entries as safe.  X is the 120 mm A-C lower arm and Y is
    the 40 mm A-B crank.
    """

    for position, (x_lower_degrees, y_crank_degrees) in enumerate(
        TESTED_SIDE_OUTPUT_ANGLE_PAIRS_DEGREES, start=1
    ):
        pivots = full_linkage_kinematics(
            x_lower_degrees, y_crank_degrees, dimensions
        )
        yield {
            "tested_position": position,
            "x_lower_output_degrees": x_lower_degrees,
            "y_crank_output_degrees": y_crank_degrees,
            "x_lower_motor_degrees": side_motor_degrees_from_joint_degrees(
                x_lower_degrees
            ),
            "y_crank_motor_degrees": side_motor_degrees_from_joint_degrees(
                y_crank_degrees
            ),
            "pivots": pivots,
            "tool_xyz": side_point_to_world(pivots.tool, base_degrees),
        }


def motor_motion_frames(
    lower_motor_start_degrees: float,
    lower_motor_end_degrees: float,
    crank_motor_start_degrees: float,
    crank_motor_end_degrees: float,
    steps: int,
    base_degrees: float = 0.0,
    dimensions: LinkageDimensions = LinkageDimensions(),
):
    """Yield evenly spaced backend frames for an animation or controller.

    Every frame contains the two commanded motor angles, all side-plane pivots
    A through T, and T in world Cartesian XYZ. This does not render anything.
    """

    if steps < 1:
        raise ValueError("steps must be at least 1")

    for index in range(steps + 1):
        progress = index / steps
        lower_motor_degrees = (
            lower_motor_start_degrees
            + (lower_motor_end_degrees - lower_motor_start_degrees) * progress
        )
        crank_motor_degrees = (
            crank_motor_start_degrees
            + (crank_motor_end_degrees - crank_motor_start_degrees) * progress
        )
        pivots, tool_xyz = full_linkage_from_motor_degrees(
            lower_motor_degrees,
            crank_motor_degrees,
            base_degrees,
            dimensions,
        )
        yield {
            "frame": index,
            "progress": progress,
            "lower_motor_degrees": lower_motor_degrees,
            "crank_motor_degrees": crank_motor_degrees,
            "pivots": pivots,
            "tool_xyz": tool_xyz,
        }


def base_step_travel_mm(tool_radius_mm: float, base_step_degrees: float) -> float:
    """Return TCP chord travel for a base rotation step at a fixed radius."""

    return 2.0 * tool_radius_mm * sin(abs(radians(base_step_degrees)) / 2.0)


def link_step_travel_mm(link_length_mm: float, link_step_degrees: float) -> float:
    """Return endpoint chord travel caused by a single link-angle step."""

    return 2.0 * link_length_mm * sin(abs(radians(link_step_degrees)) / 2.0)


def side_joint_degrees_from_microsteps(microsteps: float) -> float:
    """Return side-joint output rotation for a commanded driver microstep count.

    The reference firmware applies the 90:20 driven-to-motor pulley reduction to
    both side motors.  The returned result is an incremental angle: mechanical
    belt indexing and endstop offsets determine the physical zero position.
    """

    reduction = DRIVEN_PULLEY_TEETH / MOTOR_PULLEY_TEETH
    return microsteps * 360.0 / (MICROSTEPS_PER_MOTOR_REVOLUTION * reduction)


def side_motor_degrees_from_joint_degrees(joint_degrees: float) -> float:
    """Return required motor-shaft rotation for an incremental side-joint move."""

    return joint_degrees * DRIVEN_PULLEY_TEETH / MOTOR_PULLEY_TEETH


def side_motor_microsteps_from_joint_degrees(joint_degrees: float) -> float:
    """Return driver microsteps for an incremental side-joint output move."""

    return joint_degrees / side_joint_degrees_from_microsteps(1.0)


def degrees_from_radians(angle_radians: float) -> float:
    """Small convenience for consuming firmware angle values."""

    return degrees(angle_radians)