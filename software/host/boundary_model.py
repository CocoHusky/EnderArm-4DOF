"""Shared joint-space envelope for the UI, workspace sampling and Klipper.

Coordinates are Motor X/Y output degrees, not Cartesian millimetres.
The lower/right refit includes the owner's physically tested Right Beer pose.
Interpolated surroundings are modeled, not additional physical measurements.
"""
import math

BOUNDARY_YIELD = 5.0
TESTED_POSE = (180.50, 78.34)
REFIT_CLEARANCE = BOUNDARY_YIELD + 0.1


def original_boundary_poly():
    pts = [(0.0, 0.0), (0.0, 7.9157)]
    for i in range(1, 81):
        x = 146.0 * i / 80.0
        pts.append((x, 0.9702*x + 7.9157))
    for i in range(1, 161):
        t = i/160.0
        u = 1.0-t
        pts.append((u**3*146.0 + 3*u*u*t*194.3 + 3*u*t*t*191.0 + t**3*182.0,
                    u**3*149.56 + 3*u*u*t*154.5 + 3*u*t*t*99.0 + t**3*81.91))
    for i in range(1, 81):
        x = 182.0 + (89.02-182.0)*i/80.0
        pts.append((x, 0.8810*x - 78.4290))
    pts.append((0.0, 0.0))
    return pts


def _convex_hull(points):
    def cross(a, b, c):
        return (b[0]-a[0])*(c[1]-a[1]) - (b[1]-a[1])*(c[0]-a[0])
    ordered = sorted(set(points))
    halves = []
    for sequence in (ordered, reversed(ordered)):
        half = []
        for point in sequence:
            while len(half) >= 2 and cross(half[-2], half[-1], point) <= 0:
                half.pop()
            half.append(point)
        halves.append(half[:-1])
    return halves[0] + halves[1]


def build_boundary_poly():
    original = original_boundary_poly()
    # Only refit the lower/right chain. The upper edge and first 60% of the
    # rounded cap stay byte-for-byte identical; no global limit is disabled.
    start = 1 + 80 + 96
    chain = original[start:-1]
    # Circumscribed polygon retains at least the existing 5-degree inward
    # clearance around the tested interior point (plus rounding tolerance).
    radius = REFIT_CLEARANCE / math.cos(math.pi/64)
    support = [(TESTED_POSE[0] + radius*math.cos(2*math.pi*i/64),
                TESTED_POSE[1] + radius*math.sin(2*math.pi*i/64)) for i in range(64)]
    hull = list(reversed(_convex_hull(chain + support)))
    index = hull.index(chain[0])
    patch = []
    for offset in range(len(hull)):
        point = hull[(index+offset) % len(hull)]
        patch.append(point)
        if point == chain[-1]:
            break
    return original[:start] + patch + [original[-1]]
