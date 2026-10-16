"""Ordered arm-side recovery arc recorded by the operator, in millimetres."""
import math

TABLE_Z_MIN = -135.6
RECOVERY_ARC = (
    (235.9, -41.2), (230.2, -70.8), (217.0, -104.0),
    (204.2, -130.0), (179.2, -135.6), (154.9, -129.1),
    (131.9, -68.4),
)


def recovery_steps(radius, z, step_mm=2.0, entry_tolerance_mm=5.0):
    """Join the nearest arc segment, then follow only its recorded direction."""
    if not all(math.isfinite(v) for v in (radius, z, step_mm, entry_tolerance_mm)):
        raise ValueError('Recovery coordinates must be finite')
    if step_mm <= 0 or entry_tolerance_mm < 0:
        raise ValueError('Recovery step and entry distance are invalid')
    if z < TABLE_Z_MIN:
        raise ValueError('Current pose is below the recovery table limit')
    candidates = []
    for i, (a, b) in enumerate(zip(RECOVERY_ARC, RECOVERY_ARC[1:])):
        delta = (b[0]-a[0], b[1]-a[1])
        t = max(0.0, min(1.0, ((radius-a[0])*delta[0]+(z-a[1])*delta[1]) /
                            (delta[0]**2+delta[1]**2)))
        point = (a[0]+t*delta[0], a[1]+t*delta[1])
        candidates.append((math.dist((radius, z), point), i, point))
    distance, index, entry = min(candidates)
    if distance > entry_tolerance_mm:
        raise ValueError('Current pose is not near the recorded recovery arc')
    current = (radius, z)
    for target in (entry, *RECOVERY_ARC[index+1:]):
        count = max(1, math.ceil(math.dist(current, target)/step_mm))
        start = current
        for n in range(1, count+1):
            point = tuple(a+(b-a)*n/count for a, b in zip(start, target))
            if math.dist(current, point) > 1e-7:
                yield point
            current = point
