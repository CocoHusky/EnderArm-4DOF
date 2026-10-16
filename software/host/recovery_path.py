"""Relative radial recovery within the current arm-side workspace."""
import math

TABLE_Z_MIN = -135.6
RECOVERY_Z_MIN = TABLE_Z_MIN + 5.0
RECOVERY_TOP_Z = TABLE_Z_MIN / 2.0


def recovery_steps(radius, z, side_samples, feasible, step_mm=2.0):
    """Descend outside, turn inward, then ascend the connected inner edge.

    Radius is reach from the base axis, independent of base rotation.
    The caller approves every step against its live joint branch.
    """
    if not all(math.isfinite(v) for v in (radius, z, step_mm)) or step_mm <= 0:
        raise ValueError('Recovery coordinates and step must be valid')
    if radius <= 0 or z < RECOVERY_Z_MIN:
        raise ValueError('Current pose is below the correction height limit or at the base axis')
    samples = [(float(p[0]), float(p[1])) for p in side_samples
               if len(p) >= 2 and p[0] > 0 and math.isfinite(p[0]) and math.isfinite(p[1])]
    if not samples:
        raise ValueError('No measured workspace is available for recovery')
    nearest_z = min(abs(p[1]-z) for p in samples)
    band = [r for r, height in samples if abs(height-z) <= nearest_z+6]
    inner, outer = min(band), max(band)
    # An inner-side start continues upward instead of descending the wrong edge.
    phase = 'up' if radius <= (inner+outer)/2 else 'down'
    floor = RECOVERY_Z_MIN + 0.5
    for _ in range(600):
        if phase == 'up' and z >= RECOVERY_TOP_Z-0.5:
            return
        if phase == 'in':
            candidates = [(radius-step_mm, z)]
        else:
            angles = range(0, 76, 15) if phase == 'down' else range(15, 166, 15)
            candidates = []
            for degrees in angles:
                t = math.radians(degrees)
                if phase == 'down':
                    dr, dz = -step_mm*math.sin(t), -step_mm*math.cos(t)
                else:
                    dr, dz = -step_mm*math.cos(t), step_mm*math.sin(t)
                    if z+dz > RECOVERY_TOP_Z:
                        scale = (RECOVERY_TOP_Z-z)/dz
                        dr, dz = dr*scale, dz*scale
                candidates.append((radius+dr, z+dz))
        chosen = next(((r, h) for r, h in candidates
                       if r > 1 and h >= floor and feasible(r, h)), None)
        if chosen is None:
            if phase == 'down':
                phase = 'in'
                continue
            if phase == 'in':
                phase = 'up'
                continue
            raise ValueError('No upward recovery step is safe at the inner boundary')
        radius, z = chosen
        yield chosen
    raise ValueError('Relative recovery exceeded its step limit')
