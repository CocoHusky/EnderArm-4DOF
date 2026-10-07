"""Generate the EnderArm exact side-linkage SVG.

All dimensions and equations match docs/kinematics/README.md.
Angles are 90T output angles measured from +Z toward +Y.
"""

from math import cos, radians, sin, sqrt
from pathlib import Path

AB, AC, BF, CF, CH = 40.0, 120.0, 120.0, 40.0, 120.0
DE, HT = 45.0, 27.0
GROUND_G = (-26.933333333333334, 17.27991769527724)

QX_DEG = 22.0
QY_DEG = -59.0

def add(a, b):
    return (a[0] + b[0], a[1] + b[1])

def sub(a, b):
    return (a[0] - b[0], a[1] - b[1])

def mul(k, v):
    return (k * v[0], k * v[1])

def norm(v):
    return sqrt(v[0] ** 2 + v[1] ** 2)

def radial(length, angle_deg):
    q = radians(angle_deg)
    return (length * sin(q), length * cos(q))

def circle_intersections(p0, r0, p1, r1):
    dvec = sub(p1, p0)
    d = norm(dvec)
    u = (r0*r0 - r1*r1 + d*d) / (2*d)
    h = sqrt(max(0.0, r0*r0 - u*u))
    e = (dvec[0]/d, dvec[1]/d)
    ep = (-e[1], e[0])
    base = add(p0, mul(u, e))
    return add(base, mul(h, ep)), add(base, mul(-h, ep))

A = (0.0, 0.0)
B = radial(AB, QY_DEG)
C = radial(AC, QX_DEG)
F, _ = circle_intersections(B, BF, C, CF)  # Assembly 1
H = add(C, mul(CH / CF, sub(C, F)))
G = GROUND_G
E = add(C, G)
D = add(E, (DE, 0.0))
I = add(H, sub(D, C))
T = add(H, (HT, 0.0))

for name, point in dict(A=A, B=B, C=C, D=D, E=E, F=F, G=G, H=H, I=I, T=T).items():
    print(f"{name}: Y={point[0]:.3f} mm, Z={point[1]:.3f} mm")

print("\nReference equations:")
print("|F-B| = 120 mm")
print("|F-C| = 40 mm")
print("H = C + 3(C-F)")
print("T = H + [27, 0]")
print("motor angle = 4.5 × 90T output angle")
print("\nRendered SVG is checked in at docs/images/kinematics/enderarm-kinematics-exact.svg")
