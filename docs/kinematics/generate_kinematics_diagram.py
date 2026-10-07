"""Generate the EnderArm exact closed-linkage kinematics SVG.

The geometry here matches docs/kinematics/README.md.

Coordinate convention:
    A = (Y=0, Z=0)
    +Y = outward from the base
    +Z = upward
    qX/qY = 90T output angles measured from +Z toward +Y

Run:
    python3 generate_kinematics_diagram.py

Output:
    ../images/kinematics/enderarm-kinematics-exact.svg
"""

from math import cos, radians, sin, sqrt
from pathlib import Path

# Exact model dimensions, mm
AB = 40.0
AC = 120.0
BF = 120.0
CF = 40.0
CH = 120.0
AG_CE = 32.0
CD_HI = 25.0
DE = 45.0
GE_DI = 120.0
HT = 27.0

# Fixed A->G vector used by the current physical model.
GROUND_G_Y = -26.933333333333334
GROUND_G_Z = 17.27991769527724

# Clear reference pose used only for the drawing.
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
    angle = radians(angle_deg)
    return (length * sin(angle), length * cos(angle))


def circle_intersections(p0, r0, p1, r1):
    delta = sub(p1, p0)
    distance = norm(delta)
    along = (r0**2 - r1**2 + distance**2) / (2.0 * distance)
    across = sqrt(max(0.0, r0**2 - along**2))

    unit = (delta[0] / distance, delta[1] / distance)
    perp = (-unit[1], unit[0])
    base = add(p0, mul(along, unit))

    return add(base, mul(across, perp)), add(base, mul(-across, perp))


def solve_linkage(qx_deg=QX_DEG, qy_deg=QY_DEG):
    a = (0.0, 0.0)
    b = radial(AB, qy_deg)
    c = radial(AC, qx_deg)

    # First circle-intersection branch is physical Assembly 1.
    f, _ = circle_intersections(b, BF, c, CF)

    h = add(c, mul(CH / CF, sub(c, f)))
    g = (GROUND_G_Y, GROUND_G_Z)
    e = add(c, g)
    d = add(e, (DE, 0.0))
    i = add(h, sub(d, c))
    t = add(h, (HT, 0.0))

    return {
        "A": a, "B": b, "C": c, "D": d, "E": e,
        "F": f, "G": g, "H": h, "I": i, "T": t,
    }


def render_svg(points, output_path):
    width, height = 1500, 900
    x0, y0, plot_w, plot_h = 80, 110, 930, 700
    y_min, y_max = -55.0, 195.0
    z_min, z_max = -25.0, 150.0

    def sx(y):
        return x0 + (y - y_min) / (y_max - y_min) * plot_w

    def sy(z):
        return y0 + plot_h - (z - z_min) / (z_max - z_min) * plot_h

    def line(name1, name2, css_class):
        p1, p2 = points[name1], points[name2]
        return (
            f'<line class="{css_class}" '
            f'x1="{sx(p1[0]):.1f}" y1="{sy(p1[1]):.1f}" '
            f'x2="{sx(p2[0]):.1f}" y2="{sy(p2[1]):.1f}"/>'
        )

    members = [
        ("A", "C", "driven"),
        ("A", "B", "driven"),
        ("B", "F", "passive"),
        ("F", "C", "passive"),
        ("C", "H", "driven"),
        ("A", "G", "plate"),
        ("G", "E", "plate"),
        ("E", "C", "plate"),
        ("E", "D", "plate"),
        ("D", "C", "plate"),
        ("D", "I", "passive"),
        ("I", "H", "passive"),
        ("H", "T", "driven"),
    ]

    svg = [
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}">',
        '<rect width="100%" height="100%" fill="white"/>',
        '<style>',
        'text{font-family:Arial,Helvetica,sans-serif;fill:#111}',
        '.title{font-size:30px;font-weight:700}.sub{font-size:20px}.label{font-size:19px;font-weight:700}',
        '.small{font-size:17px}.driven{stroke:#111;stroke-width:4;fill:none}',
        '.passive{stroke:#555;stroke-width:3;stroke-dasharray:9 6;fill:none}',
        '.plate{stroke:#888;stroke-width:2.5;stroke-dasharray:3 5;fill:none}',
        '.point{fill:white;stroke:#111;stroke-width:2.5}.box{fill:white;stroke:#222;stroke-width:1.5}',
        '</style>',
        '<text x="65" y="48" class="title">EnderArm 4-DOF Kinematics — Exact Closed-Linkage Geometry</text>',
        '<text x="65" y="80" class="sub">qX = +22°, qY = −59° reference pose · dimensions in mm</text>',
    ]

    # grid
    for y in range(-50, 201, 25):
        x = sx(float(y))
        svg.append(f'<line x1="{x:.1f}" y1="{y0}" x2="{x:.1f}" y2="{y0+plot_h}" stroke="#eee"/>')
    for z in range(-25, 151, 25):
        y = sy(float(z))
        svg.append(f'<line x1="{x0}" y1="{y:.1f}" x2="{x0+plot_w}" y2="{y:.1f}" stroke="#eee"/>')

    for a, b, css in members:
        svg.append(line(a, b, css))

    offsets = {
        "A": (-18, 30), "B": (-34, 8), "C": (10, -10), "D": (10, -10),
        "E": (-34, -10), "F": (-32, -8), "G": (-34, 18), "H": (9, 30),
        "I": (10, -10), "T": (10, 6),
    }

    for name, (y, z) in points.items():
        x_px, y_px = sx(y), sy(z)
        dx, dy = offsets[name]
        svg.append(f'<circle class="point" cx="{x_px:.1f}" cy="{y_px:.1f}" r="6"/>')
        svg.append(f'<text class="label" x="{x_px+dx:.1f}" y="{y_px+dy:.1f}">{name}</text>')

    # coordinate axes
    a_x, a_y = sx(0.0), sy(0.0)
    svg.append(f'<line x1="{a_x:.1f}" y1="{a_y:.1f}" x2="{sx(65):.1f}" y2="{a_y:.1f}" stroke="#555" stroke-width="1.5"/>')
    svg.append(f'<text class="small" x="{sx(67):.1f}" y="{a_y+6:.1f}">+Y outward</text>')
    svg.append(f'<line x1="{a_x:.1f}" y1="{a_y:.1f}" x2="{a_x:.1f}" y2="{sy(65):.1f}" stroke="#555" stroke-width="1.5"/>')
    svg.append(f'<text class="small" x="{a_x+8:.1f}" y="{sy(68):.1f}">+Z</text>')

    # Main dimension labels.
    dimensions = [
        ("A", "C", "AC = 120"),
        ("A", "B", "AB = 40"),
        ("B", "F", "BF = 120"),
        ("F", "C", "CF = 40"),
        ("C", "H", "CH = 120"),
        ("H", "T", "HT = 27"),
    ]
    for p1, p2, label in dimensions:
        a, b = points[p1], points[p2]
        mid_y = (a[0] + b[0]) / 2
        mid_z = (a[1] + b[1]) / 2
        svg.append(f'<text class="small" x="{sx(mid_y)+8:.1f}" y="{sy(mid_z)-6:.1f}">{label}</text>')

    # Explanatory boxes.
    svg.extend([
        '<rect class="box" x="1050" y="115" width="400" height="250" rx="10"/>',
        '<text x="1075" y="150" class="label">Powered coordinates</text>',
        '<text x="1075" y="190" class="small">s — linear base travel</text>',
        '<text x="1075" y="225" class="small">ψ — base yaw about vertical Z</text>',
        '<text x="1075" y="260" class="small">qX — 90T output drives A → C</text>',
        '<text x="1075" y="295" class="small">qY — 90T output drives A → B</text>',
        '<text x="1075" y="330" class="small">qX/qY share the side-drive origin A</text>',
        '<rect class="box" x="1050" y="395" width="400" height="260" rx="10"/>',
        '<text x="1075" y="430" class="label">Exact dimensions</text>',
        '<text x="1075" y="470" class="small">AB 40 · AC 120 · BF 120 · CF 40</text>',
        '<text x="1075" y="505" class="small">CH 120 · AG/CE 32 · CD/HI 25</text>',
        '<text x="1075" y="540" class="small">DE 45 · GE/DI 120 · HT 27</text>',
        '<text x="1075" y="590" class="small">|F−B| = 120 · |F−C| = 40</text>',
        '<text x="1075" y="625" class="small">H = C + 3(C−F) · T = H + [27,0]</text>',
        '<rect class="box" x="1050" y="685" width="400" height="115" rx="10"/>',
        '<text x="1075" y="722" class="label">Side-drive reduction</text>',
        '<text x="1075" y="765" class="small">20T motor pulley → 90T output = 4.5:1</text>',
        '</svg>',
    ])

    output_path.write_text("\n".join(svg), encoding="utf-8")


if __name__ == "__main__":
    points = solve_linkage()
    output = (
        Path(__file__).resolve().parent.parent
        / "images"
        / "kinematics"
        / "enderarm-kinematics-exact.svg"
    )
    output.parent.mkdir(parents=True, exist_ok=True)
    render_svg(points, output)
    print(f"Wrote {output}")
    for name, (y, z) in points.items():
        print(f"{name}: Y={y:.3f} mm, Z={z:.3f} mm")
