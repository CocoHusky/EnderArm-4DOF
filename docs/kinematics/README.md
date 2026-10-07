# Kinematics and Cartesian-to-Motor Math

This document defines the mathematical model used by EnderArm. It intentionally separates the robot geometry from the Ender controller connector names.

All lengths are in millimetres. Public joint angles are in degrees unless otherwise noted.

## 1. Actuated coordinates

EnderArm has four powered coordinates:

| Symbol | Name | Meaning |
| --- | --- | --- |
| `s` | Linear base travel | Translation of the complete arm along the retained Ender linear rail |
| `ψ` | Base yaw | Rotation of the complete arm about the vertical axis |
| `qX` | Main-arm joint | Absolute angle of the 120 mm A→C driven link |
| `qY` | Crank joint | Absolute angle of the 40 mm A→B driven crank |

`qX` and `qY` are angles of the 90-tooth driven outputs, not motor-shaft angles.

The remaining labelled pivots are passive linkage points. They are determined by `qX` and `qY`.

## 2. Side-plane coordinate frame

The closed linkage is solved in the Y-Z plane.

```text
A = (Y=0, Z=0)
+Y = outward from the base toward the tool
+Z = upward

q = 0°   -> link points along +Z
q = +90° -> link points along +Y
```

Angles are therefore measured from +Z toward +Y.

The common drive origin is point A.

## 3. Link dimensions

The current physical model uses:

| Link | Length |
| --- | ---: |
| A-B | 40 mm |
| A-C | 120 mm |
| B-F | 120 mm |
| C-F | 40 mm |
| C-H | 120 mm |
| C-E / A-G | 32 mm |
| D-E | 45 mm |
| C-D / H-I | 25 mm |
| G-E / D-I | 120 mm |
| H-T | 27 mm |

The fixed support vector is

```text
G = (-26.933333, 17.279918)
```

in side-plane `(Y,Z)` coordinates.

## 4. Independent driven pivots

Let

```text
a = AB = 40
c = AC = 120
```

The crank tip B is

```text
B_y = a sin(qY)
B_z = a cos(qY)
```

or

```text
B(qY) = [40 sin(qY), 40 cos(qY)]
```

The main-arm pivot C is

```text
C_y = c sin(qX)
C_z = c cos(qX)
```

or

```text
C(qX) = [120 sin(qX), 120 cos(qX)]
```

These are the only two independent coordinates in the side linkage.

## 5. Solving the closed linkage

Point F is passive. It must simultaneously satisfy

```text
|F - B| = BF = 120
|F - C| = CF = 40
```

Therefore F is the intersection of two circles.

Define

```text
d = C - B
D = ||d||
e = d / D
```

where `e` is the unit vector from B toward C.

The distance from B to the projection of F onto the B-C line is

```text
u = (BF² - CF² + D²) / (2D)
```

The perpendicular distance from that line to F is

```text
h = sqrt(BF² - u²)
```

A perpendicular unit vector in the Y-Z plane is

```text
e_perp = [-e_z, e_y]
```

The two mathematical solutions are therefore

```text
F1 = B + u e + h e_perp
F2 = B + u e - h e_perp
```

Only one corresponds to the physical assembled arm. EnderArm uses the physical Assembly-1 branch.

If the two circles do not intersect, that pair of `qX,qY` angles is not physically reachable.

## 6. Remaining passive pivots

F, C, and H are collinear. Because

```text
CF = 40
CH = 120
```

the vector from C to H is three times the vector from F to C:

```text
H = C + (CH / CF)(C - F)
H = C + 3(C - F)
```

The lower parallelogram gives

```text
E = C + G
```

The triplate gives

```text
D = E + [45, 0]
```

The upper parallelogram gives

```text
I = H + (D - C)
```

The tool point is

```text
T = H + [27, 0]
```

Thus the complete side-plane forward-kinematics function is

```text
T = f(qX, qY)
```

The tool coordinates `T_y,T_z` are fully determined by the two driven 90T joint angles.

## 7. Forward kinematics

The complete side-plane calculation is:

```text
qX, qY
  ↓
B(qY), C(qX)
  ↓
circle intersection -> F
  ↓
H = C + 3(C-F)
  ↓
T = H + [27,0]
  ↓
(T_y, T_z)
```

This is why EnderArm must not be modeled as an ordinary serial two-link shoulder/elbow mechanism.

## 8. Side-plane inverse kinematics

Given a desired tool position

```text
T* = [Y*, Z*]
```

we need to find

```text
q = [qX, qY]^T
```

such that

```text
f(q) = T*
```

There is no simple serial-arm elbow equation because the tool position is produced by the complete closed linkage.

The reference controller uses a Newton/Jacobian solve.

Define the error

```text
e(q) = f(q) - T*
```

or

```text
e = [
    T_y(qX,qY) - Y*
    T_z(qX,qY) - Z*
]
```

The Jacobian is

```text
J = [
    ∂T_y/∂qX   ∂T_y/∂qY
    ∂T_z/∂qX   ∂T_z/∂qY
]
```

The implementation estimates the derivatives numerically with

```text
ε = 0.01°
```

For example,

```text
∂T_y/∂qX ≈ [T_y(qX+ε,qY) - T_y(qX,qY)] / ε
∂T_z/∂qX ≈ [T_z(qX+ε,qY) - T_z(qX,qY)] / ε
```

and equivalently for `qY`.

The Newton update is

```text
q_next = q - J⁻¹ e
```

For a 2×2 Jacobian

```text
J = [a b
     c d]
```

with

```text
det(J) = ad - bc
```

the correction used by the controller is

```text
ΔqX = ( d e_y - b e_z) / det(J)
ΔqY = (-c e_y + a e_z) / det(J)

qX <- qX - ΔqX
qY <- qY - ΔqY
```

The current implementation:

- starts from the current physical joint angles
- remains on Assembly 1
- stops when tool-position error is less than 0.01 mm
- allows up to 80 iterations
- rejects the solve when `|det(J)| < 1e-8`, because the linkage is near a singularity

Using the current pose as the seed is important. The closed linkage has multiple mathematical branches, and a distant arbitrary seed can converge to the wrong assembly.

## 9. Joint coordinates and physical home

Geometric zero and physical home are different.

Geometric zero is defined by

```text
qX = 0° -> A-C points along +Z
qY = 0° -> A-B points along +Z
```

After homing, the controller maps its measured joint coordinates to geometric angles with

```text
qX = qX_home + dirX (x - x_home)
qY = qY_home + dirY (y - y_home)
```

where

- `x,y` are the measured 90T output-joint coordinates
- `x_home,y_home` are the post-home reference values
- `dirX,dirY` account for motor/wiring direction
- `qX_home,qY_home` are physical calibration offsets

The current reference values in the development model are

```text
qX_home = -70°
qY_home = -104°
```

These are calibration values for the reference arm, not universal geometric constants.

## 10. 90T output angle to motor-shaft angle

Both side drives use

```text
20T motor pulley -> 90T output pulley
```

Therefore

```text
q_output = θ_motor (20 / 90)
```

and

```text
θ_motor = q_output (90 / 20)
θ_motor = 4.5 q_output
```

For commanded changes,

```text
Δθ_motor = 4.5 Δq_output
```

Examples:

| Output-joint motion | Motor-shaft motion |
| ---: | ---: |
| 1° | 4.5° |
| 5° | 22.5° |
| 10° | 45° |
| 20° | 90° |

Stepper pulse generation should be left to Klipper using the configured motor/driver parameters rather than hard-coding a microstep count into the kinematic model.

## 11. Side-plane Cartesian command to the two arm motors

The currently implemented Cartesian move performs this sequence:

```text
desired tool Y,Z
       ↓
inverse_side(Y,Z,current qX,current qY)
       ↓
desired qX,qY
       ↓
apply physical-home offsets and direction
       ↓
desired 90T output coordinates
       ↓
Klipper coordinated motion
       ↓
20T motor pulleys drive the 90T outputs
```

In equation form,

```text
(Y*, Z*)
    -> IK
(qX*, qY*)
    -> calibration transform
(x*, y*)
    -> motion planner
motor motion
```

## 12. Extending the side-plane model to world XYZ

Base yaw rotates the side-plane radial coordinate into world X-Y.

For a side-plane tool solution

```text
T_side = [r, z]
```

and base yaw `ψ`,

```text
X = r sin(ψ)
Y = r cos(ψ)
Z = z
```

Therefore, for a world target relative to the yaw-axis origin,

```text
r = sqrt(X² + Y²)
ψ = atan2(X, Y)
z = Z
```

The side-linkage inverse problem then becomes

```text
(qX,qY) = inverse_side(r,Z)
```

and the three rotary coordinates are

```text
ψ, qX, qY
```

## 13. Adding the linear base coordinate

The retained Ender rail translates the complete yaw axis.

Let

```text
s = linear-axis position
l_hat = unit vector of the physical rail in the chosen world frame
P0 = yaw-axis origin when s = 0
```

Then the moving base origin is

```text
P_base(s) = P0 + s l_hat
```

For a requested world tool point `P*`,

```text
p = P* - P_base(s)
```

Then

```text
r = sqrt(p_x² + p_y²)
ψ = atan2(p_x, p_y)
(qX,qY) = inverse_side(r,p_z)
```

This gives

```text
P* + chosen s
    ->
p relative to moving base
    ->
ψ,r,z
    ->
qX,qY
    ->
four actuator coordinates [s,ψ,qX,qY]
```

A position-only XYZ target contains three constraints while the robot has four actuated coordinates. Therefore `s` is a redundant degree of freedom for a pure XYZ command.

A controller must choose `s` using an additional rule, for example:

- keep the current linear position when the target is reachable
- choose `s` that maximizes distance from the side-linkage workspace boundary
- minimize total joint motion
- minimize motor effort
- prefer the center of linear travel

Once `s` is chosen, the remaining inverse mapping is deterministic for the selected physical linkage branch.

## 14. Complete mathematical control chain

```text
Desired world tool point P*
        +
linear-axis policy
        ↓
choose s
        ↓
P_base(s) = P0 + s l_hat
        ↓
p = P* - P_base(s)
        ↓
ψ = atan2(p_x,p_y)
r = sqrt(p_x²+p_y²)
z = p_z
        ↓
closed-linkage IK
        ↓
qX, qY
        ↓
home-offset / direction calibration
        ↓
90T output coordinates
        ↓
20T:90T ratio
        ↓
motor-shaft motion
        ↓
Klipper trajectory generation
```

The kinematic layer should produce physical actuator coordinates. Klipper should remain responsible for converting those coordinates into timed step pulses.
