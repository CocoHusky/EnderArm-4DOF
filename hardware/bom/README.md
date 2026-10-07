# Bill of Materials

EnderArm is designed to reuse as much of an Ender 3 or Ender 3 Pro as possible. The BOM is split into two parts:

1. hardware taken from the donor Ender
2. additional hardware required to complete the robotic arm

The mechanical arm is based on the ToolKnox robotic-arm design, but the EnderArm conversion replaces most of the original electronics, motors, power hardware, and several bearing/fastener parts with components already present on the Ender.

## From the Ender 3 / Ender 3 Pro

| Item | Qty | Use in EnderArm |
| --- | ---: | --- |
| Ender 3 or Ender 3 Pro | 1 | Donor machine |
| Original controller board | 1 | Klipper MCU and stepper-driver board |
| Original power supply | 1 | Powers the robot |
| Original stepper motors | 4 | Four powered robot axes |
| Original endstops | reuse | Homing / limit sensing |
| Original motor and endstop wiring | reuse | Robot wiring |
| Original Y-axis rail, carriage, belt, and extrusion | 1 set | Linear base axis |
| Original M5 screws, washers, T-nuts, and frame hardware | reuse | Mechanical assembly and converted pivots |
| Original 20T GT2 pulleys from X/Y motors | 2 | Main-arm and crank belt drives |

The donor printer replaces the separate Arduino Mega, RAMPS board, A4988 drivers, external NEMA17 motors, and separate power supply used by the upstream design.

## Additional hardware to buy

| Item | Qty | Specification | Use |
| --- | ---: | --- | --- |
| 624ZZ bearing | 6 | 4 mm bore × 13 mm OD × 5 mm wide | Use with [`624ZZ Flange Shim.stl`](../adapters/624ZZ%20Flange%20Shim.stl) |
| 686ZZ bearing | 12 | 6 mm bore × 13 mm OD × 5 mm wide | Use with [`Bearing Spacer.stl`](../adapters/Bearing%20Spacer.stl) |
| Precision steel balls | enough to fill bearing shell | 6 mm diameter | Printed loose-ball bearing replacing the purchased thrust bearing |
| GT2 closed-loop timing belt | 3 | 200 mm circumference × 6 mm wide | Three rotary belt drives |
| GT2 20T pulley | 1 | 20 tooth, 5 mm bore, for 6 mm GT2 belt | Buy or print: [GT2 20T pulley STL](https://www.printables.com/model/730635-gt2-20t-pully-5mm-bore-optimized-for-fdm) |
| M2 × 10 mm button-head screw | 6 | M2 | Arm assembly |
| M3 × 6 mm button-head screw | 36 | M3 | Arm assembly |
| M4 × 10 mm button-head screw | 6 | M4 | Arm assembly |
| M4 × 16 mm button-head screw | 8 | M4 | Arm assembly |
| M4 × 20 mm button-head screw | 6 | M4 | Arm assembly |
| M2 nut | 6 | M2 | Arm assembly |
| M3 nut | 4 | M3 | Arm assembly |
| M4 lock nut | 14 | M4 | Arm assembly |

If matching M2/M3/M4 hardware is already available, those fasteners do not need to be purchased again.

## Printed conversion parts

These parts are printed rather than purchased:

| Part | Qty | Purpose |
| --- | ---: | --- |
| [`624ZZ Flange Shim.stl`](../adapters/624ZZ%20Flange%20Shim.stl) | 6 | Adds the required flange interface to standard 624ZZ bearings |
| [`Bearing Shell Top.stl`](../adapters/Bearing%20Shell%20Top.stl) | 1 | Upper half of the loose-ball bearing shell |
| [`Bearing Shell Bottom.stl`](../adapters/Bearing%20Shell%20Bottom.stl) | 1 | Lower half of the loose-ball bearing shell |
| [`Bearing Spacer.stl`](../adapters/Bearing%20Spacer.stl) | 12 | Used with the 686ZZ bearings for the M5 pivot interface |
| [`Motor Spacer.stl`](../adapters/Motor%20Spacer.stl) | 2 | Used with the donor X/Y motors and existing 20T pulleys |

### 624ZZ flange-shim conversion

Use standard 624ZZ bearings with the printed flange shim instead of purchasing F624ZZ flanged bearings.

### Bearing-shell conversion

The printed bearing shell uses loose 6 mm steel balls in place of the more expensive purchased thrust-bearing assembly.

### 686ZZ conversion

The upstream design uses flanged F686ZZ bearings and M6 pivot hardware. EnderArm instead uses standard 686ZZ bearings with a printed spacer. The spacer creates an M5-compatible center interface and flange-like retention so the original Ender hardware can be reused.

### Motor-spacer conversion

The Ender X/Y motors already have 20T GT2 pulleys installed. The motor spacers position those motors correctly in the arm so the pulleys can stay on the shafts.

## Upstream reference

- [ToolKnox/Robotic-Arm-Arduino-project](https://github.com/ToolKnox/Robotic-Arm-Arduino-project)
- [Upstream BOM](https://github.com/ToolKnox/Robotic-Arm-Arduino-project/blob/main/BOM.pdf)
- [Mechanical model on Printables](https://www.printables.com/model/1439965-robotic-arm-arduino-project)
