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
| 686ZZ bearing | 12 | 6 mm bore × 13 mm OD × 5 mm wide | Use with [`686ZZ M5 Bearing Spacer.stl`](../adapters/686ZZ%20M5%20Bearing%20Spacer.stl) |
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

## Print settings for conversion parts

Print the Ender-specific adapter parts with a **0.4 mm nozzle, 100% infill, and 100% model scale**. The fit adjustments in these parts were tuned around a dimensionally calibrated printer with good dimensional accuracy, such as a well-tuned Ender 3 Pro, Bambu Lab printer, or another reliable FDM printer.

## Printed conversion parts

These parts are printed rather than purchased:

| Part | Qty | Purpose |
| --- | ---: | --- |
| [`624ZZ Flange Shim.stl`](../adapters/624ZZ%20Flange%20Shim.stl) | 6 | Adds the required flange interface to standard 624ZZ bearings |
| [`6mm Ball Thrust Bearing Shell Top.stl`](../adapters/6mm%20Ball%20Thrust%20Bearing%20Shell%20Top.stl) | 1 | Upper half of the loose-ball bearing shell |
| [`6mm Ball Thrust Bearing Shell Bottom.stl`](../adapters/6mm%20Ball%20Thrust%20Bearing%20Shell%20Bottom.stl) | 1 | Lower half of the loose-ball bearing shell |
| [`686ZZ M5 Bearing Spacer.stl`](../adapters/686ZZ%20M5%20Bearing%20Spacer.stl) | 12 | Used with the 686ZZ bearings for the M5 pivot interface |
| [`Ender XY 20T Pulley Motor Spacer.stl`](../adapters/Ender%20XY%2020T%20Pulley%20Motor%20Spacer.stl) | 2 | Used with the donor X/Y motors and existing 20T pulleys |

### 624ZZ flange-shim conversion

Use standard 624ZZ bearings with the printed flange shim instead of purchasing F624ZZ flanged bearings.

### Bearing-shell conversion

The printed bearing shell uses loose 6 mm steel balls in place of the more expensive purchased thrust-bearing assembly.

### 686ZZ conversion

The upstream design uses flanged F686ZZ bearings and M6 pivot hardware. EnderArm instead uses standard 686ZZ bearings with a printed spacer. The spacer creates an M5-compatible center interface and flange-like retention so the original Ender hardware can be reused.

### Motor-spacer conversion

The Ender X/Y motors already have 20T GT2 pulleys installed. The motor spacers position those motors correctly in the arm so the pulleys can stay on the shafts.

## Estimated build cost

The cost below is a **budget estimate as of October 2026**. It uses representative U.S. online prices and the project's $30 used-Ender donor target. Shipping and sales tax are not included.

| Purchase | What it covers | Estimated cost |
| --- | --- | ---: |
| Used Ender 3 / Ender 3 Pro donor | Controller, PSU, 4 steppers, endstops, wiring, linear axis, frame hardware, two existing 20T pulleys | **$30.00** |
| 624ZZ bearings | 10-pack; 6 required | $10.99 |
| 686ZZ bearings | 20-pack; 12 required | $15.49 |
| 6 mm precision steel balls | 100-pack; enough for the printed bearing shell | $9.99 |
| 200 mm × 6 mm GT2 closed belts | 3 required | $1.50 |
| M2/M3/M4/M5 button-head fastener assortment | Covers the required M2/M3/M4 screws and standard nuts | $19.99 |
| M4 nylon lock nuts | 50-pack; 14 required | $8.99 |
| PETG filament allowance | One 1 kg spool for printed arm/conversion parts; actual consumption depends on slicer settings | $17.99 |
| GT2 20T pulley | Printed using the filament allowance | $0.00 |

### Reference total

```text
Additional materials:     $84.94
Used Ender donor:         $30.00
--------------------------------
Estimated robot total:   $114.94
```

**Budget target: about $115 for the complete robot hardware** when the donor Ender is obtained for about $30 and the additional 20T pulley is printed.

If the third 20T pulley is purchased instead of printed, a representative 5-pack is about $6.99, bringing the estimate to about **$121.93**.

### Host computer

The motion host is separate from the mechanical robot BOM because an existing Linux computer, Raspberry Pi, or laptop can be used.

- Existing compatible computer: **$0 additional**
- Used Dell Wyse 5010 reference host: about **$29.95**
- Reference build including a purchased Wyse host: about **$144.89**

The donor printer is the largest variable. Current used Ender listings can be substantially higher than $30, so a more general estimate is:

```text
Robot total = donor Ender price + approximately $84.94
```

Add about $29.95 only if a dedicated Wyse host is also needed.

### Price references

Representative prices used for this estimate:

- 624ZZ 10-pack: $10.99
- 686ZZ 20-pack: $15.49
- 6 mm steel balls, 100-pack: $9.99
- 200 mm GT2 belt: $0.50 each
- M2-M5 button-head fastener assortment: $19.99
- M4 nylon lock nuts, 50-pack: $8.99
- 1 kg PETG filament allowance: $17.99
- Dell Wyse 5010: about $29.95

Prices change over time; the quantities and specifications in the BOM are the authoritative build requirements.

## Upstream reference

- [ToolKnox/Robotic-Arm-Arduino-project](https://github.com/ToolKnox/Robotic-Arm-Arduino-project)
- [Upstream BOM](https://github.com/ToolKnox/Robotic-Arm-Arduino-project/blob/main/BOM.pdf)
- [Mechanical model on Printables](https://www.printables.com/model/1439965-robotic-arm-arduino-project)
