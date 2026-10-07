# Hardware

This directory contains the mechanical files, Ender-specific conversion parts, and bill of materials for the Ender 3 / Ender 3 Pro to 4-DOF robot-arm conversion.

## Structure

- bom/ — donor hardware and additional parts to buy
- stl/ — arm printable files
- step/ — editable CAD exchange files
- adapters/ — Ender-specific conversion parts

Third-party printable files should retain their original attribution and license. Ender-specific parts are kept separately so it is clear which pieces belong to this conversion.

## Print settings used for the conversion parts

The Ender-specific adapter parts were dimensioned and fit-adjusted around the following print setup:

- **0.4 mm nozzle**
- **100% infill**
- **100% model scale**
- a dimensionally calibrated, high-quality FDM printer
- reference print quality comparable to a well-tuned Ender 3 Pro, Bambu Lab printer, or another printer with good dimensional accuracy

The bearing fits, shims, spacers, and motor spacing were adjusted for this printing condition. Print the conversion parts at 100% scale first.

Printer extrusion width, flow calibration, shrinkage, elephant foot, and hole compensation can still change the final fit. If a part is too tight or loose, correct printer calibration or local hole compensation rather than globally scaling the model.

## How the Ender conversion is assembled

The mechanical conversion keeps the useful motion hardware from the Ender and replaces the printer gantry with the articulated arm.

### 1. Retain the Ender linear base

Keep the original Ender Y-axis rail, carriage, belt drive, stepper, extrusion, and base frame.

This becomes the robot's **linear base axis**. The complete arm assembly rides on the original moving carriage.

### 2. Build the rotating arm base

The arm base mounts to the retained Ender carriage.

The source design uses a purchased thrust bearing at the rotating base. EnderArm replaces it with:

- [6mm Ball Thrust Bearing Shell Top.stl](adapters/6mm%20Ball%20Thrust%20Bearing%20Shell%20Top.stl)
- [6mm Ball Thrust Bearing Shell Bottom.stl](adapters/6mm%20Ball%20Thrust%20Bearing%20Shell%20Bottom.stl)
- loose **6 mm precision steel balls**

The two printed shells form the bearing race and the steel balls run between them.

### 3. Replace F624ZZ bearings with standard 624ZZ bearings

Where the source arm calls for F624ZZ flanged bearings, EnderArm uses:

- standard **624ZZ bearings**
- [624ZZ Flange Shim.stl](adapters/624ZZ%20Flange%20Shim.stl)

The shim provides the required flange interface while allowing inexpensive standard 624ZZ bearings to be used.

### 4. Replace F686ZZ / M6 interfaces with standard 686ZZ + original M5 hardware

The linkage uses standard **686ZZ bearings** together with:

- [686ZZ M5 Bearing Spacer.stl](adapters/686ZZ%20M5%20Bearing%20Spacer.stl)

The 686ZZ has a 6 mm bore. The printed spacer adapts the bearing to the original Ender M5 fastener and provides the locating surface needed by the linkage.

This lets the build reuse the Ender's M5 hardware instead of requiring the source design's M6 pivot arrangement or specialty flanged bearings.

### 5. Reuse the Ender X/Y motors with their 20T pulleys still installed

The donor X/Y stepper motors already have 20-tooth GT2 pulleys on their shafts.

Use:

- [Ender XY 20T Pulley Motor Spacer.stl](adapters/Ender%20XY%2020T%20Pulley%20Motor%20Spacer.stl)

The spacer moves each motor to the correct belt plane so the original pulley can stay installed.

The two side-linkage drives use the retained **20T motor pulley → 90T driven pulley** reduction.

This gives a 4.5:1 reduction between motor-shaft angle and linkage-output angle.

### 6. Install the closed-linkage arm

Assemble the source arm geometry around the two driven 90T side joints.

The two independently powered side joints are:

- **Main-arm drive** — 120 mm A→C link
- **Crank drive** — 40 mm A→B link

The remaining pivots are passive and are constrained by the closed linkage. See [docs/kinematics/README.md](../docs/kinematics/README.md) for the complete geometry and Cartesian-to-joint math.

### 7. Install the remaining rotary drive and belts

The build uses three 200 mm × 6 mm closed-loop GT2 belts for the rotary belt-driven joints.

Two 20T pulleys are reused directly from the donor X/Y motors. The additional 20T pulley can be purchased or printed:

- [GT2 20T pulley, 5 mm bore, optimized for FDM](https://www.printables.com/model/730635-gt2-20t-pully-5mm-bore-optimized-for-fdm)

### 8. Verify the mechanism before powered motion

Before connecting powered motion:

1. Move the retained linear carriage through its full travel by hand.
2. Rotate the base and confirm the printed loose-ball bearing moves smoothly.
3. Rotate both 90T linkage outputs by hand and verify that the linkage remains on the intended assembly branch.
4. Confirm that the 624ZZ flange shims stay seated and do not bind the bearings.
5. Confirm that the 686ZZ M5 spacers locate the bearings without clamping the rotating races.
6. Check that all GT2 belts track in one plane and do not rub printed parts.
7. Verify that cables can reach the full workspace without becoming tension members.

Do not proceed to high-speed motion until every axis moves freely by hand.

## Ender-specific conversion parts

| Part | Qty | Purpose |
| --- | ---: | --- |
| [624ZZ Flange Shim.stl](adapters/624ZZ%20Flange%20Shim.stl) | 6 | Allows standard 624ZZ bearings to replace F624ZZ flanged bearings |
| [6mm Ball Thrust Bearing Shell Top.stl](adapters/6mm%20Ball%20Thrust%20Bearing%20Shell%20Top.stl) | 1 | Upper race for the printed loose-ball thrust bearing |
| [6mm Ball Thrust Bearing Shell Bottom.stl](adapters/6mm%20Ball%20Thrust%20Bearing%20Shell%20Bottom.stl) | 1 | Lower race for the printed loose-ball thrust bearing |
| [686ZZ M5 Bearing Spacer.stl](adapters/686ZZ%20M5%20Bearing%20Spacer.stl) | 12 | Adapts standard 686ZZ bearings to the original Ender M5 pivot hardware |
| [Ender XY 20T Pulley Motor Spacer.stl](adapters/Ender%20XY%2020T%20Pulley%20Motor%20Spacer.stl) | 2 | Aligns donor X/Y motors while retaining their installed 20T pulleys |

For quantities of bearings, belts, fasteners, and donor parts, see the [hardware BOM](bom/).
