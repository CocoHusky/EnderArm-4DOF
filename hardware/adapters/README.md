# Ender-specific adapters

These parts were created for the EnderArm conversion so the build can reuse original Ender hardware and lower-cost commodity bearings.

## Print settings

The adapter geometry and fit were tuned around:

- **0.4 mm nozzle**
- **100% infill**
- **100% model scale**
- a dimensionally calibrated FDM printer with good dimensional accuracy

A well-tuned Ender 3 Pro, Bambu Lab printer, or another high-quality printer should be an appropriate starting point.

The bearing seats, shims, M5 interface, and motor spacing include fit adjustments based on these settings. Printer calibration still matters: flow, shrinkage, elephant foot, and hole compensation can change a nominally identical print.

## 624ZZ flange shim

[624ZZ Flange Shim.stl](624ZZ%20Flange%20Shim.stl) allows a standard 624ZZ bearing to be used where the source design calls for an F624ZZ flanged bearing.

Use one shim with each 624ZZ bearing.

## 6 mm ball thrust-bearing shell

[6mm Ball Thrust Bearing Shell Top.stl](6mm%20Ball%20Thrust%20Bearing%20Shell%20Top.stl) and [6mm Ball Thrust Bearing Shell Bottom.stl](6mm%20Ball%20Thrust%20Bearing%20Shell%20Bottom.stl) form the printed thrust-bearing race.

Fill the race with 6 mm precision steel balls during assembly. This replaces the purchased thrust-bearing assembly used by the source robot design.

## 686ZZ M5 bearing spacer

[686ZZ M5 Bearing Spacer.stl](686ZZ%20M5%20Bearing%20Spacer.stl) is used with a standard 686ZZ bearing.

The 686ZZ has a 6 mm bore. The spacer creates the M5-compatible pivot interface required to reuse the original Ender fastener and provides the locating surface needed by the linkage.

## Ender XY 20T pulley motor spacer

[Ender XY 20T Pulley Motor Spacer.stl](Ender%20XY%2020T%20Pulley%20Motor%20Spacer.stl) positions the donor X/Y stepper motors correctly for the arm belt drives.

It is specifically designed so the original 20-tooth GT2 pulley can remain installed on the motor shaft.

## Files

- 624ZZ Flange Shim.stl
- 6mm Ball Thrust Bearing Shell Top.stl
- 6mm Ball Thrust Bearing Shell Bottom.stl
- 686ZZ M5 Bearing Spacer.stl
- Ender XY 20T Pulley Motor Spacer.stl
