# Hardware

This directory contains the mechanical files and bill of materials for the Ender 3 to 4-DOF conversion.

## Structure

- `bom/` — purchased parts and reused donor hardware
- `stl/` — ready-to-print files
- `step/` — editable neutral CAD exchange files
- `adapters/` — Ender-specific conversion parts

Third-party printable files must retain their original attribution and license. Ender-specific adapters and modifications should be stored separately from unchanged upstream files.

## Ender-specific conversion parts

Four small printed parts make it possible to reuse inexpensive parts and more of the original Ender hardware:

| Part | Purpose |
| --- | --- |
| `bearing-shell-top.stl` | Upper race/shell for the printed loose-ball bearing |
| `bearing-shell-bottom.stl` | Lower race/shell for the printed loose-ball bearing |
| `bearing-spacer.stl` | Adapts a standard 686ZZ bearing to the original M5 Ender fastener and provides flange-like retention |
| `motor-spacer.stl` | Provides the required spacing to use the donor stepper motor with its existing 20-tooth GT2 pulley still installed |

### Printed loose-ball bearing

The bearing-shell top and bottom are filled with 6 mm precision steel balls. This creates the rotating bearing interface without requiring the more expensive purchased bearing assembly used by the source robot design.

The 6 mm steel balls are not a substitute for the 686ZZ bearings elsewhere in the linkage. They are a separate bearing system.

### 686ZZ adapter

A standard 686ZZ bearing has a 6 mm bore, 13 mm outside diameter, and 5 mm width. The printed bearing spacer reduces the usable center interface to the original M5 Ender fastener and adds flange-like axial retention. This avoids needing the original M6-style fastener arrangement or a specialty flanged bearing.

### Motor spacer

The donor X/Y stepper motors already have 20-tooth GT2 pulleys installed. The motor spacer positions the motor correctly so those original pulleys can be retained instead of removing them or buying replacement pulleys.

The conversion is designed to reuse the donor Ender hardware wherever possible.
