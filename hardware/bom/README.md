# Bill of materials

The goal of the conversion is to buy as little as possible and reuse the donor Ender 3 or Ender 3 Pro hardware.

## Donor machine

| Item | Notes |
| --- | --- |
| Ender 3 or Ender 3 Pro | Used donor machines can often be found for about $30 |
| Original controller board | Reused with Klipper |
| Four original stepper motors | Reassigned to the four robot DOFs |
| Original endstops and wiring | Reused where possible |
| Original power supply | Reused |
| Original Y-axis carriage/extrusion hardware | Retained as the linear base axis |
| Original M5 fasteners and T-nuts | Reused throughout the conversion where possible |
| Existing 20T GT2 pulleys on donor steppers | Retained; do not remove unless required by a specific donor configuration |

## Additional purchased hardware

| Item | Specification | Purpose |
| --- | --- | --- |
| GT2 timing belt | 200 mm long, 6 mm wide | Side-joint belt drives |
| Precision steel balls | 6 mm diameter | Run inside the printed bearing-shell top/bottom |
| 686ZZ bearing | 6 mm bore × 13 mm OD × 5 mm wide | Linkage bearing used with the printed M5 bearing spacer |

Exact quantities will be finalized from the completed reference build before the BOM is marked complete.

## Printed conversion hardware

| File | Purpose |
| --- | --- |
| `bearing-shell-top.stl` | Top half of the low-cost loose-ball bearing |
| `bearing-shell-bottom.stl` | Bottom half of the low-cost loose-ball bearing |
| `bearing-spacer.stl` | Converts the 686ZZ interface to the original M5 Ender fastener and provides flange-like retention |
| `motor-spacer.stl` | Lets the donor stepper retain its already-installed 20T pulley while matching the arm geometry |

The printed bearing shell and the 686ZZ bearing are two different bearing solutions in the same build. The 6 mm loose balls go only in the printed shell.
