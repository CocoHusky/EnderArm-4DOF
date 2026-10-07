# Bill of Materials

This BOM starts from the mechanical BOM for the upstream ToolKnox robotic-arm design and applies the substitutions used by the EnderArm conversion.

Upstream reference:

- Project: [ToolKnox/Robotic-Arm-Arduino-project](https://github.com/ToolKnox/Robotic-Arm-Arduino-project)
- Upstream BOM revision: 2026-04-30
- Upstream BOM: [BOM.pdf](https://github.com/ToolKnox/Robotic-Arm-Arduino-project/blob/main/BOM.pdf)
- Upstream mechanical model: [Robotic Arm - Arduino project](https://www.printables.com/model/1439965-robotic-arm-arduino-project)

The upstream BOM contains 28 purchased items. EnderArm removes most of the original control electronics by reusing the Ender 3/Ender 3 Pro controller, power supply, motors, wiring, endstops, and linear motion hardware.

## EnderArm donor

| Item | Qty | Source | Notes |
| --- | ---: | --- | --- |
| Ender 3 or Ender 3 Pro | 1 | Donor | Reference build was made from an Ender 3 |
| Original Ender controller board | 1 | Donor | Runs Klipper; replaces Arduino Mega + RAMPS + external A4988s |
| Original Ender power supply | 1 | Donor | Replaces the upstream 12 V / 4 A supply |
| Original Ender stepper motors | 4 | Donor | Reassigned to the four robot DOFs |
| Original Ender endstops and wiring | reuse | Donor | Reused where possible for homing |
| Original Ender linear carriage/extrusions | 1 set | Donor | Retained for the linear DOF |
| Original Ender M5 hardware and T-nuts | reuse | Donor | Used at the converted pivot/bearing interfaces where applicable |
| Original 20T GT2 pulleys on the X/Y motors | 2 | Donor | Retained on the shafts; the motor spacers avoid removing them |

## Upstream BOM conversion

### Mechatronics

| Upstream item | Upstream qty | EnderArm requirement | Action |
| --- | ---: | --- | --- |
| NEMA17 stepper motor | 3 | 4 donor Ender steppers | **Reuse donor** |
| Arduino Mega 2560 | 1 | Original Ender controller | **Remove** |
| 12 V 4 A power supply | 1 | Original Ender PSU | **Remove** |
| RAMPS 1.4 | 1 | Original Ender controller | **Remove** |
| Female 2.1 mm DC jack | 1 | Not required | **Remove** |
| A4988 stepper driver | 3 | Drivers on Ender controller | **Remove** |
| 28BYJ-48 gripper motor | 1 | Current EnderArm build has no powered gripper | **Remove** |
| ULN2003 gripper driver | 1 | Not required | **Remove** |
| End stop | 3 | Reuse donor Ender switches | **Reuse donor** |
| 12 V 5010 fan | 1 | Not required for reference build | **Remove** |
| 2GT pulley, 20 tooth | 3 | 2 donor pulleys + 1 additional/printed pulley | **Reuse 2 / add 1** |
| F624ZZ flanged bearing | 6 | F624ZZ | **Keep x6** |
| F686ZZ flanged bearing | 12 | Standard 686ZZ + printed bearing spacers | **Replace** |
| 51105 thrust bearing | 1 | Printed bearing shell + 6 mm steel balls | **Replace** |
| 2GT closed belt, 200 mm × 6 mm | 3 | Same | **Keep x3** |
| Paper clip, min. 35 mm | 2 | Gripper linkage not used | **Remove** |

### Fasteners

The M2/M3/M4 fasteners are retained from the upstream arm BOM. Matching donor hardware can be reused when available, but these quantities are the safe build quantities for the printed arm.

| Upstream item | Qty | EnderArm requirement |
| --- | ---: | --- |
| M2 × 10 mm button-head screw | 6 | Keep |
| M3 × 6 mm button-head screw | 36 | Keep |
| M4 × 10 mm button-head screw | 6 | Keep |
| M4 × 16 mm button-head screw | 8 | Keep |
| M4 × 20 mm button-head screw | 6 | Keep |
| M6 × 20 mm button-head screw | 1 | **Replaced by donor M5 pivot hardware** |
| M6 × 40 mm button-head screw | 2 | **Replaced by donor M5 pivot hardware** |
| M6 × 50 mm button-head screw | 1 | **Replaced by donor M5 pivot hardware** |
| M2 nut | 6 | Keep |
| M3 nut | 4 | Keep |
| M4 lock nut | 14 | Keep |
| M6 lock nut | 4 | **Replaced by M5-compatible donor hardware** |

The four upstream M6 pivot fasteners are not required in the EnderArm conversion. The printed bearing spacer provides an approximately 5 mm fastener interface through a standard 686ZZ bearing so M5 Ender hardware can be used instead.

## EnderArm additional hardware

These are the non-donor mechanical parts required after applying the substitutions above.

| Item | Qty | Specification | Notes |
| --- | ---: | --- | --- |
| F624ZZ flanged bearing | 6 | 4 mm bore, 13 mm OD, 5 mm wide | Same as upstream |
| 686ZZ bearing | 12 | 6 mm bore, 13 mm OD, 5 mm wide | Standard non-flanged bearing; used with printed bearing spacers |
| 6 mm precision steel balls | 15 | 6 mm diameter | Installed in the printed thrust-bearing shell in place of the 51105 bearing |
| GT2 closed-loop timing belt | 3 | 200 mm circumference, 6 mm wide | Three belt-driven rotary joints |
| GT2 20T pulley | 1 | 20 tooth, 6 mm belt | Two donor X/Y pulleys are reused; this third pulley may also be 3D printed |
| M2 × 10 mm button-head screw | 6 | M2 | Same as upstream |
| M3 × 6 mm button-head screw | 36 | M3 | Same as upstream |
| M4 × 10 mm button-head screw | 6 | M4 | Same as upstream |
| M4 × 16 mm button-head screw | 8 | M4 | Same as upstream |
| M4 × 20 mm button-head screw | 6 | M4 | Same as upstream |
| M2 nut | 6 | M2 | Same as upstream |
| M3 nut | 4 | M3 | Same as upstream |
| M4 lock nut | 14 | M4 | Same as upstream |

If suitable M2/M3/M4 fasteners are available from the donor printer or an existing hardware assortment, they do not need to be purchased again.

## EnderArm printed conversion parts

These parts are specific to the Ender conversion and are in `hardware/adapters/`.

| Printed part | Qty | Replaces / enables |
| --- | ---: | --- |
| `bearing-shell-top.stl` | 1 | Upper race for the printed loose-ball thrust bearing |
| `bearing-shell-bottom.stl` | 1 | Lower race for the printed loose-ball thrust bearing |
| `bearing-spacer.stl` | 12 | Makes a standard 686ZZ usable with an M5 fastener and provides flange-like retention |
| `motor-spacer.stl` | 2 | Allows the donor X/Y motors to retain their factory-installed 20T pulleys |

### Printed thrust bearing

The upstream design calls for one 51105 thrust bearing. EnderArm replaces it with the two printed bearing-shell halves and 15 loose 6 mm steel balls.

The shell has the same approximately 42 mm outside envelope as the original thrust-bearing location. The balls run directly in the printed race.

### 686ZZ bearing conversion

The upstream design calls for twelve F686ZZ flanged bearings. EnderArm uses twelve less-specialized 686ZZ bearings instead.

Each 686ZZ receives a printed bearing spacer. The spacer:

- fits the 686ZZ geometry
- provides an approximately 5 mm center interface for the donor M5 hardware
- supplies flange-like axial retention
- removes the need for the upstream M6 pivot hardware

### Motor spacer

The Ender X/Y stepper motors already have 20T GT2 pulleys installed. Two motor spacers position those motors correctly in the arm so the pulleys can remain on the shafts.

## Parts removed from the upstream purchase list

The following upstream items are not required for the EnderArm reference build:

- Arduino Mega 2560
- RAMPS 1.4
- three A4988 modules
- separate 12 V / 4 A power supply
- 2.1 mm DC jack
- three separate NEMA17 motors
- 28BYJ-48 gripper motor
- ULN2003 board
- 12 V 5010 fan
- 51105 thrust bearing
- twelve F686ZZ flanged bearings
- four M6 pivot screws
- four M6 lock nuts
- gripper paper-clip links

The Ender donor replaces the controller, drivers, PSU, motors, much of the wiring, the linear axis, and several pieces of hardware.
