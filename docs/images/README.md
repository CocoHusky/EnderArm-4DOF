# Build photography

Use this directory for the photographs and screenshots referenced by the build documentation.

Photographs should be clear engineering documentation rather than decorative images. Use a clean background, even lighting, and enough resolution to show hardware details. Keep the same arm orientation between related photos when possible.

## Current reference photos

These cleaned reference-build photographs are already included in the repository:

| File | Use |
| --- | --- |
| `overview/assembled-arm-on-ender-base.jpg` | Arm mounted on the retained Ender base |
| `overview/arm-linkage-assembly.jpg` | Standalone closed-linkage arm assembly |
| `overview/full-system-reference-build.jpg` | Complete working system and retained linear axis |
| `mechanical-assembly/linkage-bearing-detail.jpg` | Linkage bearing and fastener detail |
| `mechanical-assembly/base-rotation-endstops.jpg` | Base rotation and endstop arrangement |
| `wiring/controller-host-wiring.jpg` | Original Ender controller and host connection |

The repository copies are re-encoded JPEGs with camera metadata removed.

## File format

- Photos: JPG
- Diagrams and screenshots: PNG or SVG
- Video clips: MP4
- Short motion previews: GIF only when a static image cannot explain the action
- Use lowercase filenames with hyphens
- Do not include camera-generated names such as `IMG_4821.jpg`

## overview

These are the main project images.

| File | Shot |
| --- | --- |
| `overview/hero-assembled-arm.jpg` | Finished arm, front three-quarter view, neutral pose |
| `overview/hero-side-view.jpg` | Finished arm, exact side view showing the closed linkage |
| `overview/ender3-before.jpg` | Complete donor Ender 3 before disassembly |
| `overview/ender3-to-arm.jpg` | Donor printer and finished arm shown together or as a clean before/after pair |
| `overview/arm-in-motion.jpg` | Arm in a useful extended pose |
| `overview/workspace-poses.jpg` | Three or more representative poses showing the available workspace |

The main README should use `hero-assembled-arm.jpg` once it exists.

## donor-printer

Document exactly what the builder starts with.

| File | Shot |
| --- | --- |
| `donor-printer/ender3-front.jpg` | Full Ender 3 from the front |
| `donor-printer/ender3-pro-front.jpg` | Ender 3 Pro example, if available |
| `donor-printer/controller-board.jpg` | Controller board with board revision readable |
| `donor-printer/controller-connectors.jpg` | Motor, endstop, power, and USB connectors visible |
| `donor-printer/donor-motors.jpg` | Four donor stepper motors laid out and labeled by original axis |
| `donor-printer/reused-hardware.jpg` | Screws, T-nuts, endstops, belts, pulleys, wiring, and other reused hardware laid out |
| `donor-printer/y-axis-base.jpg` | Original Y-axis base assembly that remains in the robot |

## disassembly

Take each photo before removing the next major assembly.

| File | Shot |
| --- | --- |
| `disassembly/stock-printer.jpg` | Printer before disassembly |
| `disassembly/remove-top-crossbar.jpg` | Top crossbar fasteners and removal |
| `disassembly/remove-x-gantry.jpg` | X gantry before and during removal |
| `disassembly/remove-z-extrusions.jpg` | Vertical extrusions and mounting screws |
| `disassembly/remove-unused-components.jpg` | Extruder/hotend and printer-only components being removed |
| `disassembly/retained-base.jpg` | Final retained Ender base with Y-axis carriage |
| `disassembly/retained-parts-layout.jpg` | Every donor part used by the arm laid out and labeled |
| `disassembly/unused-parts-layout.jpg` | Major donor parts not required for the conversion |

## printed-parts

These images let a builder identify every printed part before assembly.

| File | Shot |
| --- | --- |
| `printed-parts/all-parts-labeled.jpg` | Complete printed-parts set with labels matching STL filenames |
| `printed-parts/print-orientation-arm-parts.jpg` | Recommended print orientation for the major linkage parts |
| `printed-parts/print-orientation-adapters.jpg` | Recommended orientation for the Ender-specific adapters |
| `printed-parts/m5-bearing-spacer.jpg` | M5-to-bearing spacer alone, close and in focus |
| `printed-parts/m5-bearing-spacer-installed.jpg` | Spacer installed in the bearing interface with an original M5 screw |
| `printed-parts/printed-20t-pulley.jpg` | Optional printed 20T pulley, if used |
| `printed-parts/90t-output-pulley.jpg` | 90T output pulley/gear used by the side joints |

## hardware

Show every non-printed purchased component at a scale that makes identification easy.

| File | Shot |
| --- | --- |
| `hardware/gt2-belt.jpg` | GT2 belt with width and length reference |
| `hardware/20t-pulley.jpg` | 20T GT2 motor pulley |
| `hardware/686zz-bearing.jpg` | 686ZZ bearing with ruler/caliper or marked dimensions |
| `hardware/6mm-steel-balls.jpg` | 6 mm precision steel balls with scale reference |
| `hardware/additional-hardware-layout.jpg` | Complete purchased hardware set for one arm |

Keep the 686ZZ bearing and 6 mm steel balls as separate documented parts. The 6 mm balls belong in the printed bearing-shell assembly; the 686ZZ uses the printed M5 bearing spacer.

## mechanical-assembly

This is the most important photo set. Photograph every joint before it is hidden by the next part.

| File | Shot |
| --- | --- |
| `mechanical-assembly/linear-base.jpg` | Retained Ender Y-axis used as the linear DOF |
| `mechanical-assembly/base-rotation.jpg` | Base rotation motor, shaft/pulley, and rotating structure |
| `mechanical-assembly/side-motor-mounts.jpg` | Two side-plane motors mounted |
| `mechanical-assembly/20t-to-90t-drive.jpg` | 20T motor pulley, GT2 belt, and 90T driven pulley in one shot |
| `mechanical-assembly/main-arm-ac.jpg` | 120 mm A-C main driven arm installed |
| `mechanical-assembly/crank-ab.jpg` | 40 mm A-B crank installed |
| `mechanical-assembly/lower-linkage.jpg` | B-F/C-F lower linkage assembled |
| `mechanical-assembly/parallelogram-linkage.jpg` | Parallel-link mechanism from the side |
| `mechanical-assembly/wrist-linkage.jpg` | H-I-T tool-side linkage |
| `mechanical-assembly/bearing-stack.jpg` | Bearing, spacer, screw, printed part, and washer stack in assembly order |
| `mechanical-assembly/m5-adapter-detail.jpg` | Close-up proving the original M5 screw works with the adapter |
| `mechanical-assembly/belt-routing.jpg` | Complete belt routing for both side drives |
| `mechanical-assembly/belt-tension.jpg` | Correct belt tension and adjustment point |
| `mechanical-assembly/endstops.jpg` | Endstop locations and trigger surfaces |
| `mechanical-assembly/complete-side-view.jpg` | Finished mechanical arm in exact side view |
| `mechanical-assembly/complete-front-view.jpg` | Finished arm from the front |
| `mechanical-assembly/complete-rear-view.jpg` | Finished arm from the rear, showing wiring path |

For the exact side-view photo, position the camera normal to the linkage plane. This image will also be used to explain the kinematic points A through T.

## wiring

The reader should be able to reproduce the wiring without guessing.

| File | Shot |
| --- | --- |
| `wiring/controller-labeled.jpg` | Controller board with each used connector labeled |
| `wiring/motor-x-connection.jpg` | First linkage motor connection |
| `wiring/motor-y-connection.jpg` | Second linkage motor connection |
| `wiring/base-rotation-connection.jpg` | Base-rotation motor connection |
| `wiring/linear-axis-connection.jpg` | Retained linear-axis motor connection |
| `wiring/endstop-connections.jpg` | All endstop connections |
| `wiring/power-input.jpg` | PSU-to-controller power connection |
| `wiring/usb-host-connection.jpg` | USB connection between host and Ender controller |
| `wiring/full-wiring-overview.jpg` | Entire electrical system in one view |
| `wiring/cable-management.jpg` | Final cable routing through full arm travel |

## host

Document the reference host without making it mandatory.

| File | Shot |
| --- | --- |
| `host/dell-wyse-reference.jpg` | Dell Wyse reference host |
| `host/dell-wyse-usb-network.jpg` | USB, Ethernet, and power connections |
| `host/raspberry-pi-example.jpg` | Optional Raspberry Pi host if one is tested |
| `host/laptop-usb-example.jpg` | Computer-hosted setup over USB |

## software

Use screenshots rather than camera photos.

| File | Shot |
| --- | --- |
| `software/klipper-ready.png` | Klipper/Moonraker showing the controller ready |
| `software/control-ui-home.png` | Main arm-control page after homing |
| `software/live-arm-view.png` | Live kinematic arm visualization |
| `software/joint-controls.png` | Joint-space controls |
| `software/cartesian-target.png` | Cartesian target controls |
| `software/workspace-boundary.png` | Measured/fitted workspace boundary |
| `software/click-to-move.png` | Click-to-move interaction |
| `software/dance-motion.png` | Dance/trajectory controls |
| `software/stop-control.png` | STOP control and motion status |

Do not include IP addresses, usernames, API tokens, Wi-Fi credentials, SSH keys, or other private information in screenshots.

## calibration

These images connect the equations to the physical machine.

| File | Shot |
| --- | --- |
| `calibration/home-pose-side.jpg` | Physical home pose from exact side view |
| `calibration/pivot-labels.jpg` | Same side view annotated A, B, C, D, E, F, G, H, I, T |
| `calibration/ac-120mm.jpg` | A-C link measurement |
| `calibration/ab-40mm.jpg` | A-B crank measurement |
| `calibration/tool-offset.jpg` | H-T tool offset measurement |
| `calibration/90t-zero-reference.jpg` | Physical zero/index reference on the 90T output |
| `calibration/linear-axis-zero.jpg` | Linear-axis home/zero point |
| `calibration/base-rotation-zero.jpg` | Base-rotation home/zero point |
| `calibration/workspace-min.jpg` | One measured workspace boundary pose |
| `calibration/workspace-max.jpg` | Opposite/extreme workspace boundary pose |

When showing a dimension, place a ruler or caliper in the same plane as the measured feature to minimize perspective error.

## validation

These are proof-of-operation images and videos.

| File | Shot |
| --- | --- |
| `validation/home-complete.jpg` | Arm successfully homed |
| `validation/linear-min.jpg` | Minimum linear travel |
| `validation/linear-max.jpg` | Maximum linear travel |
| `validation/base-min.jpg` | Base-rotation minimum |
| `validation/base-max.jpg` | Base-rotation maximum |
| `validation/workspace-extreme-1.jpg` | Arm at one validated linkage boundary |
| `validation/workspace-extreme-2.jpg` | Arm at another validated linkage boundary |
| `validation/cartesian-target.jpg` | Physical tool positioned at a commanded Cartesian target |
| `validation/dance.mp4` | Continuous coordinated motion using all four DOFs |
| `validation/click-to-move.mp4` | UI target selection followed by physical arm motion |

## Minimum photo set for the first public release

If only one photography session is available, capture these first:

1. `overview/hero-assembled-arm.jpg`
2. `overview/ender3-before.jpg`
3. `disassembly/retained-base.jpg`
4. `printed-parts/all-parts-labeled.jpg`
5. `printed-parts/m5-bearing-spacer-installed.jpg`
6. `hardware/additional-hardware-layout.jpg`
7. `mechanical-assembly/20t-to-90t-drive.jpg`
8. `mechanical-assembly/bearing-stack.jpg`
9. `mechanical-assembly/m5-adapter-detail.jpg`
10. `mechanical-assembly/complete-side-view.jpg`
11. `wiring/controller-labeled.jpg`
12. `wiring/full-wiring-overview.jpg`
13. `calibration/pivot-labels.jpg`
14. `calibration/90t-zero-reference.jpg`
15. `validation/dance.mp4`

That set is enough to make the main README, mechanical assembly guide, wiring guide, kinematics guide, and calibration guide visually complete.
