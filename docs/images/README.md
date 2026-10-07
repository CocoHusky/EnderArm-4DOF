# Build photography

Use this directory for the photographs and screenshots referenced by the build documentation.

Photographs should be clear engineering documentation rather than decorative images. Use a clean background, even lighting, and enough resolution to show hardware details. Keep the same arm orientation between related photos when possible.

## File format

- Photos: JPG
- Diagrams and screenshots: PNG or SVG
- Video clips: MP4
- Short motion previews: GIF only when a static image cannot explain the action
- Use lowercase filenames with hyphens
- Do not include camera-generated names such as `IMG_4821.jpg`

## 00-overview

These are the main project images.

| File | Shot |
| --- | --- |
| `00-overview/hero-assembled-arm.jpg` | Finished arm, front three-quarter view, neutral pose |
| `00-overview/hero-side-view.jpg` | Finished arm, exact side view showing the closed linkage |
| `00-overview/ender3-before.jpg` | Complete donor Ender 3 before disassembly |
| `00-overview/ender3-to-arm.jpg` | Donor printer and finished arm shown together or as a clean before/after pair |
| `00-overview/arm-in-motion.jpg` | Arm in a useful extended pose |
| `00-overview/workspace-poses.jpg` | Three or more representative poses showing the available workspace |

The main README should use `hero-assembled-arm.jpg` once it exists.

## 01-donor-printer

Document exactly what the builder starts with.

| File | Shot |
| --- | --- |
| `01-donor-printer/ender3-front.jpg` | Full Ender 3 from the front |
| `01-donor-printer/ender3-pro-front.jpg` | Ender 3 Pro example, if available |
| `01-donor-printer/controller-board.jpg` | Controller board with board revision readable |
| `01-donor-printer/controller-connectors.jpg` | Motor, endstop, power, and USB connectors visible |
| `01-donor-printer/donor-motors.jpg` | Four donor stepper motors laid out and labeled by original axis |
| `01-donor-printer/reused-hardware.jpg` | Screws, T-nuts, endstops, belts, pulleys, wiring, and other reused hardware laid out |
| `01-donor-printer/y-axis-base.jpg` | Original Y-axis base assembly that remains in the robot |

## 02-disassembly

Take each photo before removing the next major assembly.

| File | Shot |
| --- | --- |
| `02-disassembly/01-stock-printer.jpg` | Printer before disassembly |
| `02-disassembly/02-remove-top-crossbar.jpg` | Top crossbar fasteners and removal |
| `02-disassembly/03-remove-x-gantry.jpg` | X gantry before and during removal |
| `02-disassembly/04-remove-z-extrusions.jpg` | Vertical extrusions and mounting screws |
| `02-disassembly/05-remove-unused-components.jpg` | Extruder/hotend and printer-only components being removed |
| `02-disassembly/06-retained-base.jpg` | Final retained Ender base with Y-axis carriage |
| `02-disassembly/07-retained-parts-layout.jpg` | Every donor part used by the arm laid out and labeled |
| `02-disassembly/08-unused-parts-layout.jpg` | Major donor parts not required for the conversion |

## 03-printed-parts

These images let a builder identify every printed part before assembly.

| File | Shot |
| --- | --- |
| `03-printed-parts/all-parts-labeled.jpg` | Complete printed-parts set with labels matching STL filenames |
| `03-printed-parts/print-orientation-arm-parts.jpg` | Recommended print orientation for the major linkage parts |
| `03-printed-parts/print-orientation-adapters.jpg` | Recommended orientation for the Ender-specific adapters |
| `03-printed-parts/m5-bearing-spacer.jpg` | M5-to-bearing spacer alone, close and in focus |
| `03-printed-parts/m5-bearing-spacer-installed.jpg` | Spacer installed in the bearing interface with an original M5 screw |
| `03-printed-parts/printed-20t-pulley.jpg` | Optional printed 20T pulley, if used |
| `03-printed-parts/90t-output-pulley.jpg` | 90T output pulley/gear used by the side joints |

## 04-hardware

Show every non-printed purchased component at a scale that makes identification easy.

| File | Shot |
| --- | --- |
| `04-hardware/gt2-belt.jpg` | GT2 belt with width and length reference |
| `04-hardware/20t-pulley.jpg` | 20T GT2 motor pulley |
| `04-hardware/608zz-bearing.jpg` | 608ZZ bearing with ruler/caliper or marked dimensions |
| `04-hardware/6mm-steel-balls.jpg` | 6 mm precision steel balls with scale reference |
| `04-hardware/additional-hardware-layout.jpg` | Complete purchased hardware set for one arm |

Keep the 608ZZ bearing and 6 mm steel balls as separate documented parts.

## 05-mechanical-assembly

This is the most important photo set. Photograph every joint before it is hidden by the next part.

| File | Shot |
| --- | --- |
| `05-mechanical-assembly/01-linear-base.jpg` | Retained Ender Y-axis used as the linear DOF |
| `05-mechanical-assembly/02-base-rotation.jpg` | Base rotation motor, shaft/pulley, and rotating structure |
| `05-mechanical-assembly/03-side-motor-mounts.jpg` | Two side-plane motors mounted |
| `05-mechanical-assembly/04-20t-to-90t-drive.jpg` | 20T motor pulley, GT2 belt, and 90T driven pulley in one shot |
| `05-mechanical-assembly/05-main-arm-ac.jpg` | 120 mm A-C main driven arm installed |
| `05-mechanical-assembly/06-crank-ab.jpg` | 40 mm A-B crank installed |
| `05-mechanical-assembly/07-lower-linkage.jpg` | B-F/C-F lower linkage assembled |
| `05-mechanical-assembly/08-parallelogram-linkage.jpg` | Parallel-link mechanism from the side |
| `05-mechanical-assembly/09-wrist-linkage.jpg` | H-I-T tool-side linkage |
| `05-mechanical-assembly/10-bearing-stack.jpg` | Bearing, spacer, screw, printed part, and washer stack in assembly order |
| `05-mechanical-assembly/11-m5-adapter-detail.jpg` | Close-up proving the original M5 screw works with the adapter |
| `05-mechanical-assembly/12-belt-routing.jpg` | Complete belt routing for both side drives |
| `05-mechanical-assembly/13-belt-tension.jpg` | Correct belt tension and adjustment point |
| `05-mechanical-assembly/14-endstops.jpg` | Endstop locations and trigger surfaces |
| `05-mechanical-assembly/15-complete-side-view.jpg` | Finished mechanical arm in exact side view |
| `05-mechanical-assembly/16-complete-front-view.jpg` | Finished arm from the front |
| `05-mechanical-assembly/17-complete-rear-view.jpg` | Finished arm from the rear, showing wiring path |

For the exact side-view photo, position the camera normal to the linkage plane. This image will also be used to explain the kinematic points A through T.

## 06-wiring

The reader should be able to reproduce the wiring without guessing.

| File | Shot |
| --- | --- |
| `06-wiring/controller-labeled.jpg` | Controller board with each used connector labeled |
| `06-wiring/motor-x-connection.jpg` | First linkage motor connection |
| `06-wiring/motor-y-connection.jpg` | Second linkage motor connection |
| `06-wiring/base-rotation-connection.jpg` | Base-rotation motor connection |
| `06-wiring/linear-axis-connection.jpg` | Retained linear-axis motor connection |
| `06-wiring/endstop-connections.jpg` | All endstop connections |
| `06-wiring/power-input.jpg` | PSU-to-controller power connection |
| `06-wiring/usb-host-connection.jpg` | USB connection between host and Ender controller |
| `06-wiring/full-wiring-overview.jpg` | Entire electrical system in one view |
| `06-wiring/cable-management.jpg` | Final cable routing through full arm travel |

## 07-host

Document the reference host without making it mandatory.

| File | Shot |
| --- | --- |
| `07-host/dell-wyse-reference.jpg` | Dell Wyse reference host |
| `07-host/dell-wyse-usb-network.jpg` | USB, Ethernet, and power connections |
| `07-host/raspberry-pi-example.jpg` | Optional Raspberry Pi host if one is tested |
| `07-host/laptop-usb-example.jpg` | Computer-hosted setup over USB |

## 08-software

Use screenshots rather than camera photos.

| File | Shot |
| --- | --- |
| `08-software/klipper-ready.png` | Klipper/Moonraker showing the controller ready |
| `08-software/control-ui-home.png` | Main arm-control page after homing |
| `08-software/live-arm-view.png` | Live kinematic arm visualization |
| `08-software/joint-controls.png` | Joint-space controls |
| `08-software/cartesian-target.png` | Cartesian target controls |
| `08-software/workspace-boundary.png` | Measured/fitted workspace boundary |
| `08-software/click-to-move.png` | Click-to-move interaction |
| `08-software/dance-motion.png` | Dance/trajectory controls |
| `08-software/stop-control.png` | STOP control and motion status |

Do not include IP addresses, usernames, API tokens, Wi-Fi credentials, SSH keys, or other private information in screenshots.

## 09-calibration

These images connect the equations to the physical machine.

| File | Shot |
| --- | --- |
| `09-calibration/home-pose-side.jpg` | Physical home pose from exact side view |
| `09-calibration/pivot-labels.jpg` | Same side view annotated A, B, C, D, E, F, G, H, I, T |
| `09-calibration/ac-120mm.jpg` | A-C link measurement |
| `09-calibration/ab-40mm.jpg` | A-B crank measurement |
| `09-calibration/tool-offset.jpg` | H-T tool offset measurement |
| `09-calibration/90t-zero-reference.jpg` | Physical zero/index reference on the 90T output |
| `09-calibration/linear-axis-zero.jpg` | Linear-axis home/zero point |
| `09-calibration/base-rotation-zero.jpg` | Base-rotation home/zero point |
| `09-calibration/workspace-min.jpg` | One measured workspace boundary pose |
| `09-calibration/workspace-max.jpg` | Opposite/extreme workspace boundary pose |

When showing a dimension, place a ruler or caliper in the same plane as the measured feature to minimize perspective error.

## 10-validation

These are proof-of-operation images and videos.

| File | Shot |
| --- | --- |
| `10-validation/home-complete.jpg` | Arm successfully homed |
| `10-validation/linear-min.jpg` | Minimum linear travel |
| `10-validation/linear-max.jpg` | Maximum linear travel |
| `10-validation/base-min.jpg` | Base-rotation minimum |
| `10-validation/base-max.jpg` | Base-rotation maximum |
| `10-validation/workspace-extreme-1.jpg` | Arm at one validated linkage boundary |
| `10-validation/workspace-extreme-2.jpg` | Arm at another validated linkage boundary |
| `10-validation/cartesian-target.jpg` | Physical tool positioned at a commanded Cartesian target |
| `10-validation/dance.mp4` | Continuous coordinated motion using all four DOFs |
| `10-validation/click-to-move.mp4` | UI target selection followed by physical arm motion |

## Minimum photo set for the first public release

If only one photography session is available, capture these first:

1. `00-overview/hero-assembled-arm.jpg`
2. `00-overview/ender3-before.jpg`
3. `02-disassembly/06-retained-base.jpg`
4. `03-printed-parts/all-parts-labeled.jpg`
5. `03-printed-parts/m5-bearing-spacer-installed.jpg`
6. `04-hardware/additional-hardware-layout.jpg`
7. `05-mechanical-assembly/04-20t-to-90t-drive.jpg`
8. `05-mechanical-assembly/10-bearing-stack.jpg`
9. `05-mechanical-assembly/11-m5-adapter-detail.jpg`
10. `05-mechanical-assembly/15-complete-side-view.jpg`
11. `06-wiring/controller-labeled.jpg`
12. `06-wiring/full-wiring-overview.jpg`
13. `09-calibration/pivot-labels.jpg`
14. `09-calibration/90t-zero-reference.jpg`
15. `10-validation/dance.mp4`

That set is enough to make the main README, mechanical assembly guide, wiring guide, kinematics guide, and calibration guide visually complete.
