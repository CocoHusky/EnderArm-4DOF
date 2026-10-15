# EnderArm — an Ender 3 turned robot arm

Reuse a 3D printer's motors, controller, power supply, and frame to build a
browser-controlled robot arm. Teach it positions, edit a movement sequence,
and save programs to run again. This build uses it to operate beer taps.

The Linux host runs the interface and arm geometry; Klipper handles motion.
The camera, touch sensor, and linear rail are optional.

https://github.com/user-attachments/assets/b2309b1c-bfe8-4a8f-a414-62b6e4a15c9a

*The reference build in motion · 44 seconds · no audio.*
[Full-resolution download](docs/videos/arm-motion-full.mp4)

## Start here

1. **[Build the hardware](hardware/README.md)** — donor parts, printed adapters,
   assembly, wiring, and the [parts list](hardware/bom/README.md).
2. **[Set up the software](software/README.md)** — Linux host, Klipper,
   Moonraker, and the browser controller.
3. **[Add the optional sensor](docs/sensors.md)** — touch gestures, LED feedback,
   and acceleration readings. The arm works without it.

This build started with an Ender 3. An Ender 3 Pro can also be used, but check
the controller revision and wiring before copying pin assignments. The reference
installation currently runs with the linear rail disabled.

## What works, and what still needs testing

The reference arm has been used for homing, joint and Cartesian movement, saved
programs, and beer-tap operation. The browser UI also has a workspace view,
diagnostics, optional camera, and private remote access. Your build still needs
its own calibration and clearance checks.

Sensor firmware 0.3.0 has been flashed and live acceleration and LED commands
checked. The three press zones, two swipes, and impact/Z-stop logic are
implemented, but physical gesture accuracy and intentional impact-stop tests
are still pending. Automatic sensor actions are off by default.

Software checks cover geometry, boundaries, and sensor logic. They do not prove
that a physical movement is safe. Check free movement and endstops first, then
test slowly. The sampled sensor guards are not a hardware emergency stop.

## The build

| Arm mechanism | Tool-mounted sensor (optional) |
| --- | --- |
| ![Closed-linkage arm](docs/images/overview/arm-linkage-assembly.jpg) | ![Sensor mounting](docs/images/sensors/sensor-head-side.jpg) |

The two side motors drive a closed linkage, rather than a conventional shoulder
and elbow. Base rotation and the optional retained linear axis bring the total
to four powered axes. Most of the extra hardware is bearings, belts, fasteners,
and printed parts; the [hardware guide](hardware/README.md) lists the details.

## More detail

- [Using the controller](docs/controller.md)
- [Linkage geometry and coordinate systems](docs/kinematics/README.md)
- [Tests and manual deployment](docs/ci-cd.md)
- [Sensor and boundary change notes](docs/updates/2026-10-sensors-boundary.md)
- [Build photos](docs/images/README.md)
- [Contributing](CONTRIBUTING.md) · [Upstream sources and licensing](THIRD_PARTY.md)

The printable arm mechanism comes from an earlier open design. Check the
upstream attribution and redistribution terms before reusing third-party parts.
