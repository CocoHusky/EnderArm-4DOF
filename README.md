# EnderArm

An Ender 3 converted into a robot arm, using its motors, controller, power
supply, and frame. It runs saved movements from a browser. This one operates
beer taps.

The Linux host runs the interface and arm geometry; Klipper handles motion.
The camera, touch sensor, and linear rail are optional.

https://github.com/user-attachments/assets/b2309b1c-bfe8-4a8f-a414-62b6e4a15c9a

[Download video](docs/videos/arm-motion-full.mp4)

## Start here

1. **[Build the hardware](hardware/README.md)** — donor parts, printed adapters,
   assembly, wiring, and the [parts list](hardware/bom/README.md).
2. **[Set up the software](software/README.md)** — Linux host, Klipper,
   Moonraker, and the browser controller.
3. **[Add the optional sensor](docs/sensors.md)** — touch gestures, LED feedback,
   and acceleration readings. The arm works without it.

An Ender 3 Pro can also be used. Check the controller revision and wiring before
copying pin assignments. The linear rail is disabled on this build.

## Status

Homing, joint and Cartesian movement, saved programs, and beer-tap operation
have been tested on this arm.

Sensor readings and LED commands work with firmware 0.3.0. Presses, swipes, and
impact stops still need physical testing. Sensor-triggered motion is off by default.

Calibrate your own build, check endstops and clearances, and test slowly.
The sensor is not a replacement for a hardware emergency stop.

## The build

| Arm mechanism | Tool-mounted sensor (optional) |
| --- | --- |
| ![Closed-linkage arm](docs/images/overview/arm-linkage-assembly.jpg) | ![Sensor mounting](docs/images/sensors/sensor-head-side.jpg) |

The two side motors drive a closed linkage, rather than a conventional shoulder
and elbow. Base rotation and the optional retained linear axis bring the total
to four powered axes. Most of the extra hardware is bearings, belts, fasteners,
and printed parts; the [hardware guide](hardware/README.md) lists the details.

## Documentation

- [Using the controller](docs/controller.md)
- [Linkage geometry and coordinate systems](docs/kinematics/README.md)
- [Tests and manual deployment](docs/ci-cd.md)
- [Sensor and boundary change notes](docs/updates/2026-10-sensors-boundary.md)
- [Build photos](docs/images/README.md)
- [Contributing](CONTRIBUTING.md) · [Upstream sources and licensing](THIRD_PARTY.md)

The printable arm mechanism comes from an earlier open design. Check the
upstream attribution and redistribution terms before reusing third-party parts.
