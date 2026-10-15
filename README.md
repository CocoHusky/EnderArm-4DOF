# EnderArm

An Ender 3 converted into a robot arm, using its motors, controller, power
supply, and frame. It runs saved movements from a browser. This one operates
beer taps.

The Linux host runs the interface and arm geometry; Klipper handles motion.
The camera, touch sensor, and linear rail are optional.

```mermaid
flowchart TD
    Browser[Browser controls and saved programs] --> Host[Linux host: UI and arm geometry]
    Host --> Moonraker --> Klipper
    Klipper -->|USB| Board[Ender controller]
    Power[Ender power supply] --> Board
    Board --> Motors[Two linkage motors and base rotation]
    Board --> Rail[Linear rail: optional]
    Endstops[Homing switches] --> Board
    Motors --> Arm[Printed arm, belts and bearings]
    Sensor[FRDM-KL02Z: touch, LED and accelerometer] -. optional USB .-> Host
    Camera[USB camera] -. optional .-> Host
    Remote[Remote browser through Tailscale] -. optional .-> Host
```

https://github.com/user-attachments/assets/b2309b1c-bfe8-4a8f-a414-62b6e4a15c9a

[Download video](docs/videos/arm-motion-full.mp4)

## Build order

1. **Check the parts.** Start with the [parts list](hardware/bom/README.md).
   The main arm design comes from ToolKnox; the
   [hardware guide](hardware/README.md#base-arm-design-and-assembly-reference)
   links its models and assembly instructions.
2. **Build the mechanism.** Keep the Ender base, motors, controller, and power
   supply. Assemble the arm with the [Ender adapters](hardware/adapters/README.md)
   and follow the [conversion steps](hardware/README.md#how-the-ender-conversion-is-assembled).
3. **Wire it and install the software.** Check the donor board's pin assignments,
   then [set up Linux, Klipper, Moonraker, and the UI](software/README.md).
4. **Calibrate before running programs.** Follow the
   [first-power checks](software/README.md#first-power-checks): endstops, motor
   directions, homing, offsets, boundaries, and cable clearance.
5. **Record a short program.** Use the [controller](docs/controller.md#ui) to
   jog, record positions, edit the route, and save it. Test slowly before using
   it on a tap.
6. **Add extras last.** Set up the [touch sensor](docs/sensors.md), or enable
   the camera and remote access in [feature settings](software/README.md#optional-features).

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

The steps above are the build route. These are the files to read when changing it:

- [`hardware/`](hardware/README.md) — parts, printed adapters, and assembly
- [`software/host/`](software/host/) — browser controller, geometry, and optional devices
- [`software/klipper/`](software/klipper/) — robot motion extension
- [`firmware/frdm-kl02z/`](firmware/frdm-kl02z/README.md) — optional sensor firmware
- [`software/tests/`](software/tests/) — hardware-free tests
- [`.github/workflows/`](.github/workflows/) — checks and manual deployment

Further reading:

- [Using the controller](docs/controller.md)
- [Linkage geometry and coordinate systems](docs/kinematics/README.md)
- [Tests and manual deployment](docs/ci-cd.md)
- [Sensor and boundary change notes](docs/updates/2026-10-sensors-boundary.md)
- [Build photos](docs/images/README.md)
- [Contributing](CONTRIBUTING.md) · [Upstream sources and licensing](THIRD_PARTY.md)

The printable arm mechanism comes from an earlier open design. Check the
upstream attribution and redistribution terms before reusing third-party parts.
