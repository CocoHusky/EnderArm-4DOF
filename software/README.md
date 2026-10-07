# Software

The reference software stack separates high-level robot geometry from deterministic motor timing.

```text
Web interface
    |
Python robot controller
    |
kinematics + workspace safety
    |
Moonraker
    |
Klipper host
    |
USB
    |
original Ender controller board
    |
4 stepper axes
```

## Host

The host can be a Raspberry Pi, other Linux single-board computer, mini PC, desktop, or laptop running Debian/Linux and the required services.

Minimum recommended resources are **2 GB RAM and 2 GB available storage**.

## Responsibilities

### Python control application

- live robot state
- joint controls
- Cartesian target controls
- forward and inverse kinematics
- workspace boundary checking
- click-to-move
- coordinated demonstrations and trajectories
- user interface

### Klipper

- homing
- endstop handling
- coordinated step generation
- acceleration and velocity control
- low-latency motion execution
- USB communication with the original controller board

### Ender controller board

The original printer controller is retained as the real-time MCU and stepper-driver board. Board revisions vary between Ender 3 and Ender 3 Pro machines, so the Klipper MCU configuration must match the physical board in the donor printer.

## Source layout

- `host/` — Python control application and kinematics
- `klipper/` — Klipper extra modules and configuration
- `tests/` — kinematic and safety tests

The public release should only identify software as the reference 4-DOF controller after all four physical axes are present and validated in the checked-in source.
