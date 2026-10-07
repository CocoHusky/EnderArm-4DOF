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


## Fourth endstop: repurposing a thermistor input

The retained linear axis requires a fourth homing/limit input.

On the reference 8-bit Creality V1.1.4-style board, the unused bed-temperature input was repurposed as a digital endstop input:

```text
B-MOT / MCU PA6 -> normally-open switch -> GND
```

The Klipper-side change is to use that MCU pin as an endstop rather than as a thermistor ADC input. For the reference board, the endstop declaration uses the internal pull-up:

```ini
# Reference 8-bit Creality board only
endstop_pin: ^PA6
```

The leading `^` enables the pull-up. With the switch open, the input remains high; closing the switch pulls PA6 to ground and Klipper reports it as triggered.

This was verified on the reference controller by watching the switch state change from `open` to `TRIGGERED`.

Do not configure the same PA6 pin simultaneously as a temperature sensor and an endstop. The temperature-sensor definition for that connector must be removed/disabled when the input is repurposed.

Different Ender controller revisions use different MCU pins, so the pin value above is specific to the reference 8-bit board.
