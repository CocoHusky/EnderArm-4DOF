# Software

EnderArm splits high-level robot geometry from deterministic step timing.

```text
Browser UI
    |
    v
robot_mapper_klipper.py
    |
    +--> kinematics.py
    |
    v
Moonraker
    |
    v
Klipper
    |
   USB
    |
    v
original Ender controller board
```

## Host requirements

A Dell Wyse is **not required**. Use a Raspberry Pi, mini PC, old laptop, thin client, or other Debian/Linux machine.

Minimum practical target:

- **2 GB RAM**
- **2 GB available storage**
- Python 3
- USB connection to the Ender controller
- network access if the UI is opened from another device

The reference build used a used Dell Wyse only because it was cheaper than Raspberry Pi hardware at the time.

## Source files

| File | Purpose |
| --- | --- |
| [`host/kinematics.py`](host/kinematics.py) | Exact closed-linkage geometry, forward kinematics, inverse kinematics, coordinate transforms, motor/output conversion |
| [`host/robot_mapper_klipper.py`](host/robot_mapper_klipper.py) | Browser UI, workspace model, boundary checking, Moonraker communication, joint/tool commands |
| [`klipper/arm_robot.py`](klipper/arm_robot.py) | Klipper extra module for native robot-arm homing and coordinated A/B/C arm axes |
| [`tests/test_kinematics.py`](tests/test_kinematics.py) | Geometry and kinematics regression tests |

## Install Debian/Linux packages

```bash
sudo apt update
sudo apt install -y git python3
```

Install Klipper and Moonraker using their normal Linux installation process.

## Clone EnderArm

```bash
cd ~
git clone https://github.com/CocoHusky/EnderArm-4DOF.git
cd EnderArm-4DOF
```

## Install the Klipper arm module

For a normal Klipper checkout at `~/klipper`:

```bash
cp software/klipper/arm_robot.py ~/klipper/klippy/extras/arm_robot.py
```

Add this section to `printer.cfg`:

```ini
[arm_robot]
```

The current module expects these exact Klipper object names:

```ini
[manual_stepper joint_x]
# qX: main A -> C drive

[manual_stepper joint_y]
# qY: A -> B crank drive

[manual_stepper joint_z]
# base-yaw drive
```

Use the **actual step/dir/enable/endstop pins for the donor controller board**. Ender 3 and Ender 3 Pro boards exist in multiple revisions, so pin assignments from another board should not be copied blindly.

The retained Ender rail is the fourth powered coordinate. Calibrate that rail in millimetres using the real donor belt, pulley, motor, and travel.

## Start the web controller

```bash
python3 software/host/robot_mapper_klipper.py
```

The checked-in controller uses:

```text
Moonraker: http://127.0.0.1:7125
Web UI:    0.0.0.0:8765
```

Open it from another machine at:

```text
http://<linux-host-ip>:8765
```

There is no required fixed IP address.

## Inverse-kinematics path

For an arm-local Cartesian command:

```text
desired tool Y,Z
        |
        v
inverse_side()
        |
        v
qX, qY 90T output angles
        |
        v
home-offset / direction calibration
        |
        v
Klipper coordinated motion
```

`inverse_side()` uses the exact closed-linkage geometry, not a serial two-link shoulder/elbow approximation. It uses a finite-difference Jacobian with a 0.01° step, converges below 0.01 mm tool error, allows up to 80 iterations, and rejects near-singular solves when `|det(J)| < 1e-8`.

See [the full kinematics derivation](../docs/kinematics/README.md).

## Run the regression tests

```bash
cd software/host
python3 -m unittest -v ../tests/test_kinematics.py
```

The development source currently passes the geometry/kinematics regression suite before publication.

## First-power checks

Before normal operation:

1. confirm Klipper reaches `ready`
2. verify every endstop changes state correctly
3. verify motor directions at low speed
4. home one mechanism at a time
5. confirm the linkage remains on the physical Assembly-1 branch
6. verify the measured X/Y joint-space boundary
7. calibrate physical home offsets and linear-rail travel
8. check cable clearance over the full workspace

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

