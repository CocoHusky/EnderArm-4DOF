# Documentation

The documentation is organized in the same order a new builder should follow.

## Build sequence

- `build/` — donor printer, disassembly, printed parts, and mechanical assembly
- `electronics/` — controller board, motors, endstops, power, and wiring
- `host/` — Raspberry Pi or other Debian/Linux host setup; minimum 2 GB RAM and 2 GB available storage
- `kinematics/` — linkage geometry, coordinate frames, forward and inverse kinematics
- `calibration/` — homing, zero offsets, workspace measurement, and validation
- `troubleshooting/` — mechanical, wiring, Klipper, and motion-control problems
- `images/` — required build photographs, screenshots, and validation videos

The build guide should be usable from start to finish without requiring knowledge of the development history of the project.
