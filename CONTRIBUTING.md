# Contributing

Contributions that improve reproducibility, safety, documentation, hardware compatibility, calibration, or motion control are welcome.

## Before opening a pull request

- Describe the Ender model and controller-board revision used for testing.
- State whether the change was tested on physical hardware.
- Include measurements for geometry or calibration changes.
- Include before/after results for motion-control changes when applicable.
- Add or update tests for kinematic changes.
- Update the build documentation when a mechanical part changes.
- Add photographs when an assembly step would otherwise be ambiguous.
- Preserve upstream attribution on third-party mechanical files.

Do not commit credentials, private network information, generated cache files, build artifacts, or machine-specific temporary files.

## Mechanical changes

For modified printed parts, include:

- STL for printing
- STEP when available
- a short description of what changed
- affected fasteners/bearings
- compatibility notes
- source attribution and license

## Software changes

Keep kinematic definitions centralized rather than duplicating geometry constants across the UI and motion-control layers.

Changes that can move hardware should include boundary checks and a clear validation procedure.
