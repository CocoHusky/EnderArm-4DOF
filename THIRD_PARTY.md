# Third-party sources

EnderArm combines original Ender-specific conversion work with a printable parallel-link robot-arm mechanism derived from earlier open designs.

## Mechanical arm source

The reference mechanical design and BOM used for this conversion are from [ToolKnox/Robotic-Arm-Arduino-project](https://github.com/ToolKnox/Robotic-Arm-Arduino-project), with the corresponding mechanical project hosted on Printables. Redistribution terms for unchanged upstream STL or CAD files must be verified before those files are committed here.

The project documentation should record:

- original author
- original project URL
- exact source revision or download
- original license
- files reused unchanged
- files modified for EnderArm

If the original license restricts commercial use, derivatives of those mechanical files must preserve the required terms.

## Firmware and software dependencies

Klipper and Moonraker remain separate upstream projects and are not redistributed as part of this repository. EnderArm contains only project-specific configuration, extensions, host software, and documentation needed to use them.

`software/klipper/toolhead_extra_axis_blend.patch` modifies upstream Klipper's
`klippy/toolhead.py` lookahead for non-extruder extra axes. It includes upstream
context and is distributed under Klipper's GPL-3.0 terms. Review the target
Klipper revision before applying it. Optional OpenCV/NumPy camera dependencies
and official Tailscale binaries are installed separately, not vendored here.

## Attribution rule

The FRDM-KL02Z build downloads Freescale KSDK 1.3 from
[`theatrus/ksdk`](https://github.com/theatrus/ksdk) at commit
`ac3682bd5fe41b65e4ff5b8bebd102a606adaf14`. Startup, board configuration,
peripheral drivers, and accelerometer support are compiled from that pinned
checkout. The build injects a modern Newlib compatibility include and replaces
the example application with EnderArm's sensor application. Preserve the SDK's
license notices and review its redistribution terms when distributing firmware
binaries. The SDK and Arm GNU toolchain are downloaded separately into ignored
`.deps/`; neither is vendored in this source repository. Pyserial is an optional,
separately installed host dependency for sensor-enabled installations.

Do not remove author names, license notices, or attribution from third-party source files. Modified files should identify the upstream source and the nature of the EnderArm modification.


## BOM source

The EnderArm BOM is adapted from the ToolKnox BOM revision dated 2026-04-30. The adapted BOM preserves the upstream quantities as a baseline and documents which items are reused from the Ender donor, replaced by Ender-specific printed parts, retained, or removed.
