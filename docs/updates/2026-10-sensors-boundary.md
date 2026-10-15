# Sensor gestures and calibrated boundary update

## Included changes

- Optional FRDM-KL02Z firmware and a Sensors tab alongside the existing robot UI.
  The separate floating camera window is unchanged.
- Real MMA8451Q three-axis acceleration in g, magnitude, freshness, sensor health,
  orientation/movement diagnostics, and stationary tilt calibration.
- Five independent touch mappings: Press 1 (left third), Press 2 (center), Press 3
  (right third), Swipe left, Swipe right. Each chooses a saved program and LED
  color. Press means touching the capacitive strip, not knocking the board.
- Recognition on finger release; a two-second countdown before a mapped saved
  program runs. Another touch, Disable, a fault, or saving new mappings cancels
  the pending start. Missing mappings do not execute anything. Programs must be
  saved first; gesture control does not execute unsaved route-editor drafts.
- Explicit enable and homed/idle checks. Startup and serial reconnection disable
  automatic starts; switching tabs does not. Repeated telemetry event IDs are
  consumed once, and an old event is not replayed on connection.
- Independent impact KILL, impact red flashing, absolute-Z KILL, yellow Z-limit
  feedback, and mapped gesture-color checkboxes. Settings use Apply and reset
  OFF on service restart. Defaults: impact sample change 1.1 g, absolute Z 0.3 g
  for 0.15 s; red/off feedback approximately 5 Hz for 1.5 seconds.
- Local-only standalone Sensor Lab for board diagnostics without robot imports.
- Corrected touch-pin pull-ups and retained summed measurement precision.
  Firmware 0.3.0 includes interrupt-buffered serial commands and persistent
  gesture events. Idle raw values increased from about 2/2 to about 194/174;
  this confirms a measurement-range change, not positional accuracy.
- Mapping controls retain unsaved dropdown edits instead of rebuilding on every
  telemetry update. Gesture labels distinguish press zones from beer taps.

## Optional installation

Both `sensors` and `sensor_swipe_programs` default false in the example feature
configuration. With sensors disabled, the main host does not import the sensor
service or open a sensor device. No sensor hardware or pyserial is required.
Camera, rail, remote access, and diagnostics retain independent configuration.

For a sensor installation, install pyserial in the host environment, flash the
board using the [firmware guide](../../firmware/frdm-kl02z/README.md), set
`sensors: true`, and configure `ENDERARM_SENSOR_PORT` for its OpenSDA serial
port. Leave the separate robot motor-controller USB device alone. Enabling
`sensor_swipe_programs` only makes the runtime Enable gestures action available;
it does not arm robot motion automatically.

Calibration, feature files, saved programs, generated firmware, SDK/toolchain
downloads, serial logs and Tailscale state are excluded from source control.
Individual beer program files and this installation's program assignments are
not published. The main UI serves telemetry on its existing origin, including
through the existing private Tailscale connection; no public video/sensor port
or new cloud service is introduced.

## Confirmed boundary recalculation

The owner explicitly confirmed the saved Right Beer point 7 physically safe:
Motor X 180.50°, Motor Y 78.34°, Motor Z 99.09°, rail 0 mm. Its original signed
X/Y clearance was about -1.689° (outside). The recalculated lower/right contour
places it 5.1° inside while retaining the existing 5° inward yield. The upper
line and first 60% of the rounded cap are unchanged. This is a modeled local
continuation around the confirmed point, not a claim that surrounding poses
have all been physically measured.

`boundary_model.py` is now the source for the host, workspace sampling, plotted
contour, and Klipper boundary validation. The polygon is part of the workspace
cache fingerprint, so the old cached geometry is invalidated automatically.
Raw measurements, saved route coordinates, and global boundary protection were
not disabled or rewritten to make the route pass.

## Verification and limits

- 47 hardware-free tests pass, plus Python and embedded JavaScript syntax checks.
- Firmware 0.3.0 compiled and flashed through Linux OpenSDA; live frames confirmed
  firmware identity and healthy accelerometer/slider status in the remote UI.
- Serial LED color commands and five mapping controls were checked. Actual
  press-zone accuracy and swipe direction still require hands-on validation.
- All saved Right Beer X/Y segments pass both production boundary validators in
  regression tests; unrelated outside poses remain rejected. No physical route
  execution or intentional impact/KILL test was performed for this update.
- The new boundary and host were deployed. Reloading Klipper cleared homing;
  rehome before testing. Boundary ON/OFF remains a local operator choice.

The accelerometer has three axes, not a gyro or nine-axis orientation system.
Absolute Z in g is **not** a tilt angle; normal robot acceleration can also
trigger it. At about 20 Hz, brief impacts can be missed. Linux/USB timing and
software KILL cannot guarantee damage prevention or replace a hardware safety
interlock. KILL requires controller recovery and rehoming. Calibrate and test
LED-only feedback before deliberately enabling automatic stops or gestures.

## Deployment notes

Install `boundary_model.py` beside `kinematics.py` before updating the Klipper
extra. Updating only one side risks differing host/controller boundaries.
Klipper updates and sensor flashing are deliberate maintenance operations, not
part of automatic CI deployment. The existing manual host deployment workflow
does not flash firmware, change private state, home the arm, or run programs.
The standalone Sensor Lab also needs `sensor_lab.html` alongside its Python
entry point; it is separate from the main robot UI.
