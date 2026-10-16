# Current controller and optional capabilities

## Motion ownership

Browser → Python host → Moonraker → Klipper host planner → USB → ATmega1284P
step/endstop firmware → motor drivers. Linux performs geometry and planning;
the MCU executes timestamped step queues. Neither camera capture nor the browser
generates real-time motor pulses. ROS and LinuxCNC are not in this control path.

The Klipper extension handles independent-endstop concurrent side-joint homing,
base homing, optional rail homing, coordinated A/B/C/D targets and corner flow.
The tested reference homing seek is 30°/s. Its configured base release is 7.5°;
the extension is the source for the actual calibrated homing values.

## UI

- Control: joint jogging, arm-local tool targeting, live pose, Home, STOP, KILL.
- Workspace: world-coordinate slices, 3D model, measured raw boundary display.
- Programmed movements: capture/jog/manual poses, a page-wide coordinate switch,
  draggable 3D preview, editable/reorderable rows, speed, fixed travel duration,
  hold after arrival, corner flow, named load/save/create/delete.
- Debug: command timing, planner/MCU/network diagnostics and rolling traces.
- Settings: rail mode and endstop checks.
- Camera: separate floating window, draggable/resizable, full screen, available
  alongside controls. It remains separate from Debug.

Editing, loading and saving routes do not move hardware. Run submits current
editor values. Save overwrites the selected name after confirmation; Load
replaces the editor. Delete requires typing DELETE. A positive fixed travel time
overrides speed; blank travel means Auto. Hold begins after arrival. A hold
creates a stop; adjacent points without holds can blend.

Motor coordinates are A/B/C internally (Motor X/Y/Z θ in degrees); D is rail mm.
The Program tab retains its historical screen-facing X sign and converts that
X once at the API boundary. The backend world convention is positive yaw from
+Y toward +X. Do not negate coordinates again in the controller. Unchanged
Cartesian rows retain a verified joint hint because one tool position can have
multiple linkage solutions. Edited targets are solved sequentially in route order.

Boundary OFF disables the measured side-joint envelope and outward-tool policy.
Physical base travel, disabled rail, numerical validity and linkage reachability
still apply. A point failure must identify its waypoint; it must not silently
run a prior edited value. Workspace caches describe modeled reach, not obstacles.

With Boundary ON, every A/B segment is sampled against the measured envelope.
If a resolved world target is safe but its direct joint segment crosses a concave
part of that envelope, the host plans a short interior corridor and sends the
resulting blended joint moves instead. Boundary OFF keeps its direct Cartesian
behavior and does not use this corridor planner.

Tilt protection has one On/Off switch, a tilt limit, persistence, and a
Correct/KILL action. Correct stops producers, lets buffered motion settle,
then flashes red and recovers using Motor X/Y only. Motor Z and the rail stay
fixed throughout recovery. KILL shuts Klipper down and requires Home; it
cannot be followed directly by motion. If stopping or correction fails,
the controller is emergency-stopped and Home is required.

Correction follows the operator-recorded green side-view arc in
`software/host/recovery_path.py`, in that direction only, with no initial hop.
Each tool step is at most 2 mm. The host waits for settled, fresh accelerometer
readings after each step and stops as soon as tilt stays within the limit.
The path stays inside the measured boundary and above world Z −135.6 mm.
It refuses a starting position more than 5 mm from the recorded arc rather
than inventing a joining movement. Motor Z and rail remain fixed.

The separate KILL angle defaults to 50°. While protection is On, a valid
filtered reading at that limit emergency-stops even during correction or
after the ordinary tilt fault has latched. It does not wait for the ordinary
tilt persistence timer. Turning protection Off disables both tilt actions.

Recovery refuses to run without a valid home reference. A running route is
stopped before correction begins. Successful correction leaves the existing
home reference valid and normal motion available. If
it is stopped or errors after starting, the reference is treated as uncertain:
normal motion is locked until the operator explicitly presses Home, and
automatic homing cannot clear that lock. This is a modeled recovery path, not a
collision sensor or replacement for a hardware interlock.

## Configuration and local state

### Confirmed Right Beer boundary calibration

The owner confirmed saved Right Beer point 7 (Motor X 180.50°, Motor Y 78.34°,
Motor Z 99.09°, rail 0 mm) physically safe. `host/boundary_model.py` records its
X/Y coordinates and refits only the lower/right chain of the original envelope.
The upper line and first 60% of the rounded cap are unchanged. A local convex
continuation gives the confirmed pose 5.1° clearance, retaining the existing
5° inward yield. The intervening contour is a modeled continuation, not newly
measured physical samples. Existing raw measurements remain unchanged.

The UI, workspace sampler and Klipper all load this same model. The boundary
polygon participates in the workspace cache fingerprint, so the old map is
automatically invalidated. Regression checks cover all Right Beer segments,
agreement between both validators, and rejection of unrelated outside poses.
This is a geometry-only check; no automatic physical route test is performed.
Install `boundary_model.py` alongside `kinematics.py` when updating Klipper.

`features.json` is an installation setting, distinct from persistent rail usage.

| Feature | Missing-file default | Effect |
|---|---|---|
| camera | false | Enables the Camera control and streaming endpoints |
| remote_access | false | Supervises an installed userspace Tailscale daemon |
| diagnostics | true | Starts diagnostics sampling; Debug also offers OFF/BASIC/TRACE |
| rail | true | Allows the existing rail enable/disable Settings action |
| sensors | false | Shows Sensors and starts the optional FRDM-KL02Z reader |
| sensor_swipe_programs | false | Allows explicitly arming swipe-triggered saved programs |

Set `ENDERARM_DATA_DIR` for state storage (default `~/arm-control`) and optionally
`ENDERARM_CONFIG` for a different feature-file path. State includes
`linear_mode.json`, `boundary_mode.json`, `programmed_moves.json`, `programs/`,
`workspace_cache.json`, and the process lock. Keep it outside source control.
Boundary defaults ON on a fresh installation. No homing state is restored from disk.

## Optional sensor and swipe programs

The arm does not require a sensor. Both sensor feature flags default OFF; leave
them off for a sensor-free installation. No board is opened or sensor module
imported by the host when `sensors` is false. Installation-specific mappings in
`sensor_config.json` are ignored by Git, not shipped as everyone's defaults.

With firmware flashed and live readings visible:

1. Firmware 0.3.0 adds five independent mappings in Sensors: **Press 1 · left**,
   **Press 2 · center**, **Press 3 · right**, **Swipe left**, **Swipe right**.
   Each has its own saved program and LED color. Choose **Save mappings**.
   Existing tap slots migrate to the three presses; swipes start unassigned.
   An unassigned gesture does not run a program.
2. Check the strip first with automation disabled: slide a finger across at
   least 30% of its length in about 0.04–1.2 seconds, then lift. Activity should
   report the gesture. For a press, touch one third of the strip without sliding,
   then release within 1.2 seconds. Both are recognized on release; a swipe does
   not also count as a press. These are virtual zones, not mechanical buttons.
3. Set `sensor_swipe_programs: true` in the installation's `features.json` and
   restart the UI service. This only makes the Enable button available.
4. With the robot homed, idle, and clear, choose **Enable gestures** and confirm.
   Each gesture starts its assigned program after a two-second countdown, not
   the next/previous beer selection. Touching again or **Disable** cancels a
   pending start. Saving mappings also disables automatic starts.

Swipe control runs the saved program, not unsaved Program editor changes. Save
route edits first. A missing mapping or non-ready robot prevents execution.
Restart/reconnection disables automation; re-enable explicitly. Changing tabs
does not disable it. This is software control, not a hardware emergency stop.
Physically validate slider direction and response before arming robot motion.

Sensors → **Impact & Z protection** offers independent checkboxes for impact
KILL, impact red flashing, absolute-Z KILL, Z yellow feedback, and swipe colors.
Choose settings and Apply; KILL options require confirmation. All these options
reset OFF when the service restarts. Acknowledge clears the sensor latch, not
Klipper's emergency-stop state; recover the controller and rehome separately.
Impact is the vector change between 20 Hz samples (default 1.1 g), not a force
measurement. Z is absolute board-axis acceleration (default |Z| ≥ 0.3 g for
0.15 s), not degrees or calibrated tilt; acceleration during motion can also
trigger it. Impact feedback flashes red/off at about 5 Hz for 1.5 seconds and
takes priority over Z yellow. Gesture colors use each mapping's color; when
unarmed, enable the Gesture colors checkbox to test without motion. Automatic motion is canceled on
a detected fault. These sampled Linux/USB controls can miss brief impacts and
cannot guarantee damage prevention or replace a hardware emergency stop.

`ENDERARM_SENSOR_PORT` can override the OpenSDA serial path. The board uses an
MMA8451Q three-axis accelerometer; tilt calibration and KILL-on-fault are separate
opt-in controls. See [firmware instructions](../firmware/frdm-kl02z/README.md).

## USB camera

`arm_camera.py` selects `/dev/v4l/by-id/*-video-index0`, falling back to
`/dev/video0`. It shares one capture process across viewers, requests MJPEG
640×480 at 10 fps, encodes JPEG at quality 70, and stops capture after viewers
disconnect. Delivered frame rate depends on USB/network/CPU. The Linux account
must have camera-device access (normally the `video` group). Multiple cameras
are not configurable in this version. Close/reopen Camera to retry capture.

`/camera/stream` and `/camera/status` use the UI origin. `/controller-stream`
proxies live controller telemetry to localhost Moonraker with a same-origin
browser check, so no separate remote video or telemetry port is needed.

## Private remote access

Install official Linux Tailscale binaries under `$ENDERARM_DATA_DIR/tailscale/`
(`tailscale` and `tailscaled`), set `remote_access: true`, and restart the host.
The tested binary version is 1.104.1. Authorize interactively using:

```bash
~/arm-control/tailscale/tailscale --socket=~/arm-control/tailscale/tailscaled.sock up --hostname=ender-arm --accept-dns=false
```

Replace `~` after `--socket=` with your absolute home path (shells do not expand
that form consistently). The supervisor configures private HTTP Serve on port
8765 after authorization. Use its reported tailnet DNS name and port, with
Tailscale connected on the viewing device. The numeric IP can return a Serve
host-routing 404; use the DNS name. This does not enable public Funnel access.

The `arm-control.service` startup also starts remote access, so it does not depend
on an interactive login. Tailscale state, login links and daemon logs stay local.
Do not commit them, camera frames, private hostnames, Wi-Fi credentials or SSH keys.

## Validation record

The owner reported the pre-publication installation tested and working. During
camera/remote setup we verified live JPEG frames in a browser, Tailscale UI/state
requests, and WebSocket upgrade through the remote origin without commanding arm
motion. Repository CI checks syntax, geometry and optional-feature behavior; it
cannot establish physical motion correctness. The newly portable configuration
and CI deployment scripts still require an installation test on the controller.
The publication update also fixes the row-input listener selector: the old
`input[data-program-]` selector matched no inputs and could let telemetry replace
an edit. The listener now attaches to all inputs inside the route-table body.
