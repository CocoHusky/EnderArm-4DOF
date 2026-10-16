# Optional sensor head

The arm works without this board. Leave `sensors` and `sensor_swipe_programs`
false in your local feature file if you do not need it.

The FRDM-KL02Z provides a capacitive strip, RGB LED, and MMA8451Q three-axis
accelerometer. It does not have a gyroscope. Its OpenSDA USB connection goes to
the Linux host, separate from the robot motor controller.

![Sensor at the tool](images/sensors/sensor-head-front.jpg)

## Connect and check it

1. Build and flash using the [firmware guide](../firmware/frdm-kl02z/README.md).
   Leave the strip untouched at startup while it measures its baseline.
2. Install `pyserial` in the host's Python environment. Set `sensors: true` in
   your local `features.json` and `ENDERARM_SENSOR_PORT` to the board's OpenSDA
   serial port. Restart the UI service.
3. Open **Sensors**. Check LIVE, changing sequence numbers, and healthy sensor
   readings. Stationary acceleration magnitude should be around 1 g.
4. Leave automatic starts disabled. Enable **Gesture colors** under
   **Impact & Z protection**, then Apply, to test with colors only.

Do not open a second serial reader while the UI owns the port. The separate
[Sensor Lab](../firmware/frdm-kl02z/README.md#interactive-sensor-lab-macos) is for
testing without the arm UI running.

## Map a gesture

**Touch gestures** has five independent mappings:

| Gesture | Finger action |
| --- | --- |
| Press 1 · left | Briefly touch the left third, then lift |
| Press 2 · center | Briefly touch the middle third, then lift |
| Press 3 · right | Briefly touch the right third, then lift |
| Swipe left | Slide toward the left, then lift |
| Swipe right | Slide toward the right, then lift |

A press means touching the strip, not knocking the board. Keep it in one spot
and release within about a second. For a swipe, cross at least a third of the
strip. Recognition happens on release; check the last-gesture label first.

Choose a saved program and LED color for each row, then **Save mappings**.
Unassigned gestures do nothing. Test recognition before enabling motion.

With the arm homed, idle, and clear, **Enable gestures** permits program starts.
A gesture starts its assigned saved program after two seconds. Touching again
or choosing **Disable** cancels the countdown. Saving mappings, a fault, or a
reconnection also cancels it. Restart/reconnection requires re-enabling;
changing tabs does not disable it. Save route edits before using gestures.

## Tilt, impact, and orientation checks

Calibrate the Triplate while the arm is stationary in its normal mounted pose.
**Monitor tilt** and **KILL on confirmed fault** are separate switches; both
must be on before a sustained tilt sends KILL. A 15° limit with 0.75 seconds
of persistence means the reading has to stay over 15° for 0.75 seconds.

Tilt uses a short vector filter. Samples outside 0.75–1.25 g are ignored, so a
large bump is not treated as a gravity direction. The filter and persistence
reduce nuisance trips but do not make this a hardware safety system.

The **Lock-up recovery route** picker stores the name of a route created in
Programmed movements. It does not move the arm automatically. A KILL puts
Klipper into an emergency state, so it cannot also run a recovery route. Decide
on a separate earlier recovery threshold and test that route slowly before
adding automatic recovery behavior.

Start with LED feedback only. Each impact-protection checkbox is separate;
Apply commits the settings. These options reset OFF when the host restarts.

| Option | Meaning |
| --- | --- |
| Impact flash | Acceleration change between samples triggers red/off flashing at about 5 Hz |
| Impact KILL | The same trigger sends KILL; default change threshold is 1.1 g |
| Z color | Yellow when absolute board-axis Z acceleration reaches the threshold |
| Z KILL | KILL after the threshold persists; defaults are 0.3 g for 0.15 s |
| Gesture colors | Show mapped colors even with automatic program starts disabled |

Z acceleration in g is not a tilt angle. Robot acceleration can also change it.
For deviation from a mounted pose, use the separate **Triplate orientation**
calibration and tilt controls. Calibrate only while stationary in the normal
mounted position.

KILL requires controller recovery and rehoming. Acknowledge clears the sensor's
fault latch, not Klipper's emergency state. At roughly 20 Hz, this USB/software
system can miss a brief impact. It is not a hardware safety interlock and does
not guarantee damage prevention.

## Test status

Firmware flashing, live acceleration, and serial LED commands have been checked
on the reference build. Physical press-zone accuracy, swipe reliability, and
intentional impact-stop tests still need hands-on validation. Automated tests
cover logic, not those physical results.

[Controller details](controller.md#optional-sensor-and-swipe-programs) ·
[Firmware build](../firmware/frdm-kl02z/README.md) · [Back to the build](../README.md)
