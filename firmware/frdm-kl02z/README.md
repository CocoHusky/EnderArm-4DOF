# FRDM-KL02Z sensor firmware

Target is **FRDM-KL02Z / MKL02Z32VFM4**, not KL25Z.

This firmware uses the actual board peripherals:

- MMA8451Q 3-axis accelerometer on I2C0, address `0x1d` (PTB3 SCL / PTB4 SDA).
- GPIO capacitive slider on PTA13 and PTB12. The KL02Z does not have the KL25Z TSI peripheral.
- Active-low RGB LED on PTB6 red / PTB7 green / PTB10 blue.
- UART0 at 115200 over the board's OpenSDA virtual serial interface (PTB1/PTB2).

## Reproducible macOS build

The Homebrew `arm-none-eabi-gcc` formula installed on this Intel Mac contains the
compiler but not the Newlib headers required by KSDK 1.3. Use the complete
official Arm GNU Embedded package instead. The setup script downloads that
package through Homebrew, extracts it under ignored `.deps/`, and does not
require a system install or `sudo`.

```sh
cd /path/to/EnderArm-4DOF
./firmware/frdm-kl02z/setup_toolchain_macos.sh
./firmware/frdm-kl02z/build.sh
```

The build script pins Freescale KSDK 1.3 source to commit
`ac3682bd5fe41b65e4ff5b8bebd102a606adaf14` in ignored `.deps/ksdk`,
builds the KL02 platform library from that exact checkout, then builds this
firmware with Freescale's FRDM-KL02Z startup, clock, UART, I2C, and MMA8451Q
support. A small compile-time compatibility include is added for modern Newlib;
the pinned SDK source itself is not committed or vendored.

Generated outputs are ignored:

```
firmware/frdm-kl02z/build/enderarm_sensor.elf
firmware/frdm-kl02z/build/enderarm_sensor.hex
firmware/frdm-kl02z/build/enderarm_sensor.bin
```

The verified 0.2.1 release build on this Mac is 11,836 bytes of text, 48 bytes
of initialized data, and 1,752 bytes of BSS.

## Serial protocol

Version 0.4.0 uses a 10 ms acquisition target at the same 115200 baud. Full
status frames are sent every 100 ms; intervening acceleration packets are:

```json
{"a":[12,-8,1004],"s":43,"t":12355,"h":1}
```

`a` is acceleration in milli-g, `s` is the shared sequence counter, `t` is
device uptime in milliseconds, and `h` is accelerometer health (0 or 1).
The host requires a full frame first and retains its slider/LED metadata until
the next full frame. Touch scanning and LED feedback keep their 50 ms period.
Blocking UART transmission limits the delivered rate: about 79 samples/s was
measured on the reference board after flashing 0.4.0. The live graph buffers
all received samples, including those received between browser updates.

Version 0.3.0 adds `slider.gesture` (`none`, `press_1`, `press_2`, `press_3`,
`swipe_left`, `swipe_right`) and a persistent `slider.gesture_id` counter.
One release emits either a press or swipe, never both. Presses must remain
within 15% of the strip and use the average touched position to choose thirds;
swipes require at least 30% end-to-end travel. Valid contact lasts 40–1200 ms.
The last event remains in telemetry so the host can consume its counter once.
The host ignores an existing event on initial connection/reconnection. Keep the
strip untouched during boot baseline calibration. Physical zone and direction
recognition still require testing on the actual board.
The 0.3.0 build was flashed successfully through OpenSDA on Linux; live UI
frames confirmed version 0.3.0, gesture counter 0, and healthy sensor status.
The five mapping controls were verified in the remote UI without starting robot
motion. Physical gesture recognition remains a hands-on test.

Version 0.2.2 removes the internal touch-pin pull-ups: the board already has
510 kΩ charging resistors, and the internal pull-ups compressed measurements
to roughly two counts. Repeated readings are summed rather than truncated to
an integer average. Version 0.2.2 was successfully flashed on Linux after
remounting the reconnected programming drive; OpenSDA reported Completed and
live serial frames confirmed 0.2.2 with healthy accelerometer/slider status.
Untouched electrode readings were approximately 194/174 instead of 2/2.
Physical swipe response still needs hands-on verification; the increased raw
range alone does not establish positional accuracy.

Full frame format (0.3.0 sent these at 20 Hz):

```json
{"v":1,"device":"FRDM-KL02Z","fw":"0.1.0","seq":42,"ms":12345,"accel_mg":[12,-8,1004],"slider":{"raw_left":4,"raw_right":5,"position":500,"active":false,"swipe":"none"},"led":"off","health":{"accel":true,"slider":true,"whoami":26}}
```

- `seq`: monotonically increasing sample sequence.
- `ms`: device uptime in milliseconds.
- `accel_mg`: signed X/Y/Z acceleration in milli-g.
- `slider.position`: 0..1000, left to right.
- `slider.swipe`: `none`, `left`, or `right`.
- `health.whoami`: MMA8451Q WHO_AM_I; expected decimal 26 / hex 0x1a.

Freescale's reference KL02 demo configures the MMA8451Q for +/-4 g, so the
conversion is 2048 counts/g.

Commands are ASCII lines. LED state is reported in the next normal sensor frame:

```
LED OFF
LED RED
LED GREEN
LED BLUE
LED YELLOW
LED CYAN
LED MAGENTA
LED WHITE
```

No non-JSON acknowledgement lines are emitted, so every received line remains
a sensor-frame candidate.

## Programming this connected board

Identify your own board's P&E OpenSDA USB CDC serial port and the programming
drive labeled `FRDM-KL02Z`. Serial identifiers differ between boards; no
installation-specific serial identifier is included in the public examples.

The legacy OpenSDA FAT16 mass-storage endpoint currently fails to mount on this
Mac. Do **not** replace/update OpenSDA or raw-write its disk merely to work
around that.

Preferred programming is SWD through the existing P&E OpenSDA interface using
P&E's GDB Server for ARM devices, which supports OpenSDA on macOS. Program the
KL02 target with `build/enderarm_sensor.elf`; do not program the OpenSDA
interface MCU itself.

The robot motor controller is a separate CH340 device. Never use a
`/dev/ttyUSB*`, `/dev/cu.wchusbserial*`, or CH340 port for this sensor
firmware.

## Verification after flashing

### Linux verification (2026-10-15)

Firmware 0.2.1 was successfully copied through the existing OpenSDA programming
drive on Linux, without formatting the drive or replacing OpenSDA. The sensor
uses its installation-specific OpenSDA serial path. Set `ENDERARM_SENSOR_PORT`
to that path (the default is a generic alias that must be configured locally):

```
export ENDERARM_SENSOR_PORT=/dev/serial/by-id/YOUR_OPENSDA_DEVICE
```

Live 20 Hz frames, MMA8451Q identity (`26`), acceleration readings, and reported
red/green/blue/off command responses were verified. Version 0.2.1 uses
interrupt-buffered UART reception to prevent command bytes being lost during
sensor sampling and telemetry. Physical touch/swipe response and LED appearance
still require hands-on validation.

The main arm UI now exposes these readings and LED controls in its Sensors tab.
Tilt monitoring, KILL-on-fault, and swipe-triggered programs remain disabled
during initial testing. Do not open a second serial reader while the UI service
owns this port.

### Hands-on checks

Read only the P&E OpenSDA CDC port at 115200. A healthy stationary board should
produce JSON lines with:

- `device == "FRDM-KL02Z"`
- changing `seq` and `ms`
- `health.accel == true`
- `health.whoami == 26`
- acceleration magnitude close to 1 g while stationary
- slider raw values/position changing when touched

Then send `LED RED\n`, `LED GREEN\n`, and `LED BLUE\n` and verify the
physical LED plus the reported `led` field.

Do not enable tilt-triggered KILL or swipe-triggered programs until those real
sensor checks pass.

## Interactive Sensor Lab (macOS)

After flashing firmware 0.2.0, connect the FRDM-KL02Z's own OpenSDA USB port
to the Mac and run:

```sh
cd /path/to/EnderArm-4DOF
python3 -m pip install pyserial  # only if not already installed
PYTHONPATH=software/host python3 software/host/sensor_lab.py \
  --port /dev/cu.usbmodemYOUR_OPENSDA_DEVICE
```

Open <http://127.0.0.1:8766/> in your Mac browser. The server binds to
127.0.0.1 only, never opens the robot's CH340, and offers no motor commands.

Firmware commands (ASCII lines at 115200):
`MODE MANUAL`, `MODE ACCELERATION`, `MODE ORIENTATION`,
`MODE SLIDER`, `MODE TAP`, `MODE MOTION`.
The manual LED commands automatically switch to MANUAL mode.

0.2.0 frames add `mode` and `events` fields with `tap`, `motion`, and
`orientation`. These are *software-derived diagnostic events*, not yet the
MMA8451Q hardware interrupt tap/free-fall functions. The LED uses fixed colors
(no PWM). Acceleration mode shows the dominant absolute axis, orientation mode
shows which signed axis is most aligned with gravity, slider mode shows
red/green/blue by position when touched, tap mode flashes white on a sampled
impulse, and motion mode shows green stationary/yellow moving/magenta near
free-fall. All thresholds require physical testing.

The standalone dashboard intentionally displays no simulated values. If serial
samples are absent, bad, disconnected, or stale, its live readings are blank.
The program must be flashed before the dashboard can show values. Avoid using
the sensor for automatic KILL or robot motion until physically validated.
