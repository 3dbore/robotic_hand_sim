![ELIOS Robotic Hand Simulation Banner](paper/img/github_banner.png)

# ELIOS — 5-DOF Robotic Arm Simulation & HIL Toolkit

A 3D kinematic simulator and hardware-in-the-loop (HIL) toolkit for **ELIOS**, a
5-degree-of-freedom robotic arm built for waste sorting. The simulator renders
the arm's STL geometry in real time, solves inverse kinematics for end-effector
control, and mirrors every pose to a real Arduino-driven arm over serial —
with a simulated-camera calibration pipeline to match.

## Features

- **3D arm simulation** (`arm-sim-end.py`) — PyVista/VTK render of the 5-DOF
  arm from STL parts, driven by game-like translation/rotation controls.
  Inverse kinematics (Jacobian pseudoinverse) solves joint angles from a
  target end-effector pose; live panels show end-effector position and joint
  angles.
- **Hardware-in-the-loop toolkit** (`arm-hil-toolkit.py`, `hil_toolkit/`) — a
  PyQt5 desktop app that runs the simulator and streams each pose to an
  Arduino Uno over serial, plus a G-code interpreter/runner, joint graphs, and
  a calibration workflow, all built on the same sim.
- **Camera calibration** (`calibration.py`) — closed-form intrinsic/extrinsic
  camera calibration (Spong, Hutchinson & Vidyasagar, *Robot Modeling and
  Control*, Appendix E), camera-agnostic — consumes correspondences from
  either the simulated camera view or a real camera. The startup procedure
  runs a real-arm-safe G-code that shows the hand's AprilTag to the camera and saves
  a ground-truth file to check the real run against.
- **G-code interpreter** (`hil_toolkit/gcode.py`) — drives the wrist through
  linear/arc moves and gripper commands, in sim and on hardware.

## Repository layout

```
arm-sim-end.py          Standalone 3D simulator (IK end-effector control)
arm-hil-toolkit.py       HIL app entry point (sim + serial + calibration)
calibration.py           Camera calibration solver (book-derived, sim-agnostic)
hil_toolkit/             HIL application package
  app.py                   QApplication wiring / entry point
  main_window.py           Main application window
  sim_loader.py            Loads arm-sim-end.py as a library (headless)
  hardware_mapping.py      Sim -> servo mapping, effective joint limits
  calibration_camera.py    Simulated-camera correspondence generator
  tag_calibration.py       Startup calibration: gripper AprilTag + G-code -> ground truth
  serial_link.py           pyserial line-protocol wrapper
  gcode.py                 G-code interpreter + timed runner
  dialogs.py               Joint-graph and calibration dialogs
  widgets.py                Reusable Qt/VTK widgets
  theme.py                 Design tokens & Qt stylesheets
  paths.py                 Filesystem locations
  tags_toolkit/            AprilTags on the arm
    apriltag.py              Detection + pose (standalone webcam test tool)
    tag_model.py             Mounted tags: link, home pose, size -> ground-truth pose
    sim_tags.py              Textured tags in the simulated views/camera
arms/                     STL geometry for each arm segment, camera, FOV cone
slave_code/               Arduino firmware (5-axis and 6-axis variants)
calibration/              Camera model, correspondences, AprilTag ground truth (generated)
docs/                      Kinematics, calibration, and design reference docs
paper/                    Typst write-up
arm-sim-mapping-calculation.ipynb   Kinematics derivation notebook
Kinematics.xlsx           Sim <-> servo axis mapping reference
```

## Setup

Requires Python 3.11+ (developed on 3.13).

```bash
pip install -r requirements.txt
```

## Usage

**Simulator only** (no hardware needed):

```bash
python arm-sim-end.py
```

**Full HIL toolkit** (simulator + Arduino over serial + calibration):

```bash
python arm-hil-toolkit.py
```

Flash the matching firmware from `slave_code/` first: `6-axis-esp32/6-axis-esp32.ino`
(ESP32 + PCA9685), or `6-axis/6-axis.ino` (Arduino Uno, which also needs
`COMMAND_RESOLUTION_DEG = 1.0` in `hil_toolkit/hardware_mapping.py`). The serial
protocol is a single line of six comma-separated values plus a checksum:

```
F0,F1,F2,F3,F4,F5,CC\n
```

mapping to Gripper, Wrist Yaw, Wrist Pitch, Arm B, Arm A, and Base
respectively, in degrees with at most one decimal. 0–180 is the standard
500–2500 µs; the ESP32 firmware accepts −36–216 (100–2900 µs) so an axis offset
can carry a range past that, while the Uno firmware only takes whole degrees
0–180. An empty field releases that servo; every frame carries all six values.
`CC` is the CRC-8 (polynomial 0x07) of everything before the last comma, as two
hex digits. The ESP32 firmware drops any frame that fails it; the Uno firmware
ignores it. The sim-to-servo
mapping is derived from `Kinematics.xlsx` — see `hil_toolkit/hardware_mapping.py`.

The ESP32 firmware reports back with `BOOT reason=...` at startup and, once a
second, `STAT rx=.. bad=.. i2c=.. fix=.. reinit=.. out=...`. The toolkit logs
any rise in those counters and stops streaming if the board reboots.

**Matching the real initial pose to the sim:** connect, enable *Stream to Arm*,
then open *Axis Offsets*. Jog each axis by the amount you choose until the real
arm matches the sim's initial pose, and press *Save*. Offsets are stored per
machine in `calibration/axis_offsets.json` and loaded at startup.

An offset has no limit and moves the axis's whole operating range with it: a joint
that runs ±90° from a home of 90° runs ±90° from 95° after a +5° offset, so it sends
5–185°. Commands still stop at the firmware's −36–216° pulse limit, and the window
says when a range passes 0–180° or that limit. Past 0–180° only the servo's own stop
limits it, so back off if it buzzes at the end of its travel.

## Documentation

- [`docs/kinematics.md`](docs/kinematics.md) — joint table, link offsets, and
  coordinate conventions.
- [`docs/calculation.md`](docs/calculation.md) /
  [`docs/calculation_indonesia.md`](docs/calculation_indonesia.md) — full
  kinematics derivation (EN/ID).
- [`docs/calibration_sequence.md`](docs/calibration_sequence.md) — the sim → real
  AprilTag calibration workflow (read this first if you are an AI agent).
- [`docs/camera_calibration.md`](docs/camera_calibration.md) — the closed-form
  calibration method implemented in `calibration.py`.
- [`docs/DESIGN.md`](docs/DESIGN.md) — visual design system for the ELIO
  product UI.
- [`arm-sim-mapping-calculation.ipynb`](arm-sim-mapping-calculation.ipynb) —
  worked notebook deriving the joint/link geometry from the raw STL datums.

## Credits

The mechanical arm design (`arms/*.stl`) is by **Emre Kalem** (Makerworld) and
remains subject to its original license on Makerworld — it is not covered by
this repository's MIT license. Kinematics, simulation, calibration, and HIL
tooling by **Reza Fauzan Zulkarnaen**.

## License

MIT — see [LICENSE](LICENSE). The STL arm design is excluded; see Credits
above.
