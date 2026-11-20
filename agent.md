# agent.md — context for coding agents

ELIOS is a 5-DOF waste-sorting robot arm plus a gripper. This repo holds:

- the 3D simulator, `arm-sim-end.py`;
- an older HIL toolkit, `arm-hil-toolkit.py` with `hil_toolkit/`;
- camera calibration, `calibration.py`;
- the ESP32 firmware, `slave_code/`.

**Most active work is in the `hil/` package.** It is the new hardware-in-the-loop app for the firmware in `slave_code/6-axis-esp32-twoway-tmc2208/`. Read this file before changing `hil/` or that firmware.

## Run

```
pip install -r requirements.txt
python arm-hil-toolkit.py        # launches hil.app.main()
```

- The app opens full screen. It is laid out for a 1366×768 screen, and a compact stylesheet kicks in automatically on small screens.
- `arm-hil-toolkit-2.py` is an empty placeholder.
- The legacy `hil_toolkit/` package is the older app, with G-code, graphs and calibration. Don't mix it up with `hil/`.

## `hil/` package map

| File | Role |
|---|---|
| `app.py` | Entry point. `setup()` loads the settings and joint limits; call it before building `MainWindow`. |
| `config.py` | **Single source of truth for the hardware**: the `Axis` table, home, servo range, speed, trim, invert, wire clamp, gripper open/closed, the tank height and timing constants. Settings load and save here too. |
| `sim.py` | Imports `arm-sim-end.py` as a library as `ase`, with its `pyvista.Plotter` calls stubbed out. |
| `kinematics.py` | Closed-form FK/IK, the tank keep-out (`in_tank`, `unsafe`, `path_unsafe`), the move planner `plan(a, b)` and the `Workspace` raster masks for the 2D views. |
| `motion.py` | Joint-space smoothstep moves, timed to `SPEED_MARGIN` × each axis's firmware speed so all axes arrive together. It holds a queue of path legs (`go_path`, `final_goal`). State is in **servo degrees** for all 6 axes. |
| `link.py` | Serial link at 9600 baud: frame building with CRC-8, and parsing of the board's `STAT` lines. A reader thread emits Qt signals. |
| `main_window.py` | The UI: connection bar, 3D view, and the Joints / Go to point / Serial log tabs, plus the telemetry strip. **Every move goes through `_request()` / `_set_axis()`**, which call the tank-aware planner. |
| `sequence.py` | Saved pose sequences (`Step`, `Sequence`), loaded from and saved to `calibration/hil_sequences.json`. Poses are in servo degrees. Playback lives in `MainWindow.run_sequence` and goes through `_request()`. |
| `timeline.py` | The Sequences tab's timeline strip: one block per step (move + hold), drag to reorder, drag the right edge to set the hold, double-click to go there. |
| `widgets.py` | `JointRow` (slider, jog, INV, trim), the telemetry cards, link health, and the clickable top and side workspace views. |
| `scene.py` | The pyvista 3D view: the arm meshes, the EE and target markers, and the red tank keep-out box. |
| `theme.py` | Design tokens and Qt stylesheets (ELIO design system, `docs/DESIGN.md`). Read `theme.COMPACT` as an attribute; never `from .theme import COMPACT`. |

## Core conventions

### Axis order and serial frame

`config.AXES` is in firmware field order: F0 Gripper, F1 Wrist Yaw, F2 Wrist Pitch, F3 Arm B, F4 Arm A, F5 Base. `DISPLAY_ORDER` is only for the UI.

```
PC → board:  "F0,F1,F2,F3,F4,F5,CC\n"
```

- Each field is in degrees with at most one decimal. An empty field releases that servo.
- `CC` is the CRC-8 (poly 0x07, init 0) of everything before the last comma, written as 2 hex digits.
- The board replies with `STAT rx=.. bad=.. i2c=.. fix=.. reinit=.. spike=.. out=a,b,c,d,e,f` once a second.

### Angle spaces

Three angle spaces are involved:

1. **Sim angle:** the joint angle in `arm-sim-end.py`. Sim joints are 0..4: base, Arm A, Arm B, pitch, yaw. The gripper has no sim joint (`joint=None`).
2. **Servo angle:** `servo = home + sim`. There is no scaling and no other offset. The sim's all-zero pose equals each axis's `home`.
3. **Wire angle:** what is actually sent. `Axis.wire()` computes it in three steps:
   1. Add the trim: `v = servo + trim`.
   2. If the axis is inverted: `v = 180 - v`.
   3. Clamp to `wire_lo..wire_hi`.

Limits always live in servo or sim space: each limit is the sim limit intersected with the servo range. Trim and invert touch **only** the wire value, so they can never lock an axis.

### Hardware limits

- **MG996R servos** obey roughly -15..195° (about 330–2670 µs). Past that they go limp.
  - This is why `TRIM_MAX = 15` and the wire clamp is `WIRE_MIN..WIRE_MAX`.
  - If more correction is needed, move the servo horn by one spline tooth (about 14°) instead.
- **Arm Base** is a NEMA17 on a TMC2208 driver with an 80T→120T gear (ratio 1.5).
  - It takes servo/wire angles of -90..270, with home 90. That is ±180° from home and **never continuous**, because the cable would tangle.
  - There is no homing switch, so the base must be at home when the board powers up.
- **Gripper** is open at 180 and closed at `GRIP_CLOSED` = 75, plus `grip_closed_offset` (±`GRIP_OFFSET_MAX`). That offset shifts the closed position only. Its servo range is 60..180.
- `SIM_LIMIT_OVERRIDES` replaces the sim's limits for the base (±180) and the yaw (±90).

### Tank keep-out (safety)

- A tank sits behind the base. A world point is unsafe when Y > 0 (behind) **and** Z < `TANK_Z_MM` (150 mm).
- `kinematics.unsafe()` checks the elbow (J3), the EE (J4) and the wrist (J5). It does not check the gripper tip.
- `plan(a, b)` returns a safe path, trying these in order:
  1. a direct move;
  2. arm first, then base;
  3. base first, then arm.
  If none is safe, it returns `(None, reason)`. The UI then refuses the move and snaps the slider back.
- At the home pose the EE is at Z 90, so **the base cannot pass ±90° until the arm is raised**.
- The 2D workspace shading checks only the EE, so a point shaded as reachable can still be refused.
- **Never bypass `_request` / `_set_axis`** by calling `motion.go` or `motion.set_axis` directly from the UI.

### Kinematics reference

- `fk(angles)` returns the EE position (J4). `joint_points(angles)` returns J1..J5.
- At home: J3 = (0, -42.5, 230), EE = (0, -42.5, 90), J5 = (0, -67, 90).
- The base's forward direction is (sin θ1, -cos θ1). Forward from home is -Y, so the tank is at +Y.
- The maths is in `docs/kinematics.md`, `docs/calculation.md` and the `kinematics.py` docstring.

## Settings

`calibration/hil_settings.json` holds the user's real calibration:

- `inverted` and `trim`, per axis;
- `grip_closed_offset`;
- `port`.

The app writes it on every change. **Tests must back it up and restore it**, or set the axes in memory without calling `_save()`. Don't overwrite it with defaults.

## Firmware: `slave_code/6-axis-esp32-twoway-tmc2208/`

- An ESP32 drives a PCA9685 for the servos and a TMC2208 for the base stepper.
- It runs in stream mode, checks the CRC, and rate-limits each axis with its speed limit.
- `BASE_ANGLE_MIN` / `BASE_ANGLE_MAX` (-90..270) must match the base axis in `config.py`. So must each speed limit and the field order.
- **Any change to the frame format, limits or field order must be made in both `hil/` and the `.ino`.** The user then has to re-flash the board.

## Testing

- There is no test suite. Verify with offscreen scripts kept outside the repo:
  - set `QT_QPA_PLATFORM=offscreen` and `QT_QPA_FONTDIR=C:/Windows/Fonts`;
  - stub `hil.scene.QtInteractor` with a plain `QWidget`, because VTK can't render offscreen;
  - build `MainWindow`, drive `_tick()` in a loop, and save `widget.grab()` to a PNG to check the layout at 1366×768.
- **Don't take full-desktop screenshots.**
- On Windows, set `PYTHONIOENCODING=utf-8` when printing non-ASCII text such as `°` or `→`.
- Useful checks:
  - `kinematics.solve` never returns an `unsafe` pose;
  - every leg of `plan()` passes `path_unsafe(step=0.5)`;
  - frames from `link.make_frame` have the right CRC.

## Working agreements

- Keep changes small and in the existing style: plain functions, dataclasses, and module docstrings that explain the *why*.
- Don't commit unless asked. The working tree often holds uncommitted work from the user; don't discard it.
- The user-facing text in the UI is plain and short. Refusals say what would have gone wrong, e.g. "would hit the tank (behind the base, below Z 150 mm)".
