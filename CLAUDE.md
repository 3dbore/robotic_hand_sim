# CLAUDE.md

ELIOS: a 5-DOF waste-sorting arm plus gripper. Python sim + PyQt5 HIL app, ESP32 firmware, camera calibration. Windows, Python 3.11+.

@agent.md

`agent.md` is the detailed guide to `hil/` (axis order, angle spaces, tank keep-out, settings, firmware sync). Read it before touching `hil/` or the firmware. The README covers the legacy toolkit and the serial protocols.

## Entry points

```
python -m backend.main        # webapp backend (FastAPI, port 8000); serves frontend/dist if built
npm --prefix frontend run dev # webapp frontend dev server (Vite, port 5173, proxies /api and /ws to 8000)
npm --prefix frontend run build  # tsc -b && vite build -> frontend/dist
python arm-hil-toolkit.py     # PyQt HIL app (hil/), the reference behaviour for the webapp
python arm-sim-end.py         # standalone IK simulator (also imported as a library by hil/sim.py and hil_toolkit/sim_loader.py)
python gripper/gripper_sim.py # gripper linkage sim
pip install -r requirements.txt
```

## Which package?

- `hil/` — PyQt app and the hardware core (config, kinematics, motion, link), for `slave_code/6-axis-esp32-twoway-tmc2208/`. Default target for hardware/logic changes.
- `backend/` + `frontend/` — the webapp, a browser UI for the same system. See "Webapp" below.
- `hil_toolkit/` — legacy app (G-code, graphs, AprilTag calibration, two-way mode). Don't mix the two.
- `arm-hil-toolkit-2.py` is an empty placeholder.

## Rules

- No test suite. Verify with offscreen scripts kept outside the repo (see agent.md "Testing"). On this machine you also need `QT_QPA_FONTDIR=C:/Windows/Fonts`, a `QWidget` stub for `QtInteractor` (and its `.interactor`), and `app.processEvents()` before `widget.grab()`.
- Never take full-desktop screenshots; use `widget.grab()` only.
- `calibration/*.json` (settings, offsets, directions, sequences) is the user's real per-machine data. Back it up or avoid saving; never overwrite with defaults.
- A change to the serial frame, limits, speeds or field order must be made in both Python and the `.ino` (the user re-flashes).
- Don't commit unless asked; the tree often has the user's uncommitted work.
- Use `PYTHONIOENCODING=utf-8` when printing `°` or `→`.

## Webapp (`backend/`, `frontend/`)

- `backend/` imports `hil` (config, kinematics, motion, sequence) and adds no hardware logic of its own. `server.py` is the FastAPI app: WS `/ws` takes JSON `{action, ...}` and pushes telemetry; failures come back as `{"type": "error", "message"}`. REST under `/api/` serves the STL parts, workspace masks, cameras, sequences and settings. `robot.py` (`RobotController`) mirrors `hil/main_window.py`: every move goes through `_request_pose`, which runs `kinematics.plan`. Never call `motion.go` directly. `link_manager.py` is the serial link, `camera_manager.py` the MaixCam2 / sim camera, `scene_pose.py` the 3D part poses.
- `frontend/` is Vite + React 19 + TypeScript + three.js (lucide-react icons). `api.ts` is the WS/REST client (`send` silently drops while the WS is closed); `state.ts` has the telemetry, prefs and `useLastError` hooks; `scene3d.ts` / `View3D.tsx` draw the real STL arm, tank box and EE/target markers; `components/` are the Telemetry-page panels, `pages/` the Sequence and Settings pages.
- UI follows the mockups in `assets/` and the tokens in `DESIGN.md`, laid out for 1366×768. No gradients, glows or emoji; status and refusal text is plain and short.
- Behaviour should match the PyQt app: e.g. clicking a Control workspace view moves straight away (as `move_to`), and refusals are shown to the user.
- `POST /api/settings` writes `calibration/hil_settings.json`. Back it up before testing settings.
- Verify in the browser pane (dev server on 5173 + backend on 8000, both in `.claude/launch.json`), and run `npm run build` to type-check. `test_integration.py` exercises the REST and WS API with Starlette's `TestClient`.
- Not built yet: sequence "wait for tag", SSH toggle, window buttons, editable limits/speed/jump threshold, baud other than 9600. Not tested on real hardware.
