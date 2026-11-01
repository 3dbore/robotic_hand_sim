# Calibration sequence — sim ground truth → real arm

Guide for humans and AI agents. Read this before touching anything in `calibration/`,
`hil_toolkit/tag_calibration.py`, `CALIBRATION_GCODE`, or the tag definition.
Theory of the solver is in [`camera_calibration.md`](camera_calibration.md); this file is the *workflow*.

## 1. The idea

The calibration is run **twice**, once in each world:

| | Simulation (digital) | Real arm |
|---|---|---|
| Joint angles | exactly what G-code commands | commanded, but servo backlash / zero offsets / sag apply |
| Tag corner positions (3D) | exact forward kinematics | **unknown** — the arm may not be where it was told |
| Camera | perfect pinhole, no distortion, known K and pose | real lens, unknown K, pose, distortion |
| Output | **ground-truth file** | measurements to compare against it |

The sim has **no backlash and no offsets**, so its capture file is a perfect
reference: "if the arm and camera were ideal, at G-code line N the tag corners would be
here in 3D and land on these pixels". The real run replays the *same G-code*; any difference
between the real detections and the ground truth is real-world error (backlash, servo offset,
mounting error, lens error).

Never treat the sim ground truth as a measurement of the real hardware. It is the reference.

## 2. Files and where they come from

| File | Produced by | Role |
|---|---|---|
| `hil_toolkit/gcode.py` → `CALIBRATION_GCODE` | hand-written | The motion. 16 stations, each ends with `M240` (capture). **Identical on sim and real arm.** |
| `hil_toolkit/tags_toolkit/tag_model.py` → `CALIB_TAG` | hand-written | Tag id 0 on the hand (arm5 bottom face): `tag36h11`, print 30.0 mm, black square **18.524 mm** |
| `hil_toolkit/tag_calibration.py` → `TagCalibrationSession` | code | Captures at each `M240`, solves, saves everything |
| `calibration.py` → `calibrate()` | code | Appendix-E closed-form solver (needs ≥ 8 points; 16 captures × 4 corners = 64) |
| `calibration/apriltag_groundtruth_<stamp>.json` | sim run | Timestamped ground truth |
| `calibration/apriltag_groundtruth.json` | sim run | Copy of the **latest** run (use this one) |
| `calibration/correspondences_apriltag_<stamp>.csv` | sim run | Solver input of that run |
| `calibration/camera_model.json` | sim run (overwritten each run) | Solved camera model; also drives the GUI distance guides |

`calibration/` is generated output. A re-run of the sim overwrites `camera_model.json` and
`apriltag_groundtruth.json`, so **do not re-run the sim calibration casually** — the real-arm
comparison must use the ground truth whose `gcode.sha1` matches the G-code you ran.

## 3. Step-by-step

### Step A — Sim run (produces ground truth)

1. Launch `arm-hil-toolkit.py`.
2. **Camera Calibration** dialog → method **"AprilTag on the hand — calibration G-code"** → *Run Calibration G-code*.
   (The main-window G-code program must be reset first.)
3. The G-code runs through the normal runner; at every `M240`, `TagCalibrationSession.capture()` renders the
   simulated sensor, detects tag id 0 (sub-pixel corners), and records the capture.
4. At program end `finish()` solves, then writes the files in §2. Check the dialog report: expect
   detection error ≈ 0.2 px, reprojection RMS ≈ 0.2 px, `f_x` within ~0.3 % of true.

Never edit `CALIBRATION_GCODE` or the tag size/mount without re-running Step A, and never use an old
ground-truth file with a changed G-code (compare `gcode.sha1`).

### Step B — Real run

1. Print tag 36h11 id 0 (`paper/apriltags/tag36h11_id00_base_endeffector.png`) and mount it on the hand's
   bottom face exactly as in the sim. **Measure the black square with calipers.** If it is not 18.52 mm,
   update `print_size` of tag 0 in `TAGS` (`tag_model.py`) and redo Step A.
2. Start the arm at the same initial pose as the sim. The G-code begins with `G1 X0 Y-42.5 Z90 A0 B0`.
3. Run the **same** `CALIBRATION_GCODE` (streamed to the Arduino through the HIL toolkit; it is already limited
   to ≤ F600 and ≤ 20°/s per joint, no rapids). J1 stays at 0° because the camera rides on arm1.
4. At each `M240` grab a real frame (`tags_toolkit.apriltag.open_camera`) and detect tag id 0 with
   `AprilTagDetector` (family `tag36h11`; detector already uses `CORNER_REFINE_SUBPIX`).
5. **Check detection at all 16 stations.** The tag is seen 51–75° off its face normal. If a station misses,
   raise `A` there by up to 10° (keep J4 < 90°) and re-run Step A so sim and real stay identical.

### Step C — Use the ground truth with the real data

Match real captures to ground-truth captures by **`gcode_line`** (or `index`). Per capture the ground truth holds
`joint_deg`, `servo_cmd`, `corners_cal_mm` (3D, arm1 frame), `corners_base_mm`, `pixels_ideal_rc`,
`pixels_detected_rc` (sim detection), `distance_mm`, `pnp`.

Two uses — pick deliberately:

1. **Quantify real-world error (diagnostic).** Compare real detected corners with ground-truth pixels /
   PnP pose. Large, pose-dependent residuals point at backlash or servo zero offsets; a constant shift at all
   stations points at camera pose or mounting. Do this first.
2. **Calibrate the real camera.** Build a `CorrespondenceSet` with
   `world` = ground-truth `corners_cal_mm` (digital, ideal 3D positions) and `pixels` = **real** detected corners,
   then call `calibration.calibrate(cs)`. This gives the real camera's `R, T, f_x, f_y` *assuming the arm went
   exactly where commanded*.

Caveat for use 2: any real backlash/offset is not removed — it is absorbed into the solved camera model
(biased `T`, `R`, `f`). Trust the result only if use 1 shows small residuals. If residuals are large, fix or
characterise the mechanical error first (e.g. fit joint offsets) rather than calibrating through it.

## 4. Conventions that bite

- **Pixels are `(row, col)`**, origin at the centre of the top-left pixel. OpenCV detections are `(x, y)` =
  `(col, row)`; the code swaps with `d.corners[:, ::-1]`. Do the same for real detections.
- **Corner order** is TL, TR, BR, BL as the detector reads the tag. Keep it the same, or points mismatch.
- **Frames:** `base` (Z up, arm faces −Y at home), `cal` = arm1 = base rotated by θ1 about Z
  (`p_cal = Rz(−θ1) p_base`), `cam` = OpenCV (x right, y down, z forward). `calibrate()` outputs the "book"
  camera frame (x left, y up, z forward); `_CV_TO_BOOK = diag(−1, −1, 1)` converts.
- **Units:** mm and degrees throughout.
- **Image size** in the ground truth is 1289×720 with an ideal pinhole (FOV 81°×51°, no distortion,
  principal point assumed = image centre). A real camera differs: use the same resolution or rescale pixels, and
  **undistort real frames first** — `calibrate()` is linear and models no lens distortion. Consider a nonlinear
  refinement afterwards if accuracy matters.
- Tag corner positions assume the exact black-square size (18.524 mm). A size error scales everything, mostly `T_z`
  and `f_x`.
- The captured set is nearly coplanar per station; the roll `B` (0/30/60°) and depth/height variation are what make
  the 3D point set non-degenerate. Do not "simplify" the G-code by removing them.

## 5. Rules for agents

- Do not modify `CALIBRATION_GCODE`, `CALIB_TAG`, or the camera constants (`hardware_mapping.py` `CAM_*`) without
  re-running the sim calibration and regenerating all of `calibration/`.
- Do not delete or hand-edit files in `calibration/`; they are the reference.
- Do not "fix" real-arm discrepancies by editing the ground truth. The sim is the ideal; the discrepancy is the finding.
- Keep sim and real motion identical: same G-code, same start pose, same settle (`G4 P400`) before each `M240`.
- Verify the GUI/solver through code or `widget.grab()` only, not full-screen screenshots.
