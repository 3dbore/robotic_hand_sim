"""
CAMERA CALIBRATION FROM THE GRIPPER'S APRILTAG — the startup procedure
------------------------------------------------------------------------
gcode.CALIBRATION_GCODE turns the hand's AprilTag (tag_model.CALIB_TAG, id 0)
toward the camera at a series of poses; at every M240 the main window hands the pose to
TagCalibrationSession.capture(), which grabs the simulated sensor image,
detects the tag and records one capture:

    3D  the tag's four black-square corners from forward kinematics of the
        joint angles (and the tag's mounting on the hand), in the arm1 frame
    2D  the same corners as the detector found them in the image

The corners feed the Appendix E solver (calibration.calibrate), exactly as the
LED points did. Everything the real run needs to be checked against is saved
as ground truth: per capture the joint angles, servo commands, tag pose,
corner positions, ideal and detected pixels and the PnP distance, plus the
true intrinsics/extrinsics of the simulated camera and the G-code itself.
On the real arm the same G-code is run; the real camera's detections at the
same M240 lines are then compared capture by capture with this file.
"""
import datetime
import hashlib
import json
import os

import cv2
import numpy as np

from calibration import CorrespondenceSet, CameraModel, calibrate

from .sim_loader import ase
from .paths import CALIB_DIR, CAMERA_MODEL_FILE
from .hardware_mapping import SERVO_CHANNELS, CAM_LENS_POS, CAM_FOV_H, CAM_FOV_V, CAM_FOV_D
from .calibration_camera import base_to_cal
from .tags_toolkit.tag_model import CALIB_TAG, CALIB_TAG_FAMILY, CALIB_TAG_SIZE, CALIB_TAG_PRINT, calib_tag_spec
from .tags_toolkit.apriltag import estimate_pose

GROUND_TRUTH_FILE = os.path.join(CALIB_DIR, "apriltag_groundtruth.json")   # latest run
# Book camera frame (x left, y up, z forward) from OpenCV's (x right, y down, z forward).
_CV_TO_BOOK = np.diag([-1.0, -1.0, 1.0])


def _rot_deg(Ra, Rb):
    """Angle (deg) of the rotation taking Ra to Rb."""
    c = (np.trace(Ra.T @ Rb) - 1.0) / 2.0
    return float(np.degrees(np.arccos(np.clip(c, -1.0, 1.0))))


def true_camera_model(cam, theta1):
    """The simulated camera as a CameraModel in the calibration (arm1) frame —
    what calibrate() should recover. cam must be posed at base angle theta1."""
    R_cv, t_cv = cam.extrinsics()                       # base -> OpenCV camera
    R = _CV_TO_BOOK @ R_cv @ ase.Rz(theta1)             # arm1 -> book camera
    K = cam.camera_matrix()
    return CameraModel(R, _CV_TO_BOOK @ t_cv, K[0, 0], K[1, 1], K[0, 2], K[1, 2], cam.size,
                       meta={"source": "simulated camera (exact)"})


class TagCalibrationSession:
    """One run of the calibration G-code. camera: a SimCalibrationCamera,
    detector: an AprilTagDetector of CALIB_TAG_FAMILY."""

    def __init__(self, camera, detector, gcode_text, n_captures):
        self.cam = camera
        self.detector = detector
        self.gcode = gcode_text
        self.n_expected = n_captures
        self.K = camera.camera_matrix()
        self.captures = []
        self.started = datetime.datetime.now()
        self.result = None

    # ── per M240 ─────────────────────────────────────────────────────────────
    def capture(self, angles, line, t, gripper_cmd):
        """Grab, detect and record at this pose. Returns the capture dict
        (detected is False when the tag was not found)."""
        angles = np.array(angles, dtype=float)
        cam = self.cam
        cam.set_pose(angles, pose_meshes=False)    # the GUI has already posed the scene
        dets = [d for d in self.detector.detect(cam.capture_bgr()) if d.tag_id == CALIB_TAG.tag_id]
        estimate_pose(dets, self.K, np.zeros(5), CALIB_TAG_SIZE)

        R_tag, t_tag = CALIB_TAG.pose(angles)                  # tag -> base
        corners = CALIB_TAG.corners(angles)                     # (4,3) base, TL TR BR BL
        ideal, depth = cam.project(corners)                     # (row, col)
        R_cv, t_cv = cam.extrinsics()
        R_tc, t_tc = R_cv @ R_tag, R_cv @ t_tag + t_cv          # tag -> OpenCV camera
        to_cam = (R_cv.T @ -t_cv) - t_tag
        view_deg = float(np.degrees(np.arccos(np.clip(
            R_tag[:, 2] @ to_cam / np.linalg.norm(to_cam), -1, 1))))
        _, _, ee = ase.forward_kinematics(angles)
        servo = [gripper_cmd if ch.sim_joint is None else ch.to_servo(float(angles[ch.sim_joint]))[0]
                 for ch in SERVO_CHANNELS]

        rec = {"index": len(self.captures) + 1, "gcode_line": int(line), "t_s": round(float(t), 3),
               "joint_deg": angles.tolist(),
               "servo_cmd": dict(zip((ch.name for ch in SERVO_CHANNELS), servo)),
               "wrist_base_mm": ee.tolist(),
               "pitch_deg": float(angles[1] + angles[2] + angles[3]), "roll_deg": float(angles[4]),
               "tag_R_base": R_tag.tolist(), "tag_t_base_mm": t_tag.tolist(),
               "tag_R_cam": R_tc.tolist(), "tag_t_cam_mm": t_tc.tolist(),
               "distance_mm": float(np.linalg.norm(t_tc)), "view_angle_deg": view_deg,
               "corners_base_mm": corners.tolist(),
               "corners_cal_mm": base_to_cal(corners.T, angles[0]).T.tolist(),
               "pixels_ideal_rc": ideal.tolist(), "detected": bool(dets)}
        if dets:
            d = dets[0]
            px = d.corners[:, ::-1].astype(float)                # (x, y) -> (row, col)
            rec["pixels_detected_rc"] = px.tolist()
            rec["corner_err_px"] = np.linalg.norm(px - ideal, axis=1).tolist()
            if d.tvec is not None:
                R_est = cv2.Rodrigues(d.rvec)[0]
                t_est = d.tvec.ravel()
                rec["pnp"] = {"rvec": d.rvec.ravel().tolist(), "tvec_mm": t_est.tolist(),
                              "distance_mm": float(np.linalg.norm(t_est)),
                              "t_err_mm": float(np.linalg.norm(t_est - t_tc)),
                              "R_err_deg": _rot_deg(R_est, R_tc)}
        self.captures.append(rec)
        return rec

    @property
    def n_detected(self):
        return sum(c["detected"] for c in self.captures)

    # ── Camera View read-out ────────────────────────────────────────────────
    def hud_lines(self):
        """Short lines (ASCII, <= 19 columns) for the Camera View read-out."""
        W, H = self.cam.size
        lines = [f"CALIB {CALIB_TAG_FAMILY} id{CALIB_TAG.tag_id}",
                 f"Black  {CALIB_TAG_SIZE:5.2f} mm",
                 f"Print  {CALIB_TAG_PRINT:5.1f} mm",
                 f"Sensor {W}x{H}",
                 f"f      {self.K[0, 0]:5.1f} px",
                 f"Shot {len(self.captures)}/{self.n_expected} {4 * self.n_detected} pts"]
        if self.captures:
            c = self.captures[-1]
            pnp = c.get("pnp")
            lines += [f"L{c['gcode_line']} A{c['pitch_deg']:+.0f} B{c['roll_deg']:+.0f}",
                      f"Dist   {c['distance_mm']:5.1f} mm",
                      f"PnP    {pnp['distance_mm']:5.1f} mm" if pnp else "PnP    no tag",
                      f"Err    {max(c['corner_err_px']):5.2f} px" if c["detected"] else "Err       -",
                      f"View   {c['view_angle_deg']:5.0f} deg"]
        if self.result:
            m, d = self.result["model"], self.result["diag"]
            lines += ["SOLVED", f"fx     {m.fx:5.1f} px", f"RMS    {d['reproj_rms']:5.2f} px"]
        return lines

    # ── end of program ───────────────────────────────────────────────────────
    def correspondences(self):
        world, pixels, ideal, theta, base, idx = [], [], [], [], [], []
        for c in self.captures:
            if not c["detected"]:
                continue
            for k in range(4):
                world.append(c["corners_cal_mm"][k])
                pixels.append(c["pixels_detected_rc"][k])
                ideal.append(c["pixels_ideal_rc"][k])
                theta.append(c["joint_deg"])
                base.append(c["corners_base_mm"][k])
                idx.append([c["index"], k])
        return CorrespondenceSet(
            world, pixels, self.cam.size,
            meta={"source": "apriltag", "frame": "arm1 = base rotated by theta1 about Z; p_cal = Rz(-theta1) p_base",
                  "marker": f"{CALIB_TAG_FAMILY} id {CALIB_TAG.tag_id} black-square corners, "
                            f"{CALIB_TAG_SIZE:.3f} mm, on the hand (arm5 bottom face)"},
            extra={"theta": theta, "base": base, "ideal": ideal, "capture_corner": idx})

    def finish(self):
        """Solve, save everything and return the result dict (or raise ValueError
        when too few corners were detected)."""
        cs = self.correspondences()
        if len(cs) < 8:
            raise ValueError(f"only {len(cs)} tag corners detected — need at least 8")
        os.makedirs(CALIB_DIR, exist_ok=True)
        stamp = self.started.strftime("%Y%m%d-%H%M%S")
        csv_path = os.path.join(CALIB_DIR, f"correspondences_apriltag_{stamp}.csv")
        gt_path = os.path.join(CALIB_DIR, f"apriltag_groundtruth_{stamp}.json")
        cs.save_csv(csv_path)

        model, diag = calibrate(cs)
        model.meta.update({"correspondences": os.path.basename(csv_path),
                           "ground_truth": os.path.basename(gt_path)})
        model.save_json(CAMERA_MODEL_FILE)

        theta1 = self.captures[0]["joint_deg"][0]
        self.cam.set_pose(self.captures[0]["joint_deg"], pose_meshes=False)
        truth = true_camera_model(self.cam, theta1)
        err = np.linalg.norm(cs.pixels - cs.extra["ideal"], axis=1)
        det = [c for c in self.captures if c.get("pnp")]
        compare = {"fx_err_px": model.fx - truth.fx, "fy_err_px": model.fy - truth.fy,
                   "center_err_mm": float(np.linalg.norm(model.center - truth.center)),
                   "R_err_deg": _rot_deg(model.R, truth.R)}
        summary = {
            "captures": len(self.captures), "detected": self.n_detected, "points": len(cs),
            "detect_err_px": {"mean": float(err.mean()), "rms": float(np.sqrt((err ** 2).mean())),
                              "max": float(err.max())},
            "pnp_t_err_mm": {"mean": float(np.mean([c["pnp"]["t_err_mm"] for c in det])),
                             "max": float(np.max([c["pnp"]["t_err_mm"] for c in det]))},
            "distance_mm": [float(min(c["distance_mm"] for c in self.captures)),
                            float(max(c["distance_mm"] for c in self.captures))],
            "principal_std_mm": cs.coplanarity().tolist(),
            "solve": diag, "solved_vs_truth": compare}
        doc = {
            "about": "ELIOS camera-calibration ground truth (simulation). Run the same G-code on the "
                     "real arm, detect the same tag at each M240 and compare capture by capture.",
            "created": self.started.isoformat(timespec="seconds"),
            "frames": {"base": "arm base, mm; Z up, arm faces -Y at home",
                       "cal": "arm1 = base rotated by theta1 about Z (the camera rides on arm1)",
                       "cam": "OpenCV: x right, y down, z forward, mm",
                       "pixels": "(row, col), origin at the centre of the top-left pixel"},
            "tag": calib_tag_spec(),
            "camera": {"image_size": list(self.cam.size), "K_true": self.K.tolist(),
                       "dist_true": [0.0] * 5, "fov_deg": {"h": CAM_FOV_H, "v": CAM_FOV_V, "d": CAM_FOV_D},
                       "lens_home_base_mm": CAM_LENS_POS.tolist(),
                       "true_model_cal": truth.to_dict(), "solved_model_cal": model.to_dict()},
            "gcode": {"text": self.gcode, "sha1": hashlib.sha1(self.gcode.encode()).hexdigest(),
                      "capture_code": "M240"},
            "summary": summary,
            "captures": self.captures,
        }
        for path in (gt_path, GROUND_TRUTH_FILE):
            with open(path, "w", encoding="utf-8") as f:
                json.dump(doc, f, indent=1)
        self.result = {"model": model, "diag": diag, "truth": truth, "summary": summary,
                       "paths": {"ground_truth": gt_path, "latest": GROUND_TRUTH_FILE,
                                 "correspondences": csv_path, "model": CAMERA_MODEL_FILE}}
        return self.result


def format_result(result):
    """Text report for the calibration dialog."""
    m, d, s = result["model"], result["diag"], result["summary"]
    cmp_, truth = s["solved_vs_truth"], result["truth"]
    e, p = s["detect_err_px"], s["pnp_t_err_mm"]
    v = lambda a: "[" + ", ".join(f"{x:+.4f}" for x in a) + "]"
    ps = s["principal_std_mm"]
    return "\n".join([
        f"Tag: {CALIB_TAG_FAMILY} id {CALIB_TAG.tag_id}, black square {CALIB_TAG_SIZE:.2f} mm "
        f"(print {CALIB_TAG_PRINT:.1f} mm), on the hand's bottom face",
        f"Captures: {s['detected']}/{s['captures']} detected → {s['points']} corner points, "
        f"distance {s['distance_mm'][0]:.0f}–{s['distance_mm'][1]:.0f} mm",
        f"Principal std (mm): {ps[0]:.1f} / {ps[1]:.1f} / {ps[2]:.1f}  (last = out-of-plane)",
        f"Corner detection vs truth (px): mean {e['mean']:.3f}, RMS {e['rms']:.3f}, max {e['max']:.3f}",
        f"PnP position vs truth (mm): mean {p['mean']:.2f}, max {p['max']:.2f}",
        "",
        "Solved (Appendix E, principal point = image centre):",
        f"  R = {v(m.R[0])}", f"      {v(m.R[1])}", f"      {v(m.R[2])}",
        f"  T = [{m.T[0]:.2f}, {m.T[1]:.2f}, {m.T[2]:.2f}] mm",
        f"  f_x {m.fx:.2f} px, f_y {m.fy:.2f} px  (true {truth.fx:.2f}), o_r {m.o_r:.1f}, o_c {m.o_c:.1f}",
        f"  Reprojection RMS {d['reproj_rms']:.3f} px (max {d['reproj_max']:.3f})",
        f"  vs true camera: f_x {cmp_['fx_err_px']:+.2f} px, centre {cmp_['center_err_mm']:.2f} mm, "
        f"R {cmp_['R_err_deg']:.3f}°",
        "",
        f"Ground truth: {result['paths']['ground_truth']}",
        f"   (latest copy: {result['paths']['latest']})",
        f"Correspondences: {result['paths']['correspondences']}",
        f"Camera model: {result['paths']['model']}",
    ])
