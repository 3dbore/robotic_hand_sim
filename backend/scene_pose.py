"""Pose data for the web 3D view: the same parts and matrices as hil/scene.py,
without the pyvista/Qt rendering. The browser loads each STL once and sets its
4x4 matrix every tick, so the web view shows exactly what the PyQt app shows."""
import importlib.util
import math
import os

import numpy as np
import pyvista as pv

from hil import config, kinematics
from hil.config import CAM_LENS_POS, CAM_PART_FILE, CAM_PART_POS, CAM_PART_ROT, CAM_VIEW_DIR
from hil.sim import ase

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

_spec = importlib.util.spec_from_file_location("gripper_kinematics", os.path.join(ROOT, "gripper", "gripper_kinematics.py"))
gk = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(gk)

ARM_COLORS = {name: cfg["color"] for name, cfg in ase.ARM_CONFIG.items()}
LINK_INDEX = {k: i for i, k in enumerate(ARM_COLORS, start=1)}
ARM_LINKS = [k for k in ARM_COLORS if k not in ("arm4", "arm5")]   # replaced by the gripper model
JAW_PARTS = ("servo", "rod_geared", "rod", "finger")
GRIP_COLORS = {"wrist": ARM_COLORS["arm4"], "body": ARM_COLORS["arm5"], "body2": ARM_COLORS["arm5"],
               "servo": "#3A3A3A", "rod_geared": "#C8C8C8", "rod": "#C8C8C8", "finger": "#E0B040"}

J4_PIVOT = np.array(ase.JOINTS[3][1], float)
PITCH_HOME = 90.0
G0 = gk.rot((1, 0, 0), 90.0)
G0[:3, 3] = J4_PIVOT - G0[:3, :3] @ gk.PITCH_PIVOT

CAM_FIX = np.eye(4)                       # hand_cam.stl -> its home placement on Arm A
CAM_FIX[:3, :3] = np.array(CAM_PART_ROT, float)
CAM_FIX[:3, 3] = CAM_PART_POS
_lens = np.array(CAM_LENS_POS, float)
LENS_HOME = np.array([_lens, _lens + 100.0 * np.array(CAM_VIEW_DIR), _lens + (0, 0, 10.0)])

_grip = gk.Gripper(pv.read(gk.stl_path("finger")).points)

# part id -> STL file on disk
FILES = {k: os.path.join(ROOT, "arms", ase.ARM_CONFIG[k]["file"]) for k in ARM_LINKS}
FILES.update({k: gk.stl_path(k) for k in gk.PARTS})
FILES["cam"] = CAM_PART_FILE


def parts():
    """What the browser draws: one entry per instance (jaw parts twice)."""
    out = [{"id": k, "file": k, "color": ARM_COLORS[k]} for k in ARM_LINKS]
    for k in gk.PARTS:
        for side in (("L", "R") if k in JAW_PARTS else ("",)):
            out.append({"id": k + side, "file": k, "color": GRIP_COLORS[k]})
    out.append({"id": "cam", "file": "cam", "color": "#A0A0A0"})
    return out


def pose(sim5, grip_pct):
    """{part id: 16 floats, column-major as three.js wants} plus the lens."""
    M = ase.link_transforms(sim5)
    mats = {k: M[LINK_INDEX[k]] for k in ARM_LINKS}
    Mp, My = gk.wrist_matrices(PITCH_HOME + sim5[3], -sim5[4])
    G = M[3] @ G0
    Gp, Gy = G @ Mp, G @ My
    solved = gk.solve_jaw(_grip.servo_for_percent(min(max(grip_pct, 0.0), 100.0)))
    jaw = gk.part_matrices(solved)
    T = np.eye(4)                              # fingertip centre, on the gripper's X = 0 plane
    T[1:3, 3] = _grip.tip(solved)[1:3]
    grip = {"mat": np.round((Gy @ T).T.ravel(), 4).tolist(), "gap": round(max(_grip.gap(solved), 0.1), 2)}
    mats.update({"wrist": Gp @ gk.placement("wrist"), "body": Gy @ gk.placement("body"),
                 "body2": Gy @ gk.placement("body2")})
    for k in JAW_PARTS:
        mats[k + "L"] = Gy @ jaw[k]
        mats[k + "R"] = Gy @ gk.MIRROR_X @ jaw[k]
    mats["cam"] = M[2] @ CAM_FIX
    lens = ase.apply_transform_to_points(LENS_HOME, M[2])
    return {"mats": {k: np.round(m.T.ravel(), 4).tolist() for k, m in mats.items()},
            "lens": np.round(lens, 2).tolist(), "grip": grip}


# ── workspace masks for the 2D views, as in MainWindow._refresh_masks ──
_ws = None


def workspace():
    global _ws
    if _ws is None:
        _ws = kinematics.Workspace(config.joint_limits())
    R = 50 * math.ceil(_ws.reach / 50)
    return _ws, R


def mask_png(mode, value):
    """RGBA PNG of the reachable area, oriented for the screen.
    top: value = EE height Z; up = -Y (forward), right = -X, extent ±R.
    side: value = base heading; up = +Z, right = forward reach, h ±R, Z -150..R+50."""
    import cv2
    ws, R = workspace()
    if mode == "top":
        xs = np.linspace(-R, R, 300)
        ys = np.linspace(-R, R, 300)
        m = ws.top(xs, ys, value) if value >= config.Z_FLOOR_MM else np.zeros((300, 300), bool)
        m = m[:, ::-1]
    else:
        hs = np.linspace(-R, R, 300)
        zs = np.linspace(-150, R + 50, int(300 * (R + 200) / (2 * R)))
        m = (ws.side(hs, zs, value) & (zs[:, None] >= config.Z_FLOOR_MM))[::-1, :]
    bgra = np.zeros(m.shape + (4,), np.uint8)
    bgra[m] = (0x24, 0xB2, 0x1E, 90)      # data/positive #1eb224, BGR order
    return cv2.imencode(".png", bgra)[1].tobytes()
