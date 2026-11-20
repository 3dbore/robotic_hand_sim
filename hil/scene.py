"""3D view of the arm: the sim's meshes posed at the commanded angles, plus
the end effector and the click-to-move target.

From the wrist pitch joint (J4) on, the sim's arm4/arm5 meshes are replaced by
the gripper model in gripper/ (wrist, body, servo and both four-bar jaws).
"""
import importlib.util
import math
import os

import numpy as np
import pyvista as pv
from pyvistaqt import QtInteractor

from .config import CAM_LENS_POS, CAM_PART_FILE, CAM_PART_POS, CAM_PART_ROT, CAM_VIEW_DIR, TANK_Z_MM
from .sim import ase
from .theme import C

ARM_COLORS = {name: cfg["color"] for name, cfg in ase.ARM_CONFIG.items()}
LINK_INDEX = {k: i for i, k in enumerate(ARM_COLORS, start=1)}   # mesh -> its matrix in link_transforms
REPLACED = ("arm4", "arm5")                                      # pitch link and old gripper
ARM_LINKS = [k for k in ARM_COLORS if k not in REPLACED]

_spec = importlib.util.spec_from_file_location(
    "gripper_kinematics",
    os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "gripper", "gripper_kinematics.py"))
gk = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(gk)

# Gripper frame (Y along the gripper, X across, pitch about X through
# gk.PITCH_PIVOT) -> the arm's frame below J4 at sim angle 0: the pitch axis
# lies on the arm's X, the pitch pivot sits on J4, and gripper pitch 0 points
# straight up (+Z). The gripper's pitch is then 90 + J4, so the home pose
# (J4 = 0) is pitch +90 with the jaws reaching out along -Y like the old arm5.
J4_PIVOT = np.array(ase.JOINTS[3][1], float)
PITCH_HOME = 90.0
G0 = gk.rot((1, 0, 0), 90.0)
G0[:3, 3] = J4_PIVOT - G0[:3, :3] @ gk.PITCH_PIVOT
GRIP_COLORS = {"wrist": ARM_COLORS["arm4"], "body": ARM_COLORS["arm5"], "body2": ARM_COLORS["arm5"],
               "servo": "#3A3A3A", "rod_geared": "#C8C8C8", "rod": "#C8C8C8", "finger": "#E0B040"}
JAW_PARTS = ("servo", "rod_geared", "rod", "finger")


class Scene:
    def __init__(self, parent):
        self.plotter = QtInteractor(parent)
        self.widget = self.plotter.interactor
        p = self.plotter
        p.set_background(C["surface_lowest"])

        self.base_pts = {k: ase.original_points[k].copy() for k in ARM_LINKS}
        self.meshes = {k: ase.loaded_meshes[k].copy() for k in ARM_LINKS}
        # Gripper parts stay in their STL frames and are posed with user_matrix;
        # the jaw parts are drawn twice (left, and right = mirrored across X).
        self.grip_meshes = {k: pv.read(gk.stl_path(k)) for k in gk.PARTS}
        self.grip = gk.Gripper(self.grip_meshes["finger"].points)
        self.grip_pct = 100.0
        self.grip_actors = []          # (actor, part, side) for the main view and the POV view
        self.cam = pv.read(CAM_PART_FILE)
        self.cam_pts = (self.cam.points @ np.array(CAM_PART_ROT, float).T + CAM_PART_POS).astype(np.float32)
        self.cam.points = self.cam_pts.copy()
        lens = np.array(CAM_LENS_POS, float)
        # lens, a point it looks at, a point above it: posed with Arm A like the part
        self.lens_home = np.array([lens, lens + 100.0 * np.array(CAM_VIEW_DIR), lens + (0, 0, 10.0)])
        self.lens = self.lens_home.copy()
        self._pov = None

        self.floor = pv.Plane(center=(0, -150, 0), direction=(0, 0, 1), i_size=700, j_size=700,
                              i_resolution=14, j_resolution=14)
        p.add_mesh(self.floor, style="wireframe", color=C["surface_dim"], line_width=1)
        # Keep-out over the tank: behind the base (Y > 0), below TANK_Z_MM.
        keep_out = pv.Box(bounds=(-350, 350, 0, 350, 0, TANK_Z_MM))
        p.add_mesh(keep_out, color=C["error"], opacity=0.07)
        p.add_mesh(keep_out, style="wireframe", color=C["error"], opacity=0.35, line_width=1)
        shading = dict(smooth_shading=True, specular=0.4, specular_power=20)
        for k, m in self.meshes.items():
            p.add_mesh(m, color=ARM_COLORS[k], **shading)
        self._add_gripper(p, shading)
        p.add_mesh(self.cam, color="#A0A0A0", **shading)

        self.ee = pv.Sphere(radius=8, center=(0, 0, 0))
        self.ee_pts = self.ee.points.copy()
        p.add_mesh(self.ee, color="#FF6B35", opacity=0.85)
        self.target = pv.Sphere(radius=7, center=(0, 0, 0))
        self.target_pts = self.target.points.copy()
        self.target_actor = p.add_mesh(self.target, color=C["error"], opacity=0.55)
        self._set_target_visible(False)

        try:
            p.add_axes(line_width=2, labels_off=False)
        except Exception:
            pass
        p.camera_position = [(520, -620, 420), (0, -60, 120), (0, 0, 1)]

    def _add_gripper(self, p, shading):
        for k, m in self.grip_meshes.items():
            for side in (("L", "R") if k in JAW_PARTS else ("",)):
                self.grip_actors.append((p.add_mesh(m, color=GRIP_COLORS[k], **shading), k, side))

    def _pose_gripper(self, M, angles):
        """Wrist pitch = 90 + J4, yaw = -J5 (gripper yaw turns about its own
        long axis, which points along -Y at home), jaws from self.grip_pct."""
        Mp, My = gk.wrist_matrices(PITCH_HOME + angles[3], -angles[4])
        G = M[3] @ G0
        Gp, Gy = G @ Mp, G @ My
        servo = self.grip.servo_for_percent(min(max(self.grip_pct, 0.0), 100.0))
        jaw = gk.part_matrices(gk.solve_jaw(servo))
        mats = {"wrist": Gp @ gk.placement("wrist"), "body": Gy @ gk.placement("body"),
                "body2": Gy @ gk.placement("body2")}
        for k in JAW_PARTS:
            mats[k + "L"] = Gy @ jaw[k]
            mats[k + "R"] = Gy @ gk.MIRROR_X @ jaw[k]
        for actor, k, side in self.grip_actors:
            actor.user_matrix = mats[k + side]

    def _set_target_visible(self, on):
        try:
            self.target_actor.SetVisibility(bool(on))
        except AttributeError:
            pass

    def pose(self, angles, ee, grip_pct=None):
        """grip_pct: jaw opening, 0 = closed .. 100 = fully open (None keeps the last)."""
        if grip_pct is not None:
            self.grip_pct = float(grip_pct)
        M = ase.link_transforms(angles)
        self._last = (M, list(angles))
        for k in ARM_LINKS:
            self.meshes[k].points[:] = ase.apply_transform_to_points(self.base_pts[k], M[LINK_INDEX[k]])
        self._pose_gripper(M, angles)
        self.cam.points[:] = ase.apply_transform_to_points(self.cam_pts, M[2])
        self.lens = ase.apply_transform_to_points(self.lens_home, M[2])
        self.ee.points[:] = self.ee_pts + np.asarray(ee, np.float32)
        self.render()

    def pov_image(self, w, h, hfov_deg):
        """What the hand camera sees, as an (h, w, 3) uint8 RGB image: the arm and
        floor only (a real camera can't see the markers or the keep-out box).
        The offscreen plotter shares the posed meshes, so it is always current."""
        if self._pov is None:
            self._pov = pv.Plotter(off_screen=True, window_size=(w, h))
            self._pov.set_background(C["surface_lowest"])
            self._pov.add_mesh(self.floor, style="wireframe", color=C["surface_dim"], line_width=1)
            for k, m in self.meshes.items():
                self._pov.add_mesh(m, color=ARM_COLORS[k], smooth_shading=True, specular=0.4, specular_power=20)
            self._add_gripper(self._pov, dict(smooth_shading=True, specular=0.4, specular_power=20))
            if getattr(self, "_last", None) is not None:
                self._pose_gripper(*self._last)
        p = self._pov
        p.window_size = (w, h)
        pos, focal, up = self.lens
        cam = p.camera
        cam.position, cam.focal_point, cam.up = pos, focal, up - pos
        cam.view_angle = math.degrees(2 * math.atan(math.tan(math.radians(hfov_deg) / 2) * h / w))  # VTK wants vertical
        cam.clipping_range = (1.0, 5000.0)
        return p.screenshot(return_img=True)

    def show_target(self, xyz):
        if xyz is None:
            self._set_target_visible(False)
        else:
            self.target.points[:] = self.target_pts + np.asarray(xyz, np.float32)
            self._set_target_visible(True)
        self.render()

    def render(self):
        try:
            self.plotter.render()
        except Exception:
            pass

    def close(self):
        try:
            if self._pov is not None:
                self._pov.close()
            self.plotter.close()
        except Exception:
            pass
