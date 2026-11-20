"""3D view of the arm: the sim's meshes posed at the commanded angles, plus
the end effector and the click-to-move target."""
import math

import numpy as np
import pyvista as pv
from pyvistaqt import QtInteractor

from .config import CAM_LENS_POS, CAM_PART_FILE, CAM_PART_POS, CAM_PART_ROT, CAM_VIEW_DIR, TANK_Z_MM
from .sim import ase
from .theme import C

ARM_COLORS = {name: cfg["color"] for name, cfg in ase.ARM_CONFIG.items()}


class Scene:
    def __init__(self, parent):
        self.plotter = QtInteractor(parent)
        self.widget = self.plotter.interactor
        p = self.plotter
        p.set_background(C["surface_lowest"])

        self.base_pts = {k: ase.original_points[k].copy() for k in ARM_COLORS}
        self.meshes = {k: ase.loaded_meshes[k].copy() for k in ARM_COLORS}
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

    def _set_target_visible(self, on):
        try:
            self.target_actor.SetVisibility(bool(on))
        except AttributeError:
            pass

    def pose(self, angles, ee):
        M = ase.link_transforms(angles)
        for i, k in enumerate(ARM_COLORS, start=1):
            self.meshes[k].points[:] = ase.apply_transform_to_points(self.base_pts[k], M[i])
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
