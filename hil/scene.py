"""3D view of the arm: the sim's meshes posed at the commanded angles, plus
the end effector and the click-to-move target."""
import numpy as np
import pyvista as pv
from pyvistaqt import QtInteractor

from .config import TANK_Z_MM
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
        self.cam_pts = ase.original_cam_points.copy()
        self.cam = ase.cam_mesh.copy()

        floor = pv.Plane(center=(0, -150, 0), direction=(0, 0, 1), i_size=700, j_size=700,
                         i_resolution=14, j_resolution=14)
        p.add_mesh(floor, style="wireframe", color=C["surface_dim"], line_width=1)
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
        self.cam.points[:] = ase.apply_transform_to_points(self.cam_pts, M[1])
        self.ee.points[:] = self.ee_pts + np.asarray(ee, np.float32)
        self.render()

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
            self.plotter.close()
        except Exception:
            pass
