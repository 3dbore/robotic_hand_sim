"""
SIM TAGS — the mounted AprilTags as textured meshes in the simulated scene
----------------------------------------------------------------------------
One shared quad per tag (like ase.loaded_meshes), textured with its PNG and
re-posed on every ase.update_scene() call, so it follows the arm in every
plotter it is added to: the GUI views and the offscreen calibration camera.

    add_tags(plotter)   # once per plotter; the pose hook installs itself

Rendered unlit (flat black/white, like the LED marker) so detection in the
sim doesn't depend on how the scene lights happen to hit each face.
"""
import numpy as np
import pyvista as pv

from ..sim_loader import ase
from .tag_model import TAGS


def _home_quad(tag):
    """Full printed image (black square + white border) at the home pose, drawn
    exactly at the ground-truth pose — no lift off the face: even 0.2 mm shifts
    a close tag by ~1.5 px. The depth buffer separates it from its face because
    the tag centres sit slightly outside them (the paper).
    Points wind CCW about the tag's +Z, so back-face culling hides its rear.

    The PNGs are stored rotated 180° from how the detector reads them (tag X
    points to image-left, tag Y to image-down), so texture s runs along -X and
    t along -Y."""
    h = tag.print_size / 2.0
    local = np.array([[-h, -h, 0], [h, -h, 0], [h, h, 0], [-h, h, 0]])
    tcoords = np.array([[1, 1], [0, 1], [0, 0], [1, 0]], dtype=float)
    pts = local @ tag.R.T + tag.center
    quad = pv.PolyData(pts, faces=[4, 0, 1, 2, 3])
    quad.active_texture_coordinates = tcoords
    return quad


def _texture(tag):
    tex = pv.read_texture(tag.image)
    tex.interpolate = True
    tex.mipmap = True   # the small tags span a few dozen pixels; avoid aliasing
    return tex


TAG_MESHES = {tid: _home_quad(tag) for tid, tag in TAGS.items()}
_HOME_POINTS = {tid: mesh.points.copy() for tid, mesh in TAG_MESHES.items()}
_TEXTURES = {}


def pose_tags(angles):
    transforms = ase.link_transforms(np.asarray(angles, float))
    for tid, tag in TAGS.items():
        TAG_MESHES[tid].points[:] = ase.apply_transform_to_points(_HOME_POINTS[tid], transforms[tag.link])


def _install_pose_hook():
    update_scene = ase.update_scene
    if getattr(update_scene, "_poses_tags", False):
        return

    def update_scene_with_tags(angles):
        update_scene(angles)
        pose_tags(angles)

    update_scene_with_tags._poses_tags = True
    ase.update_scene = update_scene_with_tags  # arm-sim-end resolves this name at call time
    pose_tags(ase.current_angles)


def add_tags(plotter):
    _install_pose_hook()
    for tid, tag in TAGS.items():
        if tid not in _TEXTURES:
            _TEXTURES[tid] = _texture(tag)
        plotter.add_mesh(TAG_MESHES[tid], texture=_TEXTURES[tid], lighting=False, culling="back")
