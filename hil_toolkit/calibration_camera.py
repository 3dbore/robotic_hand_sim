"""
CAMERA CALIBRATION — STAGE 1: CORRESPONDENCE GENERATOR (simulated camera)
----------------------------------------------------------------------------
Calibration world frame = arm1 frame: the base frame rotated by θ1 about Z
(identical to the base frame at θ1 = 0). The camera rides on arm1, so it is
only rigid in this frame:  p_cal = Rz(-θ1) · p_base.
"""
import numpy as np
import pyvista as pv

from calibration import CorrespondenceSet, detect_color_blob

from .sim_loader import ase
from .theme import C
from .tags_toolkit.sim_tags import add_tags
from .hardware_mapping import (
    CAM_LENS_POS, CAM_VIEW_DIR, CAM_UP_DIR, CAM_FOV_V,
    CALIB_IMAGE_W, CALIB_IMAGE_H,
    GRIP_MARKER_LOCAL, GRIP_MARKER_RADIUS, GRIP_MARKER_RGB,
)


def camera_pose(theta1):
    """Camera (position, focal_point, up) in the base frame for base angle θ1."""
    T1 = ase.get_transform_matrix(ase.JOINTS[0][1], ase.JOINTS[0][2], theta1)
    pts = np.array([CAM_LENS_POS,
                    CAM_LENS_POS + 100.0 * CAM_VIEW_DIR,
                    CAM_LENS_POS + 10.0 * CAM_UP_DIR])
    pos, focal, up = ase.apply_transform_to_points(pts, T1).astype(float)
    return pos, focal, up - pos


def base_to_cal(p_base, theta1):
    return ase.Rz(-theta1) @ np.asarray(p_base, dtype=float)


def grip_marker_base(angles):
    M5 = ase.link_transforms(angles)[5]
    return (M5 @ np.append(GRIP_MARKER_LOCAL, 1.0))[:3]


class SimCalibrationCamera:
    """Offscreen twin of the Camera View at a fixed sensor resolution. Renders only
    physical geometry (arm + floor), the gripper LED and the AprilTags — no joint
    markers, kinematic chain or viewfinder overlay, since a real camera would not
    see them."""

    def __init__(self, width=CALIB_IMAGE_W, height=CALIB_IMAGE_H):
        self.size = (width, height)
        p = pv.Plotter(off_screen=True, window_size=self.size)
        p.set_background(C["surface_low"])
        p.add_mesh(pv.Plane(center=(0, -200, 0), direction=(0, 0, 1), i_size=800, j_size=800,
                            i_resolution=16, j_resolution=16),
                   style="wireframe", color=C["surface_dim"], line_width=1.2)
        shading = dict(show_edges=False, smooth_shading=True, specular=0.5, specular_power=25)
        for arm_name, cfg in ase.ARM_CONFIG.items():
            p.add_mesh(ase.loaded_meshes[arm_name], color=cfg["color"], **shading)
        self.marker = pv.Sphere(radius=GRIP_MARKER_RADIUS, center=GRIP_MARKER_LOCAL,
                                theta_resolution=24, phi_resolution=24)
        self._marker_home = self.marker.points.copy()
        # Unlit, so the LED renders as one flat colour like a saturated emitter
        p.add_mesh(self.marker, color=[v / 255 for v in GRIP_MARKER_RGB], lighting=False)
        add_tags(p)
        self.plotter = p

    def set_pose(self, angles, pose_meshes=True):
        """Pose this camera, its LED and (if pose_meshes) the shared scene meshes —
        pass False when the scene is already at `angles`. Does not touch
        ase.current_angles, so nothing is streamed to the hardware."""
        if pose_meshes:
            ase.update_scene(angles)
        M5 = ase.link_transforms(angles)[5]
        self.marker.points[:] = ase.apply_transform_to_points(self._marker_home, M5)
        pos, focal, up = camera_pose(angles[0])
        cam = self.plotter.camera
        cam.position, cam.focal_point, cam.up = pos, focal, up
        cam.view_angle = CAM_FOV_V
        cam.clipping_range = (1.0, 5000.0)

    def project(self, pts_base):
        """Renderer's own projection of base-frame points -> ((N,2) (r, c), (N,) depth)."""
        W, H = self.size
        M = pv.array_from_vtkmatrix(
            self.plotter.camera.GetCompositeProjectionTransformMatrix(W / H, -1, 1))
        h = np.c_[np.atleast_2d(pts_base), np.ones(len(np.atleast_2d(pts_base)))] @ M.T
        ndc = h[:, :3] / h[:, 3:4]
        x_d = (ndc[:, 0] + 1.0) / 2.0 * W
        y_d = (ndc[:, 1] + 1.0) / 2.0 * H
        return np.c_[H - y_d - 0.5, x_d - 0.5], h[:, 3]

    def capture(self):
        self.plotter.render()
        return self.plotter.screenshot(return_img=True)

    def capture_bgr(self):
        """capture() in OpenCV's channel order, for tags_toolkit.apriltag."""
        return np.ascontiguousarray(self.capture()[..., ::-1])

    def camera_matrix(self):
        """OpenCV intrinsics of this render: square pixels, principal point at the
        image centre (pixel-centre convention, as in project()), no distortion."""
        W, H = self.size
        f = (H / 2.0) / np.tan(np.radians(CAM_FOV_V) / 2.0)
        return np.array([[f, 0.0, W / 2.0 - 0.5],
                         [0.0, f, H / 2.0 - 0.5],
                         [0.0, 0.0, 1.0]])

    def extrinsics(self):
        """(R, t) base frame -> OpenCV camera frame (x right, y down, z forward), mm."""
        cam = self.plotter.camera
        pos, focal, up = (np.asarray(v, dtype=float) for v in (cam.position, cam.focal_point, cam.up))
        z = (focal - pos) / np.linalg.norm(focal - pos)
        y = -(up - np.dot(up, z) * z)
        y /= np.linalg.norm(y)
        R = np.vstack([np.cross(y, z), y, z])
        return R, -R @ pos

    def close(self):
        self.plotter.close()


CALIB_SOURCES = ("ideal", "detected")


def collect_correspondences(source, n_points=300, seed=0, min_spacing=6.0,
                            max_tries=20000, progress=None):
    """Stage 1. Random poses over the effective joint limits; keep those whose LED is
    above the floor, in front of the lens and inside the image.

    source "ideal":    (r, c) = renderer projection of the LED centre.
    source "detected": (r, c) = blob centroid in the rendered image; occluded or
                       clipped LEDs are dropped. Ideal (r, c) kept as a diagnostic.
    Returns (CorrespondenceSet, stats dict).
    """
    if source not in CALIB_SOURCES:
        raise ValueError(f"source must be one of {CALIB_SOURCES}")
    rng = np.random.default_rng(seed)
    lims = np.array(ase.JOINT_LIMITS, dtype=float)
    cam = SimCalibrationCamera()
    W, H = cam.size
    world, pixels, ideal, thetas, bases = [], [], [], [], []
    rej = dict(floor=0, view=0, spacing=0, occluded=0)
    tries = 0
    try:
        while len(world) < n_points and tries < max_tries:
            tries += 1
            angles = rng.uniform(lims[:, 0], lims[:, 1])
            joints, _, _ = ase.forward_kinematics(angles)
            p_base = grip_marker_base(angles)
            if p_base[2] < 10.0 or min(j[2] for j in joints[2:]) < 30.0:
                rej["floor"] += 1
                continue
            p_cal = base_to_cal(p_base, angles[0])
            if world and np.min(np.linalg.norm(np.array(world) - p_cal, axis=1)) < min_spacing:
                rej["spacing"] += 1
                continue
            cam.set_pose(angles, pose_meshes=source == "detected")
            (rc,), (depth,) = cam.project(p_base)
            margin = 10.0
            if depth < 40.0 or not (margin <= rc[0] <= H - 1 - margin and margin <= rc[1] <= W - 1 - margin):
                rej["view"] += 1
                continue
            if source == "detected":
                hit = detect_color_blob(cam.capture(), GRIP_MARKER_RGB)
                if hit is None:
                    rej["occluded"] += 1
                    continue
                meas = hit[0]
            else:
                meas = rc
            world.append(p_cal)
            pixels.append(meas)
            ideal.append(rc)
            thetas.append(angles)
            bases.append(p_base)
            if progress is not None:
                progress(len(world), n_points, angles)
    finally:
        cam.close()
        ase.update_scene(ase.current_angles)

    extra = {"theta": thetas, "base": bases}
    if source == "detected":
        extra["ideal"] = ideal
    cs = CorrespondenceSet(
        world, pixels, (W, H),
        meta={"source": source, "seed": seed, "tries": tries,
              "frame": "arm1 = base rotated by theta1 about Z; p_cal = Rz(-theta1) p_base",
              "marker": f"gripper LED, arm5 home {GRIP_MARKER_LOCAL.tolist()} mm, r={GRIP_MARKER_RADIUS:g} mm"},
        extra=extra)
    stats = {"n": len(cs), "tries": tries, "rejected": rej}
    if len(cs):
        stats["extent_min"] = cs.world.min(0)
        stats["extent_max"] = cs.world.max(0)
        stats["principal_std"] = cs.coplanarity()
        span = cs.pixels.max(0) - cs.pixels.min(0) + 1
        stats["pixel_bbox_frac"] = float(span[0] * span[1] / (W * H))
        if source == "detected":
            err = np.linalg.norm(cs.pixels - np.array(ideal), axis=1)
            stats["detect_err"] = {"mean": float(err.mean()), "rms": float(np.sqrt((err ** 2).mean())),
                                   "max": float(err.max())}
    return cs, stats


def format_calib_stats(stats):
    lines = [f"Points: {stats['n']} from {stats['tries']} poses "
             f"(rejected — floor {stats['rejected']['floor']}, out of view {stats['rejected']['view']}, "
             f"too close {stats['rejected']['spacing']}, "
             f"detector-rejected (occluded/clipped) {stats['rejected']['occluded']})"]
    if stats["n"]:
        lo, hi = stats["extent_min"], stats["extent_max"]
        lines.append("Extent (arm1 frame, mm): " + ", ".join(
            f"{a} {l:+.0f}…{h:+.0f}" for a, l, h in zip("xyz", lo, hi)))
        s = stats["principal_std"]
        lines.append(f"Principal std (mm): {s[0]:.1f} / {s[1]:.1f} / {s[2]:.1f}  (last = out-of-plane)")
        lines.append(f"Image coverage (bbox): {100 * stats['pixel_bbox_frac']:.0f}%")
    if "detect_err" in stats:
        e = stats["detect_err"]
        lines.append(f"Detection vs ideal (px): mean {e['mean']:.3f}, RMS {e['rms']:.3f}, max {e['max']:.3f}")
    return lines
