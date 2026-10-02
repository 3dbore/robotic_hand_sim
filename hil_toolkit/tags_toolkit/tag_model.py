"""
TAG MODEL — AprilTags mounted on the arm
------------------------------------------
Each tag is defined once at the home pose (all joints 0°) in the base frame
and rides rigidly on one link, so its pose for any joint angles is
    link_transforms(angles)[link] @ home_pose.
That is the ground truth a camera's AprilTag pose estimate is compared with.
CALIB_TAG is the tag the camera calibration uses: tag 0, already on the hand.

Tag frame = apriltag.py's object model: X right and Y up as the detector reads
the tag, Z out of the printed face. Corners come out in the detector's order
(TL, TR, BR, BL), matching apriltag.tag_object_points().

Pose estimation needs the black square's side, not the printed image's: the
PNGs in paper/apriltags/ are 664 px with a 410 px black square, so a tag
printed at 30 mm has an 18.5 mm black square.
"""
from dataclasses import dataclass

import numpy as np

from ..paths import PAPER_DIR
from ..sim_loader import ase

TAG_IMAGE_DIR = f"{PAPER_DIR}/apriltags"
PNG_BLACK_FRACTION = 410.0 / 664.0


def frame_from_normal(z, y_hint):
    """Right-handed rotation whose Z is the face normal and Y is y_hint made
    perpendicular to it. Columns = tag X, Y, Z in the base frame."""
    z = np.asarray(z, float) / np.linalg.norm(z)
    y = np.asarray(y_hint, float) - np.dot(y_hint, z) * z
    y /= np.linalg.norm(y)
    return np.column_stack([np.cross(y, z), y, z])


@dataclass(frozen=True)
class MountedTag:
    tag_id: int
    link: int               # index into ase.link_transforms(): 3 = arm3, 5 = arm5
    center: np.ndarray      # home pose, base frame, mm
    R: np.ndarray           # home pose, columns = tag X, Y, Z in the base frame
    print_size: float       # printed image side, mm
    image: str

    @property
    def size(self):
        """Black-square side (mm) — the value pose estimation needs."""
        return self.print_size * PNG_BLACK_FRACTION

    def pose(self, angles):
        """(R, t): tag frame -> base frame for these joint angles (deg), mm."""
        M = ase.link_transforms(np.asarray(angles, float))[self.link]
        return M[:3, :3] @ self.R, M[:3, :3] @ self.center + M[:3, 3]

    def corners(self, angles):
        """(4, 3) black-square corners in the base frame, detector order TL, TR, BR, BL."""
        R, t = self.pose(angles)
        s = self.size / 2.0
        local = np.array([[-s, s, 0], [s, s, 0], [s, -s, 0], [-s, -s, 0]])
        return local @ R.T + t


_DOWN_3 = np.radians(3.0)

TAGS = {
    # arm5 (end effector) bottom face, facing down.
    0: MountedTag(0, link=5, center=np.array([0.0, -97.0, 80.5]),
                  R=frame_from_normal(z=[0, 0, -1], y_hint=[0, 1, 0]),
                  print_size=30.0, image=f"{TAG_IMAGE_DIR}/tag36h11_id00_base_endeffector.png"),
    # arm3 (the link Arm B drives) rear face: faces +Y, tilted 3° down with that face.
    1: MountedTag(1, link=3, center=np.array([0.0, -29.4, 100.0]),
                  R=frame_from_normal(z=[0, np.cos(_DOWN_3), -np.sin(_DOWN_3)], y_hint=[0, 0, 1]),
                  print_size=10.0, image=f"{TAG_IMAGE_DIR}/tag36h11_id01_armb.png"),
}

# ── Camera-calibration target: the tag already on the hand ──────────────────
# Tag 0 on arm5's bottom face rides on the last two axes (J4 pitch, J5 roll).
# It faces the floor at the home pose, so gcode.CALIBRATION_GCODE pitches the
# wrist up (A +20..+40°) to turn it toward the lens; roll (B) tilts it
# sideways, moving the corners out of the arm's plane. Pose estimation and the
# ground truth use the black square: 30 mm print -> 18.52 mm black square.
CALIB_TAG = TAGS[0]
CALIB_TAG_FAMILY = "tag36h11"
CALIB_TAG_SIZE = CALIB_TAG.size            # black-square side, mm (≈18.52)
CALIB_TAG_PRINT = CALIB_TAG.print_size     # printed image side, mm (30)


def calib_tag_spec():
    """The calibration target as plain data, for the ground-truth file."""
    t = CALIB_TAG
    return {"family": CALIB_TAG_FAMILY, "id": t.tag_id, "image": t.image.rsplit("/", 1)[-1],
            "black_square_mm": round(CALIB_TAG_SIZE, 3), "print_size_mm": CALIB_TAG_PRINT,
            "link": "arm5 bottom face (hand, after J4 pitch and J5 roll)",
            "home_center_base_mm": t.center.tolist(),
            "home_R_base": t.R.tolist(),
            "home_corners_base_mm": t.corners(np.zeros(5)).tolist(),
            "corner_order": "TL, TR, BR, BL as the detector reads the tag"}
