"""
TAG MODEL — AprilTags mounted on the arm
------------------------------------------
Each tag is defined once at the home pose (all joints 0°) in the base frame
and rides rigidly on one link, so its pose for any joint angles is
    link_transforms(angles)[link] @ home_pose.
That is the ground truth a camera's AprilTag pose estimate is compared with.

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
