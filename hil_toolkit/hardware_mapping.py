"""
HARDWARE MAPPING (Kinematics.xlsx)
------------------------------------
Sim <-> servo mapping: the startup pose of the sim (all joints 0 deg) and of
the hardware (servo "Initial Position") are identical, so each axis is
mapped purely as a signed offset from its initial position:
    servo = home + sign * ratio * sim_angle

Also carries the fixed camera/marker geometry used by both live rendering
and the calibration correspondence generator, and the effective (sim n
hardware) joint limits.
"""
import numpy as np

from .sim_loader import ase
from .theme import C


class ServoChannel:
    """One serial field. Maps a sim joint angle (deg, relative to startup pose)
    to an absolute servo command:  servo = home + sign * ratio * sim_angle.

    sign  : +1 when increasing servo angle turns the joint the same way as the
            sim's positive (CCW) rotation. Flip with the INV toggle in the GUI
            if an axis moves the wrong way during bring-up.
    ratio : servo degrees per joint degree (1.0 = direct drive).
    """
    def __init__(self, name, pin, sim_joint, home, servo_min, servo_max,
                 sign=1, ratio=1.0, note=""):
        self.name = name
        self.pin = pin
        self.sim_joint = sim_joint
        self.home = home
        self.servo_min = servo_min
        self.servo_max = servo_max
        self.sign = sign
        self.ratio = ratio
        self.note = note

    def raw_servo(self, sim_deg):
        return self.home + self.sign * self.ratio * sim_deg

    def to_servo(self, sim_deg):
        """Returns (command, clipped)."""
        raw = self.raw_servo(sim_deg)
        clamped = self.servo_min if raw < self.servo_min else self.servo_max if raw > self.servo_max else raw
        cmd = int(round(clamped))
        return cmd, abs(raw - cmd) > 0.5

    def sim_range(self):
        """Hardware-reachable range expressed in sim degrees."""
        a = (self.servo_min - self.home) / (self.sign * self.ratio)
        b = (self.servo_max - self.home) / (self.sign * self.ratio)
        return min(a, b), max(a, b)


# Order == serial field order of 6-axis.ino
SERVO_CHANNELS = [
    ServoChannel("Gripper", "D2", None, home=10, servo_min=10, servo_max=90,
                 note="Not modelled in sim. 10 = open."),
    ServoChannel("Wrist Yaw", "D3", 4, home=90, servo_min=0, servo_max=180,
                 note="Real: 90° CW / 90° CCW from initial."),
    # sign=-1 verified on hardware: +1 made Pitch− move the real wrist Pitch+.
    ServoChannel("Wrist Pitch", "D4", 3, home=90, servo_min=0, servo_max=180, sign=-1,
                 note="Real: 90° CW / 90° CCW from initial."),
    # Range written 180→0 in the sheet: decreasing servo angle = CW = sim negative.
    # Full 180° CW is usable from the initial (face-down) position.
    ServoChannel("Arm B", "D5", 2, home=180, servo_min=0, servo_max=180,
                 note="Real: 180° CW only from initial."),
    # sign=-1 verified on hardware: +1 made +Z (driven mostly by J2) move the real arm −Z.
    ServoChannel("Arm A", "D6/D7", 1, home=90, servo_min=10, servo_max=180, sign=-1,
                 note="Pin 7 mirrored by firmware (180 - D6)."),
    # Planetary gear; initial stays at 90. Change `ratio` if the gear is not 1:1.
    ServoChannel("Arm Base", "D8", 0, home=90, servo_min=0, servo_max=180,
                 note="Real: 90° CW / 90° CCW from initial."),
]
GRIPPER = SERVO_CHANNELS[0]

# The sim's own limits (arm-sim-end.py) before hardware limits are applied.
SIM_BASE_LIMITS = [tuple(l) for l in ase.JOINT_LIMITS]
# Kinematics.xlsx: the sim's wrist roll only goes 180° CW, the real wrist yaw
# goes 90° each way — follow the hardware.
SIM_LIMIT_OVERRIDES = {4: (-90.0, 90.0)}

SERIAL_BAUD = 9600
BOARD_RESET_MS = 2000    # Uno auto-resets when the port opens
STREAM_INTERVAL_MS = 50  # 20 Hz; firmware limits speed to 60°/s anyway

# Camera: fixed to arm1 (follows only the J1 base rotation, no pitch).
# Lens position at the initial pose, in mm; looks straight ahead (-Y), up = +Z.
CAM_LENS_POS = np.array([-54.00, -51.10, 86.20])
CAM_VIEW_DIR = np.array([0.0, -1.0, 0.0])
CAM_UP_DIR = np.array([0.0, 0.0, 1.0])
# Lens spec: D90° H81° V51°. VTK's view_angle is the vertical FoV; the widget
# aspect ratio then sets the horizontal FoV (tan(81/2) / tan(51/2) ≈ 1.79).
CAM_FOV_H, CAM_FOV_V, CAM_FOV_D = 81.0, 51.0, 90.0
CAM_ASPECT = np.tan(np.radians(CAM_FOV_H / 2)) / np.tan(np.radians(CAM_FOV_V / 2))

# Calibration: fixed simulated sensor resolution (independent of the GUI widget).
CALIB_IMAGE_H = 720
CALIB_IMAGE_W = int(round(CALIB_IMAGE_H * CAM_ASPECT))
# Gripper LED: on arm5's camera-side (-X) face, home-pose coordinates (mm). It sits
# ~27 mm off the roll axis, so wrist roll carries it out of the arm's plane — the
# only source of non-coplanar points for a camera riding on arm1.
GRIP_MARKER_LOCAL = np.array([-26.6, -100.0, 88.2])
GRIP_MARKER_RADIUS = 3.0
GRIP_MARKER_RGB = (255, 0, 128)

# Rear-view style guides: floor distance (mm, ground distance ahead of the lens)
# and the band colour that starts at it; the last entry only closes the far band.
FLOOR_Z = 0.0
GUIDE_BANDS = ((200, "error"), (300, "warning"), (400, "primary"), (600, None))
GUIDE_HALF_WIDTH = 60.0   # mm either side of the camera's ground track

JOINT_NAMES = ["J1 Base", "J2 Arm A", "J3 Arm B", "J4 Pitch", "J5 Roll"]
JOINT_COLORS = [C["primary"], C["tertiary_container"], C["warning"], "#c25700", C["error"]]


def apply_effective_limits():
    """Effective sim limit = sim limit ∩ hardware reach. Written into
    ase.JOINT_LIMITS so the IK solver never asks for an unreachable pose.
    Returns a list of warnings."""
    warnings = []
    for ch in SERVO_CHANNELS:
        j = ch.sim_joint
        if j is None:
            continue
        s_lo, s_hi = SIM_LIMIT_OVERRIDES.get(j, SIM_BASE_LIMITS[j])
        h_lo, h_hi = ch.sim_range()
        lo, hi = max(s_lo, h_lo), min(s_hi, h_hi)
        if lo > hi:
            warnings.append(f"{ch.name}: sim and hardware ranges do not overlap — joint locked at 0°.")
            lo = hi = 0.0
        ase.JOINT_LIMITS[j] = (lo, hi)
    ase.current_angles = ase.clamp_angles(np.array(ase.current_angles, dtype=float))
    return warnings
