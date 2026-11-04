"""
HARDWARE MAPPING (Kinematics.xlsx)
------------------------------------
Sim <-> servo mapping: the startup pose of the sim (all joints 0 deg) and of
the hardware (servo "Initial Position") should be identical, so each axis is
mapped purely as a signed offset from its initial position:
    servo = home + offset + sign * ratio * sim_angle
`offset` is the per-machine correction measured in the Axis Offsets window; it
shifts the hardware only, the sim's pose (and so the kinematics) is untouched.

Also carries the fixed camera/marker geometry used by both live rendering
and the calibration correspondence generator, and the effective (sim n
hardware) joint limits.
"""
import json
import math
import os

import numpy as np

from .paths import AXIS_OFFSETS_FILE, CALIB_DIR
from .sim_loader import ase
from .theme import C

# Smallest servo step on the wire. 6-axis-esp32 accepts one decimal; set this to
# 1.0 when driving the Uno firmware (6-axis.ino), which only accepts integers.
COMMAND_RESOLUTION_DEG = 0.5

# The furthest command the ESP32 firmware will turn into a pulse (100 and 2900 us at
# 500-2500 us = 0-180°). Keep in step with ANGLE_LIMIT_MIN/MAX in 6-axis-esp32.ino.
PULSE_LIMIT_MIN_DEG = -36
PULSE_LIMIT_MAX_DEG = 216


def quantize(deg):
    return round(deg / COMMAND_RESOLUTION_DEG) * COMMAND_RESOLUTION_DEG


def format_cmd(deg):
    """Serial field text: "90" for whole degrees, "90.5" otherwise."""
    return f"{deg:g}"


class ServoChannel:
    """One serial field. Maps a sim joint angle (deg, relative to startup pose)
    to an absolute servo command:  servo = home + offset + sign * ratio * sim_angle.

    servo_min, servo_max: operating range at zero offset, in servo degrees.
    offset: servo degrees, corrects the real initial pose to match the sim's. It has
            no limit of its own and shifts the whole operating range with it.
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
        self.offset = 0.0
        self.servo_min = servo_min
        self.servo_max = servo_max
        self.sign = sign
        self.ratio = ratio
        self.note = note

    @property
    def effective_home(self):
        return self.home + self.offset

    @property
    def range_min(self):
        return self.servo_min + self.offset

    @property
    def range_max(self):
        return self.servo_max + self.offset

    def past_pulse_limit(self):
        return self.range_min < PULSE_LIMIT_MIN_DEG or self.range_max > PULSE_LIMIT_MAX_DEG

    def set_offset(self, deg):
        self.offset = float(quantize(deg))

    def command(self, raw):
        """A servo angle as it goes on the wire: clamped to the shifted range and the
        pulse limit, then quantized."""
        in_range = min(max(raw, self.range_min), self.range_max)
        return quantize(min(max(in_range, PULSE_LIMIT_MIN_DEG), PULSE_LIMIT_MAX_DEG))

    def raw_servo(self, sim_deg):
        return self.effective_home + self.sign * self.ratio * sim_deg

    def to_servo(self, sim_deg):
        """Returns (command, clipped)."""
        raw = self.raw_servo(sim_deg)
        cmd = self.command(raw)
        return cmd, abs(raw - cmd) > 0.5

    def sim_range(self):
        """Hardware-reachable range expressed in sim degrees. An offset leaves this
        unchanged until the pulse limit cuts in. Sim 0° always stays inside, so an
        offset never drags the sim off its initial pose."""
        lo = max(self.range_min, PULSE_LIMIT_MIN_DEG)
        hi = min(self.range_max, PULSE_LIMIT_MAX_DEG)
        if lo > hi:
            return 0.0, 0.0
        a = (lo - self.effective_home) / (self.sign * self.ratio)
        b = (hi - self.effective_home) / (self.sign * self.ratio)
        return min(a, b, 0.0), max(a, b, 0.0)


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


def load_axis_offsets():
    """Reads the saved offsets into SERVO_CHANNELS. A missing file means no offsets;
    a bad one is reported and ignored, never fatal. Returns a list of warnings."""
    try:
        with open(AXIS_OFFSETS_FILE, encoding="utf-8") as f:
            saved = json.load(f)["offsets_deg"]
        items = [(ch, saved.get(ch.name, 0.0)) for ch in SERVO_CHANNELS]
        if not all(isinstance(v, (int, float)) and math.isfinite(v) for _, v in items):
            raise ValueError("non-numeric offset")
    except FileNotFoundError:
        return []
    except (OSError, ValueError, KeyError, TypeError, AttributeError) as e:
        return [f"Ignoring {os.path.basename(AXIS_OFFSETS_FILE)}: {e}"]

    warnings = []
    for ch, value in items:
        ch.set_offset(value)
        if ch.past_pulse_limit():
            warnings.append(f"{ch.name}: saved offset {ch.offset:+g}° puts its range past the "
                            f"{PULSE_LIMIT_MIN_DEG}…{PULSE_LIMIT_MAX_DEG}° pulse limit; "
                            "commands stop at the limit.")
    return warnings


def save_axis_offsets():
    """Writes the current offsets; the replace keeps a crash from truncating the file."""
    os.makedirs(CALIB_DIR, exist_ok=True)
    tmp = AXIS_OFFSETS_FILE + ".tmp"
    with open(tmp, "w", encoding="utf-8") as f:
        json.dump({"offsets_deg": {ch.name: ch.offset for ch in SERVO_CHANNELS}}, f, indent=2)
    os.replace(tmp, AXIS_OFFSETS_FILE)


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
