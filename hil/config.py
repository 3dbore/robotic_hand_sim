"""
Hardware description of the arm and how each servo maps to the simulation.

Every sim joint maps to its servo with no offsets and no scaling:

    servo = home + sim_angle

and the startup pose of the sim (all joints 0°) is the servo "home" of every
axis. A joint can only go where both the sim and its servo can follow, so the
limit used everywhere is the sim limit intersected with the servo range.

Two per-axis corrections are applied only to the angle put on the wire, so
neither changes the limits and neither can lock an axis:
    trim     shifts the physical home a few degrees (wire = servo + trim);
             the whole range moves with it, so Arm B (home 180) with trim +4
             sends 184 at home and 4 at the other end
    invert   a servo mounted the other way round (wire = 180 - that)
The wire value is kept inside what the MG996R still obeys, -15..195
(about 330..2670 us); past that it takes the pulse as invalid and goes limp.
The base is a stepper, not a servo: it takes -90..270 (180 either way of home).

Behind the base (world Y > 0, where the tank is) the arm must stay above
TANK_Z_MM; see kinematics.unsafe().
"""
import json
import os
from dataclasses import dataclass
from typing import Optional

PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SIM_END_FILE = os.path.join(PROJECT_ROOT, "arm-sim-end.py")
SETTINGS_FILE = os.path.join(PROJECT_ROOT, "calibration", "hil_settings.json")
BRAND_FONT_FILE = os.path.join(PROJECT_ROOT, "fonts", "ZenDots-Regular.ttf")
APP_ICON_FILE = os.path.join(PROJECT_ROOT, "paper", "img", "ELIO_ICON.png")

# Hand camera: MaixCam2 + mount, rides on Arm A (base turn + Arm A tilt about J2).
# hand_cam.stl is arms/HandKinematics-Part.step exported as-is (FreeCAD), so it
# is in the STEP's frame: sim X/Y/Z = STEP Y/Z/X, then its datum sits at
# CAM_PART_POS. Lens and positions are sim world mm at the home pose.
CAM_PART_FILE = os.path.join(PROJECT_ROOT, "arms", "hand_cam.stl")
CAM_PART_ROT = ((0, 1, 0), (0, 0, 1), (1, 0, 0))
CAM_PART_POS = (0.0, 35.32, 82.12)
CAM_LENS_POS = (-52.0, -5.1, 137.2)
CAM_VIEW_DIR = (0.0, -1.0, 0.0)     # looks along -Y, up = +Z

SERIAL_BAUD = 9600
TICK_MS = 50                 # motion update + one stream frame (20 Hz)
RELEASE_EVERY_TICKS = 4      # while motors are off, a release frame every 200 ms
STAT_STALE_S = 3.0           # warn when the board's 1 s STAT report goes missing

# Fraction of the firmware's per-axis speed limit a planned move may use, so
# the firmware's own limiter never has to clip (and distort) the path.
SPEED_MARGIN = 0.8

# The end effector (wrist pitch joint) is never sent below the table.
Z_FLOOR_MM = 0.0

# The tank behind the base: anything of the arm past the base's ±90° (world
# Y > 0) has to be at least this high.
TANK_Z_MM = 150.0

# Furthest angle sent to a servo. Measured on Arm B: 195 (~2670 us) holds,
# 196 makes the MG996R go limp. Must stay inside the firmware's
# ANGLE_LIMIT_MIN/MAX (-36..216 in the .ino).
WIRE_MIN, WIRE_MAX = -15.0, 195.0

# Largest home trim allowed per axis, degrees. With servo ranges inside 0..180
# this keeps every trimmed angle inside WIRE_MIN..WIRE_MAX; for more, move the
# servo horn a spline tooth (~14 deg on an MG996R) instead.
TRIM_MAX = 15.0


@dataclass
class Axis:
    name: str
    channel: str               # PCA label(s) / driver, for display
    joint: Optional[int]       # sim joint index; None = not in the sim (gripper)
    home: float                # servo angle at the sim's 0° pose
    servo_lo: float            # servo range this axis may use
    servo_hi: float
    speed: float               # firmware speed limit, deg/s (servo degrees)
    sim_lo: float = 0.0        # filled in by effective limits below
    sim_hi: float = 0.0
    inverted: bool = False
    trim: float = 0.0          # home correction, degrees
    wire_lo: float = WIRE_MIN  # what may be put on the wire
    wire_hi: float = WIRE_MAX

    def to_servo(self, sim_angle):
        return self.home + sim_angle

    def to_sim(self, servo_angle):
        return servo_angle - self.home

    def wire(self, servo_angle):
        """Angle actually put on the wire (trim and mounting direction applied)."""
        v = servo_angle + self.trim
        v = 180.0 - v if self.inverted else v
        return min(self.wire_hi, max(self.wire_lo, v))

    def unwire(self, wire_angle):
        v = 180.0 - wire_angle if self.inverted else wire_angle
        return v - self.trim


# Serial field order of the firmware: F0..F5.
AXES = [
    Axis("Gripper",     "PCA 16",     None,  180.0,  60.0, 180.0, 90.0),
    Axis("Wrist Yaw",   "PCA 11",     4,      90.0,   0.0, 180.0, 45.0),
    Axis("Wrist Pitch", "PCA 8",      3,       0.0,   0.0, 180.0, 45.0),
    Axis("Arm B",       "PCA 3",      2,     180.0,   0.0, 180.0, 30.0),
    Axis("Arm A",       "PCA 1 + 2",  1,      90.0,   0.0, 180.0, 30.0),
    # NEMA17 stepper: 180 either way of home, never continuous (cable).
    Axis("Arm Base",    "NEMA17",     0,      90.0, -90.0, 270.0, 40.0,
         wire_lo=-90.0, wire_hi=270.0),
]
AXIS_BY_NAME = {a.name: a for a in AXES}
GRIPPER = AXES[0]
GRIP_OPEN, GRIP_CLOSED = 180.0, 75.0
# Correction for the closed position only (open stays at GRIP_OPEN), degrees.
GRIP_OFFSET_MAX = 15.0
grip_closed_offset = 0.0


def grip_closed():
    """Servo angle the Close button sends: GRIP_CLOSED plus its offset."""
    return GRIP_CLOSED + grip_closed_offset


def grip_percent(servo):
    span = GRIP_OPEN - grip_closed()
    return 100.0 * (servo - grip_closed()) / span if span else 0.0

# Display order: base first, as you read the arm from the floor up.
DISPLAY_ORDER = [AXES[i] for i in (5, 4, 3, 2, 1, 0)]

# Sim limits this toolkit uses instead of arm-sim-end.py's. The yaw servo
# turns ±90° around its centre, not the sim's 0..-180. The stepper base turns
# ±180°, not the sim's ±90°.
SIM_LIMIT_OVERRIDES = {0: (-180.0, 180.0), 4: (-90.0, 90.0)}


def apply_sim_limits(sim_limits):
    """Effective joint limits = sim limit ∩ servo range, per axis."""
    for ax in AXES:
        if ax.joint is None:
            ax.sim_lo, ax.sim_hi = ax.servo_lo, ax.servo_hi
            continue
        lo, hi = SIM_LIMIT_OVERRIDES.get(ax.joint, sim_limits[ax.joint])
        ax.sim_lo = max(lo, ax.to_sim(ax.servo_lo))
        ax.sim_hi = min(hi, ax.to_sim(ax.servo_hi))
        if ax.sim_lo > ax.sim_hi:
            raise ValueError(f"{ax.name}: sim and servo ranges do not overlap")


def joint_limits():
    """[(lo, hi)] for sim joints 0..4."""
    lims = [None] * 5
    for ax in AXES:
        if ax.joint is not None:
            lims[ax.joint] = (ax.sim_lo, ax.sim_hi)
    return lims


def load_settings():
    try:
        with open(SETTINGS_FILE, "r", encoding="utf-8") as f:
            data = json.load(f)
    except (OSError, ValueError):
        return {}
    for name, inv in data.get("inverted", {}).items():
        if name in AXIS_BY_NAME:
            AXIS_BY_NAME[name].inverted = bool(inv)
    for name, t in data.get("trim", {}).items():
        if name in AXIS_BY_NAME:
            try:
                AXIS_BY_NAME[name].trim = max(-TRIM_MAX, min(TRIM_MAX, float(t)))
            except (TypeError, ValueError):
                pass
    global grip_closed_offset
    try:
        grip_closed_offset = max(-GRIP_OFFSET_MAX, min(GRIP_OFFSET_MAX,
                                 float(data.get("grip_closed_offset", 0.0))))
    except (TypeError, ValueError):
        grip_closed_offset = 0.0
    return data


def save_settings(extra=None):
    data = {"inverted": {a.name: a.inverted for a in AXES},
            "trim": {a.name: a.trim for a in AXES},
            "grip_closed_offset": grip_closed_offset}
    if extra:
        data.update(extra)
    os.makedirs(os.path.dirname(SETTINGS_FILE), exist_ok=True)
    with open(SETTINGS_FILE, "w", encoding="utf-8") as f:
        json.dump(data, f, indent=2)
