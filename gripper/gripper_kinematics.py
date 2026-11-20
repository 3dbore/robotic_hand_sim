"""
Gripper kinematics (Y-type, 2-jaw radial)
-----------------------------------------
Pure geometry, no GUI. All lengths in mm, angles in degrees, in the frame of
gripper_kinematics.FCStd: Y runs along the gripper (wrist -> jaws), X across
it (the drawn jaw is on the -X side), Z is the plate stack.

Chain (gripper-kinematics.jpg):
    wrist  --Pitch (about X, through PITCH_PIVOT)-->  wrist part
           --Yaw   (about Y, through YAW_PIVOT)  -->  body, servo, both jaws

Each jaw is a four-bar linkage:
    A = Rod Geared pivot (driven: its gear meshes with the servo gear)
    B = Rod pivot        (idler)
    C = finger hole on the Rod Geared end, D = finger hole on the Rod end
    |AC| = |BD| = 30, |CD| = finger hole spacing (15.1 x 16.3 -> 22.2)
The servo gear and the Rod Geared gear are the same size (both 8.1 mm tip
radius), so the rod turns as much as the servo, the other way round.
The second jaw is the mirror image of the first across the Y axis (X -> -X).

The home pose (servo 0) is the pose in the FCStd file: Rod Geared pointing
straight out along -X.
"""
import math
import os

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))

# ── parts: STL file and FreeCAD placement (position, rotation axis, angle) ──
PARTS = {
    "wrist":      ("Robot Arm Wrist.stl",       (0.0, 32.0, 0.0),    (0, 0, 1), 180.0),
    "body":       ("Gripper Body.stl",          (0.0, 103.0, 6.0),   (0, 0, 1), 0.0),
    "body2":      ("Gripper Body2 .stl",        (0.0, 103.0, -19.0), (0, 0, 1), 0.0),
    "servo":      ("Gripper+Servo+Gear.stl",    (-10.0, 103.0, 0.0), (0, 1, 0), 180.0),
    "rod_geared": ("Gripper Rod Geared .stl",   (-25.0, 103.0, -14.0), (0, 0, 1), 180.0),
    "rod":        ("Gripper Rod.stl",           (-7.4, 118.0, -14.0), (0, 0, 1), 180.0),
    "finger":     ("Gripper Finger.stl",        None, None, None),   # placed by the linkage
}

# ── joints ──
PITCH_PIVOT = np.array([0.0, 22.0, 0.0])     # wrist pitch, axis +X (drawing: 22 mm from the wrist end)
PITCH_AXIS = np.array([1.0, 0.0, 0.0])
YAW_PIVOT = np.array([0.0, 63.0, 0.0])       # gripper yaw, axis +Y (drawing: 63 mm)
YAW_AXIS = np.array([0.0, 1.0, 0.0])
PITCH_RANGE = (-90.0, 90.0)
YAW_RANGE = (-90.0, 90.0)

# ── linkage (XY plane of the left jaw) ──
SERVO_GEAR = np.array([-10.0, 103.0])        # servo output gear centre
A = np.array([-25.0, 103.0])                 # Rod Geared pivot (15 mm from the servo gear)
B = np.array([-7.4, 118.0])                  # Rod pivot
L_ROD_GEARED = 30.0
L_ROD = 30.0
GEAR_RATIO = -1.0                            # rod angle change per servo angle change
HOME_CRANK = 180.0                           # Rod Geared direction at servo 0 (points -X)
# Finger holes in the finger STL's own frame
FINGER_C = np.array([-20.2, 22.2])           # on the Rod Geared end
FINGER_D = np.array([-5.1, 5.9])             # on the Rod end
FINGER_Z = -9.0                              # finger sits in the rods' clevis slot (rod z 5..15)
L_FINGER = float(np.linalg.norm(FINGER_D - FINGER_C))
FINGER_HOLE_ANGLE = math.degrees(math.atan2(*(FINGER_D - FINGER_C)[::-1]))


def stl_path(name):
    return os.path.join(HERE, PARTS[name][0])


# ─────────────────────────────────────────────────────────────────────────────
# 4x4 transforms
# ─────────────────────────────────────────────────────────────────────────────
def rot(axis, deg):
    """4x4 rotation about a unit axis through the origin."""
    x, y, z = np.asarray(axis, float) / np.linalg.norm(axis)
    c, s = math.cos(math.radians(deg)), math.sin(math.radians(deg))
    C = 1 - c
    M = np.eye(4)
    M[:3, :3] = [[c + x * x * C, x * y * C - z * s, x * z * C + y * s],
                 [y * x * C + z * s, c + y * y * C, y * z * C - x * s],
                 [z * x * C - y * s, z * y * C + x * s, c + z * z * C]]
    return M


def trans(v):
    M = np.eye(4)
    M[:3, 3] = v
    return M


def rot_about(point, axis, deg):
    p = np.asarray(point, float)
    if p.size == 2:
        p = np.array([p[0], p[1], 0.0])
    return trans(p) @ rot(axis, deg) @ trans(-p)


def placement(name):
    _, pos, axis, ang = PARTS[name]
    return trans(pos) @ rot(axis, ang)


MIRROR_X = np.diag([-1.0, 1.0, 1.0, 1.0])


def apply(M, pts):
    pts = np.asarray(pts, float)
    return pts @ M[:3, :3].T + M[:3, 3]


# ─────────────────────────────────────────────────────────────────────────────
# Four-bar
# ─────────────────────────────────────────────────────────────────────────────
def _circle_hits(c0, r0, c1, r1):
    d = float(np.linalg.norm(c1 - c0))
    if d < 1e-9 or d > r0 + r1 or d < abs(r0 - r1):
        return None
    a = (r0 * r0 - r1 * r1 + d * d) / (2 * d)
    h = math.sqrt(max(r0 * r0 - a * a, 0.0))
    u = (c1 - c0) / d
    n = np.array([-u[1], u[0]])
    m = c0 + a * u
    return m + h * n, m - h * n


def solve_jaw(servo_deg):
    """Left jaw at a servo angle (0 = FCStd pose). Returns a dict, or None if
    the linkage can't close at that angle."""
    crank = HOME_CRANK + GEAR_RATIO * servo_deg
    C = A + L_ROD_GEARED * np.array([math.cos(math.radians(crank)), math.sin(math.radians(crank))])
    hits = _circle_hits(C, L_FINGER, B, L_ROD)
    if hits is None:
        return None
    # open branch: the finger stays roughly parallel to its home direction
    best = None
    for D in hits:
        tilt = math.degrees(math.atan2(*(D - C)[::-1])) - FINGER_HOLE_ANGLE
        tilt = (tilt + 180.0) % 360.0 - 180.0
        if best is None or abs(tilt - 90.0) < abs(best[1] - 90.0):
            best = (D, tilt)
    D, tilt = best
    rocker = math.degrees(math.atan2(*(D - B)[::-1]))
    return {"servo": servo_deg, "crank": crank, "rocker": rocker, "finger_rot": tilt,
            "A": A, "B": B, "C": C, "D": D}


def finger_matrix(jaw):
    """Finger STL frame -> gripper frame (yaw/pitch not applied)."""
    M = rot((0, 0, 1), jaw["finger_rot"])
    c = apply(M, [[*FINGER_C, 0.0]])[0]
    M[:3, 3] = [jaw["C"][0] - c[0], jaw["C"][1] - c[1], FINGER_Z]
    return M


def part_matrices(jaw):
    """Gripper-frame matrices of the moving linkage parts of the LEFT jaw."""
    d_crank = jaw["crank"] - HOME_CRANK
    d_rocker = jaw["rocker"] - 180.0
    return {
        "servo": rot_about(SERVO_GEAR, (0, 0, 1), d_crank / GEAR_RATIO) @ placement("servo"),
        "rod_geared": rot_about(A, (0, 0, 1), d_crank) @ placement("rod_geared"),
        "rod": rot_about(B, (0, 0, 1), d_rocker) @ placement("rod"),
        "finger": finger_matrix(jaw),
    }


def wrist_matrices(pitch_deg, yaw_deg):
    """(wrist frame, gripper frame): pitch moves the wrist part; the gripper
    rides on pitch then yaw."""
    Mp = rot_about(PITCH_PIVOT, PITCH_AXIS, pitch_deg)
    My = Mp @ rot_about(YAW_PIVOT, YAW_AXIS, yaw_deg)
    return Mp, My


class Gripper:
    """Servo range and jaw gap, worked out once from the finger mesh."""

    def __init__(self, finger_points):
        self.finger_pts = np.asarray(finger_points, float)
        self.tip_local = self.finger_pts[np.argmax(self.finger_pts[:, 0])]   # blade tip
        self.reach_limit = self._limit(+1)    # furthest back the linkage can fold
        self.closed_limit = self._closed()    # servo angle where the jaws touch
        # Past the widest gap the fingers only swing back into the body, so
        # "fully open" is the widest gap, not the end of the linkage's travel.
        grid = np.arange(self.reach_limit, self.closed_limit, 0.25)
        self.open_limit = float(grid[int(np.argmax([self.gap(solve_jaw(s)) for s in grid]))])
        self.max_gap = self.gap(solve_jaw(self.open_limit))

    def finger_world(self, jaw):
        return apply(finger_matrix(jaw), self.finger_pts)

    def gap(self, jaw):
        """Distance between the two fingers (mm) — they are mirror images, so
        twice the left finger's distance from the centre line."""
        return -2.0 * float(self.finger_world(jaw)[:, 0].max())

    def tip(self, jaw):
        return apply(finger_matrix(jaw), [self.tip_local])[0]

    def _limit(self, sign, step=0.25):
        s = 0.0
        while solve_jaw(s - sign * step) is not None and abs(s) < 180:
            s -= sign * step
        return s

    def _closed(self):
        lo, hi = 0.0, None
        s = 0.0
        while True:                            # walk to the first angle with gap <= 0
            s += 1.0
            jaw = solve_jaw(s)
            if jaw is None or self.gap(jaw) <= 0.0:
                hi = s
                break
            lo = s
            if s > 180:
                return lo
        for _ in range(40):                    # then bisect
            m = 0.5 * (lo + hi)
            jaw = solve_jaw(m)
            if jaw is not None and self.gap(jaw) > 0.0:
                lo = m
            else:
                hi = m
        return lo

    def servo_for_percent(self, pct):
        """0 % = jaws touching, 100 % = fully open."""
        return self.closed_limit + (self.open_limit - self.closed_limit) * pct / 100.0

    def percent_for_servo(self, s):
        span = self.open_limit - self.closed_limit
        pct = 100.0 * (s - self.closed_limit) / span if span else 0.0
        return min(max(pct, 0.0), 100.0)      # folded back past the widest gap still reads 100 %
