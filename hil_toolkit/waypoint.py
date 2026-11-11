"""
TWO-WAY COORDINATE MODE
-------------------------
The toolkit sends one coordinate at a time. The board (6-axis-esp32-twoway) solves the
joints, runs the move, and answers when it has finished. This module holds the toolkit's
side of that:

  fk, ik, plan_move      The same kinematics and planning as the board's kinematics.h, so
                         a whole program can be checked before the arm moves and the sim
                         can follow a move while it runs. Keep the two in step.
  compile_steps          Turns a GCodeProgram into one step per stop, and rejects anything
                         the board would refuse.
  WaypointRunner         Runs those steps, waiting for the board after each one. Gives the
                         same events as GCodeRunner, plus ("send", step).
  TwoWaySession          One command at a time, sent again if no answer arrives.
"""
import math
from dataclasses import dataclass

import numpy as np

from .gcode import GCodeError
from .hardware_mapping import (SERVO_CHANNELS, PULSE_LIMIT_MIN_DEG, PULSE_LIMIT_MAX_DEG, format_cmd)
from .sim_loader import ase

# Keep these in step with kinematics.h and 6-axis-esp32-twoway.ino.
TWOWAY_FIRMWARE_ID = "twoway-1"
JOINT_COUNT = 5
D1_Y, D1_Z = 5.0, 35.5
D2_Y, D2_Z = -47.5, 194.5
D3_Z = -140.0
LIMIT_TOLERANCE_DEG = 0.05
PATH_SAMPLES = 32
PATH_MAX_SAMPLE_STEP_DEG = 30.0
PROFILE_PEAK = 1.875
MIN_DURATION_S = 0.1
MAX_SERVO_SPEED = 60.0   # MAX_SPEED_DEG_PER_SEC
SETTLE_S = 0.15          # SETTLE_MS

PLAN_OK, PLAN_REACH, PLAN_PATH = 0, 1, 2
REASONS = {
    "REACH": "the target is out of reach",
    "PATH": "the straight line leaves the arm's reach",
    "HOME": "the board has not been to the initial pose",
    "CFG": "the board has no axis mapping",
    "BUSY": "the board is still moving",
    "VALUE": "the board rejected a value",
}

REPLY_TIMEOUT_S = 0.6    # a line and its answer take about 0.1 s at 9600 baud
DONE_MARGIN_S = 2.0      # extra wait past the time the board said was left
MAX_TRIES = 4


def _wrap180(deg):
    deg = math.fmod(deg + 180.0, 360.0)
    if deg < 0.0:
        deg += 360.0
    return deg - 180.0


def fk(q):
    """Pose (x, y, z, a, b) of joint angles q: wrist position in mm, pitch and roll in degrees."""
    t2, t23 = math.radians(q[1]), math.radians(q[1] + q[2])
    r = D1_Y + (D2_Y * math.cos(t2) - D2_Z * math.sin(t2)) - D3_Z * math.sin(t23)
    z = D1_Z + (D2_Y * math.sin(t2) + D2_Z * math.cos(t2)) + D3_Z * math.cos(t23)
    t1 = math.radians(q[0])
    return (-r * math.sin(t1), r * math.cos(t1), z, q[1] + q[2] + q[3], q[4])


def _fit_limit(deg, lo, hi):
    for offset in (0.0, -360.0, 360.0):
        v = deg + offset
        if lo - LIMIT_TOLERANCE_DEG <= v <= hi + LIMIT_TOLERANCE_DEG:
            return min(max(v, lo), hi)
    return None


def ik(pose, seed, limits):
    """The valid joint angles closest to `seed`, or None if the pose is out of reach."""
    x, y, z, a, b = pose
    l1, l2 = math.hypot(D2_Y, D2_Z), abs(D3_Z)
    alpha10 = math.atan2(D2_Z, D2_Y)
    gamma0 = math.atan2(D3_Z, 0.0) - alpha10

    rho = math.hypot(x, y)
    if rho < 1e-6:
        branches = [(seed[0], 0.0)]
    else:
        branches = [(math.degrees(math.atan2(x, -y)), -rho), (math.degrees(math.atan2(-x, y)), rho)]

    best, best_score = None, 0.0
    for t1, r in branches:
        q1 = _fit_limit(t1, *limits[0])
        if q1 is None:
            continue
        dy, dz = r - D1_Y, z - D1_Z
        cos_g = (dy * dy + dz * dz - l1 * l1 - l2 * l2) / (2.0 * l1 * l2)
        if abs(cos_g) > 1.0 + 1e-9:
            continue
        g = math.acos(min(max(cos_g, -1.0), 1.0))
        for gamma in (g, -g):
            q3 = _fit_limit(_wrap180(math.degrees(gamma - gamma0)), *limits[2])
            if q3 is None:
                continue
            alpha1 = math.atan2(dz, dy) - math.atan2(l2 * math.sin(gamma), l1 + l2 * math.cos(gamma))
            q2 = _fit_limit(_wrap180(math.degrees(alpha1 - alpha10)), *limits[1])
            if q2 is None:
                continue
            q4 = _fit_limit(a - q2 - q3, *limits[3])
            if q4 is None:
                continue
            q5 = _fit_limit(b, *limits[4])
            if q5 is None:
                continue
            candidate = (q1, q2, q3, q4, q5)
            # Squared: J2+J3+J4 is fixed by the pitch, so plain differences often tie exactly
            # and rounding would pick the elbow.
            score = sum((c - s) ** 2 for c, s in zip(candidate, seed))
            if best is None or score < best_score:
                best, best_score = candidate, score
    return best


def profile(tau):
    """Minimum-jerk: speed and acceleration are both zero at each end of the move."""
    if tau <= 0.0:
        return 0.0
    if tau >= 1.0:
        return 1.0
    return tau * tau * tau * (10.0 + tau * (-15.0 + 6.0 * tau))


def _lerp(a, b, s):
    return tuple(u + (v - u) * s for u, v in zip(a, b))


@dataclass
class Plan:
    linear: bool
    duration: float
    start: tuple
    end: tuple
    start_pose: tuple
    end_pose: tuple
    limits: list

    def angles_at(self, t, q):
        """Joint angles t seconds in; q holds the previous angles."""
        if self.duration <= 0.0 or t >= self.duration:
            return self.end
        s = profile(t / self.duration)
        if not self.linear:
            return _lerp(self.start, self.end, s)
        return ik(_lerp(self.start_pose, self.end_pose, s), q, self.limits) or q


def plan_move(start, target, linear, speed, limits, ratios, max_servo_speed=MAX_SERVO_SPEED):
    """Returns (PLAN_*, Plan or None). See planMove() in kinematics.h."""
    start = tuple(float(v) for v in start)
    start_pose = fk(start)
    end = ik(target, start, limits)
    if end is None:
        return PLAN_REACH, None

    peak = 0.0
    if linear:
        prev = start
        for k in range(1, PATH_SAMPLES + 1):
            q = ik(_lerp(start_pose, target, k / PATH_SAMPLES), prev, limits)
            if q is None:
                return PLAN_PATH, None
            for j in range(JOINT_COUNT):
                step = abs(q[j] - prev[j])
                if step > PATH_MAX_SAMPLE_STEP_DEG:
                    return PLAN_PATH, None
                peak = max(peak, step * PATH_SAMPLES * abs(ratios[j]))
            prev = q
    else:
        peak = max(abs(end[j] - start[j]) * abs(ratios[j]) for j in range(JOINT_COUNT))

    length = math.dist(target[:3], start_pose[:3])
    if peak < 1e-3 and length < 1e-3:
        duration = 0.0
    else:
        duration = PROFILE_PEAK * peak / max_servo_speed
        if speed > 0.0:
            duration = max(duration, length / speed)
        duration = max(duration, MIN_DURATION_S)
    return PLAN_OK, Plan(linear, duration, start, end, start_pose, target, limits)


def wire_pose(pose):
    """A pose as the board will receive it: two decimals."""
    return tuple(round(float(v), 2) for v in pose)


def joint_limits():
    return [tuple(ase.JOINT_LIMITS[j]) for j in range(JOINT_COUNT)]


def joint_ratios():
    ratios = [1.0] * JOINT_COUNT
    for ch in SERVO_CHANNELS:
        if ch.sim_joint is not None:
            ratios[ch.sim_joint] = abs(ch.ratio)
    return ratios


def config_fields():
    """The fields after the sequence number of one C command per joint."""
    fields = []
    for ch in SERVO_CHANNELS:
        if ch.sim_joint is None:
            continue
        lo, hi = ase.JOINT_LIMITS[ch.sim_joint]
        fields.append(f"{ch.sim_joint},{ch.effective_home:g},{ch.sign * ch.ratio:g},"
                      f"{max(ch.range_min, PULSE_LIMIT_MIN_DEG):g},{min(ch.range_max, PULSE_LIMIT_MAX_DEG):g},"
                      f"{lo:.3f},{hi:.3f}")
    return fields


def move_fields(pose, linear, speed, grip):
    x, y, z, a, b = pose
    return (f"{int(linear)},{x:.2f},{y:.2f},{z:.2f},{a:.2f},{b:.2f},"
            f"{speed:.1f},{format_cmd(grip)}")


def poses_match(p, q):
    return math.dist(p[:3], q[:3]) < 0.05 and abs(p[3] - q[3]) < 0.1 and abs(p[4] - q[4]) < 0.1


def compile_steps(program, board_q, sim_q):
    """One step per stop. Starts from where the board is (board_q), going to the sim's pose
    first if the two differ. Raises GCodeError for any move the board would refuse."""
    limits, ratios = joint_limits(), joint_ratios()
    q = tuple(float(v) for v in board_q)
    steps = []

    def add_move(line, pose, linear, speed):
        nonlocal q
        pose = wire_pose(pose)
        speed = round(speed, 1)
        code, plan = plan_move(q, pose, linear, speed, limits, ratios)
        if code == PLAN_REACH:
            raise GCodeError(line, f"X{pose[0]:g} Y{pose[1]:g} Z{pose[2]:g} A{pose[3]:g} B{pose[4]:g} "
                                   "is out of reach")
        if code == PLAN_PATH:
            raise GCodeError(line, f"the straight line to X{pose[0]:g} Y{pose[1]:g} Z{pose[2]:g} "
                                   "leaves the arm's reach")
        if plan.duration <= 0.0:
            return
        steps.append({"kind": "move", "line": line, "pose": pose, "linear": linear,
                      "speed": speed, "plan": plan})
        q = plan.end

    sim_pose = fk(sim_q)
    if program.segments and not poses_match(fk(q), sim_pose):
        add_move(program.segments[0]["line"], sim_pose, False, 0.0)

    for seg in program.segments:
        if seg["kind"] == "move":
            pts, cum, length = seg["points"], seg["cum"], seg["length"]
            for i in range(1, len(pts)):
                f = cum[i] / length if length > 1e-9 else 1.0
                pose = (*pts[i],
                        seg["a"][0] + (seg["a"][1] - seg["a"][0]) * f,
                        seg["b"][0] + (seg["b"][1] - seg["b"][0]) * f)
                add_move(seg["line"], pose, not seg["rapid"], seg["speed"] / 60.0)
        elif seg["kind"] == "grip":
            pose = wire_pose(fk(q))
            steps.append({**seg, "pose": pose, "linear": False, "speed": 0.0, "plan": None})
            q = ik(pose, q, limits) or q   # the board solves the rounded pose
        else:
            steps.append(dict(seg))
    return steps


class WaypointRunner:
    """Runs compile_steps() output. A step that goes to the board stays current until
    step_done() is called with the angles the board reports."""
    def __init__(self, steps, start_q):
        self.steps = steps
        self.idx = 0
        self.t = 0.0
        self.q = tuple(float(v) for v in start_q)
        self.sent = False
        self.done_q = None
        self.pause_requested = False

    @property
    def step(self):
        return self.steps[self.idx] if self.idx < len(self.steps) else None

    @property
    def line(self):
        step = self.step
        return step["line"] if step else None

    @property
    def duration(self):
        total = 0.0
        for s in self.steps:
            if s["kind"] == "move":
                total += s["plan"].duration + SETTLE_S
            elif s["kind"] == "dwell":
                total += s["duration"]
            elif s["kind"] == "grip":
                total += SETTLE_S
        return total

    @property
    def in_flight(self):
        return self.sent and self.done_q is None

    def step_done(self, q):
        self.done_q = tuple(q)

    def _next(self):
        self.idx += 1
        self.t = 0.0
        self.sent = False
        self.done_q = None

    def _set_angles(self, q):
        self.q = tuple(q)
        ase.current_angles = np.array(self.q, dtype=float)

    def tick(self, dt):
        """Advance by dt seconds. Returns events: ("send", step), ("grip", value),
        ("capture", line), ("pause", line) for M0, ("paused", line) once a requested
        pause has reached the end of its stop, ("end", line)."""
        step = self.step
        if step is None:
            return [("end", None)]
        events = []
        kind = step["kind"]
        if kind in ("move", "grip"):
            if not self.sent:
                self.sent = True
                if kind == "grip":
                    events.append(("grip", step["value"]))
                events.append(("send", step))
                return events
            self.t += dt
            if step["plan"] is not None and self.done_q is None:
                self._set_angles(step["plan"].angles_at(self.t, self.q))
            if self.done_q is None:
                return events
            self._set_angles(self.done_q)
            ase.desired_global_pitch = step["pose"][3]
            ase.desired_global_roll = step["pose"][4]
            line = step["line"]
            self._next()
            if self.pause_requested:
                self.pause_requested = False
                events.append(("paused", line))
        elif kind == "dwell":
            self.t += dt
            if self.t >= step["duration"]:
                self._next()
        elif kind == "pause":
            events.append(("pause", step["line"]))
            self._next()
        elif kind == "capture":
            events.append(("capture", step["line"]))
            self._next()
        elif kind == "end":
            events.append(("end", step["line"]))
            self.idx = len(self.steps)
            return events
        if self.step is None:
            events.append(("end", None))
        return events


class TwoWaySession:
    """One command at a time, in order. `send` takes a line and returns False if the link
    failed. A command with no answer in time is sent again under the same number, which
    the board never acts on twice."""
    def __init__(self, send, clock):
        self._send = send
        self._clock = clock
        self._seq = 0
        self._queue = []
        self._current = None
        self.closed = False

    @property
    def busy(self):
        return self._current is not None or bool(self._queue)

    def request(self, command, fields="", on_done=None, on_fail=None):
        """on_done(reply) gets the OK or DONE answer, on_fail(reason) an ERR reason or "TIMEOUT"."""
        self._seq = self._seq % 65535 + 1
        line = f"{command},{self._seq}" + (f",{fields}" if fields else "")
        self._queue.append({"seq": self._seq, "line": line, "on_done": on_done, "on_fail": on_fail, "tries": 0})
        self._pump()

    def abort(self, on_done=None, on_fail=None):
        """Drops everything outstanding and stops the board where it is."""
        self._queue = []
        self._current = None
        self.request("X", on_done=on_done, on_fail=on_fail)

    def close(self, stop_board=False):
        """Ends the session, with one last stop command if asked. A stop that gets lost
        only means the current move runs to its end."""
        was_open, self.closed = not self.closed, True
        self._queue = []
        self._current = None
        if stop_board and was_open:
            self._seq = self._seq % 65535 + 1
            self._send(f"X,{self._seq}")

    def _pump(self):
        if self._current is None and self._queue and not self.closed:
            self._current = self._queue.pop(0)
            self._transmit()

    def _transmit(self):
        current = self._current
        current["tries"] += 1
        current["deadline"] = self._clock() + REPLY_TIMEOUT_S
        self._send(current["line"])

    def tick(self):
        current = self._current
        if current is None or self.closed or self._clock() < current["deadline"]:
            return
        if current["tries"] >= MAX_TRIES:
            self._fail("TIMEOUT")
        else:
            self._transmit()

    def _fail(self, reason):
        current, self._current = self._current, None
        self._queue = []
        if current["on_fail"] is not None:
            current["on_fail"](reason)

    def on_reply(self, reply):
        current = self._current
        if current is None or reply["seq"] != current["seq"]:
            return
        kind = reply["kind"]
        if kind == "ACK":
            current["tries"] = 0
            current["deadline"] = self._clock() + reply.get("t", 0.0) + DONE_MARGIN_S
        elif kind == "ERR":
            self._fail(reply.get("reason", "UNKNOWN"))
        else:
            self._current = None
            if current["on_done"] is not None:
                current["on_done"](reply)
            self._pump()
