"""
Joint-space motion: every move is a synchronised smoothstep from where the
command is now to a goal, timed so no axis needs more than SPEED_MARGIN of
its firmware speed limit. All axes start and arrive together.

State is in servo degrees for all six axes (field order of config.AXES);
the sim angles are derived from it.
"""
import time

import numpy as np

from .config import AXES, SPEED_MARGIN

N = len(AXES)
_SPEED = np.array([a.speed for a in AXES]) * SPEED_MARGIN


def leg_time(a, b):
    """Seconds a move from servo pose a to b takes (no planner detours)."""
    delta = np.abs(np.asarray(b, float) - np.asarray(a, float))
    return max(float(np.max(1.5 * delta / _SPEED)), 0.15) if np.any(delta > 1e-6) else 0.0


def servo_limits():
    lo = np.array([a.to_servo(a.sim_lo) if a.joint is not None else a.servo_lo for a in AXES])
    hi = np.array([a.to_servo(a.sim_hi) if a.joint is not None else a.servo_hi for a in AXES])
    return lo, hi


class Motion:
    def __init__(self):
        self.home = np.array([a.home for a in AXES], float)
        self.lo, self.hi = servo_limits()
        self.cmd = self.home.copy()
        self.goal = self.home.copy()
        self._start = self.cmd.copy()
        self._t0 = 0.0
        self._dur = 0.0
        self.queue = []             # further legs after the current one (servo goals)

    @property
    def moving(self):
        return self._dur > 0.0

    def clamp(self, servo):
        return np.minimum(np.maximum(servo, self.lo), self.hi)

    def go_path(self, goals):
        """Move through `goals` (servo angle arrays) one leg after another."""
        self.go(goals[0])
        self.queue = [self.clamp(np.asarray(g, float)) for g in goals[1:]]
        if self._dur == 0.0 and self.queue:
            self.go_path(self.queue)

    def go(self, goal):
        """Start a synchronised move to `goal` (six servo angles)."""
        self.queue = []
        goal = self.clamp(np.asarray(goal, float))
        delta = np.abs(goal - self.cmd)
        # smoothstep peaks at 1.5x its mean speed
        dur = float(np.max(1.5 * delta / _SPEED))
        self.goal = goal
        self._start = self.cmd.copy()
        self._t0 = time.monotonic()
        self._dur = max(dur, 0.15) if np.any(delta > 1e-6) else 0.0
        if self._dur == 0.0:
            self.cmd = goal.copy()

    def set_axis(self, i, servo):
        g = self.final_goal.copy()
        g[i] = servo
        self.go(g)

    def stop(self):
        """Freeze at the current command."""
        self.queue = []
        self.goal = self.cmd.copy()
        self._dur = 0.0

    def jump(self, servo):
        """Set the command immediately, no ramp (e.g. motors are off)."""
        self.queue = []
        self.cmd = self.clamp(np.asarray(servo, float))
        self.goal = self.cmd.copy()
        self._dur = 0.0

    def tick(self):
        """Advance the command; returns True if it changed."""
        if self._dur <= 0.0:
            return False
        s = (time.monotonic() - self._t0) / self._dur
        if s >= 1.0:
            self.cmd = self.goal.copy()
            self._dur = 0.0
            if self.queue:
                self.go_path(self.queue)
            return True
        s = s * s * (3.0 - 2.0 * s)
        self.cmd = self._start + (self.goal - self._start) * s
        return True

    def remaining(self):
        t = 0.0 if self._dur <= 0.0 else max(0.0, self._dur - (time.monotonic() - self._t0))
        prev = self.goal
        for g in self.queue:
            t += max(leg_time(prev, g), 0.15)
            prev = g
        return t

    @property
    def final_goal(self):
        """Where the whole path ends."""
        return self.queue[-1] if self.queue else self.goal

    # ── sim view of the command ──
    @staticmethod
    def to_sim(servo):
        angles = np.zeros(5)
        for i, a in enumerate(AXES):
            if a.joint is not None:
                angles[a.joint] = a.to_sim(servo[i])
        return angles

    @staticmethod
    def from_sim(angles, gripper):
        servo = np.zeros(N)
        for i, a in enumerate(AXES):
            servo[i] = gripper if a.joint is None else a.to_servo(angles[a.joint])
        return servo

    def sim_cmd(self):
        return self.to_sim(self.cmd)

    def sim_goal(self):
        return self.to_sim(self.final_goal)

    def wire(self):
        return [a.wire(v) for a, v in zip(AXES, self.cmd)]
