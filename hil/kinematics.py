"""
Closed-form inverse kinematics for the arm, and its reachable workspace.

The arm (arm-sim-end.py) is a base yaw J1 (Z) followed by a planar chain:
J2, J3, J4 all rotate about X, so after undoing the base yaw everything up to
the end effector (the J4 pitch joint) lies in one vertical plane. In that
plane a point is a complex number  y + i·z  and a rotation about X by `a` is a
multiplication by e^{ia}, so

    J4 = c1 + e^{iθ2}·D2 + e^{i(θ2+θ3)}·D3

which is the textbook two-link problem, solved exactly below. J4 (pitch) and
J5 (roll) don't move the end effector; pitch keeps the tool's global angle
(θ2+θ3+θ4) where it was, roll is left alone.
"""
import math

import numpy as np

from .config import TANK_Z_MM
from .sim import ase

_d1, _d2, _d3 = ase.D_OFFSETS[0], ase.D_OFFSETS[1], ase.D_OFFSETS[2]
C1 = complex(_d1[1], _d1[2])            # J2 in the arm plane (y, z)
D2 = complex(_d2[1], _d2[2])            # J2 → J3
D3 = complex(_d3[1], _d3[2])            # J3 → J4
L1, L2 = abs(D2), abs(D3)
PHI2, PHI3 = math.atan2(D2.imag, D2.real), math.atan2(D3.imag, D3.real)


def fk(angles):
    """End-effector (J4) position, world mm."""
    return np.asarray(ase.forward_kinematics(np.asarray(angles, float))[2])


def joint_points(angles):
    """World positions of J1..J5."""
    return [np.asarray(p) for p in ase.forward_kinematics(np.asarray(angles, float))[0]]


def in_tank(y, z):
    """True where a world point is over the tank: behind the base, too low."""
    return (np.asarray(y) > 1e-3) & (np.asarray(z) < TANK_Z_MM)


def unsafe(angles):
    """Why this pose would hit the tank, or None. The elbow (J3), the end
    effector (J4) and the wrist (J5) are checked."""
    pts = joint_points(angles)
    for name, p in (("elbow", pts[2]), ("end effector", pts[3]), ("wrist", pts[4])):
        if in_tank(p[1], p[2]):
            return f"the {name} would hit the tank (behind the base, below Z {TANK_Z_MM:.0f} mm)"
    return None


def path_unsafe(a, b, step=2.0):
    """Why the straight joint-space move a → b would hit the tank, or None.
    (Every move is joint-space: all axes start and arrive together.)"""
    a, b = np.asarray(a, float), np.asarray(b, float)
    n = max(1, int(math.ceil(np.max(np.abs(b - a)) / step)))
    for s in np.linspace(0.0, 1.0, n + 1):
        why = unsafe(a + (b - a) * s)
        if why:
            return why
    return None


def plan(a, b):
    """Joint-space waypoints from a to b that keep clear of the tank.

    Returns (waypoints, None) or (None, reason). Tries the straight move, then
    arm first (lift in place, then turn), then base first (turn, then lower)."""
    a, b = np.asarray(a, float), np.asarray(b, float)
    why = unsafe(b)
    if why:
        return None, why
    if not path_unsafe(a, b):
        return [b], None
    arm_first = b.copy()
    arm_first[0] = a[0]
    base_first = a.copy()
    base_first[0] = b[0]
    for via in (arm_first, base_first):
        if not unsafe(via) and not path_unsafe(a, via) and not path_unsafe(via, b):
            return [via, b], None
    return None, "the way there passes over the tank too low; raise the arm first"


def _wrap(a):
    return (a + 180.0) % 360.0 - 180.0


def _fit(angle, lo, hi, tol=1e-6):
    """angle (deg), shifted by whole turns into [lo, hi] if possible."""
    for k in (0, -360, 360):
        a = angle + k
        if lo - tol <= a <= hi + tol:
            return min(max(a, lo), hi)
    return None


def solve(target, current, limits, keep_pitch=True, check=unsafe):
    """Joint angles that put the end effector at `target` (world mm).

    Returns (angles, None) or (None, reason). Of all solutions inside
    `limits` the one closest to `current` is chosen.
    """
    X, Y, Z = (float(v) for v in target)
    r = math.hypot(X, Y)
    cur = np.asarray(current, float)
    best = None
    reason = "out of reach"

    # The arm plane points forward (-y local) or backward (+y local).
    for y_local, t1 in ((-r, math.degrees(math.atan2(X, -Y))),
                        (r, math.degrees(math.atan2(-X, Y)))):
        if r < 1e-9:
            t1 = cur[0]
        t1 = _fit(t1, *limits[0])
        if t1 is None:
            reason = "base can't turn that far"
            continue
        v = complex(y_local, Z) - C1
        c = (abs(v) ** 2 - L1 ** 2 - L2 ** 2) / (2 * L1 * L2)
        if c < -1 - 1e-9 or c > 1 + 1e-9:
            continue
        g0 = math.acos(max(-1.0, min(1.0, c)))
        psi = math.atan2(v.imag, v.real)
        for g in (g0, -g0):
            t3 = _fit(_wrap(math.degrees(g + PHI2 - PHI3)), *limits[2])
            t2 = _fit(_wrap(math.degrees(psi - math.atan2(L2 * math.sin(g), L1 + L2 * math.cos(g)) - PHI2)),
                      *limits[1])
            if t2 is None or t3 is None:
                reason = "outside the arm's joint limits"
                continue
            a = cur.copy()
            a[0], a[1], a[2] = t1, t2, t3
            if keep_pitch:
                pitch = cur[1] + cur[2] + cur[3]
                a[3] = min(max(pitch - t2 - t3, limits[3][0]), limits[3][1])
            if np.linalg.norm(fk(a) - (X, Y, Z)) > 0.5:
                continue
            if check is not None:
                why = check(a)
                if why:
                    reason = why
                    continue
            cost = float(np.sum(np.abs(a - cur)))
            if best is None or cost < best[0]:
                best = (cost, a)
    if best is None:
        return None, reason
    return best[1], None


class Workspace:
    """Where the end effector can go, as boolean rasters.

    The planar chain's reach (θ2, θ3 inside limits) is rasterised once in the
    arm plane. The base turns ±180°, so a world point (X, Y, Z) is reachable
    when its arm-plane point is, facing it (-r) or leaning back over it (+r),
    and it isn't over the tank (only the end effector is checked here)."""

    CELL = 2.0     # mm per raster cell

    def __init__(self, limits):
        (a2, b2), (a3, b3) = limits[1], limits[2]
        t2 = np.radians(np.linspace(a2, b2, int((b2 - a2) / 0.25) + 1))
        t3 = np.radians(np.linspace(a3, b3, int((b3 - a3) / 0.25) + 1))
        T2, T3 = np.meshgrid(t2, t3, indexing="ij")
        p = C1 + np.exp(1j * T2) * D2 + np.exp(1j * (T2 + T3)) * D3
        reach = L1 + L2 + abs(C1) + 10
        self.y0, self.z0 = -reach, -reach
        n = int(2 * reach / self.CELL) + 1
        mask = np.zeros((n, n), bool)
        iy = ((p.real - self.y0) / self.CELL).astype(int).ravel()
        iz = ((p.imag - self.z0) / self.CELL).astype(int).ravel()
        mask[iy, iz] = True
        # close the one-cell gaps the sampling leaves at full stretch
        grown = mask.copy()
        grown[1:, :] |= mask[:-1, :]
        grown[:-1, :] |= mask[1:, :]
        grown[:, 1:] |= mask[:, :-1]
        grown[:, :-1] |= mask[:, 1:]
        self.plane = grown
        self.n = n
        self.reach = reach

    def plane_lookup(self, y_local, z):
        iy = np.floor((np.asarray(y_local) - self.y0) / self.CELL).astype(int)
        iz = np.floor((np.asarray(z) - self.z0) / self.CELL).astype(int)
        ok = (iy >= 0) & (iy < self.n) & (iz >= 0) & (iz < self.n)
        out = np.zeros(np.shape(iy), bool)
        out[ok] = self.plane[iy[ok], iz[ok]]
        return out

    def top(self, xs, ys, z):
        """Reachable mask over the X–Y grid (xs, ys) at height z."""
        X, Y = np.meshgrid(xs, ys)
        r = np.hypot(X, Y)
        Z = np.full_like(X, z)
        return (self.plane_lookup(-r, Z) | self.plane_lookup(r, Z)) & ~in_tank(Y, Z)

    def side(self, hs, zs, heading=0.0):
        """Reachable mask over (forward reach h, height z) in the arm plane at
        base angle `heading`, h = -y_local."""
        H, Zg = np.meshgrid(hs, zs)
        Y = -H * math.cos(math.radians(heading))
        return self.plane_lookup(-H, Zg) & ~in_tank(Y, Zg)
