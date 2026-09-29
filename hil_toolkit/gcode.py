"""
G-CODE INTERPRETER
--------------------
Coordinates are the wrist (J4) position in the base frame, in mm.
  G0 / G1        rapid / feed move            X Y Z  A (wrist pitch°) B (wrist roll°)  F (mm/min)
  G2 / G3        CW / CCW arc (helix if the   I J K = centre offset from the start point
                 3rd axis also changes)       start == end → full circle
  G17/G18/G19    arc plane XY / ZX / YZ       G4 P<ms> or S<s>   dwell
  G90 / G91      absolute / relative          G28                return to initial pose
  G21            millimetres (G20 rejected)   M0/M1 pause · M2/M30 end
  M3 [S<deg>]    close gripper                M5                 open gripper
Comments: ";" to end of line, or "( … )".  N line numbers are ignored.
"""
import math
import re

import numpy as np

from .sim_loader import ase
from .hardware_mapping import GRIPPER

GCODE_FEED_DEFAULT = 1200.0   # mm/min
GCODE_FEED_MAX = 3000.0       # mm/min
GCODE_RAPID = 2400.0          # mm/min for G0 / G28
GCODE_ROT_SPEED = 60.0        # °/s for A/B changes (firmware servo limit)
GCODE_ARC_SEG = 2.0           # mm per arc chord
GCODE_TICK_MS = 50
GCODE_TRACK_TOL = 2.0         # mm; larger final error = target out of reach
GCODE_PITCH_TOL = 1.0         # °

_GCODE_WORD = re.compile(r"([A-Z])([-+]?(?:\d+\.?\d*|\.\d+))")
_GCODE_LINE = re.compile(r"(?:[A-Z][-+]?(?:\d+\.?\d*|\.\d+))+")

GCODE_HELP = (
    "G0/G1 X Y Z A(pitch°) B(roll°) F(mm/min) · G2/G3 arc with I J K · "
    "G17/G18/G19 plane · G90/G91 · G4 P(ms)/S(s) · G28 home · "
    "M3 [S°] grip · M5 release · M0 pause · M30 end")

# Demo program: verified reachable within the hardware limits, joint speeds <= 60°/s
DEFAULT_GCODE = """\
; ELIOS demo: pick, inspect, trace, place
; mm, wrist (J4) position in the base frame
G21 G90 G17            ; mm, absolute, XY arc plane
G28                    ; start from the initial pose
M5                     ; open gripper
G4 P500

; 1. Pick
G0 X0 Y-110 Z150       ; rapid above the part
G1 Z110 F600           ; slow descent
M3 S70                 ; grip
G4 P600
G1 Z150 F900           ; lift

; 2. Show the part to the camera (pitch A, roll B)
G1 Y-170 Z210 A-20 F1200
G1 B60
G1 B-60
G1 B0 A0
G1 Y-130 Z150

; 3. Rectangle + circles in the XY plane
G1 X40 Y-100 F1500
G1 Y-160
G1 X-40
G1 Y-100
G1 X0
G2 X0 Y-100 I0 J-30    ; full circle, R30
G3 X30 Y-130 I0 J-30   ; 3/4 circle CCW

; 4. Helix: one turn rising 40 mm
G91                    ; relative
G2 X0 Y0 Z40 I-30 J0
G90

; 5. Vertical circle in the YZ plane
G19
G2 J-30 K0
G17

; 6. Relative sweep across the table
G91
G1 X50 F1800
G1 X-100
G1 X50
G90

; 7. Place
G0 X70 Y-120 Z150
G1 Z110 F600
M5                     ; release
G4 P500
G1 Z150 F900
G28                    ; home
M30
"""


class GCodeError(Exception):
    def __init__(self, line, msg):
        super().__init__(f"line {line}: {msg}")
        self.line = line


class GCodeProgram:
    """Compiles G-code text into timed segments (move / dwell / grip / pause / end)."""
    # (u, v, normal) axis indices per plane; G18 is ZX by convention
    PLANES = {17: (0, 1, 2), 18: (2, 0, 1), 19: (1, 2, 0)}

    def __init__(self, text, pos, pitch, roll):
        self.segments = []
        self.warnings = []
        self._compile(text, np.array(pos, dtype=float), float(pitch), float(roll))

    @property
    def duration(self):
        return sum(s.get("duration", 0.0) for s in self.segments)

    def _compile(self, text, pos, a, b):
        mode, plane, relative, feed = None, 17, False, GCODE_FEED_DEFAULT
        _, _, home = ase.forward_kinematics(np.zeros(5))
        for n, raw in enumerate(text.splitlines(), 1):
            line = re.sub(r"\(.*?\)", "", raw).split(";")[0].upper()
            line = re.sub(r"\s+", "", line)
            if not line:
                continue
            if not _GCODE_LINE.fullmatch(line):
                raise GCodeError(n, f"cannot parse '{raw.strip()}'")
            gs, ms, words = [], [], {}
            for letter, num in _GCODE_WORD.findall(line):
                if letter == "G":
                    gs.append(float(num))
                elif letter == "M":
                    ms.append(float(num))
                elif letter == "N":
                    continue
                elif letter in words:
                    raise GCodeError(n, f"{letter} given twice")
                elif letter not in "XYZABIJKFPS":
                    raise GCodeError(n, f"unsupported word '{letter}'")
                else:
                    words[letter] = float(num)

            motion_now = None
            for g in gs:
                if g != int(g):
                    raise GCodeError(n, f"unsupported G{g:g}")
                g = int(g)
                if g in (0, 1, 2, 3):
                    mode = motion_now = g
                elif g in (17, 18, 19):
                    plane = g
                elif g == 21:
                    pass
                elif g == 20:
                    raise GCodeError(n, "G20 (inches) not supported, use mm")
                elif g in (90, 91):
                    relative = g == 91
                elif g == 4:
                    if "P" in words:
                        secs = words["P"] / 1000.0
                    elif "S" in words:
                        secs = words["S"]
                    else:
                        raise GCodeError(n, "G4 needs P<ms> or S<seconds>")
                    self.segments.append({"kind": "dwell", "line": n, "duration": max(0.0, secs)})
                elif g == 28:
                    self._move(n, np.array([pos, home]), (a, 0.0), (b, 0.0), GCODE_RAPID)
                    pos, a, b = home.copy(), 0.0, 0.0
                else:
                    raise GCodeError(n, f"unsupported G{g}")

            if "F" in words:
                if words["F"] <= 0:
                    raise GCodeError(n, "F must be positive")
                if words["F"] > GCODE_FEED_MAX:
                    self.warnings.append(f"line {n}: F{words['F']:g} capped to F{GCODE_FEED_MAX:g}")
                feed = min(words["F"], GCODE_FEED_MAX)

            if any(k in words for k in "XYZAB") or (motion_now in (2, 3) and any(k in words for k in "IJK")):
                if mode is None:
                    raise GCodeError(n, "no motion mode active (use G0/G1/G2/G3)")
                target = pos.copy()
                for i, ax in enumerate("XYZ"):
                    if ax in words:
                        target[i] = pos[i] + words[ax] if relative else words[ax]
                a1 = (a + words["A"] if relative else words["A"]) if "A" in words else a
                b1 = (b + words["B"] if relative else words["B"]) if "B" in words else b
                if mode in (0, 1):
                    self._move(n, np.array([pos, target]), (a, a1), (b, b1),
                               GCODE_RAPID if mode == 0 else feed)
                else:
                    self._arc(n, pos, target, words, mode == 2, plane, (a, a1), (b, b1), feed)
                pos, a, b = target, a1, b1
            elif any(k in words for k in "IJK"):
                raise GCodeError(n, "I/J/K are only valid with G2/G3")

            for m in ms:
                m = int(m)
                if m in (0, 1):
                    self.segments.append({"kind": "pause", "line": n})
                elif m in (2, 30):
                    self.segments.append({"kind": "end", "line": n})
                    return
                elif m == 3:
                    v = words.get("S", GRIPPER.servo_max)
                    self.segments.append({"kind": "grip", "line": n, "value": int(round(
                        np.clip(v, GRIPPER.servo_min, GRIPPER.servo_max)))})
                elif m == 5:
                    self.segments.append({"kind": "grip", "line": n, "value": GRIPPER.servo_min})
                else:
                    raise GCodeError(n, f"unsupported M{m}")

    def _move(self, n, pts, a, b, speed):
        cum = np.concatenate([[0.0], np.cumsum(np.linalg.norm(np.diff(pts, axis=0), axis=1))])
        length = cum[-1]
        if length < 1e-9 and a[0] == a[1] and b[0] == b[1]:
            return
        duration = max(length / (speed / 60.0),
                       abs(a[1] - a[0]) / GCODE_ROT_SPEED,
                       abs(b[1] - b[0]) / GCODE_ROT_SPEED, 1e-6)
        self.segments.append({"kind": "move", "line": n, "points": pts, "cum": cum,
                              "length": length, "a": a, "b": b, "duration": duration})

    def _arc(self, n, start, end, words, cw, plane, a, b, feed):
        u, v, w = self.PLANES[plane]
        if not any("IJK"[i] in words for i in (u, v)):
            raise GCodeError(n, f"G{2 if cw else 3} needs centre offsets ({'IJK'[u]}/{'IJK'[v]}) for G{plane}")
        cu = start[u] + words.get("IJK"[u], 0.0)
        cv = start[v] + words.get("IJK"[v], 0.0)
        r0 = math.hypot(start[u] - cu, start[v] - cv)
        r1 = math.hypot(end[u] - cu, end[v] - cv)
        if r0 < 0.1:
            raise GCodeError(n, "arc radius is zero")
        if abs(r0 - r1) > 0.5:
            raise GCodeError(n, f"arc end is not on the circle (r {r0:.1f} vs {r1:.1f} mm)")
        t0 = math.atan2(start[v] - cv, start[u] - cu)
        t1 = math.atan2(end[v] - cv, end[u] - cu)
        if cw:
            sweep = -((t0 - t1) % (2 * math.pi)) or -2 * math.pi
        else:
            sweep = ((t1 - t0) % (2 * math.pi)) or 2 * math.pi
        helix = end[w] - start[w]
        k = max(8, math.ceil(math.hypot(abs(sweep) * r0, helix) / GCODE_ARC_SEG))
        pts = np.empty((k + 1, 3))
        for i in range(k + 1):
            s = i / k
            t, r = t0 + sweep * s, r0 + (r1 - r0) * s
            pts[i, u] = cu + r * math.cos(t)
            pts[i, v] = cv + r * math.sin(t)
            pts[i, w] = start[w] + helix * s
        pts[-1] = end
        self._move(n, pts, a, b, feed)


class GCodeRunner:
    """Steps a compiled program in time, driving the sim through its IK."""
    def __init__(self, program):
        self.segments = program.segments
        self.idx = 0
        self.t = 0.0

    @property
    def segment(self):
        return self.segments[self.idx] if self.idx < len(self.segments) else None

    @property
    def line(self):
        seg = self.segment
        return seg["line"] if seg else None

    def _next(self):
        self.idx += 1
        self.t = 0.0

    @staticmethod
    def _point_at(seg, s):
        cum, pts = seg["cum"], seg["points"]
        i = int(np.clip(np.searchsorted(cum, s, side="right") - 1, 0, len(pts) - 2))
        span = cum[i + 1] - cum[i]
        f = 0.0 if span < 1e-9 else (s - cum[i]) / span
        return pts[i] + min(1.0, f) * (pts[i + 1] - pts[i])

    def tick(self, dt):
        """Advance by dt seconds. Returns events: ("warn", msg), ("grip", value),
        ("pause", line), ("end", line)."""
        seg = self.segment
        if seg is None:
            return [("end", None)]
        events = []
        kind = seg["kind"]
        if kind == "move":
            self.t += dt
            f = min(1.0, self.t / seg["duration"])
            target = self._point_at(seg, f * seg["length"])
            ase.desired_global_pitch = seg["a"][0] + (seg["a"][1] - seg["a"][0]) * f
            ase.desired_global_roll = seg["b"][0] + (seg["b"][1] - seg["b"][0]) * f
            _, _, ee = ase.forward_kinematics(ase.current_angles)
            ase.current_angles = ase.ik_step(ase.current_angles, target - ee)
            if f >= 1.0:
                ang = ase.current_angles
                _, _, ee = ase.forward_kinematics(ang)
                err = np.linalg.norm(target - ee)
                if err > GCODE_TRACK_TOL:
                    events.append(("warn", f"Line {seg['line']}: X{target[0]:.1f} Y{target[1]:.1f} "
                                           f"Z{target[2]:.1f} out of reach (off by {err:.1f} mm)"))
                pitch = ang[1] + ang[2] + ang[3]
                if abs(pitch - seg["a"][1]) > GCODE_PITCH_TOL:
                    events.append(("warn", f"Line {seg['line']}: wrist pitch limited to "
                                           f"A{pitch:.1f} (asked A{seg['a'][1]:g})"))
                self._next()
        elif kind == "dwell":
            self.t += dt
            if self.t >= seg["duration"]:
                self._next()
        elif kind == "grip":
            events.append(("grip", seg["value"]))
            self._next()
        elif kind == "pause":
            events.append(("pause", seg["line"]))
            self._next()
        elif kind == "end":
            events.append(("end", seg["line"]))
            self.idx = len(self.segments)
            return events
        if self.segment is None:
            events.append(("end", None))
        return events
