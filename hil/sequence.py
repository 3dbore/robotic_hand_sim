"""
Sequences: named lists of saved poses ("steps") that the arm plays through
one after another, starting from wherever it is.

A step stores all six servo angles (field order of config.AXES, gripper
included) and how long to hold there before moving on. Poses are stored in
servo degrees, not wire degrees, so changing trim or invert later does not
change what a saved sequence means.

Every move between steps still goes through MainWindow._request(), so the
tank-aware planner can refuse a step; the run then stops there.

A step may also carry `tag`: what the gripper camera saw when the step was
saved (AprilTag ids and image positions plus the camera's azimuth / elevation;
see arm-hil-toolkit-2.py). Steps without a tag simply have tag = None.

Saved to calibration/hil_sequences.json, rewritten on every edit.
"""
import copy
import json
import os
from dataclasses import dataclass, field
from typing import List, Optional

from .config import AXES, PROJECT_ROOT

SEQUENCES_FILE = os.path.join(PROJECT_ROOT, "calibration", "hil_sequences.json")
HOLD_MAX_S = 60.0


@dataclass
class Step:
    name: str
    servo: List[float]          # six servo angles, config.AXES order
    hold: float = 0.5           # seconds to wait at this pose
    tag: Optional[dict] = None  # AprilTag capture taken at this pose, if any
    # Speed profile used to move INTO this step (arm-hil-toolkit-2.py):
    # {axis name: {"vmax": deg/s, "accel": fraction, "decel": fraction}}. None = default.
    motion: Optional[dict] = None

    def to_json(self):
        d = {"name": self.name, "hold": round(self.hold, 2),
             "servo": {a.name: round(float(v), 2) for a, v in zip(AXES, self.servo)}}
        if self.tag:
            d["tag"] = self.tag
        if self.motion:
            d["motion"] = self.motion
        return d

    def copy(self, name=None):
        return Step(self.name if name is None else name, list(self.servo), self.hold,
                    copy.deepcopy(self.tag), copy.deepcopy(self.motion))

    @classmethod
    def from_json(cls, d):
        s = d.get("servo", {})
        servo = [float(s.get(a.name, a.home)) for a in AXES]
        hold = max(0.0, min(HOLD_MAX_S, float(d.get("hold", 0.5))))
        tag = d.get("tag")
        mo = d.get("motion")
        return cls(str(d.get("name", "Step")), servo, hold, tag if isinstance(tag, dict) else None,
                   mo if isinstance(mo, dict) else None)


@dataclass
class Sequence:
    name: str
    steps: List[Step] = field(default_factory=list)


def load():
    """[Sequence] from disk; an empty list if the file is missing or broken."""
    try:
        with open(SEQUENCES_FILE, "r", encoding="utf-8") as f:
            data = json.load(f)
    except (OSError, ValueError):
        return []
    out = []
    for d in data.get("sequences", []):
        try:
            out.append(Sequence(str(d["name"]), [Step.from_json(s) for s in d.get("steps", [])]))
        except (KeyError, TypeError, ValueError):
            continue
    return out


def save(seqs):
    data = {"sequences": [{"name": q.name, "steps": [s.to_json() for s in q.steps]} for q in seqs]}
    os.makedirs(os.path.dirname(SEQUENCES_FILE), exist_ok=True)
    tmp = SEQUENCES_FILE + ".tmp"
    with open(tmp, "w", encoding="utf-8") as f:
        json.dump(data, f, indent=2)
    os.replace(tmp, SEQUENCES_FILE)


def unique_name(base, taken):
    if base not in taken:
        return base
    n = 2
    while f"{base} {n}" in taken:
        n += 1
    return f"{base} {n}"
