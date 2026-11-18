"""
Sequences: named lists of saved poses ("steps") that the arm plays through
one after another, starting from wherever it is.

A step stores all six servo angles (field order of config.AXES, gripper
included) and how long to hold there before moving on. Poses are stored in
servo degrees, not wire degrees, so changing trim or invert later does not
change what a saved sequence means.

Every move between steps still goes through MainWindow._request(), so the
tank-aware planner can refuse a step; the run then stops there.

Saved to calibration/hil_sequences.json, rewritten on every edit.
"""
import json
import os
from dataclasses import dataclass, field
from typing import List

from .config import AXES, PROJECT_ROOT

SEQUENCES_FILE = os.path.join(PROJECT_ROOT, "calibration", "hil_sequences.json")
HOLD_MAX_S = 60.0


@dataclass
class Step:
    name: str
    servo: List[float]          # six servo angles, config.AXES order
    hold: float = 0.5           # seconds to wait at this pose

    def to_json(self):
        return {"name": self.name, "hold": round(self.hold, 2),
                "servo": {a.name: round(float(v), 2) for a, v in zip(AXES, self.servo)}}

    @classmethod
    def from_json(cls, d):
        s = d.get("servo", {})
        servo = [float(s.get(a.name, a.home)) for a in AXES]
        hold = max(0.0, min(HOLD_MAX_S, float(d.get("hold", 0.5))))
        return cls(str(d.get("name", "Step")), servo, hold)


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
