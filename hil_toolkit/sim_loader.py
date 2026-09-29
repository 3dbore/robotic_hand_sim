"""
Loads arm-sim-end.py (kinematics + scene meshes) as a library, without
letting its standalone GUI construction pop a real window.

This module is imported exactly once no matter how many other hil_toolkit
modules do `from .sim_loader import ase` — Python caches module execution in
sys.modules, so the STL loads and the dynamic exec_module below only ever
run a single time per process. Every other module must go through this
module rather than repeating the importlib dance itself.
"""
import sys
import importlib.util

import numpy as np
import pyvista as pv

from .paths import SIM_END_FILE


class _DummyCamera:
    def zoom(self, *args):
        pass


class _DummyInteractor:
    def __init__(self):
        self.interactor = self

    def AddObserver(self, *args):
        pass

    def GetKeySym(self):
        return ""


class _DummyPlotter:
    """Swallows every pyvista.Plotter call arm-sim-end.py makes at module
    scope, so importing it never builds a real VTK render window."""

    def __init__(self, *args, **kwargs):
        self.camera = _DummyCamera()
        self.iren = _DummyInteractor()
        self.interactor = self.iren
        self.camera_position = None

    def __getattr__(self, attr):
        return lambda *a, **kw: None

    def __setattr__(self, key, value):
        self.__dict__[key] = value


def _load_arm_sim_end():
    real_plotter = pv.Plotter
    pv.Plotter = _DummyPlotter
    try:
        spec = importlib.util.spec_from_file_location("arm_sim_end", SIM_END_FILE)
        module = importlib.util.module_from_spec(spec)
        sys.modules["arm_sim_end"] = module
        spec.loader.exec_module(module)
    finally:
        pv.Plotter = real_plotter
    return module


ase = _load_arm_sim_end()

# arm1.stl offset compensation (see arm-sim-toolkit-2.py)
ARM1_OFFSET = np.array([0.0, 0.0, 0.0], dtype=np.float32)
if hasattr(ase, "original_points") and "arm1" in ase.original_points:
    ase.original_points["arm1"] = ase.original_points["arm1"] + ARM1_OFFSET
if hasattr(ase, "loaded_meshes") and "arm1" in ase.loaded_meshes:
    ase.loaded_meshes["arm1"].points[:] = ase.loaded_meshes["arm1"].points + ARM1_OFFSET
