"""
Loads arm-sim-end.py as a library: its kinematics, constants and meshes,
without its standalone window (every pyvista.Plotter call it makes at import
is swallowed by a dummy).
"""
import importlib.util
import sys

import pyvista as pv

from .config import SIM_END_FILE


class _Dummy:
    def __init__(self, *args, **kwargs):
        pass

    def __getattr__(self, attr):
        return _Dummy()

    def __call__(self, *args, **kwargs):
        return _Dummy()

    def __setattr__(self, key, value):
        pass


def _load():
    real = pv.Plotter
    pv.Plotter = _Dummy
    try:
        spec = importlib.util.spec_from_file_location("arm_sim_end", SIM_END_FILE)
        module = importlib.util.module_from_spec(spec)
        sys.modules["arm_sim_end"] = module
        spec.loader.exec_module(module)
    finally:
        pv.Plotter = real
    return module


ase = _load()
