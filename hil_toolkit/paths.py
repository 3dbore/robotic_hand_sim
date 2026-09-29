"""Filesystem locations shared across the toolkit modules.

Computed once here (not re-derived per module with `dirname(dirname(...))`
chains) so every module agrees on the same project root.
"""
import os

PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

ARMS_DIR = os.path.join(PROJECT_ROOT, "arms")
FONTS_DIR = os.path.join(PROJECT_ROOT, "fonts")
CALIB_DIR = os.path.join(PROJECT_ROOT, "calibration")
PAPER_DIR = os.path.join(PROJECT_ROOT, "paper")

SIM_END_FILE = os.path.join(PROJECT_ROOT, "arm-sim-end.py")
BRAND_FONT_FILE = os.path.join(FONTS_DIR, "ZenDots-Regular.ttf")
APP_ICON_FILE = os.path.join(PAPER_DIR, "img", "ELIO_ICON.png")
CAMERA_MODEL_FILE = os.path.join(CALIB_DIR, "camera_model.json")
