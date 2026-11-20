"""
ELIOS Gripper Simulation
------------------------
3D view of the Y-type (2-jaw radial) gripper on the left, controls on the
right. The geometry and the linkage maths live in gripper_kinematics.py.

    python gripper/gripper_sim.py

Controls
    Jaws    opening %, servo angle (0 = pose in gripper_kinematics.FCStd),
            Open / Close / Home move at SERVO_SPEED, Cycle loops open<->close
    Wrist   Pitch (about X) and Yaw (about the gripper's long axis)
    View    linkage lines + pivots, finger-tip paths, see-through body,
            camera presets
"""
import os
import sys

import numpy as np
import pyvista as pv
from PyQt5.QtCore import Qt, QTimer, pyqtSignal
from PyQt5.QtGui import QFontDatabase, QIcon
from PyQt5.QtWidgets import (
    QApplication, QCheckBox, QFrame, QGridLayout, QHBoxLayout, QLabel, QMainWindow, QPushButton,
    QScrollArea, QSlider, QSplitter, QVBoxLayout, QWidget,
)
from pyvistaqt import QtInteractor

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(1, os.path.dirname(HERE))          # for the shared ELIO theme in ../hil

import gripper_kinematics as gk                     # noqa: E402
from hil import theme                               # noqa: E402
from hil.theme import C                             # noqa: E402

SERVO_SPEED = 120.0          # deg/s for Open / Close / Home / Cycle
TICK_MS = 16
LINK_COLOR = "#7ed321"       # the green of the kinematics drawing
PIVOT_COLOR = "#2e7d32"

PART_STYLE = {               # colour, opacity
    "wrist": ("#b8c2cc", 1.0),
    "body": ("#c97a70", 1.0),
    "body2": ("#a8605a", 1.0),
    "servo": ("#4a4f55", 1.0),
    "rod_geared": ("#8d9aa6", 1.0),
    "rod": ("#9fb0bf", 1.0),
    "finger": ("#d9dee3", 1.0),
}
SEE_THROUGH = {"body": 0.25, "body2": 0.25, "wrist": 0.35}
JAW_PARTS = ("rod_geared", "rod", "finger")


# ─────────────────────────────────────────────────────────────────────────────
# 3D scene
# ─────────────────────────────────────────────────────────────────────────────
class GripperScene:
    """All parts are loaded once in their STL frames and posed with each
    actor's user_matrix; the right jaw reuses the left jaw's meshes with an
    X mirror in front of its matrix."""

    def __init__(self, plotter):
        self.plotter = p = plotter
        p.set_background(C["surface_lowest"])
        self.meshes = {k: pv.read(gk.stl_path(k)) for k in gk.PARTS}
        self.gripper = gk.Gripper(self.meshes["finger"].points)

        self.actors = {}
        for k, mesh in self.meshes.items():
            color, op = PART_STYLE[k]
            sides = ("L", "R") if k in JAW_PARTS else ("",)
            for side in sides:
                self.actors[k + side] = p.add_mesh(mesh, color=color, opacity=op, specular=0.3,
                                                   specular_power=15, show_edges=False)
        # linkage: ground A-B, servo gear - A, crank A-C, coupler C-D, rocker B-D (both jaws)
        self.link_pts = np.zeros((12, 3))
        lines = []
        for j in (0, 6):          # points per jaw: S, A, B, C, D, (unused)
            S, A, B, Cc, D = j, j + 1, j + 2, j + 3, j + 4
            lines += [2, S, A, 2, A, B, 2, A, Cc, 2, Cc, D, 2, B, D]
        self.link = pv.PolyData(self.link_pts.copy(), lines=np.array(lines))
        self.link_actor = p.add_mesh(self.link, color=LINK_COLOR, line_width=4, render_lines_as_tubes=True)
        self.pivots = pv.PolyData(self.link_pts.copy())
        self.pivot_actor = p.add_mesh(self.pivots, color=PIVOT_COLOR, point_size=12,
                                      render_points_as_spheres=True)
        for a in (self.link_actor, self.pivot_actor):    # draw the linkage over the plates
            a.GetProperty().SetOpacity(1.0)

        # finger-tip paths over the whole servo range, in the gripper frame
        g = self.gripper
        tips = np.array([g.tip(gk.solve_jaw(s))
                         for s in np.linspace(g.open_limit, g.closed_limit, 80)])
        self.path_home = np.vstack([tips, tips * [-1, 1, 1]])
        n = len(tips)
        self.path = pv.PolyData(self.path_home.copy(),
                                lines=np.hstack([[n], np.arange(n), [n], np.arange(n, 2 * n)]))
        self.path_actor = p.add_mesh(self.path, color=C["tertiary"], line_width=2)
        self.tip_dot = pv.PolyData(np.zeros((2, 3)))
        self.tip_actor = p.add_mesh(self.tip_dot, color=C["tertiary"], point_size=10,
                                    render_points_as_spheres=True)

        try:
            p.add_axes(line_width=2)
        except Exception:
            pass
        self.view("iso")

    # ── posing ──
    def update(self, servo, pitch, yaw):
        """Pose everything; returns the jaw dict (with tip / gap) or None."""
        jaw = gk.solve_jaw(servo)
        if jaw is None:
            return None
        Mp, My = gk.wrist_matrices(pitch, yaw)
        self._set("wrist", Mp @ gk.placement("wrist"))
        for k in ("body", "body2"):
            self._set(k, My @ gk.placement(k))
        mats = gk.part_matrices(jaw)
        self._set("servo", My @ mats["servo"])
        for k in JAW_PARTS:
            self._set(k + "L", My @ mats[k])
            self._set(k + "R", My @ gk.MIRROR_X @ mats[k])

        z = -14.0 + 10.0          # linkage drawn mid-way through the rods' clevis
        S, A, B, Cc, D = (np.array([*v, z]) for v in (gk.SERVO_GEAR, jaw["A"], jaw["B"], jaw["C"], jaw["D"]))
        left = np.array([S, A, B, Cc, D, D])
        pts = np.vstack([left, left * [-1, 1, 1]])
        pts[6] = S                # one servo gear drives both sides
        self.link.points = gk.apply(My, pts)
        self.pivots.points = self.link.points
        self.path.points = gk.apply(My, self.path_home)
        tip = self.gripper.tip(jaw)
        self.tip_dot.points = gk.apply(My, [tip, tip * [-1, 1, 1]])

        jaw["gap"] = self.gripper.gap(jaw)
        jaw["tip"] = tip
        jaw["tip_world"] = gk.apply(My, [tip])[0]
        self.render()
        return jaw

    def _set(self, key, M):
        a = self.actors[key] if key in self.actors else None
        if a is None:
            return
        a.user_matrix = M

    # ── display options ──
    def show_linkage(self, on):
        for a in (self.link_actor, self.pivot_actor):
            a.SetVisibility(bool(on))
        self.render()

    def show_paths(self, on):
        for a in (self.path_actor, self.tip_actor):
            a.SetVisibility(bool(on))
        self.render()

    def show_wrist(self, on):
        self.actors["wrist"].SetVisibility(bool(on))
        self.render()

    def see_through(self, on):
        for k, op in SEE_THROUGH.items():
            self.actors[k].GetProperty().SetOpacity(op if on else PART_STYLE[k][1])
        self.render()

    def view(self, name):
        c = (0.0, 110.0, -5.0)
        views = {
            # like the drawing: looking down Z, Y to the right, X down
            "top": [(c[0], c[1], 420), c, (-1, 0, 0)],
            "side": [(420, c[1], c[2]), c, (0, 0, 1)],
            "front": [(0, 420, c[2]), c, (0, 0, 1)],
            "iso": [(-230, -60, 260), c, (0, 0, 1)],
        }
        self.plotter.camera_position = views[name]
        self.plotter.reset_camera_clipping_range()
        self.render()

    def render(self):
        try:
            self.plotter.render()
        except Exception:
            pass

    def close(self):
        try:
            self.plotter.close()
        except Exception:
            pass


# ─────────────────────────────────────────────────────────────────────────────
# Widgets
# ─────────────────────────────────────────────────────────────────────────────
def label(text="", name=None):
    w = QLabel(text)
    if name:
        w.setObjectName(name)
    return w


def card(title, subtitle=None):
    f = QFrame()
    f.setObjectName("Card")
    lay = QVBoxLayout(f)
    m = 10 if theme.COMPACT else 14
    lay.setContentsMargins(m, m - 2, m, m)
    lay.setSpacing(8)
    head = QHBoxLayout()
    head.addWidget(label(title, "CardTitle"))
    head.addStretch()
    if subtitle:
        head.addWidget(label(subtitle, "CardAction"))
    lay.addLayout(head)
    return f, lay


class ValueSlider(QWidget):
    """Name, slider, live value; works in floats (0.1 resolution)."""

    changed = pyqtSignal(float)
    SCALE = 10

    def __init__(self, name, lo, hi, unit="°", decimals=1):
        super().__init__()
        self.unit, self.decimals = unit, decimals
        lay = QGridLayout(self)
        lay.setContentsMargins(0, 0, 0, 0)
        lay.setHorizontalSpacing(8)
        lay.setVerticalSpacing(0)
        n = label(name, "RowName")
        n.setMinimumWidth(78)
        lay.addWidget(n, 0, 0)
        self.slider = QSlider(Qt.Horizontal)
        self.slider.valueChanged.connect(lambda v: self._emit())
        lay.addWidget(self.slider, 0, 1)
        self.value = label("", "MetricSmall")
        self.value.setMinimumWidth(62)
        self.value.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
        lay.addWidget(self.value, 0, 2)
        self.range_lbl = label("", "Muted")
        lay.addWidget(self.range_lbl, 1, 1)
        lay.setColumnStretch(1, 1)
        self.set_range(lo, hi)

    def set_range(self, lo, hi):
        self.slider.setRange(int(round(lo * self.SCALE)), int(round(hi * self.SCALE)))
        self.range_lbl.setText(f"{lo:.{self.decimals}f}{self.unit} … {hi:.{self.decimals}f}{self.unit}")

    def get(self):
        return self.slider.value() / self.SCALE

    def set(self, v, emit=False):
        self.slider.blockSignals(not emit)
        self.slider.setValue(int(round(v * self.SCALE)))
        self.slider.blockSignals(False)
        self._show()

    def _show(self):
        self.value.setText(f"{self.get():.{self.decimals}f}{self.unit}")

    def _emit(self):
        self._show()
        self.changed.emit(self.get())


# ─────────────────────────────────────────────────────────────────────────────
# Main window
# ─────────────────────────────────────────────────────────────────────────────
class GripperWindow(QMainWindow):
    def __init__(self, interactor_cls=QtInteractor):
        super().__init__()
        self.setWindowTitle("ELIOS Gripper Simulation")
        self.servo = 0.0                 # commanded (shown) servo angle
        self.target = None               # where the timer is moving it, if anywhere
        self.cycling = False
        self.pitch = self.yaw = 0.0

        split = QSplitter(Qt.Horizontal)
        split.setChildrenCollapsible(False)
        self.setCentralWidget(split)

        # left: 3D
        view_card, vlay = card("Gripper", "Y-type · 2-jaw radial")
        host = QWidget()
        hl = QVBoxLayout(host)
        hl.setContentsMargins(0, 0, 0, 0)
        self.plotter = interactor_cls(host)
        hl.addWidget(self.plotter.interactor if hasattr(self.plotter, "interactor") else self.plotter)
        vlay.addWidget(host, 1)
        self.scene = GripperScene(self.plotter)
        self.g = self.scene.gripper
        self.servo = self.g.open_limit   # start fully open
        self.pose_lbl = label("", "Mono")
        vlay.addWidget(self.pose_lbl)
        split.addWidget(view_card)

        # right: controls
        side = QWidget()
        sl = QVBoxLayout(side)
        sl.setContentsMargins(0, 0, 0, 0)
        sl.setSpacing(10)
        sl.addWidget(self._jaw_card())
        sl.addWidget(self._wrist_card())
        sl.addWidget(self._readout_card())
        sl.addWidget(self._view_card())
        sl.addStretch()
        scroll = QScrollArea()
        scroll.setWidget(side)
        scroll.setWidgetResizable(True)
        scroll.setFrameShape(QFrame.NoFrame)
        scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        scroll.setMinimumWidth(360)
        split.addWidget(scroll)
        split.setStretchFactor(0, 1)
        split.setStretchFactor(1, 0)
        split.setSizes([1100, 400])

        self.timer = QTimer(self)
        self.timer.timeout.connect(self._tick)
        self.setStyleSheet(theme.STYLE_SHEET + (theme.COMPACT_STYLE_SHEET if theme.COMPACT else ""))
        self._apply()

    # ── cards ──
    def _jaw_card(self):
        f, lay = card("Jaws", "servo drives Rod Geared")
        self.open_sl = ValueSlider("Opening", 0, 100, " %")
        self.open_sl.changed.connect(lambda v: self._set_servo(self.g.servo_for_percent(v)))
        lay.addWidget(self.open_sl)
        self.servo_sl = ValueSlider("Servo", self.g.reach_limit, self.g.closed_limit)
        self.servo_sl.changed.connect(self._set_servo)
        lay.addWidget(self.servo_sl)
        row = QHBoxLayout()
        for text, fn, name in (("Open", lambda: self._move_to(self.g.open_limit), "Primary"),
                               ("Close", lambda: self._move_to(self.g.closed_limit), "Primary"),
                               ("Home", lambda: self._move_to(0.0), "")):
            b = QPushButton(text)
            if name:
                b.setObjectName(name)
            b.clicked.connect(fn)
            row.addWidget(b)
        self.cycle_btn = QPushButton("Cycle")
        self.cycle_btn.setObjectName("Toggle")
        self.cycle_btn.setCheckable(True)
        self.cycle_btn.setToolTip("Open and close continuously")
        self.cycle_btn.toggled.connect(self._cycle)
        row.addWidget(self.cycle_btn)
        lay.addLayout(row)
        note = label("Opening 100 % = widest gap; the servo can fold the jaws further back than that. "
                     "Home = servo 0°, the pose in gripper_kinematics.FCStd. The servo gear and the "
                     "Rod Geared gear are equal (1:1, opposite turn); the second jaw mirrors the first.",
                     "Muted")
        note.setWordWrap(True)
        lay.addWidget(note)
        return f

    def _wrist_card(self):
        f, lay = card("Wrist")
        self.pitch_sl = ValueSlider("Pitch", *gk.PITCH_RANGE)
        self.pitch_sl.changed.connect(lambda v: self._set_wrist(pitch=v))
        lay.addWidget(self.pitch_sl)
        self.yaw_sl = ValueSlider("Yaw", *gk.YAW_RANGE)
        self.yaw_sl.changed.connect(lambda v: self._set_wrist(yaw=v))
        lay.addWidget(self.yaw_sl)
        b = QPushButton("Centre wrist")
        b.clicked.connect(lambda: (self.pitch_sl.set(0, True), self.yaw_sl.set(0, True)))
        lay.addWidget(b)
        return f

    def _readout_card(self):
        f, lay = card("Linkage")
        grid = QGridLayout()
        grid.setHorizontalSpacing(12)
        grid.setVerticalSpacing(4)
        self.read = {}
        rows = [("gap", "Jaw gap"), ("servo", "Servo gear"), ("crank", "Rod Geared"),
                ("rocker", "Rod"), ("finger", "Finger tilt"), ("tip", "Tip (gripper)"),
                ("tip_w", "Tip (wrist)")]
        for i, (k, name) in enumerate(rows):
            grid.addWidget(label(name, "Secondary"), i, 0)
            v = label("—", "Metric" if k == "gap" else "MetricSmall")
            v.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
            grid.addWidget(v, i, 1)
            self.read[k] = v
        lay.addLayout(grid)
        note = label(f"Angles are directions in the XY plane (0° = +X). Jaws touch at servo "
                     f"{self.g.closed_limit:.1f}°, widest gap {self.g.max_gap:.1f} mm at "
                     f"{self.g.open_limit:.1f}°.", "Muted")
        note.setWordWrap(True)
        lay.addWidget(note)
        return f

    def _view_card(self):
        f, lay = card("View")
        checks = [("Linkage + pivots", True, self.scene.show_linkage),
                  ("Finger-tip paths", True, self.scene.show_paths),
                  ("See-through body", False, self.scene.see_through),
                  ("Wrist", True, self.scene.show_wrist)]
        grid = QGridLayout()
        for i, (text, on, fn) in enumerate(checks):
            cb = QCheckBox(text)
            cb.setChecked(on)
            cb.toggled.connect(fn)
            fn(on)
            grid.addWidget(cb, i // 2, i % 2)
        lay.addLayout(grid)
        row = QHBoxLayout()
        for name in ("top", "side", "front", "iso"):
            b = QPushButton(name.capitalize())
            b.clicked.connect(lambda _, n=name: self.scene.view(n))
            row.addWidget(b)
        lay.addLayout(row)
        return f

    # ── control ──
    def _set_servo(self, s):
        self.target = None
        if self.cycling:
            self.cycle_btn.setChecked(False)
        self.servo = float(np.clip(s, self.g.reach_limit, self.g.closed_limit))
        self._apply()

    def _set_wrist(self, pitch=None, yaw=None):
        if pitch is not None:
            self.pitch = pitch
        if yaw is not None:
            self.yaw = yaw
        self._apply()

    def _move_to(self, s):
        if self.cycling and s not in (self.g.open_limit, self.g.closed_limit):
            self.cycle_btn.setChecked(False)
        self.target = s
        self.timer.start(TICK_MS)

    def _cycle(self, on):
        self.cycling = on
        if on:
            mid = 0.5 * (self.g.open_limit + self.g.closed_limit)
            self._move_to(self.g.closed_limit if self.servo < mid else self.g.open_limit)

    def _tick(self):
        if self.target is None:
            self.timer.stop()
            return
        step = SERVO_SPEED * TICK_MS / 1000.0
        d = self.target - self.servo
        if abs(d) <= step:
            self.servo = self.target
            if self.cycling:
                self.target = self.g.open_limit if self.target == self.g.closed_limit else self.g.closed_limit
            else:
                self.target = None
        else:
            self.servo += step if d > 0 else -step
        self._apply()

    def _apply(self):
        jaw = self.scene.update(self.servo, self.pitch, self.yaw)
        if jaw is None:
            return
        self.servo_sl.set(self.servo)
        self.open_sl.set(self.g.percent_for_servo(self.servo))
        r = self.read
        r["gap"].setText(f"{jaw['gap']:.1f} mm")
        r["servo"].setText(f"{self.servo:+.1f}°")
        r["crank"].setText(f"{jaw['crank']:.1f}°")
        r["rocker"].setText(f"{jaw['rocker'] % 360:.1f}°")
        r["finger"].setText(f"{jaw['finger_rot'] - 90.0:+.1f}°")
        t, tw = jaw["tip"], jaw["tip_world"]
        r["tip"].setText(f"{t[0]:.1f},{t[1]:.1f}, {t[2]:.1f}")
        r["tip_w"].setText(f"{tw[0]:.1f}, {tw[1]:.1f}, {tw[2]:.1f}")
        self.pose_lbl.setText(f"servo {self.servo:+6.1f}°   opening {self.g.percent_for_servo(self.servo):5.1f} %   "
                              f"gap {jaw['gap']:5.1f} mm   pitch {self.pitch:+.1f}°   yaw {self.yaw:+.1f}°")

    def closeEvent(self, e):
        self.timer.stop()
        self.scene.close()
        super().closeEvent(e)


def main():
    if sys.platform == "win32":
        import ctypes
        ctypes.windll.shell32.SetCurrentProcessExplicitAppUserModelID("ELIO.GripperSim")
    app = QApplication(sys.argv)
    try:
        from hil import config
        if os.path.exists(config.APP_ICON_FILE):
            app.setWindowIcon(QIcon(config.APP_ICON_FILE))
        if "Zen Dots" not in QFontDatabase().families() and os.path.exists(config.BRAND_FONT_FILE):
            QFontDatabase.addApplicationFont(config.BRAND_FONT_FILE)
    except Exception:
        pass
    avail = app.primaryScreen().availableGeometry()
    theme.COMPACT = avail.width() <= theme.COMPACT_MAX_W or avail.height() <= theme.COMPACT_MAX_H
    w = GripperWindow()
    w.resize(int(avail.width() * 0.9), int(avail.height() * 0.9))
    w.showMaximized()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
