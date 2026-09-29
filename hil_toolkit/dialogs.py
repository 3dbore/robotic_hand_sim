"""Secondary windows: the joint-angle graph and the camera calibration dialog."""
import datetime
import os
import time

from PyQt5.QtWidgets import (QApplication, QDialog, QVBoxLayout, QGridLayout,
                             QComboBox, QSpinBox, QPushButton, QLabel, QTextEdit)
import pyqtgraph as pg

from calibration import calibrate, CameraModel

from .theme import C
from .paths import CALIB_DIR, CAMERA_MODEL_FILE
from .hardware_mapping import JOINT_NAMES, JOINT_COLORS, CALIB_IMAGE_W, CALIB_IMAGE_H
from .calibration_camera import collect_correspondences, format_calib_stats
from .widgets import make_card, group_label


class JointGraphWindow(QDialog):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWindowTitle("Joint Rotational Movement")
        self.resize(820, 560)

        layout = QVBoxLayout(self)
        layout.setContentsMargins(24, 24, 24, 24)
        card, card_lay = make_card("Joint Rotational Movement", "G-code run · sim degrees")
        layout.addWidget(card)

        self.plot_widget = pg.PlotWidget()
        self.plot_widget.setBackground(C["surface_lowest"])
        self.plot_widget.showGrid(x=True, y=True, alpha=0.15)
        for side in ("left", "bottom"):
            ax = self.plot_widget.getAxis(side)
            ax.setPen(pg.mkPen(C["surface_dim"]))
            ax.setTextPen(pg.mkPen(C["outline"]))
        self.plot_widget.setLabel("left", "Angle (deg)", color=C["outline"])
        self.plot_widget.setLabel("bottom", "Time (s)", color=C["outline"])
        legend = self.plot_widget.addLegend(offset=(10, 10))
        legend.setLabelTextColor(C["on_surface_variant"])
        card_lay.addWidget(self.plot_widget, 1)

        self.t_data = []
        self.y_data = [[] for _ in range(5)]
        self.lines = [
            self.plot_widget.plot([], [], name=JOINT_NAMES[i],
                                  pen=pg.mkPen(color=JOINT_COLORS[i], width=1.8))
            for i in range(5)
        ]

    def update_plot(self, t, angles):
        self.t_data.append(t)
        for i in range(5):
            self.y_data[i].append(angles[i])
            self.lines[i].setData(self.t_data, self.y_data[i])

    def reset_plot(self):
        self.t_data = []
        self.y_data = [[] for _ in range(5)]
        for line in self.lines:
            line.setData([], [])


class CalibrationDialog(QDialog):
    """Stage 1: collect (x, y, z) ↔ (r, c) correspondences from the simulated camera."""

    def __init__(self, gui):
        super().__init__(gui)
        self.gui = gui
        self.setWindowTitle("Camera Calibration")
        self.setModal(True)  # scene meshes are shared: keep the main window idle while posing
        self.resize(680, 520)

        layout = QVBoxLayout(self)
        layout.setContentsMargins(24, 24, 24, 24)
        card, lay = make_card("Correspondences", f"{CALIB_IMAGE_W}×{CALIB_IMAGE_H} px · arm1 frame")
        layout.addWidget(card)

        form = QGridLayout()
        form.setSpacing(6)
        form.addWidget(group_label("Pixel source"), 0, 0)
        self.source = QComboBox()
        self.source.addItem("Idealized — renderer projection", "ideal")
        self.source.addItem("Realistic — blob detection on render", "detected")
        form.addWidget(self.source, 0, 1)
        form.addWidget(group_label("Points"), 1, 0)
        self.n_points = QSpinBox()
        self.n_points.setRange(12, 5000)
        self.n_points.setValue(300)
        form.addWidget(self.n_points, 1, 1)
        form.addWidget(group_label("Seed"), 2, 0)
        self.seed = QSpinBox()
        self.seed.setRange(0, 99999)
        form.addWidget(self.seed, 2, 1)
        lay.addLayout(form)

        self.btn_run = QPushButton("Collect")
        self.btn_run.setObjectName("Primary")
        self.btn_run.clicked.connect(self.run)
        lay.addWidget(self.btn_run)
        self.status = QLabel("Poses are sampled over the effective joint limits; "
                             "the gripper LED is the tracked marker.")
        self.status.setWordWrap(True)
        self.status.setObjectName("Secondary")
        lay.addWidget(self.status)
        self.result = QTextEdit()
        self.result.setReadOnly(True)
        lay.addWidget(self.result, 1)
        self.last_set = None

    def run(self):
        if self.gui.streaming or self.gui.gcode_timer.isActive():
            self.status.setText("Pause streaming and G-code first.")
            return
        source = self.source.currentData()
        n = self.n_points.value()
        self.btn_run.setEnabled(False)

        last = [0.0]

        def progress(k, total, angles):
            self.status.setText(f"Collecting… {k}/{total}")
            now = time.monotonic()
            if now - last[0] > 0.1 or k == total:   # ~10 fps preview
                last[0] = now
                self.gui.preview_calibration_pose(angles)
            QApplication.processEvents()

        try:
            cs, stats = collect_correspondences(source, n, seed=self.seed.value(), progress=progress)
        finally:
            self.btn_run.setEnabled(True)
            self.gui.refresh_views()
        os.makedirs(CALIB_DIR, exist_ok=True)
        stamp = datetime.datetime.now().strftime("%Y%m%d-%H%M%S")
        path = os.path.join(CALIB_DIR, f"correspondences_{source}_{stamp}.csv")
        cs.save_csv(path)
        self.last_set = cs
        lines = format_calib_stats(stats) + [f"Saved: {path}", ""]
        self.gui.log_message(f"Calibration: {len(cs)} {source} correspondences → {os.path.basename(path)}", "ok")
        if len(cs) < 8:
            self.result.setPlainText("\n".join(lines + ["Too few points to solve."]))
            return

        model, diag = calibrate(cs)
        model.meta["correspondences"] = os.path.basename(path)
        model.save_json(CAMERA_MODEL_FILE)
        self.gui.set_camera_model(model)
        np_fmt = lambda v: "[" + ", ".join(f"{x:+.4f}" for x in v) + "]"
        lines += ["Solved (Appendix E, principal point = image centre):",
                  f"  R = {np_fmt(model.R[0])}", f"      {np_fmt(model.R[1])}", f"      {np_fmt(model.R[2])}",
                  f"  T = [{model.T[0]:.2f}, {model.T[1]:.2f}, {model.T[2]:.2f}] mm",
                  f"  f_x {model.fx:.2f} px, f_y {model.fy:.2f} px, o_r {model.o_r:.1f}, o_c {model.o_c:.1f}",
                  f"  Reprojection RMS {diag['reproj_rms']:.3f} px (max {diag['reproj_max']:.3f})",
                  f"Saved: {CAMERA_MODEL_FILE}"]
        self.result.setPlainText("\n".join(lines))
        self.status.setText(("Done" if len(cs) == n else f"Only {len(cs)}/{n} points found")
                            + " — distance guides updated in Camera View.")
        self.gui.log_message(f"Calibration solved: reprojection RMS {diag['reproj_rms']:.3f} px.", "ok")
