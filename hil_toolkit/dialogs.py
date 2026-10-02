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
from .tags_toolkit.tag_model import CALIB_TAG, CALIB_TAG_FAMILY, CALIB_TAG_SIZE, CALIB_TAG_PRINT
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


TAG_INFO = (f"Target: {CALIB_TAG_FAMILY} id {CALIB_TAG.tag_id}, the tag on the hand's bottom face "
            f"(last two axes). Print {CALIB_TAG_PRINT:.0f} mm, black square {CALIB_TAG_SIZE:.2f} mm. "
            "The wrist pitches up to show it to the camera. "
            "Runs the calibration G-code (slow, ≤ 20°/s per joint, streamed to the arm if streaming) "
            "and captures the tag at every M240.")
LED_INFO = ("Poses are sampled at random over the effective joint limits (sim only — the jumps "
            "between poses are not a real-arm motion); the gripper LED is the tracked marker.")


class CalibrationDialog(QDialog):
    """Stage 1: collect (x, y, z) ↔ (r, c) correspondences from the simulated camera —
    the AprilTag calibration G-code (default, runnable on the real arm) or random
    LED poses."""

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
        form.addWidget(group_label("Method"), 0, 0)
        self.source = QComboBox()
        self.source.addItem("AprilTag on the hand — calibration G-code", "apriltag")
        self.source.addItem("LED, random poses — renderer projection", "ideal")
        self.source.addItem("LED, random poses — blob detection on render", "detected")
        self.source.currentIndexChanged.connect(self.on_method_changed)
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
        self.status = QLabel()
        self.status.setWordWrap(True)
        self.status.setObjectName("Secondary")
        lay.addWidget(self.status)
        self.result = QTextEdit()
        self.result.setReadOnly(True)
        lay.addWidget(self.result, 1)
        self.last_set = None
        self.on_method_changed()

    def on_method_changed(self):
        tag = self.source.currentData() == "apriltag"
        self.n_points.setEnabled(not tag)
        self.seed.setEnabled(not tag)
        self.btn_run.setText("Run Calibration G-code" if tag else "Collect")
        self.status.setText(TAG_INFO if tag else LED_INFO)

    def show_tag_result(self, text):
        """Called by the main window when the calibration G-code has finished."""
        self.source.setCurrentIndex(self.source.findData("apriltag"))
        self.result.setPlainText(text)
        self.status.setText("Done — camera model and ground truth saved; distance guides updated.")
        self.show()
        self.raise_()

    def run_apriltag(self):
        if self.gui.gcode_runner is not None:
            self.status.setText("Reset the G-code program in the main window first.")
            return
        self.result.setPlainText("Running the calibration G-code in the main window — "
                                 "this dialog reopens with the result when it ends.")
        self.hide()   # modal: the main window must stay usable to watch / pause the run
        if not self.gui.start_tag_calibration():
            self.show()
            self.status.setText("Could not start — see the System Log.")

    def run(self):
        source = self.source.currentData()
        if source == "apriltag":
            self.run_apriltag()
            return
        if self.gui.streaming or self.gui.gcode_timer.isActive():
            self.status.setText("Pause streaming and G-code first.")
            return
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
