"""Secondary windows: the joint-angle graph and the camera calibration dialog."""
import datetime
import os
import time

import numpy as np
from PyQt5.QtCore import QTimer
from PyQt5.QtWidgets import (QApplication, QDialog, QVBoxLayout, QHBoxLayout, QGridLayout,
                             QComboBox, QSpinBox, QDoubleSpinBox, QPushButton, QLabel,
                             QTextEdit, QMessageBox)
import pyqtgraph as pg

from calibration import calibrate, CameraModel

from .theme import C
from .paths import CALIB_DIR, CAMERA_MODEL_FILE
from .sim_loader import ase
from .hardware_mapping import (JOINT_NAMES, JOINT_COLORS, CALIB_IMAGE_W, CALIB_IMAGE_H,
                               SERVO_CHANNELS, GRIPPER, COMMAND_RESOLUTION_DEG, quantize,
                               PULSE_LIMIT_MIN_DEG, PULSE_LIMIT_MAX_DEG, save_axis_offsets)
from .serial_link import STAT_COUNTERS
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


class AxisOffsetDialog(QDialog):
    """Jog each axis until the real initial pose matches the sim's, then save the
    corrections. Non-modal so the main window's stream and pose controls stay usable.
    Offsets act live on the stream; Save is what makes them survive a restart."""

    STEP_PRESETS = (0.5, 1.0, 2.0, 5.0)

    def __init__(self, gui):
        super().__init__(gui)
        self.gui = gui
        self.setWindowTitle("Axis Offsets")
        self.resize(760, 520)
        self.saved = self.current_offsets()

        layout = QVBoxLayout(self)
        layout.setContentsMargins(24, 24, 24, 24)
        card, lay = make_card("Initial Pose Correction", "servo degrees")
        layout.addWidget(card)

        self.status = QLabel()
        self.status.setWordWrap(True)
        self.status.setObjectName("Secondary")
        lay.addWidget(self.status)

        step_row = QHBoxLayout()
        step_row.setSpacing(6)
        step_row.addWidget(group_label("Move per click"))
        self.step = QDoubleSpinBox()
        self.step.setDecimals(1)
        self.step.setRange(COMMAND_RESOLUTION_DEG, 30.0)
        self.step.setSingleStep(COMMAND_RESOLUTION_DEG)
        self.step.setSuffix("°")
        self.step.setValue(1.0)
        self.step.setToolTip(f"Any amount; snaps to {COMMAND_RESOLUTION_DEG:g}° steps")
        self.step.editingFinished.connect(lambda: self.step.setValue(self.step_size()))
        step_row.addWidget(self.step)
        for value in self.STEP_PRESETS:
            if value >= COMMAND_RESOLUTION_DEG:
                preset = QPushButton(f"{value:g}°")
                preset.setObjectName("Pad")
                preset.clicked.connect(lambda _, v=value: self.step.setValue(v))
                step_row.addWidget(preset)
        step_row.addStretch()
        lay.addLayout(step_row)

        grid = QGridLayout()
        grid.setHorizontalSpacing(12)
        grid.setVerticalSpacing(6)
        for col, title in enumerate(("Axis", "Home", "", "Offset", "", "New home", "Sent", "Output")):
            if title:
                grid.addWidget(group_label(title), 0, col)
        self.rows = []
        for r, ch in enumerate(SERVO_CHANNELS, start=1):
            name = QLabel(ch.name)
            name.setObjectName("RowName")
            name.setToolTip(ch.note)
            home = QLabel(f"{ch.home}°")
            home.setObjectName("Mono")
            minus = self.make_jog_button("−", ch, -1)
            plus = self.make_jog_button("+", ch, +1)
            offset, new_home, sent, output = (self.make_value_label() for _ in range(4))
            for col, w in enumerate((name, home, minus, offset, plus, new_home, sent, output)):
                grid.addWidget(w, r, col)
            self.rows.append(dict(ch=ch, minus=minus, plus=plus, offset=offset,
                                  new_home=new_home, sent=sent, output=output))
        grid.setColumnStretch(0, 1)
        lay.addLayout(grid)

        self.limit_note = QLabel()
        self.limit_note.setWordWrap(True)
        self.limit_note.setObjectName("Muted")
        lay.addWidget(self.limit_note)

        btns = QHBoxLayout()
        btns.setSpacing(6)
        self.btn_home = QPushButton("Go to Initial Pose")
        self.btn_home.clicked.connect(self.go_to_initial_pose)
        self.btn_zero = QPushButton("Zero All")
        self.btn_zero.setObjectName("Danger")
        self.btn_zero.clicked.connect(lambda: self.set_offsets(dict.fromkeys(self.saved, 0.0)))
        self.btn_revert = QPushButton("Revert to Saved")
        self.btn_revert.clicked.connect(lambda: self.set_offsets(self.saved))
        self.btn_save = QPushButton("Save")
        self.btn_save.setObjectName("Primary")
        self.btn_save.clicked.connect(self.save)
        for b in (self.btn_home, self.btn_zero, self.btn_revert):
            btns.addWidget(b)
        btns.addStretch()
        btns.addWidget(self.btn_save)
        lay.addLayout(btns)

        lay.addWidget(group_label("Link Health"))
        self.health = QLabel()
        self.health.setObjectName("Mono")
        lay.addWidget(self.health)
        note = QLabel("Output is the pulse the board is really generating, read back from the "
                      "hardware and rounded to the driver's 0.44° step. These servos report no "
                      "shaft angle, so it cannot show a joint that was pushed off position or "
                      "lost servo power.")
        note.setWordWrap(True)
        note.setObjectName("Muted")
        lay.addWidget(note)
        lay.addStretch()

        self.timer = QTimer(self)
        self.timer.timeout.connect(self.refresh)

    # ── Building blocks ──────────────────────────────────────────────────────
    def make_jog_button(self, text, ch, direction):
        btn = QPushButton(text)
        btn.setObjectName("Pad")
        btn.setAutoRepeat(True)   # hold to keep moving; firmware still limits the speed
        btn.setAutoRepeatDelay(400)
        btn.setAutoRepeatInterval(150)
        btn.clicked.connect(lambda: self.jog(ch, direction))
        return btn

    @staticmethod
    def make_value_label():
        label = QLabel()
        label.setObjectName("Mono")
        label.setMinimumWidth(52)   # room for "-180.0°", so columns hold still as values change
        return label

    @staticmethod
    def current_offsets():
        return {ch.name: ch.offset for ch in SERVO_CHANNELS}

    def step_size(self):
        return max(COMMAND_RESOLUTION_DEG, quantize(self.step.value()))

    def is_dirty(self):
        return self.current_offsets() != self.saved

    def block_reason(self):
        """Why jogging is unavailable right now, or None."""
        gui = self.gui
        if not (gui.link.is_open() and gui.board_ready):
            return "Connect to the board, then enable Stream to Arm to jog."
        if gui.twoway_switch.isChecked():
            return "Offsets are jogged in stream mode. Turn Two-way off, then enable Stream to Arm."
        if not gui.streaming:
            return "Enable Stream to Arm to jog."
        if gui.gcode_runner is not None:
            return "A G-code program is loaded. Press Reset on it to jog."
        return None

    @staticmethod
    def at_initial_pose(gui):
        return not np.any(np.abs(ase.current_angles) > 0.5) and gui.gripper_cmd == GRIPPER.home

    # ── Actions ──────────────────────────────────────────────────────────────
    def jog(self, ch, direction):
        if self.block_reason() is None:
            ch.set_offset(ch.offset + direction * self.step_size())
            self.gui.on_offsets_changed()
            self.refresh()

    def set_offsets(self, offsets):
        for ch in SERVO_CHANNELS:
            ch.set_offset(offsets[ch.name])
        self.gui.on_offsets_changed()
        self.refresh()

    def go_to_initial_pose(self):
        self.gui.reset_arm_ui()
        self.gui.grip_slider.setValue(GRIPPER.home)
        self.refresh()

    def save(self):
        try:
            save_axis_offsets()
        except OSError as e:
            self.gui.log_message(f"Could not save axis offsets: {e}", "err")
            self.status.setText(f"Could not save: {e}")
            return False
        self.saved = self.current_offsets()
        applied = ", ".join(f"{k} {v:+g}°" for k, v in self.saved.items() if v) or "none"
        self.gui.log_message(f"Axis offsets saved: {applied}.", "ok")
        self.gui.log_effective_limits()
        self.refresh()
        return True

    # ── Live state ───────────────────────────────────────────────────────────
    @staticmethod
    def range_note():
        """What the shifted ranges mean for the servos, or "" while all is standard."""
        beyond = [f"{ch.name} ({ch.range_min:g}…{ch.range_max:g}°)" for ch in SERVO_CHANNELS
                  if ch.range_min < 0 or ch.range_max > 180]
        capped = [ch.name for ch in SERVO_CHANNELS if ch.past_pulse_limit()]
        parts = []
        if beyond:
            parts.append("Range past the standard 0–180° pulse: " + ", ".join(beyond) + ". Back off "
                         "if a servo buzzes at the end of its travel: holding against its stop "
                         "overheats it.")
        if capped:
            parts.append(f"Commands stop at the {PULSE_LIMIT_MIN_DEG}…{PULSE_LIMIT_MAX_DEG}° "
                         "pulse limit on: " + ", ".join(capped) + ".")
        return " ".join(parts)

    def refresh(self):
        gui = self.gui
        blocked = self.block_reason()
        cmds = gui.compute_commands()[0]
        out = gui.fw_stat["out"] if gui.fw_stat else None
        for i, (row, cmd) in enumerate(zip(self.rows, cmds)):
            ch = row["ch"]
            row["minus"].setEnabled(blocked is None)
            row["plus"].setEnabled(blocked is None)
            row["offset"].setText(f"{ch.offset:+.1f}°")
            row["new_home"].setText(f"{ch.effective_home:.1f}°")
            row["sent"].setText(f"{cmd:.1f}°" if gui.streaming else "–")
            has_out = out is not None and i < len(out)
            row["output"].setText("–" if not has_out else "off" if out[i] is None else f"{out[i]:.1f}°")

        if blocked:
            self.status.setText(blocked)
        elif not self.at_initial_pose(gui):
            self.status.setText("The sim is away from its initial pose. "
                                "Press Go to Initial Pose so you compare the same pose.")
        else:
            self.status.setText("Jog each axis until the real arm matches the sim's initial pose, "
                                "then Save. + raises the servo angle, whichever way that turns the joint.")
        self.limit_note.setText(self.range_note())

        dirty = self.is_dirty()
        self.btn_save.setEnabled(dirty)
        self.btn_revert.setEnabled(dirty)
        self.btn_zero.setEnabled(any(ch.offset for ch in SERVO_CHANNELS))

        if gui.fw_stat:
            text = " · ".join(f"{k} {gui.fw_stat[k]}" for k in STAT_COUNTERS)
        else:
            text = "no reports from the board"
        if gui.fw_boot_reason:
            text += f" · boot {gui.fw_boot_reason}"
        self.health.setText(text)

    def showEvent(self, event):
        super().showEvent(event)
        self.refresh()
        self.timer.start(200)

    def hideEvent(self, event):
        self.timer.stop()
        super().hideEvent(event)

    def reject(self):
        self.close()   # Esc takes the same unsaved-changes path as the close button

    def closeEvent(self, event):
        if self.is_dirty():
            answer = QMessageBox.question(
                self, "Unsaved Offsets", "Save the axis offsets before closing?",
                QMessageBox.Save | QMessageBox.Discard | QMessageBox.Cancel, QMessageBox.Save)
            if answer == QMessageBox.Cancel or (answer == QMessageBox.Save and not self.save()):
                event.ignore()
                return
            if answer == QMessageBox.Discard:
                self.set_offsets(self.saved)
        event.accept()   # not QDialog.closeEvent: it calls reject(), which lands back here


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
        if self.gui.streaming or self.gui.twoway_switch.isChecked() or self.gui.gcode_timer.isActive():
            self.status.setText("Pause streaming, Two-way and G-code first.")
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
