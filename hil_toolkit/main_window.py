"""The main application window: layout, live telemetry, G-code and hardware link."""
import datetime
import os

import numpy as np
import pyvista as pv
from PyQt5.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout,
                             QHBoxLayout, QLabel, QFrame, QTextEdit, QGridLayout,
                             QPushButton, QComboBox, QSlider, QSizePolicy)
from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtGui import QColor, QTextCursor, QTextFormat
from pyvistaqt import QtInteractor

from calibration import CameraModel

from . import theme
from .theme import C
from .paths import CAMERA_MODEL_FILE
from .sim_loader import ase
from .hardware_mapping import (
    SERVO_CHANNELS, GRIPPER, SERIAL_BAUD, BOARD_RESET_MS, STREAM_INTERVAL_MS,
    apply_effective_limits, CAM_ASPECT, CAM_FOV_H, CAM_FOV_V, CAM_FOV_D,
)
from .serial_link import SerialLink, serial
from .calibration_camera import camera_pose
from .gcode import GCodeProgram, GCodeError, GCodeRunner, DEFAULT_GCODE, GCODE_HELP, GCODE_TICK_MS
from .widgets import (StatusDot, AspectBox, FocusSplitter, ServoRow, make_card,
                      group_label, add_soft_shadow, add_viewfinder, add_distance_guides)
from .dialogs import JointGraphWindow, CalibrationDialog


class HILToolkitGUI(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("ELIOS Hardware-in-the-Loop Toolkit")
        self.setStyleSheet(theme.STYLE_SHEET + (theme.COMPACT_STYLE_SHEET if theme.COMPACT else ""))
        avail = QApplication.primaryScreen().availableGeometry()
        self.resize(min(1680, avail.width()), min(960, avail.height()))

        self.link = SerialLink()
        self.board_ready = False
        self.streaming = False
        self.last_sent = None
        self.frames_sent = 0
        self.gripper_cmd = GRIPPER.home

        root = QWidget()
        root.setObjectName("Page")
        root.setAttribute(Qt.WA_StyledBackground, True)
        self.setCentralWidget(root)
        page = QVBoxLayout(root)
        # DESIGN.md: 24px container margin, 16px gutter (halved on compact screens)
        gap = 8 if theme.COMPACT else 16
        margin = 12 if theme.COMPACT else 24
        page.setContentsMargins(margin, margin, margin, margin)
        page.setSpacing(gap)

        page.addLayout(self.build_header())

        # Row 1: log + gcode | camera ⇆ 3D (resizable) | servo telemetry
        row1 = QHBoxLayout()
        row1.setSpacing(gap)
        left = QVBoxLayout()
        left.setSpacing(gap)
        left.addWidget(self.build_log_card(), 1)
        left.addWidget(self.build_gcode_card(), 1)
        row1.addLayout(left, 3)
        cam_share = 3
        self.view_split = FocusSplitter([cam_share, 5])
        self.view_split.setHandleWidth(gap)
        cam_card = self.build_camera_card()
        viz_card = self.build_viz_card()
        cam_card.setMinimumWidth(160)
        viz_card.setMinimumWidth(220)
        self.view_split.addWidget(cam_card)
        self.view_split.addWidget(viz_card)
        self.view_split.setStretchFactor(0, cam_share)
        self.view_split.setStretchFactor(1, 5)
        row1.addWidget(self.view_split, cam_share + 5)
        row1.addWidget(self.build_telemetry_card(), 3)
        page.addLayout(row1, 5 if theme.COMPACT else 3)

        # Row 2: EE projections | manual override | hardware link
        row2 = QHBoxLayout()
        row2.setSpacing(gap)
        row2.addWidget(self.build_ee_card(), 7)
        row2.addWidget(self.build_override_card(), 3)
        row2.addWidget(self.build_hardware_card(), 4)
        page.addLayout(row2, 3 if theme.COMPACT else 2)

        self.graph_window = JointGraphWindow(self)
        self.calib_dialog = CalibrationDialog(self)

        self.log_message("System initialized.", "ok")
        for w in apply_effective_limits():
            self.log_message(w, "warn")
        self.log_effective_limits()
        if serial is None:
            self.log_message("pyserial not installed — run: pip install pyserial", "err")

        self.setup_ee_workspace_projections()
        self.setup_3d_scene()

        # G-code state
        self.gcode_timer = QTimer()
        self.gcode_timer.timeout.connect(self.gcode_tick)
        self.gcode_runner = None
        self._gcode_shown_line = None
        self.gcode_time_elapsed = 0.0

        # Telemetry + serial streaming
        self.sync_timer = QTimer()
        self.sync_timer.timeout.connect(self.sync_tick)
        self.sync_timer.start(STREAM_INTERVAL_MS)

        self.refresh_ports()
        self.update_link_ui()

    def showEvent(self, event):
        super().showEvent(event)
        if not getattr(self, "_split_initialized", False):
            self._split_initialized = True
            # Apply the default camera/3D split once the real widths are known
            QTimer.singleShot(0, self.view_split.reset_sizes)

    # ── Layout builders ──────────────────────────────────────────────────────
    def build_header(self):
        header = QHBoxLayout()
        header.setSpacing(10)
        titles = QVBoxLayout()
        titles.setSpacing(2)
        title = QLabel("ELIOS Hardware-in-the-Loop")
        title.setObjectName("PageTitle")
        subtitle = QLabel("Hardware-in-the-loop toolkit · simulation mirrored to Arduino Uno (6-axis.ino)")
        subtitle.setObjectName("PageSubtitle")
        titles.addWidget(title)
        titles.addWidget(subtitle)
        header.addLayout(titles)
        header.addStretch()

        pill = QFrame()
        pill.setObjectName("Pill")
        pill.setFixedHeight(26 if theme.COMPACT else 32)
        pill_lay = QHBoxLayout(pill)
        pill_lay.setContentsMargins(10, 0, 14, 0)
        pill_lay.setSpacing(6)
        self.header_dot = StatusDot(C["outline"], pulse=True)
        self.header_status = QLabel("Disconnected")
        self.header_status.setObjectName("Secondary")
        pill_lay.addWidget(self.header_dot)
        pill_lay.addWidget(self.header_status)
        header.addWidget(pill)

        btn_graphs = QPushButton("Joint Graphs")
        btn_graphs.clicked.connect(lambda: self.graph_window.show())
        header.addWidget(btn_graphs)
        btn_calib = QPushButton("Camera Calibration")
        btn_calib.clicked.connect(lambda: self.calib_dialog.show())
        header.addWidget(btn_calib)
        return header

    def build_log_card(self):
        card, lay = make_card("System Log")
        self.log_text = QTextEdit()
        self.log_text.setReadOnly(True)
        lay.addWidget(self.log_text, 1)
        return card

    def build_gcode_card(self):
        self.gcode_status = QLabel("Ready")
        self.gcode_status.setObjectName("CardAction")
        card, lay = make_card("G-Code Execution", self.gcode_status)
        self.gcode_text = QTextEdit()
        self.gcode_text.setAcceptRichText(False)
        self.gcode_text.setLineWrapMode(QTextEdit.NoWrap)
        self.gcode_text.setPlainText(DEFAULT_GCODE)
        self.gcode_text.setToolTip(GCODE_HELP)
        self.gcode_text.setStyleSheet(f"color: {C['on_surface']};")
        lay.addWidget(self.gcode_text, 1)
        btns = QHBoxLayout()
        btns.setSpacing(6)
        run = QPushButton("Run")
        run.setObjectName("Primary")
        add_soft_shadow(run)
        pause = QPushButton("Pause")
        reset = QPushButton("Reset")
        run.clicked.connect(self.run_gcode)
        pause.clicked.connect(self.pause_gcode)
        reset.clicked.connect(self.reset_gcode)
        for b in (run, pause, reset):
            btns.addWidget(b)
        lay.addLayout(btns)
        return card

    def build_camera_card(self):
        card, lay = make_card("Camera View", f"H{CAM_FOV_H:g}° V{CAM_FOV_V:g}° D{CAM_FOV_D:g}°")
        self.cam_plotter = QtInteractor(card)
        self.cam_plotter.set_background(C["surface_low"])
        # Aspect must match the lens so the horizontal FoV comes out at 81°
        # at any size the splitter gives the card.
        lay.addWidget(AspectBox(self.cam_plotter.interactor, CAM_ASPECT), 1)
        return card

    def build_viz_card(self):
        card, lay = make_card("3D Visualization", "sim pose = hardware target")
        self.plotter = QtInteractor(card)
        self.plotter.set_background(C["surface_lowest"])
        lay.addWidget(self.plotter.interactor, 1)
        return card

    def build_telemetry_card(self):
        card, lay = make_card("Servo Telemetry", "command sent per axis")
        lay.setSpacing(6)
        self.servo_rows = {}
        # Kinematic order for reading; serial order is kept in SERVO_CHANNELS
        for ch in [SERVO_CHANNELS[i] for i in (5, 4, 3, 2, 1, 0)]:
            row = ServoRow(ch, self.on_invert_toggled)
            self.servo_rows[ch.name] = row
            lay.addWidget(row)
        lay.addStretch()
        return card

    def build_ee_card(self):
        card, lay = make_card("End-Effector Workspace", "2D projections · reachable within hardware limits")
        self.ee_plotter = QtInteractor(card, shape=(1, 3), border_color=C["surface_dim"], border_width=1)
        lay.addWidget(self.ee_plotter.interactor, 1)
        return card

    def build_override_card(self):
        card, lay = make_card("Manual Override", f"{ase.IK_STEP_SIZE:g} mm · {ase.IK_ROT_STEP:g}° step")
        body = QHBoxLayout()
        body.setSpacing(14)

        hand = QGridLayout()
        hand.setSpacing(4)
        hand.addWidget(group_label("Hand"), 0, 0, 1, 3, Qt.AlignHCenter)
        pads = {
            "+Z": (1, 1), "-X": (2, 0), "+Y": (2, 1), "+X": (2, 2), "-Y": (3, 1), "-Z": (4, 1),
        }
        for label, (r, c) in pads.items():
            b = QPushButton(label)
            b.setObjectName("Pad")
            b.setAutoRepeat(True)
            b.setAutoRepeatInterval(60)
            b.clicked.connect(lambda _, d=label: self.move_arm(d, ase.IK_STEP_SIZE))
            hand.addWidget(b, r, c)

        # Base rotation (J1), directly below the X+ / X- pad buttons.
        base_left = QPushButton("◀ Base")
        base_left.setObjectName("Pad")
        base_left.setAutoRepeat(True)
        base_left.setAutoRepeatInterval(60)
        base_left.clicked.connect(lambda: self.rotate_base_ui(-ase.IK_ROT_STEP))
        hand.addWidget(base_left, 3, 0)

        base_right = QPushButton("Base ▶")
        base_right.setObjectName("Pad")
        base_right.setAutoRepeat(True)
        base_right.setAutoRepeatInterval(60)
        base_right.clicked.connect(lambda: self.rotate_base_ui(ase.IK_ROT_STEP))
        hand.addWidget(base_right, 3, 2)

        body.addLayout(hand)

        right = QVBoxLayout()
        right.setSpacing(4)
        right.addWidget(group_label("End effector"))
        pr = QGridLayout()
        pr.setSpacing(4)
        for (label, axis, amt), (r, c) in zip(
                (("Pitch +", "PITCH", 1), ("Pitch −", "PITCH", -1),
                 ("Roll +", "ROLL", 1), ("Roll −", "ROLL", -1)),
                ((0, 0), (0, 1), (1, 0), (1, 1))):
            b = QPushButton(label)
            b.setAutoRepeat(True)
            b.clicked.connect(lambda _, ax=axis, a=amt: self.rotate_ee_ui(ax, a * ase.IK_ROT_STEP))
            pr.addWidget(b, r, c)
        right.addLayout(pr)
        right.addStretch()
        body.addLayout(right)
        lay.addLayout(body, 1)

        reset = QPushButton("Reset to Initial Position")
        reset.clicked.connect(self.reset_arm_ui)
        lay.addWidget(reset)
        return card

    def build_hardware_card(self):
        self.link_badge = QLabel(f"{SERIAL_BAUD} baud")
        self.link_badge.setObjectName("CardAction")
        card, lay = make_card("Hardware Link", self.link_badge)

        lay.addWidget(group_label("Serial port"))
        port_row = QHBoxLayout()
        port_row.setSpacing(6)
        self.port_combo = QComboBox()
        self.port_combo.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        # Long port descriptions must not force the card (and window) wider
        self.port_combo.setSizeAdjustPolicy(QComboBox.AdjustToMinimumContentsLengthWithIcon)
        self.port_combo.setMinimumContentsLength(10)
        self.btn_scan = QPushButton("Scan")
        self.btn_scan.clicked.connect(self.refresh_ports)
        self.btn_connect = QPushButton("Connect")
        self.btn_connect.setObjectName("Primary")
        add_soft_shadow(self.btn_connect)
        self.btn_connect.clicked.connect(self.toggle_connection)
        port_row.addWidget(self.port_combo, 1)
        port_row.addWidget(self.btn_scan)
        port_row.addWidget(self.btn_connect)
        lay.addLayout(port_row)

        ctl_row = QHBoxLayout()
        ctl_row.setSpacing(6)
        self.btn_stream = QPushButton("Stream to Arm")
        self.btn_stream.setObjectName("Toggle")
        self.btn_stream.setCheckable(True)
        self.btn_stream.toggled.connect(self.set_streaming)
        self.btn_release = QPushButton("Release Servos")
        self.btn_release.setObjectName("Danger")
        self.btn_release.setToolTip("Sends empty fields: firmware detaches every servo (no PWM).")
        self.btn_release.clicked.connect(self.release_servos)
        ctl_row.addWidget(self.btn_stream, 1)
        ctl_row.addWidget(self.btn_release)
        lay.addLayout(ctl_row)

        tx_row = QHBoxLayout()
        tx_col = QVBoxLayout()
        tx_col.setSpacing(2)
        tx_col.addWidget(group_label("Last frame  D2,D3,D4,D5,D6,D8"))
        self.tx_label = QLabel("—")
        self.tx_label.setObjectName("MetricSmall")
        self.tx_label.setTextInteractionFlags(Qt.TextSelectableByMouse)
        tx_col.addWidget(self.tx_label)
        tx_row.addLayout(tx_col, 1)
        cnt_col = QVBoxLayout()
        cnt_col.setSpacing(2)
        cnt_col.addWidget(group_label("Frames"))
        self.frames_label = QLabel("0")
        self.frames_label.setObjectName("MetricSmall")
        cnt_col.addWidget(self.frames_label)
        tx_row.addLayout(cnt_col)
        lay.addLayout(tx_row)

        div = QFrame()
        div.setObjectName("Divider")
        div.setFixedHeight(1)
        lay.addWidget(div)

        grip_head = QHBoxLayout()
        grip_head.addWidget(group_label("Gripper · D2"))
        grip_head.addStretch()
        self.grip_value = QLabel(f"{GRIPPER.home:03d}°")
        self.grip_value.setObjectName("MetricSmall")
        grip_head.addWidget(self.grip_value)
        lay.addLayout(grip_head)
        grip_row = QHBoxLayout()
        grip_row.setSpacing(6)
        btn_open = QPushButton("Open")
        btn_close = QPushButton("Close")
        self.grip_slider = QSlider(Qt.Horizontal)
        self.grip_slider.setRange(GRIPPER.servo_min, GRIPPER.servo_max)
        self.grip_slider.setValue(GRIPPER.home)
        self.grip_slider.valueChanged.connect(self.set_gripper)
        btn_open.clicked.connect(lambda: self.grip_slider.setValue(GRIPPER.servo_min))
        btn_close.clicked.connect(lambda: self.grip_slider.setValue(GRIPPER.servo_max))
        grip_row.addWidget(btn_open)
        grip_row.addWidget(self.grip_slider, 1)
        grip_row.addWidget(btn_close)
        lay.addLayout(grip_row)
        lay.addStretch()
        return card

    # ── Logging ──────────────────────────────────────────────────────────────
    def log_message(self, msg, level="info"):
        colors = {"info": C["on_surface_variant"], "ok": C["primary"], "warn": C["warning"],
                  "err": C["error"], "rx": C["outline"]}
        ts = datetime.datetime.now().strftime("%H:%M:%S")
        safe = msg.replace("&", "&amp;").replace("<", "&lt;").replace(">", "&gt;")
        self.log_text.append(
            f"<span style='color:{C['outline']}'>{ts}</span>&nbsp;&nbsp;"
            f"<span style='color:{colors.get(level, C['on_surface_variant'])}'>{safe}</span>")
        sb = self.log_text.verticalScrollBar()
        sb.setValue(sb.maximum())

    def log_effective_limits(self):
        parts = []
        for ch in SERVO_CHANNELS[1:][::-1]:
            lo, hi = ase.JOINT_LIMITS[ch.sim_joint]
            parts.append(f"{ch.name} {lo:+.0f}…{hi:+.0f}°")
        self.log_message("Effective joint limits (sim ∩ hardware): " + ", ".join(parts))

    # ── Serial link ──────────────────────────────────────────────────────────
    def refresh_ports(self):
        current = self.port_combo.currentData()
        self.port_combo.clear()
        ports = SerialLink.available_ports()
        for device, desc in ports:
            self.port_combo.addItem(f"{device} — {desc}", device)
        if not ports:
            self.port_combo.addItem("No ports found", None)
        idx = self.port_combo.findData(current)
        if idx < 0:
            # Prefer something that looks like an Arduino
            for i, (_, desc) in enumerate(ports):
                if "arduino" in desc.lower() or "ch340" in desc.lower() or "usb serial" in desc.lower():
                    idx = i
                    break
        if idx >= 0:
            self.port_combo.setCurrentIndex(idx)

    def toggle_connection(self):
        if self.link.is_open():
            self.disconnect_link()
            return
        if serial is None:
            self.log_message("pyserial not installed — run: pip install pyserial", "err")
            return
        device = self.port_combo.currentData()
        if not device:
            self.log_message("No serial port selected.", "warn")
            return
        try:
            self.link.open(device, SERIAL_BAUD)
        except Exception as e:
            self.log_message(f"Could not open {device}: {e}", "err")
            return
        self.board_ready = False
        self.log_message(f"Opened {device} @ {SERIAL_BAUD}. Waiting for board reset…")
        QTimer.singleShot(BOARD_RESET_MS, self.on_board_ready)
        self.update_link_ui()

    def on_board_ready(self):
        if self.link.is_open():
            self.board_ready = True
            self.log_message("Board ready. Enable 'Stream to Arm' to drive the hardware.", "ok")
            self.update_link_ui()

    def disconnect_link(self, reason=None):
        if self.btn_stream.isChecked():
            self.btn_stream.setChecked(False)
        self.link.close()
        self.board_ready = False
        self.log_message(reason or "Disconnected.", "err" if reason else "info")
        self.update_link_ui()

    def update_link_ui(self):
        connected = self.link.is_open()
        self.port_combo.setEnabled(not connected)
        self.btn_scan.setEnabled(not connected)
        self.btn_connect.setText("Disconnect" if connected else "Connect")
        self.btn_stream.setEnabled(connected and self.board_ready)
        self.btn_release.setEnabled(connected and self.board_ready)
        for row in self.servo_rows.values():
            if row.inv is not None:
                row.inv.setEnabled(not self.streaming)

        if not connected:
            state, color, glow = "Disconnected", C["outline"], False
        elif not self.board_ready:
            state, color, glow = "Board resetting…", C["warning"], True
        elif self.streaming:
            state, color, glow = f"Streaming · {self.link.port.port}", C["primary"], True
        else:
            state, color, glow = f"Connected · {self.link.port.port} · idle", C["primary"], False
        self.header_status.setText(state)
        self.header_dot.set_state(color, glow)

    def send_line(self, line):
        try:
            self.link.write_line(line)
        except Exception as e:
            self.disconnect_link(f"Serial write failed: {e}")
            return False
        self.last_sent = line
        self.frames_sent += 1
        self.tx_label.setText(line)
        self.frames_label.setText(str(self.frames_sent))
        return True

    def home_line(self):
        return ",".join(str(ch.home) for ch in SERVO_CHANNELS)

    def set_streaming(self, on):
        if on and not (self.link.is_open() and self.board_ready):
            self.btn_stream.setChecked(False)
            return
        self.streaming = on
        if on:
            # First frame attaches each servo instantly at the given angle, so
            # attach at the known initial pose; later frames are speed-limited.
            if self.send_line(self.home_line()):
                if np.any(np.abs(ase.current_angles) > 0.5) or self.gripper_cmd != GRIPPER.home:
                    self.log_message("Streaming: attached at initial position, "
                                     "arm will travel to current sim pose at firmware speed.", "warn")
                else:
                    self.log_message("Streaming enabled.", "ok")
        else:
            self.log_message("Streaming paused. Servos hold their last position.")
        self.update_link_ui()

    def release_servos(self):
        if self.btn_stream.isChecked():
            self.btn_stream.setChecked(False)
        if self.link.is_open():
            if self.send_line("," * (len(SERVO_CHANNELS) - 1)):
                self.log_message("All servos released (no PWM).", "warn")

    def on_invert_toggled(self, ch, checked):
        ch.sign = -1 if checked else 1
        for w in apply_effective_limits():
            self.log_message(w, "warn")
        self.log_message(f"{ch.name}: direction {'inverted' if checked else 'normal'}.", "warn")
        self.log_effective_limits()
        ase.update_scene(ase.current_angles)
        self.refresh_views()

    def set_gripper(self, value):
        self.gripper_cmd = int(value)
        self.grip_value.setText(f"{self.gripper_cmd:03d}°")

    # ── Periodic sync: telemetry + streaming + RX ────────────────────────────
    def compute_commands(self):
        cmds, statuses = [], []
        angles = ase.current_angles
        for ch in SERVO_CHANNELS:
            if ch.sim_joint is None:
                cmds.append(self.gripper_cmd)
                statuses.append("ok")
                continue
            sim_deg = float(angles[ch.sim_joint])
            cmd, clipped = ch.to_servo(sim_deg)
            lo, hi = ase.JOINT_LIMITS[ch.sim_joint]
            if clipped:
                status = "clip"
            elif sim_deg - lo < 1.0 or hi - sim_deg < 1.0:
                status = "near"
            else:
                status = "ok"
            cmds.append(cmd)
            statuses.append(status)
        return cmds, statuses

    def sync_tick(self):
        cmds, statuses = self.compute_commands()
        for ch, cmd, status in zip(SERVO_CHANNELS, cmds, statuses):
            sim_deg = 0.0 if ch.sim_joint is None else float(ase.current_angles[ch.sim_joint])
            lim = None if ch.sim_joint is None else ase.JOINT_LIMITS[ch.sim_joint]
            self.servo_rows[ch.name].refresh(cmd, sim_deg, lim, status)

        if self.link.is_open():
            try:
                for line in self.link.read_lines():
                    self.log_message(f"RX  {line}", "rx")
            except Exception as e:
                self.disconnect_link(f"Serial read failed: {e}")
                return
            if self.streaming:
                line = ",".join(str(c) for c in cmds)
                if line != self.last_sent:
                    self.send_line(line)

    # ── G-code ───────────────────────────────────────────────────────────────
    def run_gcode(self):
        if self.gcode_timer.isActive():
            return
        if self.gcode_runner is None:
            _, _, ee = ase.forward_kinematics(ase.current_angles)
            try:
                program = GCodeProgram(self.gcode_text.toPlainText(), ee,
                                       ase.desired_global_pitch, ase.desired_global_roll)
            except GCodeError as e:
                self.log_message(f"G-code: {e}", "err")
                self.highlight_gcode_line(e.line, C["error_container"])
                self.gcode_status.setText(f"Error · L{e.line}")
                return
            for w in program.warnings:
                self.log_message(f"G-code: {w}", "warn")
            if not program.segments:
                self.log_message("G-code: nothing to run.", "warn")
                return
            self.gcode_runner = GCodeRunner(program)
            self.gcode_time_elapsed = 0.0
            self.graph_window.reset_plot()
            self.gcode_text.setReadOnly(True)
            self.log_message(f"G-code: started · {len(program.segments)} steps · "
                             f"~{program.duration:.0f} s", "ok")
        else:
            self.log_message("G-code: resumed.")
        self.gcode_timer.start(GCODE_TICK_MS)

    def pause_gcode(self):
        if self.gcode_timer.isActive():
            self.gcode_timer.stop()
            self.gcode_status.setText(f"Paused · L{self.gcode_runner.line}")
            self.log_message("G-code: paused.")

    def reset_gcode(self):
        self.gcode_timer.stop()
        self.finish_gcode()
        self.gcode_time_elapsed = 0.0
        self.graph_window.reset_plot()
        self.log_message("G-code: reset.")

    def finish_gcode(self, status="Ready"):
        self.gcode_runner = None
        self._gcode_shown_line = None
        self.gcode_text.setReadOnly(False)
        self.gcode_text.setExtraSelections([])
        self.gcode_status.setText(status)

    def highlight_gcode_line(self, line, color):
        block = self.gcode_text.document().findBlockByNumber(line - 1)
        if not block.isValid():
            return
        sel = QTextEdit.ExtraSelection()
        sel.format.setBackground(QColor(color))
        sel.format.setProperty(QTextFormat.FullWidthSelection, True)
        sel.cursor = QTextCursor(block)
        self.gcode_text.setExtraSelections([sel])
        self.gcode_text.setTextCursor(sel.cursor)  # scrolls the line into view
        self.gcode_text.ensureCursorVisible()

    def gcode_tick(self):
        runner = self.gcode_runner
        dt = GCODE_TICK_MS / 1000.0
        self.gcode_time_elapsed += dt
        line = runner.line
        events = runner.tick(dt)
        ase.update_scene(ase.current_angles)
        self.refresh_views()
        self.graph_window.update_plot(self.gcode_time_elapsed, ase.current_angles)

        for kind, payload in events:
            if kind == "warn":
                self.log_message(f"G-code: {payload}", "warn")
            elif kind == "grip":
                self.grip_slider.setValue(payload)
                self.log_message(f"G-code: gripper → {payload:03d}°")
            elif kind == "pause":
                self.gcode_timer.stop()
                self.gcode_status.setText(f"Paused · L{payload}")
                self.log_message(f"G-code: M0 pause at line {payload} — press Run to continue.", "warn")
            elif kind == "end":
                self.gcode_timer.stop()
                self.finish_gcode("Done")
                self.log_message(f"G-code: completed in {self.gcode_time_elapsed:.1f} s.", "ok")
                return

        line = runner.line or line
        if line is not None and self.gcode_timer.isActive():
            if line != self._gcode_shown_line:
                self.highlight_gcode_line(line, C["primary_fixed"])
                self._gcode_shown_line = line
            self.gcode_status.setText(f"Running · L{line}")

    # ── Manual override ──────────────────────────────────────────────────────
    def refresh_views(self):
        self.update_camera_view()
        self.update_ee_plot()
        self.plotter.interactor.Render()

    def move_arm(self, direction, amount):
        ase.move_ee(direction, amount)
        self.refresh_views()

    def rotate_ee_ui(self, axis, amount):
        ase.rotate_ee(axis, amount)
        self.refresh_views()

    def rotate_base_ui(self, amount):
        ase.rotate_base(amount)
        self.refresh_views()

    def reset_arm_ui(self):
        ase.current_angles = np.zeros(5)
        ase.cam_elevation = 0.0
        ase.desired_global_pitch = 0.0
        ase.desired_global_roll = 0.0
        ase.update_scene(ase.current_angles)
        self.refresh_views()
        self.log_message("Manual override: reset to initial position.")

    # ── Scene rendering ──────────────────────────────────────────────────────
    def update_camera_view(self):
        # Camera rides on arm1: only the J1 (base, Z-axis) rotation applies
        pos, focal, up = camera_pose(ase.current_angles[0])
        self.cam_plotter.camera.position = pos
        self.cam_plotter.camera.focal_point = focal
        self.cam_plotter.camera.up = up
        self.cam_plotter.interactor.Render()

    def preview_calibration_pose(self, angles):
        """Show a calibration sample in both views, with the Camera View turned by
        that sample's θ1 — what the calibration camera saw. Display only:
        ase.current_angles (the streamed pose) is untouched."""
        ase.update_scene(angles)
        pos, focal, up = camera_pose(angles[0])
        self.cam_plotter.camera.position = pos
        self.cam_plotter.camera.focal_point = focal
        self.cam_plotter.camera.up = up
        self.cam_plotter.interactor.Render()
        self.plotter.interactor.Render()

    def update_ee_plot(self):
        if getattr(self, "dot_xy_actor", None) is None:
            return
        _, _, ee = ase.forward_kinematics(ase.current_angles)
        self.dot_xy_actor.position = (ee[0], ee[1], 0.0)
        self.dot_yz_actor.position = (ee[1], ee[2], 0.0)
        self.dot_xz_actor.position = (ee[0], ee[2], 0.0)
        self.ee_plotter.interactor.Render()

    def setup_3d_scene(self):
        floor = pv.Plane(center=(0, -200, 0), direction=(0, 0, 1), i_size=800, j_size=800,
                         i_resolution=16, j_resolution=16)
        for p in (self.plotter, self.cam_plotter):
            p.add_mesh(floor, style="wireframe", color=C["surface_dim"], line_width=1.2)
        self.plotter.add_axes_at_origin(labels_off=True, line_width=3)

        shading = dict(show_edges=False, smooth_shading=True, specular=0.5, specular_power=25)
        for p in (self.plotter, self.cam_plotter):
            for arm_name, cfg in ase.ARM_CONFIG.items():
                p.add_mesh(ase.loaded_meshes[arm_name], color=cfg["color"], opacity=1.0, **shading)
            if p is self.plotter:  # the camera must not render its own housing
                p.add_mesh(ase.cam_mesh, color="#A0A0A0", **shading)
            p.add_mesh(ase.chain_poly, color=C["on_surface_variant"], line_width=2.5, opacity=0.45)
            p.add_mesh(ase.ee_sphere, color="#FF6B35", opacity=0.8)
            p.add_mesh(ase.ee_glow, color="#FF6B35", opacity=0.15)
            idx = 0
            for _, _, _, jcolor in ase.JOINTS:
                p.add_mesh(ase.marker_meshes[idx], color=jcolor, opacity=0.15)
                p.add_mesh(ase.marker_meshes[idx + 1], color=jcolor, opacity=1.0)
                p.add_mesh(ase.marker_meshes[idx + 2], color=jcolor, line_width=3, opacity=0.75)
                idx += 3
        # Cyan = vision/scanning state in DESIGN.md
        self.plotter.add_mesh(ase.fov_mesh, color=C["tertiary_container"], opacity=0.15,
                              show_edges=False, smooth_shading=True)
        add_viewfinder(self.cam_plotter, C["tertiary_container"], CAM_ASPECT)

        self.plotter.camera_position = "iso"
        self.plotter.reset_camera()
        self.plotter.camera.zoom(0.8)
        self.cam_plotter.camera.view_angle = CAM_FOV_V  # vertical FoV

        self.camera_model = None
        self.guide_actors = []
        if os.path.exists(CAMERA_MODEL_FILE):
            try:
                self.set_camera_model(CameraModel.load_json(CAMERA_MODEL_FILE))
                self.log_message(f"Loaded camera calibration: {os.path.basename(CAMERA_MODEL_FILE)}")
            except (OSError, ValueError, KeyError) as e:
                self.log_message(f"Could not load camera calibration: {e}", "warn")

        ase.update_scene(ase.current_angles)
        self.refresh_views()

    def set_camera_model(self, model):
        for act in self.guide_actors:
            self.cam_plotter.renderer.RemoveActor2D(act)
        self.camera_model = model
        self.guide_actors = add_distance_guides(self.cam_plotter, model)
        self.cam_plotter.interactor.Render()

    def setup_ee_workspace_projections(self):
        # Sampled over the *effective* (hardware-reachable) limits
        n = 20000
        j1 = np.radians(np.random.uniform(*ase.JOINT_LIMITS[0], n))
        j2 = np.radians(np.random.uniform(*ase.JOINT_LIMITS[1], n))
        j3 = np.radians(np.random.uniform(*ase.JOINT_LIMITS[2], n))
        s1, c1 = np.sin(j1), np.cos(j1)
        s2, c2 = np.sin(j2), np.cos(j2)
        dy2 = -47.5 * c2 - 194.5 * s2
        dz2 = -47.5 * s2 + 194.5 * c2
        dy3 = 140.0 * np.sin(j2 + j3)
        dz3 = -140.0 * np.cos(j2 + j3)
        X = s1 * (-5.0 - dy2 - dy3)
        Y = c1 * (5.0 + dy2 + dy3)
        Z = 35.5 + dz2 + dz3

        cmap = [C["primary_fixed"], C["primary_fixed_dim"], C["primary"]]
        views = [
            ((0, 0), X, Y, "X (lateral mm)", "Y (depth mm)", "XY · top"),
            ((0, 1), Y, Z, "Y (depth mm)", "Z (height mm)", "YZ · side"),
            ((0, 2), X, Z, "X (lateral mm)", "Z (height mm)", "XZ · front"),
        ]
        for (r, c), a, b, xt, yt, label in views:
            self.ee_plotter.subplot(r, c)
            self.ee_plotter.set_background(C["surface_lowest"])
            H, ae, be = np.histogram2d(a, b, bins=50)
            grid = pv.ImageData()
            grid.dimensions = (51, 51, 1)
            grid.origin = (ae[0], be[0], 0.0)
            grid.spacing = (ae[1] - ae[0], be[1] - be[0], 1.0)
            grid.cell_data["density"] = H.flatten(order="F")
            self.ee_plotter.add_mesh(grid.threshold(1e-5, scalars="density"), scalars="density",
                                     cmap=cmap, opacity=0.55, show_scalar_bar=False)
            self.ee_plotter.add_mesh(grid.outline(), color=C["surface_dim"], line_width=1)
            self.ee_plotter.show_bounds(xtitle=xt, ytitle=yt, grid=True, color=C["outline"],
                                        font_size=8, location="outer")
            self.ee_plotter.add_text(label, position="upper_left", color=C["on_surface_variant"], font_size=8)
            self.ee_plotter.view_xy()
            self.ee_plotter.reset_camera()
            self.ee_plotter.camera.zoom(0.8)
            self.ee_plotter.enable_parallel_projection()
            self.ee_plotter.enable_2d_style()

        dot = pv.Sphere(radius=12.0, center=(0.0, 0.0, 0.0))
        actors = []
        for c in range(3):
            self.ee_plotter.subplot(0, c)
            actors.append(self.ee_plotter.add_mesh(dot, color=C["on_surface"]))
        self.dot_xy_actor, self.dot_yz_actor, self.dot_xz_actor = actors

    def closeEvent(self, event):
        self.sync_timer.stop()
        self.gcode_timer.stop()
        self.link.close()
        for p in (self.plotter, self.cam_plotter, self.ee_plotter):
            p.close()
        super().closeEvent(event)
