"""Main window: connection bar, 3D view, joint / go-to-point / sequence / log
tabs, and the servo telemetry strip."""
import math
import time

import numpy as np
from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtWidgets import (
    QCheckBox, QComboBox, QDoubleSpinBox, QFrame, QGridLayout, QHBoxLayout, QInputDialog, QLabel,
    QLineEdit, QMainWindow, QMessageBox, QPushButton, QScrollArea, QSplitter, QTabWidget, QTextEdit,
    QVBoxLayout, QWidget,
)

from . import config, kinematics, sequence, theme
from .config import AXES, DISPLAY_ORDER, GRIPPER, GRIP_CLOSED, GRIP_OFFSET_MAX, GRIP_OPEN, TICK_MS, RELEASE_EVERY_TICKS, TANK_Z_MM, Z_FLOOR_MM
from .link import Link, list_ports, make_frame
from .motion import Motion, leg_time
from .scene import Scene
from .theme import C
from .timeline import Timeline
from .widgets import Chip, JointRow, LinkHealth, ServoCard, WorkspaceView, label, make_card

RELEASE_FRAME = make_frame([None] * 6)
TELEMETRY_EVERY_TICKS = 4


class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("ELIOS HIL Toolkit")
        self.setStyleSheet(theme.STYLE_SHEET + (theme.COMPACT_STYLE_SHEET if theme.COMPACT else ""))

        self.settings = config.load_settings()
        self.limits = config.joint_limits()
        self.ws = kinematics.Workspace(self.limits)
        self.motion = Motion()
        self.link = Link()
        self.motors_on = False
        self.stat = None
        self.target = None          # world xyz of the last accepted click
        self._tick_n = 0
        self._was_moving = False
        self._mask_z = None
        self.seqs = sequence.load()
        self.run = None             # playing sequence: {"seq", "idx", "hold_until", "auto"}

        page = QWidget()
        page.setObjectName("Page")
        root = QVBoxLayout(page)
        m = 10 if theme.COMPACT else 16
        root.setContentsMargins(m, m, m, m)
        root.setSpacing(8 if theme.COMPACT else 12)
        root.addLayout(self._build_top_bar())

        split = QSplitter(Qt.Horizontal)
        split.setChildrenCollapsible(False)
        split.addWidget(self._build_view_card())
        split.addWidget(self._build_tabs())
        split.setStretchFactor(0, 5)
        split.setStretchFactor(1, 6)
        split.setSizes([5000, 6000])
        root.addWidget(split, 1)
        root.addWidget(self._build_telemetry())
        self.setCentralWidget(page)

        self.link.line.connect(self._on_line)
        self.link.stat.connect(self._on_stat)
        self.link.boot.connect(self._on_boot)
        self.link.closed.connect(self._on_link_lost)

        self._refresh_ports()
        self._apply_motor_state()
        self._refresh_masks(force=True)
        self._goal_changed()
        self._show_pose()
        self._refresh_telemetry()
        self._refresh_seq_lists()

        self.timer = QTimer(self)
        self.timer.setInterval(TICK_MS)
        self.timer.timeout.connect(self._tick)
        self.timer.start()

    # ───────────────────────────── layout ─────────────────────────────
    def _build_top_bar(self):
        bar = QHBoxLayout()
        bar.setSpacing(8)
        title = label("ELIOS HIL", "PageTitle")
        bar.addWidget(title)
        if not theme.COMPACT:
            bar.addWidget(label("manual joints · go to point · servo telemetry", "PageSubtitle"))
        bar.addStretch()

        self.port_box = QComboBox()
        self.port_box.setMinimumWidth(110)
        bar.addWidget(self.port_box)
        refresh = QPushButton("↻")
        refresh.setToolTip("Rescan serial ports")
        refresh.clicked.connect(self._refresh_ports)
        bar.addWidget(refresh)
        self.connect_btn = QPushButton("Connect")
        self.connect_btn.setObjectName("Primary")
        self.connect_btn.clicked.connect(self._toggle_connect)
        bar.addWidget(self.connect_btn)
        self.conn_chip = Chip("Offline", "idle")
        bar.addWidget(self.conn_chip)

        bar.addSpacing(14)
        self.motor_btn = QPushButton("Motors OFF")
        self.motor_btn.setObjectName("Toggle")
        self.motor_btn.setCheckable(True)
        self.motor_btn.setToolTip("ON: stream the commanded pose to the arm.\n"
                                  "OFF: release every servo (they go limp).")
        self.motor_btn.toggled.connect(self._set_motors)
        bar.addWidget(self.motor_btn)
        home = QPushButton("Home")
        home.setToolTip("Move every axis back to its start pose (sim 0°)")
        home.clicked.connect(self.go_home)
        bar.addWidget(home)
        stop = QPushButton("STOP · Release")
        stop.setObjectName("Danger")
        stop.setToolTip("Stop the move and release every servo now (Esc)")
        stop.clicked.connect(self.emergency_stop)
        bar.addWidget(stop)
        self.fs_btn = QPushButton("Full screen")
        self.fs_btn.setToolTip("Full screen on/off (F11)")
        self.fs_btn.clicked.connect(self.toggle_full_screen)
        bar.addWidget(self.fs_btn)
        return bar

    def _build_view_card(self):
        card, lay = make_card("Arm", "sim pose = command sent")
        self.scene = Scene(card)
        lay.addWidget(self.scene.widget, 1)
        self.pose_lbl = label("", "Mono")
        self.pose_lbl.setWordWrap(True)
        lay.addWidget(self.pose_lbl)
        return card

    def _build_tabs(self):
        tabs = QTabWidget()
        tabs.setDocumentMode(True)
        tabs.tabBar().setElideMode(Qt.ElideNone)
        tabs.tabBar().setExpanding(False)
        tabs.addTab(self._build_joints_tab(), "Joints")
        tabs.addTab(self._build_point_tab(), "Go to point")
        tabs.addTab(self._build_sequence_tab(), "Sequences")
        tabs.addTab(self._build_log_tab(), "Serial log")
        tabs.setStyleSheet(
            f"QTabBar::tab {{ padding: 6px 14px; min-width: 80px; font-weight: 700; color: {C['outline']}; "
            f"background: transparent; border: none; border-bottom: 2px solid transparent; }}"
            f"QTabBar::tab:selected {{ color: {C['primary']}; border-bottom: 2px solid {C['primary']}; }}"
            f"QTabWidget::pane {{ border: none; }}")
        self.tabs = tabs
        return tabs

    def _build_joints_tab(self):
        card, lay = make_card("Manual joints", "no kinematics · each servo on its own")
        hint = label("Drag a slider or hold −/+. Moves are ramped to stay under each "
                     "axis's speed limit, and all axes arrive together.", "Secondary")
        hint.setWordWrap(True)
        lay.addWidget(hint)
        step_row = QHBoxLayout()
        step_row.addWidget(label("Jog step", "GroupLabel"))
        self.step_box = QComboBox()
        for s in ("0.5°", "1°", "2°", "5°", "10°"):
            self.step_box.addItem(s, float(s[:-1]))
        self.step_box.setCurrentIndex(2)
        step_row.addWidget(self.step_box)
        step_row.addStretch()
        lay.addLayout(step_row)

        self.rows = {}
        for ax in DISPLAY_ORDER:
            i = AXES.index(ax)
            row = JointRow(i, ax, self.motion.lo[i], self.motion.hi[i])
            row.moved.connect(self._slider_moved)
            row.jog.connect(self._jog)
            row.inverted.connect(self._set_inverted)
            row.trimmed.connect(self._set_trim)
            self.rows[i] = row
            lay.addWidget(row)

        grip = QHBoxLayout()
        grip.addWidget(label("Gripper", "GroupLabel"))
        b_open = QPushButton(f"Open ({GRIP_OPEN:.0f}°)")
        self.close_btn = QPushButton()
        b_open.clicked.connect(lambda: self._set_axis(0, GRIP_OPEN))
        self.close_btn.clicked.connect(lambda: self._set_axis(0, config.grip_closed()))
        self.close_offset = QDoubleSpinBox()
        self.close_offset.setRange(-GRIP_OFFSET_MAX, GRIP_OFFSET_MAX)
        self.close_offset.setDecimals(1)
        self.close_offset.setSingleStep(0.5)
        self.close_offset.setPrefix("close offset ")
        self.close_offset.setSuffix("°")
        self.close_offset.setKeyboardTracking(False)
        self.close_offset.setValue(config.grip_closed_offset)
        self.close_offset.setToolTip(f"Shifts only the closed position (base {GRIP_CLOSED:.0f}°). "
                                     "Open is not affected.")
        self.close_offset.valueChanged.connect(self._set_close_offset)
        grip.addWidget(b_open)
        grip.addWidget(self.close_btn)
        grip.addWidget(self.close_offset)
        self._update_close_btn()
        grip.addStretch()
        lay.addLayout(grip)

        line = QFrame()
        line.setObjectName("Divider")
        line.setFixedHeight(1)
        lay.addWidget(line)
        run = QHBoxLayout()
        run.addWidget(label("Sequence", "GroupLabel"))
        self.run_box = QComboBox()
        self.run_box.setMinimumWidth(140)
        self.run_box.setToolTip("Saved sequences (make them in the Sequences tab)")
        run.addWidget(self.run_box, 1)
        b_run = QPushButton("▶ Run")
        b_run.setObjectName("Primary")
        b_run.setToolTip("Play every step in order, starting from the current pose")
        b_run.clicked.connect(lambda: self.run_sequence(self._run_choice(), auto=True))
        b_next = QPushButton("Next step")
        b_next.setToolTip("Move to the next step only, one per click")
        b_next.clicked.connect(lambda: self.step_sequence(self._run_choice()))
        b_stop = QPushButton("■ Stop")
        b_stop.setToolTip("Stop the sequence where it is (servos stay powered)")
        b_stop.clicked.connect(self.stop_sequence)
        for b in (b_run, b_next, b_stop):
            run.addWidget(b)
        lay.addLayout(run)
        self.run_status = label("", "Secondary")
        self.run_status.setWordWrap(True)
        lay.addWidget(self.run_status)
        lay.addStretch()
        return card

    def _build_point_tab(self):
        card, lay = make_card("Go to point", "click where the end effector should go")
        hint = label("Shaded = reachable within every joint's limits. Top view is at the "
                     "end effector's current height; side view is in the arm's current "
                     "heading. Hatched = below the table. Behind the base (past ±90°) "
                     f"the arm has to stay above Z {TANK_Z_MM:.0f} mm, clear of the tank.",
                     "Secondary")
        hint.setWordWrap(True)
        lay.addWidget(hint)
        R = 50 * math.ceil(self.ws.reach / 50)
        self.top_view = WorkspaceView("top", (-R, R, -R, R), "Top view (from above)")
        self.side_view = WorkspaceView("side", (-R, R, -150, R + 50), "Side view (arm plane)")
        self.side_view.floor = Z_FLOOR_MM
        self.top_view.picked.connect(self._pick_top)
        self.side_view.picked.connect(self._pick_side)
        views = QHBoxLayout()
        views.setSpacing(8)
        views.addWidget(self.top_view, 1)
        views.addWidget(self.side_view, 1)
        lay.addLayout(views, 1)

        form = QHBoxLayout()
        form.setSpacing(6)
        self.spin = {}
        for key in ("X", "Y", "Z"):
            form.addWidget(label(key, "GroupLabel"))
            s = QDoubleSpinBox()
            s.setRange(-600, 600)
            s.setDecimals(1)
            s.setSuffix(" mm")
            s.setSingleStep(5)
            self.spin[key] = s
            form.addWidget(s)
        form.addWidget(label("Pitch", "GroupLabel"))
        self.pitch_spin = QDoubleSpinBox()
        self.pitch_spin.setRange(-360, 360)
        self.pitch_spin.setDecimals(1)
        self.pitch_spin.setSuffix(" °")
        self.pitch_spin.setToolTip("Tool pitch relative to the floor (θ2 + θ3 + θ4). Kept while moving.")
        form.addWidget(self.pitch_spin)
        go = QPushButton("Go")
        go.setObjectName("Primary")
        go.clicked.connect(self._go_typed)
        form.addWidget(go)
        here = QPushButton("Use current")
        here.clicked.connect(self._fill_current)
        form.addWidget(here)
        form.addStretch()
        lay.addLayout(form)
        self.point_status = label("", "Secondary")
        self.point_status.setWordWrap(True)
        lay.addWidget(self.point_status)
        self._fill_current()
        return card

    def _build_sequence_tab(self):
        card, lay = make_card("Sequences", "saved poses, played in order")
        hint = label("Pose the arm (Joints or Go to point), then add it as a step. Drag a block "
                     "to reorder, drag its right edge to change the hold, double-click to go "
                     "there. Ctrl + wheel zooms. Runs start from wherever the arm is.", "Secondary")
        hint.setWordWrap(True)
        lay.addWidget(hint)

        top = QHBoxLayout()
        top.addWidget(label("Sequence", "GroupLabel"))
        self.seq_box = QComboBox()
        self.seq_box.setMinimumWidth(140)
        self.seq_box.currentIndexChanged.connect(self._seq_selected)
        top.addWidget(self.seq_box, 1)
        for text, fn in (("New", self._seq_new), ("Rename", self._seq_rename),
                         ("Duplicate", self._seq_duplicate), ("Delete", self._seq_delete)):
            b = QPushButton(text)
            b.clicked.connect(fn)
            if text == "Delete":
                b.setObjectName("Danger")
            top.addWidget(b)
        lay.addLayout(top)

        self.timeline = Timeline()
        self.timeline.selected.connect(self._show_step)
        self.timeline.moved.connect(self._step_moved)
        self.timeline.hold_changed.connect(self._step_hold)
        self.timeline.activated.connect(self._step_go)
        scroll = QScrollArea()
        scroll.setWidget(self.timeline)
        scroll.setWidgetResizable(True)
        scroll.setVerticalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        scroll.setFixedHeight(self.timeline.height() + 14)
        scroll.setFrameShape(QFrame.NoFrame)
        lay.addWidget(scroll)

        tools = QHBoxLayout()
        add = QPushButton("+ Add current pose")
        add.setObjectName("Primary")
        add.setToolTip("Insert the pose the arm is heading to, after the selected step")
        add.clicked.connect(self._step_add)
        tools.addWidget(add)
        for text, tip, fn in (
                ("Update", "Replace the selected step's pose with the current pose", self._step_update),
                ("Go to", "Move the arm to the selected step", lambda: self._step_go(self.timeline.sel)),
                ("Copy", "Duplicate the selected step", self._step_copy),
                ("◀", "Move the selected step earlier", lambda: self._step_shift(-1)),
                ("▶", "Move the selected step later", lambda: self._step_shift(1))):
            b = QPushButton(text)
            b.setToolTip(tip)
            b.clicked.connect(fn)
            tools.addWidget(b)
        rm = QPushButton("Remove")
        rm.setObjectName("Danger")
        rm.clicked.connect(self._step_remove)
        tools.addWidget(rm)
        tools.addStretch()
        lay.addLayout(tools)

        edit = QHBoxLayout()
        edit.addWidget(label("Step", "GroupLabel"))
        self.step_name = QLineEdit()
        self.step_name.setPlaceholderText("name")
        self.step_name.editingFinished.connect(self._step_rename)
        edit.addWidget(self.step_name, 1)
        self.step_hold = QDoubleSpinBox()
        self.step_hold.setRange(0.0, sequence.HOLD_MAX_S)
        self.step_hold.setDecimals(1)
        self.step_hold.setSingleStep(0.5)
        self.step_hold.setPrefix("hold ")
        self.step_hold.setSuffix(" s")
        self.step_hold.setKeyboardTracking(False)
        self.step_hold.setToolTip("How long to stay at this pose before moving on")
        self.step_hold.valueChanged.connect(lambda v: self._step_hold(self.timeline.sel, v))
        edit.addWidget(self.step_hold)
        lay.addLayout(edit)
        self.step_pose = label("", "Mono")
        self.step_pose.setWordWrap(True)
        lay.addWidget(self.step_pose)

        play = QHBoxLayout()
        b_play = QPushButton("▶ Play from current pose")
        b_play.setObjectName("Primary")
        b_play.clicked.connect(lambda: self.run_sequence(self._seq(), auto=True))
        b_stop = QPushButton("■ Stop")
        b_stop.clicked.connect(self.stop_sequence)
        play.addWidget(b_play)
        play.addWidget(b_stop)
        play.addStretch()
        lay.addLayout(play)
        self.seq_status = label("", "Secondary")
        self.seq_status.setWordWrap(True)
        lay.addWidget(self.seq_status)
        lay.addStretch()
        return card

    def _build_log_tab(self):
        card, lay = make_card("Serial log", "lines from the board")
        opts = QHBoxLayout()
        self.log_stat = QCheckBox("show STAT")
        self.log_stat.setChecked(True)
        self.log_sent = QCheckBox("show frames sent")
        opts.addWidget(self.log_stat)
        opts.addWidget(self.log_sent)
        opts.addStretch()
        clear = QPushButton("Clear")
        opts.addWidget(clear)
        lay.addLayout(opts)
        self.log = QTextEdit()
        self.log.setReadOnly(True)
        self.log.document().setMaximumBlockCount(600)
        clear.clicked.connect(self.log.clear)
        lay.addWidget(self.log, 1)
        return card

    def _build_telemetry(self):
        box = QWidget()
        lay = QVBoxLayout(box)
        lay.setContentsMargins(0, 0, 0, 0)
        lay.setSpacing(6)
        head = QHBoxLayout()
        head.addWidget(label("Servo telemetry", "CardTitle"))
        head.addWidget(label("▼ sent   ● board reports   | home   green band = allowed range", "Muted"))
        head.addStretch()
        lay.addLayout(head)
        self.health = LinkHealth()
        lay.addWidget(self.health)
        cards = QHBoxLayout()
        cards.setSpacing(8)
        self.cards = {}
        for ax in DISPLAY_ORDER:
            i = AXES.index(ax)
            c = ServoCard(ax, self.motion.lo[i], self.motion.hi[i])
            self.cards[i] = c
            cards.addWidget(c, 1)
        lay.addLayout(cards)
        return box

    # ───────────────────────────── connection ─────────────────────────────
    def _refresh_ports(self):
        cur = self.port_box.currentText() or self.settings.get("port", "")
        self.port_box.clear()
        ports = list_ports()
        self.port_box.addItems(ports)
        if cur in ports:
            self.port_box.setCurrentText(cur)

    def _toggle_connect(self):
        if self.link.is_open:
            self._disconnect()
            return
        dev = self.port_box.currentText()
        if not dev:
            QMessageBox.information(self, "No port", "No serial port found. Plug the board in and press ↻.")
            return
        try:
            self.link.open(dev)
        except Exception as e:
            QMessageBox.warning(self, "Can't open port", f"{dev}: {e}")
            return
        self.settings["port"] = dev
        self._save()
        self.health.reset()
        self.stat = None
        self._log(f"── connected to {dev} @ {config.SERIAL_BAUD} ──", C["primary"])
        self._apply_motor_state()

    def _disconnect(self):
        if self.link.is_open:
            self.link.send(RELEASE_FRAME)
        self.link.close()
        self.motor_btn.setChecked(False)
        self.stat = None
        self._log("── disconnected ──", C["outline"])
        self._apply_motor_state()

    def _on_link_lost(self, reason):
        self.link.close()
        self.motor_btn.setChecked(False)
        self.stat = None
        self._log(f"── link lost: {reason} ──", C["error"])
        self._apply_motor_state()

    def _set_motors(self, on):
        if on and not self.link.is_open:
            self.motor_btn.blockSignals(True)
            self.motor_btn.setChecked(False)
            self.motor_btn.blockSignals(False)
            QMessageBox.information(self, "Not connected", "Connect to the board first.")
            return
        self.motors_on = on
        if not on and self.link.is_open:
            self.link.send(RELEASE_FRAME)
        self._apply_motor_state()

    def _apply_motor_state(self):
        self.motors_on = self.motor_btn.isChecked() and self.link.is_open
        self.motor_btn.setText("Motors ON" if self.motors_on else "Motors OFF")
        self.connect_btn.setText("Disconnect" if self.link.is_open else "Connect")
        self.port_box.setEnabled(not self.link.is_open)
        if not self.link.is_open:
            self.conn_chip.set("Offline · sim only", "idle")
        elif self.motors_on:
            self.conn_chip.set("Driving the arm", "ok")
        else:
            self.conn_chip.set("Connected · servos released", "info")

    def emergency_stop(self):
        self._end_run("Sequence stopped (STOP).", error=True)
        self.motion.stop()
        self.motor_btn.setChecked(False)
        if self.link.is_open:
            self.link.send(RELEASE_FRAME)
        self._goal_changed()
        self._log("── STOP: all servos released ──", C["error"])

    def toggle_full_screen(self):
        if self.isFullScreen():
            self.showMaximized()
        else:
            self.showFullScreen()

    def changeEvent(self, e):
        super().changeEvent(e)
        if hasattr(self, "fs_btn"):
            self.fs_btn.setText("Windowed" if self.isFullScreen() else "Full screen")

    def keyPressEvent(self, e):
        if e.key() == Qt.Key_Escape:
            self.emergency_stop()
        elif e.key() == Qt.Key_F11:
            self.toggle_full_screen()
        else:
            super().keyPressEvent(e)

    # ───────────────────────────── serial in ─────────────────────────────
    def _on_line(self, text):
        if text.startswith("STAT") and not self.log_stat.isChecked():
            return
        color = C["warning"] if text.startswith("BOOT") else None
        self._log(text, color)

    def _on_stat(self, st):
        self.stat = st
        self.health.on_stat(st)

    def _on_boot(self, reason):
        self.health.on_boot(reason)
        self._log(f"board restarted ({reason}); its servos are released until the next frame",
                  C["warning"])

    def _log(self, text, color=None):
        if color:
            self.log.append(f'<span style="color:{color}">{text}</span>')
        else:
            self.log.append(text.replace("<", "&lt;"))

    # ───────────────────────────── motion ─────────────────────────────
    def _request(self, servo_goal):
        """Start a move to `servo_goal` (six servo angles) along a path that
        keeps clear of the tank. Returns None, or why it was refused."""
        goal = self.motion.clamp(np.asarray(servo_goal, float))
        path, why = kinematics.plan(self.motion.sim_cmd(), self.motion.to_sim(goal))
        if path is None:
            return why
        self.motion.go_path([self.motion.from_sim(w, goal[0]) for w in path])
        return None

    def _set_axis(self, i, servo, skip_row=None):
        self._end_run("Sequence stopped: manual move.")
        goal = self.motion.final_goal.copy()
        goal[i] = servo
        why = self._request(goal)
        if why:
            self._refuse(f"{AXES[i].name} not moved: {why}.")
            skip_row = None          # put the slider back
        self._goal_changed(skip_row=skip_row)

    def _refuse(self, msg):
        self.point_status.setStyleSheet(f"color: {C['error']};")
        self.point_status.setText(msg)
        now = time.monotonic()
        if msg != getattr(self, "_last_refusal", (None, 0))[0] or now - self._last_refusal[1] > 2.0:
            self._log(msg, C["warning"])
        self._last_refusal = (msg, now)

    def _slider_moved(self, i, servo):
        self._set_axis(i, servo, skip_row=i)

    def _jog(self, i, sign):
        step = self.step_box.currentData()
        self._set_axis(i, self.motion.final_goal[i] + sign * step)

    def _set_inverted(self, i, on):
        AXES[i].inverted = on
        self._save()
        self._log(f"{AXES[i].name}: {'inverted' if on else 'normal'} direction", C["warning"])

    def _set_trim(self, i, deg):
        AXES[i].trim = deg
        self._save()
        self._refresh_telemetry()

    def _update_close_btn(self):
        self.close_btn.setText(f"Close ({config.grip_closed():.1f}°)".replace(".0°", "°"))

    def _set_close_offset(self, deg):
        was_closed = abs(self.motion.final_goal[0] - config.grip_closed()) < 0.05
        config.grip_closed_offset = deg
        self._save()
        self._update_close_btn()
        if was_closed:                       # keep a closed gripper closed
            self._set_axis(0, config.grip_closed())
        self._refresh_telemetry()

    def go_home(self):
        self._end_run("Sequence stopped: going home.")
        why = self._request(self.motion.home)
        if why:
            self._refuse(f"Can't go home: {why}.")
        self.target = None
        self._goal_changed()

    def _goal_changed(self, skip_row=None):
        for i, row in self.rows.items():
            if i != skip_row:
                row.show_goal(self.motion.final_goal[i])
        goal_sim = self.motion.sim_goal()
        ee = kinematics.fk(goal_sim)
        self.top_view.goal = (ee[0], ee[1])
        self.side_view.goal = self._side_coords(ee, self.motion.sim_cmd()[0])
        self.scene.show_target(self.target)
        self.top_view.update()
        self.side_view.update()

    def _tick(self):
        self._tick_n += 1
        changed = self.motion.tick()
        if self.run is not None and not self.motion.moving:
            self._run_tick()
        if self.link.is_open:
            if self.motors_on:
                self.link.send(make_frame(self.motion.wire()))
                if self.log_sent.isChecked():
                    self._log("→ " + make_frame(self.motion.wire()).strip(), C["outline"])
            elif self._tick_n % RELEASE_EVERY_TICKS == 0:
                self.link.send(RELEASE_FRAME)
        if changed:
            self._show_pose()
        moving = self.motion.moving
        if self._was_moving and not moving:
            self._refresh_masks()
            self._refresh_timeline()
            if self.target is not None:
                err = np.linalg.norm(kinematics.fk(self.motion.sim_cmd()) - self.target)
                self.point_status.setStyleSheet("")
                for v in (self.top_view, self.side_view):
                    v.message = ""
                    v.update()
                self.point_status.setText(f"Arrived (within {err:.1f} mm of the target).")
        self._was_moving = moving
        if self._tick_n % TELEMETRY_EVERY_TICKS == 0:
            self._refresh_telemetry()

    def _show_pose(self):
        sim = self.motion.sim_cmd()
        ee = kinematics.fk(sim)
        self.scene.pose(sim, ee)
        for i, row in self.rows.items():
            row.show_cmd(self.motion.cmd[i])
        pitch = sim[1] + sim[2] + sim[3]
        self.pose_lbl.setText(
            f"EE  X {ee[0]:.1f}  Y {ee[1]:.1f}  Z {ee[2]:.1f} mm   pitch {pitch:+.1f}°\n"
            f"joints " + " ".join(f"{a:+6.1f}" for a in sim))
        pts = kinematics.joint_points(sim)
        self.top_view.arm = [(p[0], p[1]) for p in pts]
        self.top_view.ee = (ee[0], ee[1])
        self.top_view.heading = sim[0]
        self.side_view.arm = [self._side_coords(p, sim[0]) for p in pts]
        self.side_view.ee = self._side_coords(ee, sim[0])
        if self.target is not None:
            self.top_view.target = (self.target[0], self.target[1])
            self.side_view.target = self._side_coords(self.target, sim[0])
        else:
            self.top_view.target = self.side_view.target = None
        self.top_view.update()
        self.side_view.update()

    def _refresh_telemetry(self):
        connected = self.link.is_open
        feedback = connected and self.health.feedback_ok()
        out = self.stat.out if (self.stat is not None and feedback) else [None] * 6
        for i, c in self.cards.items():
            rb = out[i]
            if rb is not None:
                rb = AXES[i].unwire(rb)
            c.update_state(self.motion.cmd[i], rb, self.motors_on, self.motion.moving, feedback)
        self.health.refresh(connected, self.motors_on)

    # ───────────────────────────── go to point ─────────────────────────────
    @staticmethod
    def _side_coords(p, heading_deg):
        t = math.radians(heading_deg)
        return (p[0] * math.sin(t) - p[1] * math.cos(t), p[2])

    def _refresh_masks(self, force=False):
        sim = self.motion.sim_cmd()
        z = float(kinematics.fk(sim)[2])
        heading = float(sim[0])
        if force or abs(heading - getattr(self, "_mask_heading", 1e9)) > 1.0:
            u0, u1, v0, v1 = self.side_view.extent
            hs = np.linspace(u0, u1, 300)
            zs = np.linspace(v0, v1, int(300 * (v1 - v0) / (u1 - u0)))
            m = self.ws.side(hs, zs, heading) & (zs[:, None] >= Z_FLOOR_MM)
            self.side_view.set_mask(m)
            self._mask_heading = heading
        if force or self._mask_z is None or abs(z - self._mask_z) > 1.0:
            u0, u1, v0, v1 = self.top_view.extent
            xs = np.linspace(u0, u1, 300)
            ys = np.linspace(v0, v1, 300)
            m = self.ws.top(xs, ys, z) if z >= Z_FLOOR_MM else np.zeros((300, 300), bool)
            self.top_view.set_mask(m)
            self.top_view.title = f"Top view at Z = {z:.0f} mm"
            self._mask_z = z

    def _pick_top(self, x, y):
        z = float(kinematics.fk(self.motion.sim_goal())[2])
        self.move_to((x, y, z))

    def _pick_side(self, h, z):
        t = math.radians(self.motion.sim_cmd()[0])
        self.move_to((h * math.sin(t), -h * math.cos(t), z))

    def _go_typed(self):
        goal = self.motion.sim_goal()
        goal[3] += self.pitch_spin.value() - (goal[1] + goal[2] + goal[3])
        self.move_to((self.spin["X"].value(), self.spin["Y"].value(), self.spin["Z"].value()),
                     start=goal)

    def _fill_current(self):
        sim = self.motion.sim_goal()
        ee = kinematics.fk(sim)
        for k, v in zip("XYZ", ee):
            self.spin[k].setValue(float(v))
        self.pitch_spin.setValue(float(sim[1] + sim[2] + sim[3]))

    def move_to(self, xyz, start=None):
        """Solve IK for world point xyz and start the move. Returns True if accepted."""
        xyz = np.asarray(xyz, float)
        self._end_run("Sequence stopped: manual move.")
        for v in (self.top_view, self.side_view):
            v.message = ""
        if xyz[2] < Z_FLOOR_MM:
            return self._reject(xyz, "below the table")
        cur = self.motion.sim_goal() if start is None else np.asarray(start, float)
        angles, why = kinematics.solve(xyz, cur, self.limits)
        if angles is None:
            return self._reject(xyz, why)
        why = self._request(self.motion.from_sim(angles, self.motion.final_goal[0]))
        if why:
            return self._reject(xyz, why)
        self.target = xyz
        self._goal_changed()
        self._show_pose()
        pitch = angles[1] + angles[2] + angles[3]
        want = cur[1] + cur[2] + cur[3]
        note = "" if abs(pitch - want) < 0.05 else f"  Pitch limited to {pitch:+.1f}° (wanted {want:+.1f}°)."
        self.point_status.setStyleSheet("")
        self.point_status.setText(
            f"Moving to X {xyz[0]:.0f}, Y {xyz[1]:.0f}, Z {xyz[2]:.0f} mm — about "
            f"{self.motion.remaining():.1f} s.{note}" + ("" if self.motors_on else "  (motors off: sim only)"))
        for k, v in zip("XYZ", xyz):
            self.spin[k].setValue(float(v))
        self.pitch_spin.setValue(float(pitch))
        return True

    def _reject(self, xyz, why):
        msg = f"Can't reach X {xyz[0]:.0f}, Y {xyz[1]:.0f}, Z {xyz[2]:.0f} mm: {why}."
        self.point_status.setStyleSheet(f"color: {C['error']};")
        self.point_status.setText(msg)
        for v in (self.top_view, self.side_view):
            v.message = "unreachable: " + why
            v.update()
        return False

    # ───────────────────────────── sequences: editing ─────────────────────────────
    def _seq(self):
        i = self.seq_box.currentIndex()
        return self.seqs[i] if 0 <= i < len(self.seqs) else None

    def _run_choice(self):
        i = self.run_box.currentIndex()
        return self.seqs[i] if 0 <= i < len(self.seqs) else None

    def _refresh_seq_lists(self, select=None):
        """Refill both sequence pickers; `select` = the sequence to open in the editor."""
        cur_edit = select or self._seq()
        cur_run = self._run_choice()
        names = [f"{q.name}  ({len(q.steps)} steps)" for q in self.seqs]
        for box, cur in ((self.seq_box, cur_edit), (self.run_box, cur_run or cur_edit)):
            box.blockSignals(True)
            box.clear()
            box.addItems(names)
            if cur in self.seqs:
                box.setCurrentIndex(self.seqs.index(cur))
            box.blockSignals(False)
        self._seq_selected()

    def _save_seqs(self):
        try:
            sequence.save(self.seqs)
        except OSError as e:
            self._seq_msg(f"Could not save sequences: {e}", error=True)

    def _seq_edited(self, select_step=None):
        """After a change to the open sequence: save, redraw, refresh the pickers."""
        self._save_seqs()
        if select_step is not None:
            self.timeline.sel = select_step
        self._refresh_seq_lists()

    def _seq_selected(self, *_):
        q = self._seq()
        if q is not None and q.steps and not 0 <= self.timeline.sel < len(q.steps):
            self.timeline.sel = 0
        self._refresh_timeline()
        self._show_step()

    def _refresh_timeline(self):
        q = self._seq()
        steps = q.steps if q else []
        prev, move_t = self.motion.final_goal, []
        for s in steps:
            move_t.append(leg_time(prev, s.servo))
            prev = s.servo
        self.timeline.set_steps(steps, move_t)

    def _sel_step(self):
        q, i = self._seq(), self.timeline.sel
        return (q, i) if q and 0 <= i < len(q.steps) else (None, -1)

    def _show_step(self, *_):
        q, i = self._sel_step()
        s = q.steps[i] if q else None
        for w in (self.step_name, self.step_hold):
            w.setEnabled(s is not None)
            w.blockSignals(True)
        self.step_name.setText(s.name if s else "")
        self.step_hold.setValue(s.hold if s else 0.0)
        for w in (self.step_name, self.step_hold):
            w.blockSignals(False)
        if s is None:
            self.step_pose.setText("")
            return
        ee = kinematics.fk(self.motion.to_sim(np.asarray(s.servo)))
        self.step_pose.setText(
            f"EE  X {ee[0]:.0f}  Y {ee[1]:.0f}  Z {ee[2]:.0f} mm   "
            f"gripper {config.grip_percent(s.servo[0]):.0f}% open\n"
            + "  ".join(f"{ax.name} {s.servo[AXES.index(ax)]:.1f}°" for ax in DISPLAY_ORDER))

    def _seq_msg(self, text, error=False):
        self.seq_status.setStyleSheet(f"color: {C['error']};" if error else "")
        self.seq_status.setText(text)
        if error:
            self._log(text, C["warning"])

    def _names(self):
        return {q.name for q in self.seqs}

    def _seq_new(self):
        name, ok = QInputDialog.getText(self, "New sequence", "Name:")
        if not ok or not name.strip():
            return
        q = sequence.Sequence(sequence.unique_name(name.strip(), self._names()))
        self.seqs.append(q)
        self.timeline.sel = -1
        self._save_seqs()
        self._refresh_seq_lists(select=q)

    def _seq_rename(self):
        q = self._seq()
        if q is None:
            return
        name, ok = QInputDialog.getText(self, "Rename sequence", "Name:", text=q.name)
        if ok and name.strip() and name.strip() != q.name:
            q.name = sequence.unique_name(name.strip(), self._names())
            self._seq_edited()

    def _seq_duplicate(self):
        q = self._seq()
        if q is None:
            return
        c = sequence.Sequence(sequence.unique_name(q.name + " copy", self._names()),
                              [s.copy() for s in q.steps])
        self.seqs.insert(self.seqs.index(q) + 1, c)
        self._save_seqs()
        self._refresh_seq_lists(select=c)

    def _seq_delete(self):
        q = self._seq()
        if q is None:
            return
        if QMessageBox.question(self, "Delete sequence",
                                f"Delete “{q.name}” and its {len(q.steps)} steps?") != QMessageBox.Yes:
            return
        if self.run and self.run["seq"] is q:
            self._end_run("Sequence stopped: it was deleted.")
        self.seqs.remove(q)
        self.timeline.sel = -1
        self._save_seqs()
        self._refresh_seq_lists()

    def _step_add(self):
        q = self._seq()
        if q is None:                        # the first step makes a sequence
            q = sequence.Sequence(sequence.unique_name("Sequence", self._names()))
            self.seqs.append(q)
            self._refresh_seq_lists(select=q)
        name = sequence.unique_name(f"Pose {len(q.steps) + 1}", {s.name for s in q.steps})
        at = self.timeline.sel + 1 if 0 <= self.timeline.sel < len(q.steps) else len(q.steps)
        q.steps.insert(at, sequence.Step(name, [float(v) for v in self.motion.final_goal]))
        self._seq_edited(select_step=at)
        self._seq_msg(f"Added “{name}” as step {at + 1}.")

    def _step_update(self):
        q, i = self._sel_step()
        if q:
            q.steps[i].servo = [float(v) for v in self.motion.final_goal]
            self._seq_edited()
            self._seq_msg(f"“{q.steps[i].name}” now holds the current pose.")

    def _step_copy(self):
        q, i = self._sel_step()
        if q:
            s = q.steps[i]
            name = sequence.unique_name(s.name, {x.name for x in q.steps})
            q.steps.insert(i + 1, s.copy(name))
            self._seq_edited(select_step=i + 1)

    def _step_remove(self):
        q, i = self._sel_step()
        if q:
            del q.steps[i]
            self._seq_edited(select_step=min(i, len(q.steps) - 1))

    def _step_shift(self, d):
        q, i = self._sel_step()
        if q and 0 <= i + d < len(q.steps):
            self._step_moved(i, i + d)

    def _step_moved(self, i, to):
        q = self._seq()
        if q:
            q.steps.insert(to, q.steps.pop(i))
            self._seq_edited(select_step=to)

    def _step_hold(self, i, hold):
        q = self._seq()
        if q and 0 <= i < len(q.steps) and q.steps[i].hold != hold:
            q.steps[i].hold = hold
            self._save_seqs()
            self._refresh_timeline()
            if i == self.timeline.sel:
                self.step_hold.blockSignals(True)
                self.step_hold.setValue(hold)
                self.step_hold.blockSignals(False)

    def _step_rename(self):
        q, i = self._sel_step()
        name = self.step_name.text().strip()
        if q and name and name != q.steps[i].name:
            q.steps[i].name = name
            self._save_seqs()
            self._refresh_timeline()

    def _step_go(self, i):
        q = self._seq()
        if not q or not 0 <= i < len(q.steps):
            return
        self._end_run("Sequence stopped: manual move.")
        s = q.steps[i]
        why = self._request(s.servo)
        self.target = None
        self._goal_changed()
        if why:
            self._seq_msg(f"Can't go to “{s.name}”: {why}.", error=True)
        else:
            self._seq_msg(f"Going to “{s.name}”, about {self.motion.remaining():.1f} s.")

    # ───────────────────────────── sequences: playing ─────────────────────────────
    def run_sequence(self, q, auto=True):
        """Play `q` from the current pose: auto = every step, else one step per call."""
        if q is None or not q.steps:
            self._run_msg("Pick a sequence with at least one step (make one in the Sequences tab).",
                          error=True)
            return
        self._end_run(None)
        self.run = {"seq": q, "idx": -1, "hold_until": None, "auto": auto}
        if self._seq() is not q:
            self.seq_box.setCurrentIndex(self.seqs.index(q))
        self._run_next()

    def step_sequence(self, q):
        r = self.run
        if r and r["seq"] is q and not r["auto"]:
            if self.motion.moving:
                self._run_msg("Still moving to the current step.")
                return
            self._run_next()
        else:
            self.run_sequence(q, auto=False)

    def stop_sequence(self):
        if self.run:
            self.motion.stop()
            self._goal_changed()
        self._end_run("Sequence stopped.")

    def _run_next(self):
        r = self.run
        q = r["seq"]
        r["idx"] += 1
        r["hold_until"] = None
        if r["idx"] >= len(q.steps):
            self._end_run(f"“{q.name}” done ({len(q.steps)} steps).")
            return
        s = q.steps[r["idx"]]
        why = self._request(s.servo)
        if why:
            self._end_run(f"“{q.name}” stopped at step {r['idx'] + 1} “{s.name}”: {why}.", error=True)
            return
        self.target = None
        self._goal_changed()
        self._refresh_timeline()
        self.timeline.set_active(r["idx"])
        self._run_msg(f"“{q.name}” step {r['idx'] + 1}/{len(q.steps)}: moving to “{s.name}”"
                      + ("" if self.motors_on else "  (motors off: sim only)"))

    def _run_tick(self):
        """Each tick while a sequence is loaded and the arm has stopped moving."""
        r = self.run
        q = r["seq"]
        if r["idx"] >= len(q.steps):         # steps removed while running
            self._end_run(f"“{q.name}” done.")
            return
        s = q.steps[r["idx"]]
        n = f"“{q.name}” step {r['idx'] + 1}/{len(q.steps)}"
        if not r["auto"]:                    # Next step mode: wait for the next click
            if r["hold_until"] is None:
                r["hold_until"] = 0.0
                self.timeline.set_active(r["idx"], holding=True)
                if r["idx"] + 1 >= len(q.steps):
                    self._end_run(f"{n}: at “{s.name}”. That was the last step.")
                else:
                    self._run_msg(f"{n}: at “{s.name}”. Press Next step to go on.")
            return
        now = time.monotonic()
        if r["hold_until"] is None:
            r["hold_until"] = now + s.hold
            self.timeline.set_active(r["idx"], holding=True)
            if s.hold > 0:
                self._run_msg(f"{n}: holding “{s.name}” for {s.hold:.1f} s")
        elif now >= r["hold_until"]:
            self._run_next()

    def _end_run(self, msg, error=False):
        if self.run is None:
            return
        self.run = None
        self.timeline.set_active(-1)
        if msg:
            self._run_msg(msg, error)

    def _run_msg(self, text, error=False):
        style = f"color: {C['error']};" if error else ""
        for w in (self.run_status, self.seq_status):
            w.setStyleSheet(style)
            w.setText(text)
        if error:
            self._log(text, C["warning"])

    # ───────────────────────────── misc ─────────────────────────────
    def _save(self):
        try:
            config.save_settings({"port": self.settings.get("port", "")})
        except OSError as e:
            self._log(f"could not save settings: {e}", C["error"])

    def closeEvent(self, e):
        self.timer.stop()
        if self.link.is_open:
            self.link.send(RELEASE_FRAME)
            time.sleep(0.05)
        self.link.close()
        self.scene.close()
        super().closeEvent(e)
