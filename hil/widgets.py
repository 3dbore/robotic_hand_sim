"""Reusable widgets: cards, joint rows, servo telemetry cards, link health,
and the clickable 2D workspace views."""
import math
import time

import numpy as np
from PyQt5.QtCore import Qt, QPointF, QRectF, pyqtSignal
from PyQt5.QtGui import QColor, QImage, QPainter, QPen, QBrush, QFont, QPolygonF
from PyQt5.QtWidgets import (
    QCheckBox, QDoubleSpinBox, QFrame, QGridLayout, QHBoxLayout, QLabel, QPushButton, QSizePolicy,
    QSlider, QVBoxLayout, QWidget,
)

from . import theme
from .config import STAT_STALE_S
from .theme import C, MONO

# ─────────────────────────────────────────────────────────────────────────────
# Small helpers
# ─────────────────────────────────────────────────────────────────────────────


def label(text="", name=None, tip=None):
    w = QLabel(text)
    if name:
        w.setObjectName(name)
    if tip:
        w.setToolTip(tip)
    return w


def make_card(title, subtitle=None):
    card = QFrame()
    card.setObjectName("Card")
    lay = QVBoxLayout(card)
    m = 10 if theme.COMPACT else 14
    lay.setContentsMargins(m, m - 2, m, m)
    lay.setSpacing(6 if theme.COMPACT else 8)
    head = QHBoxLayout()
    head.addWidget(label(title, "CardTitle"))
    head.addStretch()
    if subtitle:
        head.addWidget(label(subtitle, "CardAction"))
    lay.addLayout(head)
    return card, lay


class Chip(QLabel):
    """Rounded status chip; set(text, tone) with tone in TONES."""

    TONES = {
        "ok": (C["primary"], "#e3f6ee"),
        "info": (C["tertiary"], "#e1f2f7"),
        "warn": (C["warning"], "#fbf0d9"),
        "bad": (C["error"], C["error_container"]),
        "idle": (C["outline"], C["surface_container"]),
    }

    def __init__(self, text="", tone="idle"):
        super().__init__()
        self.setAlignment(Qt.AlignCenter)
        self.set(text, tone)

    def set(self, text, tone):
        fg, bg = self.TONES[tone]
        self.setText(text)
        fs = 10 if theme.COMPACT else 11
        self.setStyleSheet(
            f"QLabel {{ color: {fg}; background: {bg}; border-radius: 9px; padding: 1px 8px;"
            f" font-size: {fs}px; font-weight: 700; }}")


def fmt_deg(v, signed=False):
    if v is None:
        return "—"
    return f"{v:+.1f}°" if signed else f"{v:.1f}°"


# ─────────────────────────────────────────────────────────────────────────────
# Joints tab: one row per axis
# ─────────────────────────────────────────────────────────────────────────────


class JointRow(QWidget):
    """Name · slider (servo degrees) · value · jog −/+ · invert."""

    moved = pyqtSignal(int, float)        # axis index, servo goal
    jog = pyqtSignal(int, float)          # axis index, signed step
    inverted = pyqtSignal(int, bool)
    trimmed = pyqtSignal(int, float)      # axis index, home trim (deg)

    SCALE = 10                            # slider units per degree

    def __init__(self, idx, axis, lo, hi):
        super().__init__()
        self.idx, self.axis = idx, axis
        lay = QGridLayout(self)
        lay.setContentsMargins(0, 2, 0, 2)
        lay.setHorizontalSpacing(8)
        lay.setVerticalSpacing(0)

        name = label(axis.name, "RowName")
        name.setMinimumWidth(92)
        lay.addWidget(name, 0, 0)
        lay.addWidget(label(axis.channel, "Muted"), 1, 0)

        self.slider = QSlider(Qt.Horizontal)
        self.slider.setRange(int(round(lo * self.SCALE)), int(round(hi * self.SCALE)))
        self.slider.setSingleStep(self.SCALE)
        self.slider.setPageStep(10 * self.SCALE)
        self.slider.valueChanged.connect(self._slid)
        lay.addWidget(self.slider, 0, 1)
        self.lo, self.hi = lo, hi
        self.range_lbl = label("", "Muted")
        self._show_range()
        lay.addWidget(self.range_lbl, 1, 1)

        self.value = label("", "MetricSmall")
        self.value.setMinimumWidth(64)
        self.value.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
        lay.addWidget(self.value, 0, 2)
        self.sub = label("", "Muted")
        self.sub.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
        lay.addWidget(self.sub, 1, 2)

        minus, plus = QPushButton("−"), QPushButton("+")
        for b, sgn in ((minus, -1.0), (plus, 1.0)):
            b.setObjectName("Pad")
            b.setAutoRepeat(True)
            b.setAutoRepeatDelay(350)
            b.setAutoRepeatInterval(120)
            b.clicked.connect(lambda _, s=sgn: self.jog.emit(self.idx, s))
        lay.addWidget(minus, 0, 3, 2, 1)
        lay.addWidget(plus, 0, 4, 2, 1)

        self.inv = QCheckBox("INV")
        self.inv.setToolTip("Servo mounted the other way round: sends 180° − angle")
        self.inv.setChecked(axis.inverted)
        self.inv.toggled.connect(lambda on: self.inverted.emit(self.idx, on))
        lay.addWidget(self.inv, 0, 5)

        from .config import TRIM_MAX
        self.trim = QDoubleSpinBox()
        self.trim.setRange(-TRIM_MAX, TRIM_MAX)
        self.trim.setDecimals(1)
        self.trim.setSingleStep(0.5)
        self.trim.setPrefix("trim ")
        self.trim.setSuffix("°")
        self.trim.setKeyboardTracking(False)
        self.trim.setValue(axis.trim)
        self.trim.setToolTip("Home correction: shifts what is sent to this servo by a few degrees\n"
                             "so the real joint lines up with the sim's home pose. Saved.")
        self.trim.valueChanged.connect(self._trim_changed)
        lay.addWidget(self.trim, 1, 5)
        lay.setColumnStretch(1, 1)

    def _trim_changed(self, v):
        self.trimmed.emit(self.idx, v)
        self._show_range()

    def _show_range(self):
        a = self.axis
        text = f"servo {self.lo:.0f}°…{self.hi:.0f}°   home {a.home:.0f}°"
        if a.trim:
            w0, w1 = sorted((a.wire(self.lo), a.wire(self.hi)))
            text += f"   → sends {w0:.1f}°…{w1:.1f}° (home {a.wire(a.home):.1f}°)"
        self.range_lbl.setText(text)

    def _slid(self, v):
        self.moved.emit(self.idx, v / self.SCALE)

    def show_goal(self, servo):
        self.slider.blockSignals(True)
        self.slider.setValue(int(round(servo * self.SCALE)))
        self.slider.blockSignals(False)

    def show_cmd(self, servo):
        self.value.setText(fmt_deg(servo))
        if self.axis.joint is None:
            from .config import grip_percent
            self.sub.setText(f"{grip_percent(servo):.0f}% open")
        else:
            self.sub.setText(f"joint {self.axis.to_sim(servo):+.1f}°")


# ─────────────────────────────────────────────────────────────────────────────
# Telemetry: per-axis card + link health
# ─────────────────────────────────────────────────────────────────────────────


class RangeBar(QWidget):
    """Servo range 0..180 with the allowed band, home tick, the angle sent
    (filled triangle above) and the angle the board reports (dot below)."""

    def __init__(self, axis, lo, hi):
        super().__init__()
        self.axis, self.lo, self.hi = axis, lo, hi
        self.cmd = None
        self.rb = None
        self.setMinimumHeight(26)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)

    def set(self, cmd, rb):
        self.cmd, self.rb = cmd, rb
        self.update()

    def paintEvent(self, _):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        w, h = self.width(), self.height()
        x0, x1 = 6.0, w - 6.0
        cy = h / 2.0

        def X(a):
            return x0 + (x1 - x0) * min(max(a, 0.0), 180.0) / 180.0

        p.setPen(Qt.NoPen)
        p.setBrush(QColor(C["surface_high"]))
        p.drawRoundedRect(QRectF(x0, cy - 3, x1 - x0, 6), 3, 3)
        p.setBrush(QColor(C["primary_fixed"]))
        p.drawRoundedRect(QRectF(X(self.lo), cy - 3, X(self.hi) - X(self.lo), 6), 3, 3)
        p.setPen(QPen(QColor(C["outline"]), 1.5))
        hx = X(self.axis.home)
        p.drawLine(QPointF(hx, cy - 7), QPointF(hx, cy + 7))
        if self.cmd is not None:
            cx = X(self.cmd)
            p.setPen(Qt.NoPen)
            p.setBrush(QColor(C["primary"]))
            p.drawPolygon(QPolygonF([QPointF(cx - 5, cy - 11), QPointF(cx + 5, cy - 11), QPointF(cx, cy - 3)]))
        if self.rb is not None:
            rx = X(self.rb)
            p.setPen(QPen(QColor(C["surface_lowest"]), 1.5))
            p.setBrush(QColor(C["tertiary"]))
            p.drawEllipse(QPointF(rx, cy + 7), 4, 4)
        p.end()


class ServoCard(QFrame):
    """One axis: what we send, what the board says it is doing, and whether
    those agree, in words."""

    SETTLE_TOL = 2.5      # deg; readback within this of the command = OK
    MISMATCH_AFTER = 2.0  # s of disagreement after a move before calling it out

    def __init__(self, axis, lo, hi):
        super().__init__()
        self.setObjectName("Card")
        self.axis = axis
        self.lo, self.hi = lo, hi
        self._bad_since = None
        lay = QVBoxLayout(self)
        m = 8 if theme.COMPACT else 10
        lay.setContentsMargins(m + 2, m, m + 2, m)
        lay.setSpacing(3)

        head = QHBoxLayout()
        head.addWidget(label(axis.name, "RowName"))
        head.addStretch()
        self.chip = Chip("Released", "idle")
        head.addWidget(self.chip)
        lay.addLayout(head)
        lay.addWidget(label(axis.channel + ("  ·  mirrored pair" if "+" in axis.channel else ""), "Muted"))

        grid = QGridLayout()
        grid.setHorizontalSpacing(8)
        grid.setVerticalSpacing(0)
        grid.addWidget(label("Sent", "Muted"), 0, 0)
        grid.addWidget(label("Board", "Muted"), 0, 1)
        grid.addWidget(label("Joint" if axis.joint is not None else "Open", "Muted"), 0, 2)
        self.sent = label("—", "MetricSmall")
        self.board = label("—", "MetricSmall")
        self.board.setStyleSheet(f"color: {C['tertiary']};")
        self.joint = label("—", "MetricSmall")
        self.joint.setStyleSheet(f"color: {C['on_surface_variant']};")
        grid.addWidget(self.sent, 1, 0)
        grid.addWidget(self.board, 1, 1)
        grid.addWidget(self.joint, 1, 2)
        lay.addLayout(grid)

        self.bar = RangeBar(axis, lo, hi)
        lay.addWidget(self.bar)
        self.note = label("", "Muted")
        self.note.setWordWrap(True)
        lay.addWidget(self.note)

    def update_state(self, cmd, rb, motors_on, moving, feedback_ok):
        a = self.axis
        self.sent.setText(fmt_deg(cmd) if motors_on else "off")
        self.board.setText(fmt_deg(rb))
        if a.joint is None:
            from .config import grip_percent
            self.joint.setText(f"{grip_percent(cmd):.0f}%")
        else:
            self.joint.setText(fmt_deg(a.to_sim(cmd), signed=True))
        self.bar.set(cmd if motors_on else None, rb)

        notes = []
        if a.inverted or a.trim:
            tags = ", ".join(t for t in ("inverted" if a.inverted else "",
                                         f"trim {a.trim:+.1f}°" if a.trim else "") if t)
            notes.append(f"{tags}: wire {a.wire(cmd):.1f}°")
        if motors_on and (cmd <= self.lo + 0.5 or cmd >= self.hi - 0.5):
            notes.append("at its limit")

        now = time.monotonic()
        if not motors_on:
            self._bad_since = None
            if rb is not None:
                self.chip.set("Still driven", "warn")
                notes.append("board still holds it after release")
            else:
                self.chip.set("Released", "idle")
        elif not feedback_ok:
            self._bad_since = None
            self.chip.set("No feedback", "idle")
        elif rb is None:
            self.chip.set("Not driven", "warn")
            notes.append("board isn't pulsing this servo yet")
        else:
            diff = abs(rb - cmd)
            if diff <= self.SETTLE_TOL:
                self._bad_since = None
                self.chip.set("Moving" if moving else "OK", "info" if moving else "ok")
            elif moving:
                self._bad_since = None
                self.chip.set("Following", "info")
            else:
                if self._bad_since is None:
                    self._bad_since = now
                if now - self._bad_since >= self.MISMATCH_AFTER:
                    self.chip.set(f"Off by {diff:.1f}°", "bad")
                else:
                    self.chip.set("Settling", "info")
        self.note.setText("  ·  ".join(notes))


class LinkHealth(QFrame):
    """Board link status in plain words, from the 1 s STAT reports."""

    def __init__(self):
        super().__init__()
        self.setObjectName("Card")
        lay = QHBoxLayout(self)
        m = 8 if theme.COMPACT else 10
        lay.setContentsMargins(m + 4, m, m + 4, m)
        lay.setSpacing(18)
        lay.addWidget(label("Board link", "RowName"))
        self.chip = Chip("Disconnected", "idle")
        lay.addWidget(self.chip)
        self.items = {}
        for key, title, tip in (
            ("rate", "Frames in", "Commands the board accepted in the last second"),
            ("bad", "Rejected", "Frames dropped for a bad checksum or garbled text (wiring / noise)"),
            ("i2c", "I2C errors", "Failed writes to the servo board"),
            ("reinit", "Servo board resets", "Times the firmware had to re-initialise the PCA9685"),
            ("spike", "Jumps ignored", "Sudden jumps >20° held back until a second frame agreed"),
            ("age", "Last report", "Time since the board's last STAT line"),
            ("boot", "Last boot", "Why the board last restarted"),
        ):
            box = QVBoxLayout()
            box.setSpacing(0)
            box.addWidget(label(title, "Muted", tip))
            v = label("—", "MetricSmall", tip)
            box.addWidget(v)
            lay.addLayout(box)
            self.items[key] = v
        lay.addStretch()
        self.reset()

    def reset(self):
        self.last = None
        self.prev = None
        self.t_last = None
        self.boot_reason = None
        self.base = None
        for v in self.items.values():
            v.setText("—")
            v.setStyleSheet("")

    def on_stat(self, st):
        if self.base is None:
            self.base = st
        self.prev, self.last = self.last, st
        self.t_last = time.monotonic()

    def on_boot(self, reason):
        self.boot_reason = reason
        self.base = None

    def refresh(self, connected, sending):
        def put(key, text, tone=None):
            w = self.items[key]
            w.setText(text)
            w.setStyleSheet(f"color: {Chip.TONES[tone][0]};" if tone else "")

        if self.boot_reason:
            put("boot", self.boot_reason, "warn" if self.boot_reason in ("BROWNOUT", "PANIC", "WDT",
                                                                         "TASK_WDT", "INT_WDT") else None)
        if not connected:
            self.chip.set("Disconnected", "idle")
            return
        if self.last is None:
            self.chip.set("Waiting for board…", "info")
            return
        age = time.monotonic() - self.t_last
        put("age", f"{age:.1f} s ago", "bad" if age > STAT_STALE_S else None)
        st, prev = self.last, self.prev
        rate = st.rx - prev.rx if prev is not None and st.rx >= prev.rx else None
        put("rate", f"{rate}/s" if rate is not None else "—",
            "warn" if sending and rate is not None and rate < 10 else None)

        def counter(key, attr):
            total = getattr(st, attr)
            new = prev is not None and total > getattr(prev, attr)
            put(key, f"{total}" + ("  ▲" if new else ""), "warn" if new else None)
            return new

        new_bad = counter("bad", "bad")
        new_i2c = counter("i2c", "i2c")
        new_reinit = counter("reinit", "reinit")
        counter("spike", "spike")

        if age > STAT_STALE_S:
            self.chip.set(f"No report for {age:.0f} s", "bad")
        elif new_reinit or new_i2c:
            self.chip.set("Servo board trouble", "warn")
        elif new_bad:
            self.chip.set("Noisy line", "warn")
        else:
            self.chip.set("Healthy", "ok")

    def feedback_ok(self):
        return self.t_last is not None and time.monotonic() - self.t_last <= STAT_STALE_S


# ─────────────────────────────────────────────────────────────────────────────
# Workspace: clickable 2D views of where the end effector can go
# ─────────────────────────────────────────────────────────────────────────────


class WorkspaceView(QWidget):
    """2D projection with the reachable area shaded. Click to pick a target.

    mode "top":  looking down; up on screen = forward (-Y), right = -X
                 (as seen standing behind the arm). Coordinates (X, Y).
    mode "side": the arm's own plane at its current heading; right = forward
                 reach h, up = Z. Coordinates (h, Z).
    """

    picked = pyqtSignal(float, float)

    def __init__(self, mode, extent, title):
        super().__init__()
        self.mode = mode
        self.title = title
        # world rectangle (u0, u1, v0, v1) in the view's own coordinates
        self.extent = extent
        self.mask_img = None
        self._mask_buf = None
        self.arm = []           # projected joint points (u, v)
        self.ee = None
        self.goal = None
        self.target = None
        self.hover = None
        self.floor = None
        self.heading = None
        self.message = ""
        self.setMouseTracking(True)
        self.setMinimumSize(220, 200)
        self.setCursor(Qt.CrossCursor)

    # world (u, v) ↔ screen
    def _frame(self):
        u0, u1, v0, v1 = self.extent
        pad = 26
        w, h = self.width() - 2 * pad, self.height() - 2 * pad - 14
        s = min(w / (u1 - u0), h / (v1 - v0))
        ox = pad + (w - s * (u1 - u0)) / 2
        oy = pad + 14 + (h - s * (v1 - v0)) / 2
        return s, ox, oy

    def to_screen(self, u, v):
        u0, u1, v0, v1 = self.extent
        s, ox, oy = self._frame()
        if self.mode == "top":           # u = X (right = -X), v = Y (up = -Y)
            return QPointF(ox + (u1 - u) * s, oy + (v - v0) * s)
        return QPointF(ox + (u - u0) * s, oy + (v1 - v) * s)

    def to_world(self, x, y):
        u0, u1, v0, v1 = self.extent
        s, ox, oy = self._frame()
        if self.mode == "top":
            return u1 - (x - ox) / s, v0 + (y - oy) / s
        return u0 + (x - ox) / s, v1 - (y - oy) / s

    def set_mask(self, mask):
        """mask[i_v, i_u] over the extent, row 0 = v0, col 0 = u0."""
        rgba = np.zeros(mask.shape + (4,), np.uint8)
        col = QColor(C["primary_fixed_dim"])
        rgba[mask] = (col.red(), col.green(), col.blue(), 110)
        if self.mode == "top":
            rgba = rgba[:, ::-1]          # right = -X
        else:
            rgba = rgba[::-1, :]          # up = +Z
        self._mask_buf = np.ascontiguousarray(rgba)
        h, w = self._mask_buf.shape[:2]
        self.mask_img = QImage(self._mask_buf.data, w, h, 4 * w, QImage.Format_RGBA8888)
        self.update()

    def mouseMoveEvent(self, e):
        self.hover = self.to_world(e.x(), e.y())
        self.update()

    def leaveEvent(self, _):
        self.hover = None
        self.update()

    def mousePressEvent(self, e):
        if e.button() == Qt.LeftButton:
            u, v = self.to_world(e.x(), e.y())
            u0, u1, v0, v1 = self.extent
            if u0 <= u <= u1 and v0 <= v <= v1:
                self.picked.emit(u, v)

    def paintEvent(self, _):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        p.fillRect(self.rect(), QColor(C["surface_lowest"]))
        u0, u1, v0, v1 = self.extent
        a, b = self.to_screen(u0, v0), self.to_screen(u1, v1)
        area = QRectF(a, b).normalized()
        p.fillRect(area, QColor(C["surface_low"]))
        if self.mask_img is not None:
            p.drawImage(area, self.mask_img)

        # grid every 50 mm, axes through 0
        p.setFont(QFont("Segoe UI", 7))
        for val in range(int(math.ceil(u0 / 50)) * 50, int(u1) + 1, 50):
            pa, pb = self.to_screen(val, v0), self.to_screen(val, v1)
            p.setPen(QPen(QColor(C["outline"] if val == 0 else C["surface_dim"]), 1, Qt.SolidLine if val == 0 else Qt.DotLine))
            p.drawLine(pa, pb)
        for val in range(int(math.ceil(v0 / 50)) * 50, int(v1) + 1, 50):
            pa, pb = self.to_screen(u0, val), self.to_screen(u1, val)
            p.setPen(QPen(QColor(C["outline"] if val == 0 else C["surface_dim"]), 1, Qt.SolidLine if val == 0 else Qt.DotLine))
            p.drawLine(pa, pb)
        if self.floor is not None:
            pa, pb = self.to_screen(u0, self.floor), self.to_screen(u1, self.floor)
            p.setPen(QPen(QColor(C["on_surface_variant"]), 2))
            p.drawLine(pa, pb)
            low = QRectF(pa, self.to_screen(u1, v0)).normalized()
            p.fillRect(low, QBrush(QColor(C["outline_variant"]), Qt.BDiagPattern))
        if self.heading is not None:
            # base heading ray (top view): forward = Rz(θ1)·(0, -1, 0)
            ang = math.radians(self.heading)
            r = max(abs(u0), abs(u1))
            p.setPen(QPen(QColor(C["outline"]), 1, Qt.DashLine))
            p.drawLine(self.to_screen(0, 0), self.to_screen(r * math.sin(ang), -r * math.cos(ang)))

        # arm
        if len(self.arm) >= 2:
            pts = [self.to_screen(*q) for q in self.arm]
            p.setPen(QPen(QColor(C["on_surface_variant"]), 3, Qt.SolidLine, Qt.RoundCap, Qt.RoundJoin))
            for i in range(len(pts) - 1):
                p.drawLine(pts[i], pts[i + 1])
            p.setPen(Qt.NoPen)
            p.setBrush(QColor(C["inverse_surface"]))
            for q in pts:
                p.drawEllipse(q, 3, 3)
        # goal ring, target cross, ee dot
        if self.goal is not None:
            q = self.to_screen(*self.goal)
            p.setPen(QPen(QColor(C["primary"]), 2, Qt.DashLine))
            p.setBrush(Qt.NoBrush)
            p.drawEllipse(q, 8, 8)
        if self.target is not None:
            q = self.to_screen(*self.target)
            p.setPen(QPen(QColor(C["error"]), 2))
            p.drawLine(QPointF(q.x() - 6, q.y() - 6), QPointF(q.x() + 6, q.y() + 6))
            p.drawLine(QPointF(q.x() - 6, q.y() + 6), QPointF(q.x() + 6, q.y() - 6))
        if self.ee is not None:
            q = self.to_screen(*self.ee)
            p.setPen(QPen(QColor(C["surface_lowest"]), 2))
            p.setBrush(QColor("#FF6B35"))
            p.drawEllipse(q, 5.5, 5.5)

        # labels
        p.setPen(QColor(C["on_surface"]))
        p.setFont(QFont("Segoe UI", 9, QFont.DemiBold))
        p.drawText(QPointF(8, 16), self.title)
        p.setFont(QFont("Segoe UI", 8))
        p.setPen(QColor(C["outline"]))
        if self.mode == "top":
            p.drawText(area.adjusted(0, -14, 0, 0), Qt.AlignHCenter | Qt.AlignTop, "forward ↑")
            p.drawText(area.adjusted(4, 0, 0, -2), Qt.AlignLeft | Qt.AlignBottom, "+X ←")
        else:
            p.drawText(area.adjusted(0, 0, -4, -2), Qt.AlignRight | Qt.AlignBottom, "forward →")
            p.drawText(area.adjusted(4, 2, 0, 0), Qt.AlignLeft | Qt.AlignTop, "Z ↑")
        if self.hover is not None:
            u, v = self.hover
            names = ("X", "Y") if self.mode == "top" else ("reach", "Z")
            txt = f"{names[0]} {u:.0f}  {names[1]} {v:.0f} mm"
            p.setFont(QFont(MONO.split(",")[0].strip("' "), 8))
            p.setPen(QColor(C["on_surface_variant"]))
            p.drawText(self.rect().adjusted(0, 4, -8, 0), Qt.AlignRight | Qt.AlignTop, txt)
        if self.message:
            p.setPen(QColor(C["error"]))
            p.setFont(QFont("Segoe UI", 8, QFont.DemiBold))
            p.drawText(self.rect().adjusted(8, 0, -8, -4), Qt.AlignLeft | Qt.AlignBottom, self.message)
        p.end()
