"""Small reusable Qt/VTK widgets shared by the main window and dialogs."""
import cv2
import numpy as np
from PyQt5.QtWidgets import (QWidget, QVBoxLayout, QHBoxLayout, QLabel, QFrame,
                             QCheckBox, QSizePolicy, QSplitter, QSplitterHandle,
                             QGraphicsDropShadowEffect, QAbstractButton)
from PyQt5.QtCore import Qt, QTimer, QPointF, QRectF, QSize
from PyQt5.QtGui import QPainter, QColor, QPen
import pyvista as pv
from vtkmodules.vtkRenderingCore import vtkActor2D, vtkCoordinate, vtkPolyDataMapper2D, vtkTextActor

from . import theme
from .theme import C
from .hardware_mapping import FLOOR_Z, GUIDE_BANDS, GUIDE_HALF_WIDTH


class StatusDot(QWidget):
    """Status dot. With glow, draws the DESIGN.md "Robot Pulse" ring; with
    pulse=True the ring animates outward while glowing."""
    def __init__(self, color=C["outline"], glow=False, pulse=False):
        super().__init__()
        self.setFixedSize(14, 14)
        self._color = color
        self._glow = glow
        self._phase = 0.0
        self._timer = None
        if pulse:
            self._timer = QTimer(self)
            self._timer.timeout.connect(self._tick)
        self._sync_timer()

    def set_state(self, color, glow=False):
        if color != self._color or glow != self._glow:
            self._color, self._glow = color, glow
            self._sync_timer()
            self.update()

    def _sync_timer(self):
        if self._timer is None:
            return
        if self._glow and not self._timer.isActive():
            self._timer.start(40)
        elif not self._glow:
            self._timer.stop()
            self._phase = 0.0

    def _tick(self):
        self._phase = (self._phase + 0.03) % 1.0
        self.update()

    def paintEvent(self, event):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        c = QColor(self._color)
        center = QPointF(7, 7)
        if self._glow:
            animated = self._timer is not None and self._timer.isActive()
            radius = 3.5 + 3.0 * self._phase if animated else 5.5
            ring = QColor(c)
            ring.setAlpha(int(170 * (1.0 - self._phase)) if animated else 110)
            p.setPen(QPen(ring, 1.5))
            p.setBrush(Qt.NoBrush)
            p.drawEllipse(center, radius, radius)
        p.setPen(Qt.NoPen)
        p.setBrush(c)
        p.drawEllipse(center, 3.0, 3.0)
        p.end()


class ToggleSwitch(QAbstractButton):
    """On/off switch with its label; the label is part of the click target.
    Eco Green track when on, as for the other active states in DESIGN.md."""
    def __init__(self, text, checked=False):
        super().__init__()
        self.setObjectName("Switch")
        self.setText(text)
        self.setCheckable(True)
        self.setChecked(checked)
        self.setCursor(Qt.PointingHandCursor)

    def _track_size(self):
        h = 14 if theme.COMPACT else 16
        return 1.8 * h, h

    def sizeHint(self):
        self.ensurePolished()
        tw, th = self._track_size()
        fm = self.fontMetrics()
        return QSize(int(tw) + 8 + fm.horizontalAdvance(self.text()) + 2, max(int(th), fm.height()) + 4)

    def paintEvent(self, event):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        tw, th = self._track_size()
        track = QRectF(1, (self.height() - th) / 2, tw, th)
        on, enabled = self.isChecked(), self.isEnabled()
        track_color = C["surface_high"] if not enabled else C["primary"] if on else C["outline_variant"]
        p.setPen(Qt.NoPen)
        p.setBrush(QColor(track_color))
        p.drawRoundedRect(track, th / 2, th / 2)
        if self.hasFocus():
            p.setPen(QPen(QColor(C["primary"]), 1))
            p.setBrush(Qt.NoBrush)
            p.drawRoundedRect(track.adjusted(-1, -1, 1, 1), th / 2 + 1, th / 2 + 1)
        d = th - 4
        x = track.right() - d - 2 if on else track.left() + 2
        p.setPen(Qt.NoPen)
        p.setBrush(QColor(C["surface_lowest"]))
        p.drawEllipse(QRectF(x, track.top() + 2, d, d))
        p.setPen(QColor(C["on_surface_variant"] if enabled else C["outline_variant"]))
        p.drawText(QRectF(tw + 8, 0, self.width() - tw - 8, self.height()),
                   Qt.AlignVCenter | Qt.AlignLeft, self.text())
        p.end()


class AspectBox(QWidget):
    """Keeps its child at a fixed aspect ratio, centred, at the largest size
    that fits. Used for the camera so its FoV stays true while resizing."""
    def __init__(self, child, aspect):
        super().__init__()
        self._child = child
        self._aspect = aspect
        child.setParent(self)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.setMinimumSize(120, int(120 / aspect))

    def resizeEvent(self, event):
        w, h = self.width(), max(1, self.height())
        if w / h > self._aspect:
            cw, ch = int(h * self._aspect), h
        else:
            cw, ch = w, int(w / self._aspect)
        self._child.setGeometry((w - cw) // 2, (h - ch) // 2, cw, ch)
        super().resizeEvent(event)


class SplitHandle(QSplitterHandle):
    """Drag grip between splitter panes; double-click restores the default split."""
    def __init__(self, orientation, parent):
        super().__init__(orientation, parent)
        self.setAttribute(Qt.WA_Hover, True)
        self.setToolTip("Drag to resize · double-click to reset")

    def enterEvent(self, event):
        self.update()
        super().enterEvent(event)

    def leaveEvent(self, event):
        self.update()
        super().leaveEvent(event)

    def mouseDoubleClickEvent(self, event):
        self.splitter().reset_sizes()

    def paintEvent(self, event):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        p.setPen(Qt.NoPen)
        p.setBrush(QColor(C["primary"] if self.underMouse() else C["outline_variant"]))
        w, h = 4.0, 36.0
        p.drawRoundedRect(QRectF((self.width() - w) / 2, (self.height() - h) / 2, w, h), 2, 2)
        p.end()


class FocusSplitter(QSplitter):
    """Horizontal splitter whose panes the user can resize to focus one view."""
    def __init__(self, stretches):
        super().__init__(Qt.Horizontal)
        self._stretches = stretches
        self.setChildrenCollapsible(False)

    def createHandle(self):
        return SplitHandle(self.orientation(), self)

    def reset_sizes(self):
        total = sum(self.sizes())
        weight = sum(self._stretches)
        self.setSizes([int(total * s / weight) for s in self._stretches])


def add_soft_shadow(widget):
    """Extra-diffused, low-opacity lift for primary buttons (DESIGN.md)."""
    fx = QGraphicsDropShadowEffect(widget)
    fx.setBlurRadius(18)
    fx.setOffset(0, 4)
    fx.setColor(QColor(0, 105, 76, 60))
    widget.setGraphicsEffect(fx)


def add_viewfinder(plotter, color, aspect):
    """DESIGN.md "Vision Viewfinder": corner brackets and a centre crosshair in
    thin cyan strokes, drawn in normalised viewport coordinates."""
    mx, lx = 0.04, 0.08                  # horizontal inset / bracket length
    my, ly = mx * aspect, lx * aspect    # same on-screen length vertically
    segs = []
    for x, sx in ((mx, 1), (1 - mx, -1)):
        for y, sy in ((my, 1), (1 - my, -1)):
            segs += [((x, y), (x + sx * lx, y)), ((x, y), (x, y + sy * ly))]
    gx, cx = 0.012, 0.045                # crosshair gap / arm length
    gy, cy = gx * aspect, cx * aspect
    segs += [((0.5 - cx, 0.5), (0.5 - gx, 0.5)), ((0.5 + gx, 0.5), (0.5 + cx, 0.5)),
             ((0.5, 0.5 - cy), (0.5, 0.5 - gy)), ((0.5, 0.5 + gy), (0.5, 0.5 + cy))]
    pts = np.array([(x, y, 0.0) for seg in segs for (x, y) in seg])
    lines = np.hstack([[2, 2 * i, 2 * i + 1] for i in range(len(segs))])
    poly = pv.PolyData(pts, lines=lines)

    coord = vtkCoordinate()
    coord.SetCoordinateSystemToNormalizedViewport()
    mapper = vtkPolyDataMapper2D()
    mapper.SetInputData(poly)
    mapper.SetTransformCoordinate(coord)
    actor = vtkActor2D()
    actor.SetMapper(mapper)
    actor.GetProperty().SetColor(QColor(color).getRgbF()[:3])
    actor.GetProperty().SetLineWidth(1.5)
    actor.GetProperty().SetOpacity(0.9)
    plotter.renderer.AddActor2D(actor)


def _normalized_line_actor(segs, color, width):
    pts = np.array([(x, y, 0.0) for seg in segs for (x, y) in seg])
    lines = np.hstack([[2, 2 * i, 2 * i + 1] for i in range(len(segs))])
    coord = vtkCoordinate()
    coord.SetCoordinateSystemToNormalizedViewport()
    mapper = vtkPolyDataMapper2D()
    mapper.SetInputData(pv.PolyData(pts, lines=lines))
    mapper.SetTransformCoordinate(coord)
    actor = vtkActor2D()
    actor.SetMapper(mapper)
    actor.GetProperty().SetColor(QColor(color).getRgbF()[:3])
    actor.GetProperty().SetLineWidth(width)
    return actor


def add_distance_guides(plotter, model):
    """Rear-view camera guides drawn from a calibrated CameraModel only: floor
    lines at fixed ground distances ahead of the lens, joined by side rails.
    Returns the added actors so they can be removed on recalibration."""
    W, H = model.image_size
    cen = model.center
    fwd = model.R[2].copy()                  # optical axis in world coordinates
    fwd[2] = 0.0
    fwd /= np.linalg.norm(fwd)
    lat = np.cross([0.0, 0.0, 1.0], fwd)
    foot = np.array([cen[0], cen[1], FLOOR_Z])

    def to_view(p):
        (row, col), depth = (v[0] for v in model.project(p))
        return ((col + 0.5) / W, 1.0 - (row + 0.5) / H) if depth > 0 else None

    def ground(d, side):
        return to_view(foot + d * fwd + side * GUIDE_HALF_WIDTH * lat)

    actors = []
    keys = [k for _, k in GUIDE_BANDS[:-1]]
    for i, (d, _) in enumerate(GUIDE_BANDS):
        key = keys[min(i, len(keys) - 1)]
        segs = []
        if i < len(keys):                    # side rails up to the next distance
            for side in (-1, 1):
                pts = [ground(s, side) for s in np.linspace(d, GUIDE_BANDS[i + 1][0], 9)]
                segs += [(a, b) for a, b in zip(pts[:-1], pts[1:]) if a and b]
        a, b = ground(d, -1), ground(d, 1)
        if a and b:
            segs.append((a, b))
        if segs:
            actors.append(_normalized_line_actor(segs, C[key], 2.5))
        if a and b:
            label = vtkTextActor()
            label.SetInput(f"{d / 10:g} cm")
            prop = label.GetTextProperty()
            prop.SetColor(QColor(C[key]).getRgbF()[:3])
            prop.SetFontSize(12)
            prop.SetBold(True)
            prop.SetVerticalJustificationToCentered()
            label.GetPositionCoordinate().SetCoordinateSystemToNormalizedViewport()
            # Alternate sides so labels of close-together far lines never overlap
            if i % 2 == 0:
                end = max(a, b)
                label.SetPosition(end[0] + 0.01, end[1])
            else:
                end = min(a, b)
                prop.SetJustificationToRight()
                label.SetPosition(end[0] - 0.01, end[1])
            actors.append(label)
    for act in actors:
        plotter.renderer.AddActor2D(act)
    return actors


# X/Y/Z in cv2.drawFrameAxes' colours, so the overlay reads like apriltag.py.
TAG_AXIS_COLORS = ("#ff0000", "#00ff00", "#0000ff")


def add_tag_overlay(plotter, detections, camera_matrix, image_size, axis_length):
    """tags_toolkit.apriltag's drawing — outline, centre, id and X/Y/Z axes — as
    2D actors over a view whose viewport shows the same image as image_size
    (the camera the detections were made in). axis_length: one length or
    {tag_id: length}, in the unit of the detections' tvec. Returns the actors."""
    W, H = image_size
    aspect = W / H

    def nv(uv):
        return ((uv[0] + 0.5) / W, 1.0 - (uv[1] + 0.5) / H)

    actors = []
    for det in detections:
        c = [nv(p) for p in det.corners]
        actors.append(_normalized_line_actor([(c[i], c[(i + 1) % 4]) for i in range(4)],
                                             C["primary_container"], 2.0))
        cx, cy = nv(det.center)
        ring = [(cx + 0.006 * np.cos(t), cy + 0.006 * aspect * np.sin(t))
                for t in np.linspace(0, 2 * np.pi, 13)]
        actors.append(_normalized_line_actor(list(zip(ring[:-1], ring[1:])), C["error"], 2.0))

        label = vtkTextActor()
        label.SetInput(f"id={det.tag_id}")
        prop = label.GetTextProperty()
        prop.SetColor(QColor(C["primary_container"]).getRgbF()[:3])
        prop.SetFontSize(12)
        prop.SetBold(True)
        label.GetPositionCoordinate().SetCoordinateSystemToNormalizedViewport()
        label.SetPosition(c[0][0], c[0][1] + 0.02)
        actors.append(label)

        if det.rvec is not None:
            L = axis_length.get(det.tag_id) if isinstance(axis_length, dict) else axis_length
            ends, _ = cv2.projectPoints(np.float64([[0, 0, 0], [L, 0, 0], [0, L, 0], [0, 0, L]]),
                                        det.rvec, det.tvec, camera_matrix, None)
            o, *axes = [nv(p) for p in ends.reshape(-1, 2)]
            for end, color in zip(axes, TAG_AXIS_COLORS):
                actors.append(_normalized_line_actor([(o, end)], color, 2.5))
    for act in actors:
        plotter.renderer.AddActor2D(act)
    return actors


class RangeBar(QWidget):
    """Servo travel 0..180 with the allowed band, home tick and current marker."""
    def __init__(self):
        super().__init__()
        self.setFixedHeight(12)
        self.lo = 0
        self.hi = 180
        self.home = 90
        self.value = 90
        self.color = C["primary"]

    def set_values(self, lo, hi, home, value, color):
        new = (lo, hi, home, value, color)
        if new != (self.lo, self.hi, self.home, self.value, self.color):
            self.lo, self.hi, self.home, self.value, self.color = new
            self.update()

    def paintEvent(self, event):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        w = self.width() - 8
        x0 = 4.0
        cy = self.height() / 2.0

        def x(v):
            return x0 + w * (max(0.0, min(180.0, v)) / 180.0)

        p.setPen(Qt.NoPen)
        p.setBrush(QColor(C["surface_high"]))
        p.drawRoundedRect(QRectF(x0, cy - 2, w, 4), 2, 2)

        p.setBrush(QColor(C["primary_fixed_dim"]))
        lo, hi = sorted((self.lo, self.hi))
        p.drawRoundedRect(QRectF(x(lo), cy - 2, max(2.0, x(hi) - x(lo)), 4), 2, 2)

        p.setPen(QPen(QColor(C["outline"]), 1))
        p.drawLine(QPointF(x(self.home), cy - 5), QPointF(x(self.home), cy + 5))

        p.setPen(QPen(QColor(C["surface_lowest"]), 1.5))
        p.setBrush(QColor(self.color))
        p.drawEllipse(QPointF(x(self.value), cy), 4.0, 4.0)
        p.end()


class ServoRow(QWidget):
    def __init__(self, ch, on_invert=None):
        super().__init__()
        self.ch = ch
        lay = QVBoxLayout(self)
        lay.setContentsMargins(0, 1 if theme.COMPACT else 2, 0, 1 if theme.COMPACT else 2)
        lay.setSpacing(2 if theme.COMPACT else 3)

        self.dot = StatusDot(C["primary"])
        name = QLabel(ch.name)
        name.setObjectName("RowName")
        self.value = QLabel("---°")
        self.value.setObjectName("MetricSmall")
        self.value.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
        self.sim = QLabel("")
        self.sim.setObjectName("Mono")
        self.limits = QLabel("")
        self.limits.setObjectName("Muted")
        self.inv = None
        if ch.sim_joint is not None:
            self.inv = QCheckBox("INV")
            self.inv.setToolTip("Invert direction of this servo relative to the sim")
            self.inv.setChecked(ch.sign < 0)
            if on_invert is not None:
                self.inv.toggled.connect(lambda checked, c=ch: on_invert(c, checked))
        self.bar = RangeBar()

        top = QHBoxLayout()
        top.setSpacing(6)
        top.addWidget(self.dot)
        top.addWidget(name)
        if theme.COMPACT:
            # Two lines per axis: [dot name … sim value INV] / [bar limits]
            top.addStretch()
            top.addWidget(self.sim)
            top.addWidget(self.value)
            if self.inv is not None:
                top.addWidget(self.inv)
            lay.addLayout(top)

            bottom = QHBoxLayout()
            bottom.setSpacing(6)
            bottom.setContentsMargins(18, 0, 0, 0)
            bottom.addWidget(self.bar, 1)
            bottom.addWidget(self.limits)
            lay.addLayout(bottom)
            self.setToolTip(f"{ch.pin} — {ch.note}")
        else:
            pin = QLabel(ch.pin)
            pin.setObjectName("Muted")
            top.addWidget(pin)
            top.addStretch()
            top.addWidget(self.value)
            lay.addLayout(top)

            mid = QHBoxLayout()
            mid.setSpacing(8)
            mid.setContentsMargins(18, 0, 0, 0)
            mid.addWidget(self.sim)
            mid.addStretch()
            mid.addWidget(self.limits)
            if self.inv is not None:
                mid.addWidget(self.inv)
            lay.addLayout(mid)

            bar_wrap = QHBoxLayout()
            bar_wrap.setContentsMargins(14, 0, 0, 0)
            bar_wrap.addWidget(self.bar)
            lay.addLayout(bar_wrap)
            self.setToolTip(ch.note)

    def refresh(self, cmd, sim_deg, lim, status):
        color = {"ok": C["primary"], "near": C["warning"], "clip": C["error"]}[status]
        self.dot.set_state(color, glow=(status != "ok"))
        self.value.setText(f"{cmd:03d}°")
        if self.ch.sim_joint is None:
            self.sim.setText("hardware only")
            self.limits.setText(f"{self.ch.servo_min}…{self.ch.servo_max}° servo")
            band = (self.ch.servo_min, self.ch.servo_max)
        else:
            self.sim.setText(f"sim {sim_deg:+6.1f}°")
            self.limits.setText(f"{lim[0]:+.0f}…{lim[1]:+.0f}° sim")
            band = (self.ch.raw_servo(lim[0]), self.ch.raw_servo(lim[1]))
        self.bar.set_values(band[0], band[1], self.ch.home, cmd, color)


def make_card(title, action=None):
    card = QFrame()
    card.setObjectName("Card")
    lay = QVBoxLayout(card)
    # DESIGN.md: 24px card padding; compact screens halve it to fit 768px height
    pad = 12 if theme.COMPACT else 24
    lay.setContentsMargins(pad, pad, pad, pad)
    lay.setSpacing(8 if theme.COMPACT else 16)
    head = QHBoxLayout()
    head.setSpacing(8)
    t = QLabel(title)
    t.setObjectName("CardTitle")
    head.addWidget(t)
    head.addStretch()
    if isinstance(action, str):
        action = QLabel(action)
        action.setObjectName("CardAction")
    if action is not None:
        head.addWidget(action)
    lay.addLayout(head)
    div = QFrame()
    div.setObjectName("Divider")
    div.setFixedHeight(1)
    lay.addWidget(div)
    return card, lay


def group_label(text):
    lbl = QLabel(text)
    lbl.setObjectName("GroupLabel")
    return lbl
