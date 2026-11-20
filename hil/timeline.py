"""
Timeline strip for the Sequences tab, laid out like a video editor track:
one block per step, left to right, each as wide as the move into it plus
its hold (hatched tail). The ruler marks the time at each block's start.

  click            select a step
  drag a block     reorder
  drag right edge  change that step's hold time
  double-click     move the arm to that step
  Ctrl + wheel     zoom
"""
from PyQt5.QtCore import QPointF, QRectF, Qt, pyqtSignal
from PyQt5.QtGui import QBrush, QColor, QFont, QPainter, QPen
from PyQt5.QtWidgets import QSizePolicy, QWidget

from .sequence import HOLD_MAX_S
from .theme import C

RULER_H = 18
BLOCK_H = 62
PAD = 8
GAP = 4
MIN_W = 78
EDGE = 7


class Timeline(QWidget):
    selected = pyqtSignal(int)
    moved = pyqtSignal(int, int)          # from index, to index
    hold_changed = pyqtSignal(int, float)
    activated = pyqtSignal(int)

    def __init__(self):
        super().__init__()
        self.steps = []
        self.move_t = []                  # seconds of the move into each step
        self.sel = -1
        self.active = -1                  # step being played, -1 = none
        self.active_hold = False          # playing its hold (not its move)
        self.pps = 40.0                   # pixels per second
        self._drag = None                 # ("move" | "hold", index, start x)
        self._drag_x = 0
        self.setMouseTracking(True)
        self.setFixedHeight(RULER_H + BLOCK_H + 2 * PAD)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)

    # ── data ──
    def set_steps(self, steps, move_t):
        self.steps, self.move_t = steps, move_t
        if self.sel >= len(steps):
            self.sel = len(steps) - 1
        self._relayout()

    def set_active(self, idx, holding=False):
        self.active, self.active_hold = idx, holding
        self.update()

    def _relayout(self):
        x = PAD
        self.rects = []
        for s, mt in zip(self.steps, self.move_t):
            w = max(MIN_W, (mt + s.hold) * self.pps)
            self.rects.append(QRectF(x, RULER_H + PAD, w, BLOCK_H))
            x += w + GAP
        self.setMinimumWidth(int(x + PAD + 40))
        self.update()

    def _hit(self, pos):
        for i, r in enumerate(self.rects):
            if r.left() <= pos.x() <= r.right() + GAP and r.top() <= pos.y() <= r.bottom():
                return i, pos.x() >= r.right() - EDGE
        return -1, False

    def _insert_at(self, x):
        for i, r in enumerate(self.rects):
            if x < r.center().x():
                return i
        return len(self.rects)

    # ── mouse ──
    def mousePressEvent(self, e):
        if e.button() != Qt.LeftButton:
            return
        i, edge = self._hit(e.pos())
        if i < 0:
            return
        self.sel = i
        self.selected.emit(i)
        self._drag = ("hold" if edge else "press", i, e.pos().x())
        self.update()

    def mouseMoveEvent(self, e):
        if self._drag is None:
            _, edge = self._hit(e.pos())
            self.setCursor(Qt.SizeHorCursor if edge else Qt.ArrowCursor)
            return
        kind, i, x0 = self._drag
        if kind == "press" and abs(e.pos().x() - x0) > 6:
            self._drag, kind = ("move", i, x0), "move"
            self.setCursor(Qt.ClosedHandCursor)
        if kind == "move":
            self._drag_x = e.pos().x()
            self.update()
        elif kind == "hold":
            r = self.rects[i]
            hold = (e.pos().x() - r.left()) / self.pps - self.move_t[i]
            hold = round(max(0.0, min(HOLD_MAX_S, hold)) * 10) / 10
            if hold != self.steps[i].hold:
                self.hold_changed.emit(i, hold)

    def mouseReleaseEvent(self, e):
        if self._drag and self._drag[0] == "move":
            i = self._drag[1]
            to = self._insert_at(e.pos().x())
            to = to - 1 if to > i else to
            if to != i:
                self.sel = to
                self.moved.emit(i, to)
        self._drag = None
        self.setCursor(Qt.ArrowCursor)
        self.update()

    def mouseDoubleClickEvent(self, e):
        i, _ = self._hit(e.pos())
        if i >= 0:
            self.activated.emit(i)

    def wheelEvent(self, e):
        if e.modifiers() & Qt.ControlModifier:
            f = 1.15 if e.angleDelta().y() > 0 else 1 / 1.15
            self.pps = max(10.0, min(200.0, self.pps * f))
            self._relayout()
            e.accept()
        else:
            e.ignore()

    # ── paint ──
    def paintEvent(self, _):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        p.fillRect(self.rect(), QColor(C["surface_low"]))
        p.fillRect(QRectF(0, 0, self.width(), RULER_H), QColor(C["surface_container"]))
        small = QFont(self.font())
        small.setPointSizeF(7.5)
        bold = QFont(self.font())
        bold.setBold(True)

        if not self.steps:
            p.setPen(QColor(C["outline"]))
            p.drawText(self.rect().adjusted(0, RULER_H, 0, 0), Qt.AlignCenter,
                       "No steps yet. Pose the arm, then press “Add current pose”.")
            return

        t = 0.0
        for i, (r, s) in enumerate(zip(self.rects, self.steps)):
            p.setPen(QPen(QColor(C["outline"]), 1))
            p.drawLine(QPointF(r.left(), RULER_H - 6), QPointF(r.left(), RULER_H))
            p.setFont(small)
            p.drawText(QRectF(r.left() + 3, 0, r.width(), RULER_H), Qt.AlignVCenter, f"{t:.1f} s")
            t += self.move_t[i] + s.hold
            self._paint_block(p, i, r, s, bold, small)
        end = self.rects[-1].right()
        p.setPen(QPen(QColor(C["outline"]), 1))
        p.drawLine(QPointF(end, RULER_H - 6), QPointF(end, RULER_H))
        p.drawText(QRectF(end + 3, 0, 60, RULER_H), Qt.AlignVCenter, f"{t:.1f} s")

        if self._drag and self._drag[0] == "move":
            k = self._insert_at(self._drag_x)
            x = self.rects[k].left() - GAP / 2 if k < len(self.rects) else self.rects[-1].right() + GAP / 2
            p.setPen(QPen(QColor(C["primary"]), 3))
            p.drawLine(QPointF(x, RULER_H + 2), QPointF(x, RULER_H + PAD + BLOCK_H + 4))

    def _paint_block(self, p, i, r, s, bold, small):
        active = i == self.active
        total = self.move_t[i] + s.hold
        hold_w = r.width() * (s.hold / total) if total > 0 else r.width()
        move_r = QRectF(r.left(), r.top(), r.width() - hold_w, r.height())
        hold_r = QRectF(move_r.right(), r.top(), hold_w, r.height())

        p.save()
        clip = QRectF(r)
        p.setPen(Qt.NoPen)
        p.setBrush(QColor(C["primary"] if active else C["primary_fixed"]))
        p.drawRoundedRect(clip, 8, 8)
        p.setClipRect(hold_r)
        p.setBrush(QColor(C["primary_container"] if active else C["primary_fixed_dim"]))
        p.drawRoundedRect(clip, 8, 8)
        p.setBrush(QBrush(QColor(255, 255, 255, 90), Qt.BDiagPattern))
        p.drawRoundedRect(clip, 8, 8)
        p.restore()

        if active:                             # mark which phase is playing
            ph = hold_r if self.active_hold else move_r
            p.fillRect(QRectF(ph.left() + 4, r.bottom() - 6, max(0.0, ph.width() - 8), 3),
                       QColor(C["on_primary"]))

        sel = i == self.sel
        p.setBrush(Qt.NoBrush)
        p.setPen(QPen(QColor(C["inverse_surface"] if sel else C["primary_fixed_dim"]), 2 if sel else 1))
        p.drawRoundedRect(r.adjusted(0.5, 0.5, -0.5, -0.5), 8, 8)

        fg = QColor(C["on_primary"] if active else C["on_primary_fixed_variant"])
        p.setPen(fg)
        p.setFont(bold)
        inner = r.adjusted(8, 6, -6, -6)
        tag = getattr(s, "tag", None)
        badge = "◉" + ",".join(str(t["id"]) for t in tag.get("tags", [])) if tag else ""
        badge_w = 0
        if badge:                              # step carries an AprilTag capture
            p.setFont(small)
            badge_w = p.fontMetrics().horizontalAdvance(badge) + 4
            p.drawText(inner, Qt.AlignRight | Qt.AlignTop, badge)
            p.setFont(bold)
        name = p.fontMetrics().elidedText(f"{i + 1}  {s.name}", Qt.ElideRight,
                                          int(inner.width() - badge_w))
        p.drawText(inner, Qt.AlignLeft | Qt.AlignTop, name)
        p.setFont(small)
        p.drawText(inner, Qt.AlignLeft | Qt.AlignBottom,
                   f"move {self.move_t[i]:.1f}s\nhold {s.hold:.1f}s")
