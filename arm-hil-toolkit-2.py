"""
ELIOS Hardware-in-the-Loop Toolkit + MaixCam2 camera
----------------------------------------------------
Same toolkit as arm-hil-toolkit.py (joints, go to point, sequences, serial
log, servo telemetry), plus a live view of the MaixCam2 on the gripper.

The camera panel floats in the top-right corner of the 3D "Arm" view:
    - type the camera's IP (same Wi-Fi), press Connect
    - SSH shows the login user / password (MaixCam default root / root) and
      the capture resolution
    - drag the grip in the bottom-left corner to resize, Hide folds it away

How the frames get here (nothing has to be installed on the camera):
    1. the PC opens an SSH session to the camera (paramiko)
    2. it copies DEVICE_SCRIPT below to /tmp/elios_cam_stream.py and runs it
    3. the script reads the camera like vision/camera/camera_capture.py
       (and still shows the image on the camera's own screen), encodes each
       frame as JPEG and writes it to stdout as  b"MXJP" + u32 length + JPEG
    4. those bytes come back over the same SSH channel and are drawn here
Disconnect (or closing the window) kills the script, which frees the camera.

AprilTags in sequences: frames are searched for tag36h11 tags on the PC and
each tag's centre gets a dot. In the Sequences tab, with "Tag on save" on,
"+ Add current pose" / "Update" also store, if a tag is in view, only where
the tag's centre sits along the horizontal axis: its offset from the camera's
vertical centre line (pixels, normalised and degrees; the tag nearest that
line if there are several). Steps with no tag in view are saved without one.
"Capture tag" / "Clear tag" edit a step's tag.

Playing a step that has a tag: once the arm arrives, Arm Base alone steers the
live tag until its centre is back on the saved vertical line (tag left of the
line -> base turns left, right -> right), then the step's hold starts. If the
tag never shows up it carries on after ALIGN_WAIT_S. The angles come from
CAM_HFOV_DEG (pinhole model); AZ_SIGN flips the turn direction.
Only one program can hold the camera: stop camera_capture.py / MaixVision
runs first.

Requires: paramiko  (pip install paramiko)
"""
import math
import os
import socket
import struct
import sys
import threading
import time

from PyQt5.QtCore import QRectF, QSettings, QSize, Qt, QThread, pyqtSignal
from PyQt5.QtGui import QColor, QFont, QFontDatabase, QIcon, QImage, QPainter, QPen
from PyQt5.QtWidgets import (
    QApplication, QCheckBox, QComboBox, QFrame, QHBoxLayout, QLineEdit, QPushButton, QSizePolicy, QStackedWidget,
    QTabBar, QVBoxLayout, QWidget,
)

import numpy as np

from hil import config, theme
from hil.app import setup
from hil.scene import Scene
from hil.theme import C, MONO
from hil.widgets import Chip, label, make_card

# ─────────────────────────────────────────────────────────────────────────────
# Runs on the MaixCam2 (python3 /tmp/elios_cam_stream.py W H QUALITY SHOW)
# ─────────────────────────────────────────────────────────────────────────────
DEVICE_SCRIPT = r'''
import os, struct, sys

# fd 1 carries the JPEG stream; everything else printed (Python or the maix
# C++ libraries) is pushed to stderr so it can't corrupt the frames.
out = os.fdopen(os.dup(1), "wb")
os.dup2(2, 1)
sys.stdout = sys.stderr

W, H, Q, SHOW = int(sys.argv[1]), int(sys.argv[2]), int(sys.argv[3]), sys.argv[4] == "1"
print("PID:%d" % os.getpid(), flush=True)

from maix import app, camera, display, time

cam = camera.Camera(W, H)
disp = None
if SHOW:
    try:
        disp = display.Display()
    except Exception as e:
        print("no local display: %s" % e, flush=True)

cv_encode = None


def encode(img):
    global cv_encode
    if cv_encode is None:
        data = img.to_jpeg(Q).to_bytes()
        if data[:2] == b"\xff\xd8":
            return data
        print("to_jpeg gave no JPEG, using OpenCV", flush=True)
        import cv2
        from maix import image
        cv_encode = lambda im: cv2.imencode(".jpg", image.image2cv(im, False, False),
                                            [cv2.IMWRITE_JPEG_QUALITY, Q])[1].tobytes()
    return cv_encode(img)


print("READY %dx%d q%d" % (W, H, Q), flush=True)
n, t0 = 0, time.ticks_ms()
try:
    while not app.need_exit():
        img = cam.read()
        if disp is not None:
            disp.show(img)
        data = encode(img)
        out.write(b"MXJP" + struct.pack(">I", len(data)) + data)
        out.flush()
        n += 1
        if time.ticks_ms() - t0 >= 5000:
            print("fps %.1f" % (n * 1000.0 / (time.ticks_ms() - t0)), flush=True)
            n, t0 = 0, time.ticks_ms()
except (BrokenPipeError, OSError):
    pass                               # PC disconnected
'''

REMOTE_PATH = "/tmp/elios_cam_stream.py"
MAGIC = b"MXJP"
MAX_FRAME = 4 * 1024 * 1024
RESOLUTIONS = ["512x320", "320x240", "640x480", "1280x720"]
FIRST_FRAME_TIMEOUT_S = 12
STALL_TIMEOUT_S = 5
CMD_TIMEOUT_S = 6                      # every SSH step gives up after this

# ── AprilTag capture for sequence steps ──
# Horizontal field of view of the MaixCam2 at the capture resolution. A
# placeholder: measure it (or calibrate) and set it here. Raw pixels are saved
# too, so captures can be re-derived later with a better value.
CAM_HFOV_DEG = 60.0
# +1: turning Arm Base to larger angles pans the camera to the right (a tag
# right of the target line is followed by turning right). Flip it if the base
# turns the wrong way round.
AZ_SIGN = +1.0
TAG_MAX_AGE_S = 1.0                    # detection must be at most this old
TAG_SETTLE_S = 0.4                     # ...and taken this long after the arm stopped
# Centring during playback (Arm Base steers the tag onto its saved line)
ALIGN_TOL_DEG = 1.0                    # close enough
ALIGN_GAIN = 0.8                       # fraction of the error corrected per move
ALIGN_MAX_STEP_DEG = 10.0              # biggest single base correction
ALIGN_MAX_MOVES = 8
ALIGN_WAIT_S = 4.0                     # give up when no usable frame / tag for this long


def tag_center(corners):
    """Centre of a tag: where its diagonals cross (exact under perspective,
    unlike the corner average). corners are TL, TR, BR, BL."""
    p = np.asarray(corners, dtype=float).reshape(4, 2)
    d1, d2 = p[2] - p[0], p[3] - p[1]
    den = d1[0] * d2[1] - d1[1] * d2[0]
    if abs(den) < 1e-9:
        return p.mean(axis=0)
    t = ((p[1] - p[0])[0] * d2[1] - (p[1] - p[0])[1] * d2[0]) / den
    return p[0] + t * d1


def annotate_tags(jpeg, detector):
    """Decode jpeg, find AprilTags, draw outline + a dot on each tag's centre.
    Returns (QImage or None, [{"id", "center": (u, v), "corners"}])."""
    import cv2
    bgr = cv2.imdecode(np.frombuffer(jpeg, np.uint8), cv2.IMREAD_COLOR)
    if bgr is None:
        return None, []
    dets = detector.detect(bgr)
    r = max(3, bgr.shape[1] // 100)
    found = []
    for d in dets:
        cv2.polylines(bgr, [d.corners.astype(np.int32).reshape(-1, 1, 2)], True, (0, 255, 0), 2)
        c = tag_center(d.corners)
        found.append({"id": d.tag_id, "center": (float(c[0]), float(c[1])),
                      "corners": d.corners.astype(float).tolist()})
        cx, cy = (int(round(v)) for v in c)
        cv2.circle(bgr, (cx, cy), r + 2, (255, 255, 255), -1, cv2.LINE_AA)
        cv2.circle(bgr, (cx, cy), r, (0, 0, 255), -1, cv2.LINE_AA)
        cv2.putText(bgr, f"id{d.tag_id}", (cx + r + 4, cy - r - 2), cv2.FONT_HERSHEY_SIMPLEX,
                    0.5, (0, 255, 255), 1, cv2.LINE_AA)
    rgb = np.ascontiguousarray(bgr[:, :, ::-1])
    h, w = rgb.shape[:2]
    return QImage(rgb.data, w, h, 3 * w, QImage.Format_RGB888).copy(), found


def h_angle(u, w):
    """Horizontal angle (deg) of image column u off the vertical centre line;
    positive = right of it (pinhole model, CAM_HFOV_DEG)."""
    f = (w / 2.0) / math.tan(math.radians(CAM_HFOV_DEG) / 2.0)    # focal length, px
    return math.degrees(math.atan2(u - w / 2.0, f))


def tag_record(found, size, servo):
    """What gets saved in a sequence step: only where the tag's centre sits
    along the horizontal axis, i.e. its offset from the camera's vertical centre
    line. With several tags in view, the one closest to that line is kept.
    `servo` (six servo angles) is only stored for reference."""
    base = config.AXIS_BY_NAME["Arm Base"]
    base_servo = float(servo[config.AXES.index(base)])
    w, h = size
    t = min(found, key=lambda t: abs(t["center"][0] - w / 2.0))
    u = t["center"][0]
    dx = u - w / 2.0
    return {
        "time": time.strftime("%Y-%m-%dT%H:%M:%S"),
        "image": [int(w), int(h)],
        "hfov_deg": CAM_HFOV_DEG,
        "camera": {"azimuth_deg": round(base.to_sim(base_servo), 2), "base_servo": round(base_servo, 2)},
        "tags": [{
            "id": int(t["id"]),
            "x_px": round(u, 1),
            "offset_px": round(dx, 1),                       # + right of the centre line
            "offset_norm": round(dx / (w / 2.0), 4),
            "offset_deg": round(h_angle(u, w), 2),
        }],
    }


def saved_offset_deg(t):
    """Saved horizontal offset (deg) of a stored tag; also reads the old
    format where offset_deg was [horizontal, vertical]."""
    v = t["offset_deg"]
    return float(v[0] if isinstance(v, (list, tuple)) else v)


def describe_tag(tag):
    parts = [f"id{t['id']} centre {saved_offset_deg(t):+.1f}° off the vertical line "
             f"({t.get('x_px', t['px'][0] if 'px' in t else 0):.0f} px)" for t in tag.get("tags", [])]
    return "Tag  " + ("   ".join(parts) or "no tags")


class CameraClient(QThread):
    """SSH into the MaixCam2, run DEVICE_SCRIPT and turn its stdout into QImages."""

    frame = pyqtSignal(QImage)
    status = pyqtSignal(str, str)      # text, Chip tone
    log = pyqtSignal(str)

    def __init__(self, host, user, password, width, height, quality=70, show_on_device=True):
        super().__init__()
        self.host, self.user, self.password = host, user, password
        self.width, self.height, self.quality = width, height, quality
        self.show_on_device = show_on_device
        self.last_jpeg = None          # newest frame, for later vision work
        self._stop = threading.Event()
        self.detect_tags = True        # toggled live from the panel
        self.tag_count = 0
        self.last_tags = None          # (monotonic time, (w, h), [tags]) of the newest frame
        self._detector = None
        self._ssh = None
        self._pid = None

    def stop(self):
        self._stop.set()

    def run(self):
        try:
            import paramiko
        except ImportError:
            self.status.emit("paramiko missing", "bad")
            self.log.emit("pip install paramiko")
            return
        import logging
        logging.getLogger("paramiko").setLevel(logging.CRITICAL)   # we report errors ourselves
        ssh = paramiko.SSHClient()
        ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
        self._ssh = ssh
        last_err = ""
        try:
            self.status.emit("Connecting…", "info")
            ssh.connect(self.host, port=22, username=self.user, password=self.password,
                        timeout=6, banner_timeout=6, auth_timeout=6,
                        allow_agent=False, look_for_keys=False)
            if self._stop.is_set():
                return
            # a copy left over from an earlier run would still hold the camera
            self._exec(ssh, "pkill -f 'elios_cam_strea[m].py'; true")
            code, err = self._exec(ssh, f"cat > {REMOTE_PATH}", DEVICE_SCRIPT.encode())
            if code != 0:
                raise RuntimeError(f"could not write {REMOTE_PATH} on the camera: {err or code}")
            if self._stop.is_set():
                return

            chan = ssh.get_transport().open_session(timeout=CMD_TIMEOUT_S)
            chan.settimeout(0.3)
            chan.exec_command(
                "P=python3; command -v python3 >/dev/null 2>&1 || P=python; "
                f"exec $P -u {REMOTE_PATH} {self.width} {self.height} {self.quality} "
                f"{1 if self.show_on_device else 0}")
            self.status.emit("Starting camera…", "info")

            buf = bytearray()
            err_buf = b""
            started = time.monotonic()
            last_frame = None
            while not self._stop.is_set():
                while chan.recv_stderr_ready():
                    err_buf += chan.recv_stderr(4096)
                    *lines, err_buf = err_buf.split(b"\n")
                    for ln in lines:
                        text = ln.decode("utf-8", "replace").rstrip()
                        if text.startswith("PID:"):
                            self._pid = int(text[4:])
                        elif text:
                            last_err = text
                            self.log.emit(text)
                try:
                    data = chan.recv(65536)
                except socket.timeout:
                    data = None
                if data == b"":
                    break                              # script ended
                if data:
                    buf += data
                    for jpeg in self._frames(buf):
                        img = self._decode(jpeg)
                        if img is not None:
                            self.last_jpeg = jpeg
                            if last_frame is None:
                                self.status.emit("Live", "ok")
                            last_frame = time.monotonic()
                            self.frame.emit(img)
                now = time.monotonic()
                if last_frame is None and now - started > FIRST_FRAME_TIMEOUT_S:
                    raise RuntimeError(last_err or "no frames from the camera")
                if last_frame is not None and now - last_frame > STALL_TIMEOUT_S:
                    self.status.emit("Stalled", "warn")
                    last_frame = now
            if not self._stop.is_set():
                time.sleep(0.2)
                while chan.recv_stderr_ready():
                    err_buf += chan.recv_stderr(4096)
                tail = [l for l in err_buf.decode("utf-8", "replace").splitlines() if l.strip()]
                for t in tail:
                    self.log.emit(t)
                raise RuntimeError((tail[-1] if tail else last_err) or "camera script stopped")
            self.status.emit("Offline", "idle")
        except Exception as e:
            if self._stop.is_set():
                return                         # user pressed Disconnect: stay quiet
            raw = f"{type(e).__name__}: {e}"
            msg = str(e) or type(e).__name__
            if "Authentication" in raw:
                msg = "SSH login refused (check user / password)"
            elif isinstance(e, (socket.timeout, TimeoutError)) or "timed out" in raw.lower():
                msg = f"no answer from {self.host} (same Wi-Fi? IP right?)"
            elif isinstance(e, ConnectionRefusedError) or "Unable to connect" in raw:
                msg = f"{self.host} refused SSH on port 22"
            elif "banner" in raw.lower() or "No existing session" in raw:
                msg = f"{self.host} answered but not as an SSH server (busy? retry in a few s)"
            print(f"[camera] {raw}", flush=True)
            self.log.emit(msg)
            self.status.emit("Error", "bad")
        finally:
            self._cleanup()

    def _decode(self, jpeg):
        """JPEG -> QImage, with AprilTags marked when detection is on."""
        if self.detect_tags:
            try:
                if self._detector is None:
                    from hil_toolkit.tags_toolkit.apriltag import AprilTagDetector
                    self._detector = AprilTagDetector()
                img, found = annotate_tags(jpeg, self._detector)
                if img is not None:
                    self.tag_count = len(found)
                    self.last_tags = (time.monotonic(), (img.width(), img.height()), found)
                    return img
            except Exception as e:
                self.detect_tags = False
                self.log.emit(f"AprilTag detection off: {e}")
        self.tag_count = 0
        img = QImage.fromData(jpeg, "JPG")
        return None if img.isNull() else img

    @staticmethod
    def _frames(buf):
        """Pull every complete frame out of buf (resyncing on MAGIC)."""
        while True:
            i = buf.find(MAGIC)
            if i < 0:
                del buf[:max(0, len(buf) - 3)]
                return
            if i:
                del buf[:i]
            if len(buf) < 8:
                return
            n = struct.unpack(">I", bytes(buf[4:8]))[0]
            if n > MAX_FRAME:
                del buf[:4]
                continue
            if len(buf) < 8 + n:
                return
            jpeg = bytes(buf[8:8 + n])
            del buf[:8 + n]
            yield jpeg

    @staticmethod
    def _exec(ssh, cmd, stdin=None, timeout=CMD_TIMEOUT_S):
        """Run cmd on the camera; never blocks longer than timeout.
        Returns (exit code, or None on timeout / failure; stderr text)."""
        try:
            chan = ssh.get_transport().open_session(timeout=timeout)
        except Exception as e:
            return None, str(e)
        try:
            chan.settimeout(timeout)
            chan.exec_command(cmd)
            if stdin is not None:
                chan.sendall(stdin)
            chan.shutdown_write()
            end = time.monotonic() + timeout
            while not chan.exit_status_ready():
                if time.monotonic() > end:
                    return None, "timed out"
                time.sleep(0.05)
            err = b""
            while chan.recv_stderr_ready():
                err += chan.recv_stderr(4096)
            return chan.recv_exit_status(), err.decode("utf-8", "replace").strip()
        except Exception as e:
            return None, str(e)
        finally:
            chan.close()

    def _cleanup(self):
        ssh, self._ssh = self._ssh, None
        if ssh is None:
            return
        try:
            t = ssh.get_transport()
            if self._pid and t is not None and t.is_active():
                self._exec(ssh, f"kill {self._pid}", timeout=3)
        finally:
            try:
                ssh.close()
            except Exception:
                pass


# ─────────────────────────────────────────────────────────────────────────────
# Widgets
# ─────────────────────────────────────────────────────────────────────────────
class CameraView(QWidget):
    """Letterboxed camera image with a small fps / size readout."""

    def __init__(self):
        super().__init__()
        self.img = None
        self.info = ""
        self.placeholder = "Type the MaixCam IP and press Connect"
        self.setMinimumSize(120, 75)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)

    def set_image(self, img, info):
        self.img, self.info = img, info
        self.update()

    def clear(self, placeholder=None):
        self.img, self.info = None, ""
        if placeholder:
            self.placeholder = placeholder
        self.update()

    def paintEvent(self, _):
        p = QPainter(self)
        p.setRenderHint(QPainter.SmoothPixmapTransform)
        r = self.rect()
        p.fillRect(r, QColor(C["inverse_surface"]))
        if self.img is None:
            p.setPen(QColor(C["outline_variant"]))
            p.drawText(r.adjusted(8, 8, -8, -8), Qt.AlignCenter | Qt.TextWordWrap, self.placeholder)
            return
        s = self.img.size().scaled(r.size(), Qt.KeepAspectRatio)
        x, y = (r.width() - s.width()) // 2, (r.height() - s.height()) // 2
        p.drawImage(QRectF(x, y, s.width(), s.height()), self.img)
        if self.info:
            f = QFont(MONO)
            f.setPixelSize(10)
            p.setFont(f)
            tw = p.fontMetrics().horizontalAdvance(self.info) + 10
            box = QRectF(x + 6, y + s.height() - 22, tw, 16)
            p.fillRect(box, QColor(0, 0, 0, 150))
            p.setPen(QColor("#ffffff"))
            p.drawText(box, Qt.AlignCenter, self.info)


class CornerGrip(QWidget):
    """Bottom-left drag handle: the panel is pinned top-right, so dragging left
    / down makes it wider / taller."""

    def __init__(self, panel):
        super().__init__(panel)
        self.setFixedSize(16, 16)
        self.setAttribute(Qt.WA_TransparentForMouseEvents)   # the panel does the drag

    def paintEvent(self, _):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        p.setPen(QPen(QColor(C["outline"]), 1.4))
        for k in (4, 8, 12):
            p.drawLine(1, 15 - k, 1 + k, 15)


class CameraPanel(QFrame):
    """Floating card: header with a Simulated / Real tab + the image. Simulated
    renders the scene from the hand camera's lens; Real is the live MaixCam with
    its IP / connect row (+ SSH settings). The real stream keeps running (and
    detecting tags) while the Simulated tab is shown."""

    MIN = QSize(230, 150)
    DEFAULT = QSize(360, 330)
    EDGE = 8                           # px band on the left / bottom edge that resizes
    CORNER = 20                        # bottom-left square that resizes both ways

    def __init__(self, parent, scene):
        super().__init__(parent)
        self.scene = scene
        self.setObjectName("Card")
        # The VTK view is a native window; a native panel stacks above it.
        self.setAttribute(Qt.WA_NativeWindow)
        self.prefs = QSettings("ELIO", "HILToolkit")
        self.client = None
        self._retired = []             # disconnected clients still cleaning up
        self._drag = None              # (edges, global start pos, start size)
        self.setMouseTracking(True)
        self.res = (512, 320)
        self._fps_t, self._fps_n, self.fps = time.monotonic(), 0, 0.0
        sz = self.prefs.value("camera/size", self.DEFAULT)
        self.user_size = sz if isinstance(sz, QSize) else QSize(self.DEFAULT)

        lay = QVBoxLayout(self)
        m = 8 if theme.COMPACT else 10
        lay.setContentsMargins(m, m - 2, m, m)
        lay.setSpacing(6)

        head = QHBoxLayout()
        head.setSpacing(6)
        head.addWidget(label("Camera", "CardTitle"))
        self.tabs = QTabBar()
        self.tabs.setDrawBase(False)
        self.tabs.setExpanding(False)
        self.tabs.addTab("Simulated")
        self.tabs.addTab("Real")
        self.tabs.setStyleSheet(
            f"QTabBar::tab {{ padding: 2px 8px; font-weight: 700; color: {C['outline']}; "
            f"background: transparent; border: none; border-bottom: 2px solid transparent; }}"
            f"QTabBar::tab:selected {{ color: {C['primary']}; border-bottom: 2px solid {C['primary']}; }}")
        head.addWidget(self.tabs)
        head.addStretch()
        self.chip = Chip("Offline", "idle")
        head.addWidget(self.chip)
        self.fold_btn = QPushButton("Hide")
        self.fold_btn.setToolTip("Fold / unfold the camera panel")
        self.fold_btn.clicked.connect(lambda: self.set_folded(not self.folded))
        head.addWidget(self.fold_btn)
        lay.addLayout(head)

        self.body = QStackedWidget()
        self.sim_view = CameraView()
        self.sim_view.placeholder = "Simulated hand camera"
        self.body.addWidget(self.sim_view)
        real = QWidget()
        self.body.addWidget(real)
        body = QVBoxLayout(real)
        body.setContentsMargins(0, 0, 0, 0)
        body.setSpacing(6)
        self.view = CameraView()
        body.addWidget(self.view, 1)

        row = QHBoxLayout()
        row.setSpacing(6)
        self.ip_edit = QLineEdit(self.prefs.value("camera/ip", "", type=str))
        self.ip_edit.setPlaceholderText("camera IP, e.g. 192.168.1.42")
        self.ip_edit.returnPressed.connect(self.toggle_connect)
        row.addWidget(self.ip_edit, 1)
        self.gear_btn = QPushButton("SSH")
        self.gear_btn.setObjectName("Toggle")
        self.gear_btn.setCheckable(True)
        self.gear_btn.setToolTip("SSH login and capture resolution")
        row.addWidget(self.gear_btn)
        self.tag_btn = QPushButton("Tags")
        self.tag_btn.setObjectName("Toggle")
        self.tag_btn.setCheckable(True)
        self.tag_btn.setChecked(self.prefs.value("camera/tags", True, type=bool))
        self.tag_btn.setToolTip("Detect AprilTags (tag36h11) and mark each centre with a dot")
        self.tag_btn.toggled.connect(self._on_tags_toggled)
        row.addWidget(self.tag_btn)
        self.conn_btn = QPushButton("Connect")
        self.conn_btn.setObjectName("Primary")
        self.conn_btn.clicked.connect(self.toggle_connect)
        row.addWidget(self.conn_btn)
        body.addLayout(row)

        self.settings_row = QWidget()
        srow = QHBoxLayout(self.settings_row)
        srow.setContentsMargins(0, 0, 0, 0)
        srow.setSpacing(6)
        self.user_edit = QLineEdit(self.prefs.value("camera/user", "root", type=str))
        self.user_edit.setPlaceholderText("user")
        self.user_edit.setToolTip("SSH user (MaixCam default: root)")
        srow.addWidget(self.user_edit, 1)
        self.pass_edit = QLineEdit("root")
        self.pass_edit.setEchoMode(QLineEdit.Password)
        self.pass_edit.setPlaceholderText("password")
        self.pass_edit.setToolTip("SSH password (MaixCam default: root; not saved)")
        srow.addWidget(self.pass_edit, 1)
        self.res_box = QComboBox()
        self.res_box.addItems(RESOLUTIONS)
        self.res_box.setCurrentText(self.prefs.value("camera/res", RESOLUTIONS[0], type=str))
        self.res_box.setToolTip("Capture resolution (applies on the next Connect)")
        srow.addWidget(self.res_box)
        self.settings_row.setVisible(False)
        self.gear_btn.toggled.connect(self._show_settings)
        body.addWidget(self.settings_row)

        self.msg = label("", "Muted")
        self.msg.setWordWrap(True)
        self.msg.setVisible(False)
        body.addWidget(self.msg)
        lay.addWidget(self.body, 1)

        self.grip = CornerGrip(self)
        self.folded = False
        self.tabs.setCurrentIndex(self.prefs.value("camera/tab", 0, type=int))
        self.body.setCurrentIndex(self.tabs.currentIndex())
        self.tabs.currentChanged.connect(self._on_tab)
        self.set_folded(self.prefs.value("camera/folded", False, type=bool), save=False)

    # ── simulated camera ──
    def _on_tab(self, i):
        self.body.setCurrentIndex(i)
        self.refresh_sim()
        self.save_prefs()
        if self.parent():
            self.parent().place_overlay()

    def refresh_sim(self):
        """Re-render the simulated view; only while it is the one on screen."""
        if self.folded or self.tabs.currentIndex() != 0:
            return
        w, h = (int(v) for v in self.res_box.currentText().split("x"))
        rgb = np.ascontiguousarray(self.scene.pov_image(w, h, CAM_HFOV_DEG))
        img = QImage(rgb.data, w, h, 3 * w, QImage.Format_RGB888).copy()
        self.sim_view.set_image(img, f"sim  {w}×{h}  HFOV {CAM_HFOV_DEG:.0f}°")

    # ── geometry (the host places the panel; we only report the size) ──
    def resize_to(self, size):
        self.user_size = size.expandedTo(self.MIN)
        if self.parent():
            self.parent().place_overlay()

    def target_size(self, avail):
        w = min(self.user_size.width(), max(self.MIN.width(), avail.width()))
        if self.folded:
            return QSize(w, self.layout().sizeHint().height())
        need = self.minimumSizeHint().height()
        h = min(max(self.user_size.height(), need), max(need, avail.height()))
        return QSize(w, h)

    def _edges(self, pos):
        """Which edges a press at pos would drag: '', 'l', 'b' or 'lb'."""
        if self.folded:
            return ""
        x, y, h = pos.x(), pos.y(), self.height()
        if x <= self.CORNER and y >= h - self.CORNER:
            return "lb"
        return ("l" if x <= self.EDGE else "") + ("b" if y >= h - self.EDGE else "")

    def mousePressEvent(self, e):
        edges = self._edges(e.pos())
        if e.button() == Qt.LeftButton and edges:
            self._drag = (edges, e.globalPos(), QSize(self.user_size))
            e.accept()
        else:
            super().mousePressEvent(e)

    def mouseMoveEvent(self, e):
        if self._drag:
            edges, p0, s0 = self._drag
            d = e.globalPos() - p0
            self.resize_to(QSize(s0.width() - d.x() if "l" in edges else s0.width(),
                                 s0.height() + d.y() if "b" in edges else s0.height()))
            return
        cursor = {"l": Qt.SizeHorCursor, "b": Qt.SizeVerCursor, "lb": Qt.SizeBDiagCursor}
        edges = self._edges(e.pos())
        if edges:
            self.setCursor(cursor[edges])
        else:
            self.unsetCursor()

    def mouseReleaseEvent(self, e):
        if self._drag:
            self._drag = None
            self.save_prefs()
        super().mouseReleaseEvent(e)

    def resizeEvent(self, e):
        self.grip.move(2, self.height() - self.grip.height() - 2)
        self.grip.raise_()
        super().resizeEvent(e)

    def set_folded(self, on, save=True):
        self.folded = on
        self.body.setVisible(not on)
        self.grip.setVisible(not on)
        self.fold_btn.setText("Show" if on else "Hide")
        self.refresh_sim()
        if self.parent():
            self.parent().place_overlay()
        if save:
            self.save_prefs()

    def tag_snapshot(self, since):
        """Tags in the newest frame, if it was taken after `since` (monotonic).
        Returns ((w, h), [tags]) or a reason string."""
        c = self.client
        if c is None or self.chip.text() not in ("Live", "Stalled"):
            return "camera not connected"
        if not c.detect_tags:
            return "tag detection is off (Tags button)"
        snap = c.last_tags
        if snap is None or time.monotonic() - snap[0] > TAG_MAX_AGE_S:
            return "no recent frame from the camera"
        if snap[0] < since:
            return "no frame since the arm stopped yet, try again"
        return snap[1], snap[2]

    def _on_tags_toggled(self, on):
        if self.client is not None:
            self.client.detect_tags = on

    def _show_settings(self, on):
        self.settings_row.setVisible(on)
        if self.parent():
            self.parent().place_overlay()

    def save_prefs(self):
        self.prefs.setValue("camera/ip", self.ip_edit.text().strip())
        self.prefs.setValue("camera/user", self.user_edit.text().strip())
        self.prefs.setValue("camera/res", self.res_box.currentText())
        self.prefs.setValue("camera/size", self.user_size)
        self.prefs.setValue("camera/folded", self.folded)
        self.prefs.setValue("camera/tags", self.tag_btn.isChecked())
        self.prefs.setValue("camera/tab", self.tabs.currentIndex())

    # ── connection ──
    def toggle_connect(self):
        if self.client is not None:
            self.disconnect_camera()
            return
        ip = self.ip_edit.text().strip()
        if not ip:
            self._message("Type the camera's IP address first.", error=True)
            self.ip_edit.setFocus()
            return
        w, h = (int(v) for v in self.res_box.currentText().split("x"))
        self.res = (w, h)
        self.save_prefs()
        self._message("")
        self.client = CameraClient(ip, self.user_edit.text().strip() or "root", self.pass_edit.text(),
                                   w, h)
        self.client.detect_tags = self.tag_btn.isChecked()
        self.client.frame.connect(self._on_frame)
        self.client.status.connect(self._on_status)
        self.client.log.connect(self._on_log)
        self.client.finished.connect(self._on_finished)
        self._fps_t, self._fps_n, self.fps = time.monotonic(), 0, 0.0
        self.view.clear(f"Connecting to {ip}…")
        self.conn_btn.setText("Disconnect")
        self.conn_btn.setObjectName("")
        self._restyle(self.conn_btn)
        for w_ in (self.ip_edit, self.user_edit, self.pass_edit, self.res_box):
            w_.setEnabled(False)
        self.client.start()

    def disconnect_camera(self):
        """Let the client clean up in the background and free the panel at
        once, so Connect can be pressed again straight away."""
        c = self.client
        if c is None:
            return
        for sig, slot in ((c.frame, self._on_frame), (c.status, self._on_status),
                          (c.log, self._on_log), (c.finished, self._on_finished)):
            try:
                sig.disconnect(slot)
            except TypeError:
                pass
        c.stop()
        self._retired.append(c)
        c.finished.connect(lambda c=c: self._retired.remove(c) if c in self._retired else None)
        self.chip.set("Offline", "idle")
        self._on_finished()

    def shutdown(self):
        clients = [c for c in [self.client] + self._retired if c is not None]
        for c in clients:
            c.stop()
        for c in clients:
            c.wait(4000)

    def _on_finished(self):
        self.client = None
        self.conn_btn.setEnabled(True)
        self.conn_btn.setText("Connect")
        self.conn_btn.setObjectName("Primary")
        self._restyle(self.conn_btn)
        for w_ in (self.ip_edit, self.user_edit, self.pass_edit, self.res_box):
            w_.setEnabled(True)
        if self.chip.text() not in ("Error", "paramiko missing"):
            self.chip.set("Offline", "idle")
            self.view.clear("Disconnected")
        else:
            self.view.clear("Connection failed (see below). Press Connect to try again.")

    def _on_status(self, text, tone):
        self.chip.set(text, tone)

    def _on_log(self, text):
        print(f"[camera] {text}", flush=True)
        low = text.lower()
        if self.chip.text() in ("Live",) and not any(k in low for k in ("error", "fail", "traceback")):
            return
        self._message(text, error=True)

    def _on_frame(self, img):
        self._fps_n += 1
        now = time.monotonic()
        if now - self._fps_t >= 1.0:
            self.fps = self._fps_n / (now - self._fps_t)
            self._fps_t, self._fps_n = now, 0
        if self.msg.isVisible():
            self._message("")
        tags = f"  {self.client.tag_count} tag(s)" if self.client and self.client.detect_tags else ""
        self.view.set_image(img, f"{img.width()}×{img.height()}  {self.fps:4.1f} fps{tags}")

    def _message(self, text, error=False):
        self.msg.setText(text)
        self.msg.setStyleSheet(f"color: {C['error']};" if error else "")
        self.msg.setVisible(bool(text))
        if self.parent():
            self.parent().place_overlay()

    @staticmethod
    def _restyle(w):
        w.style().unpolish(w)
        w.style().polish(w)


class ViewHost(QWidget):
    """Holds the 3D view (filling it) and the camera panel pinned top-right."""

    MARGIN = 10

    def __init__(self):
        super().__init__()
        self.scene_widget = None
        self.overlay = None

    def set_children(self, scene_widget, overlay):
        self.scene_widget, self.overlay = scene_widget, overlay
        overlay.raise_()
        self.place_overlay()

    def place_overlay(self):
        if self.scene_widget is not None:
            self.scene_widget.setGeometry(self.rect())
        if self.overlay is None:
            return
        m = self.MARGIN
        avail = QSize(self.width() - 2 * m, self.height() - 2 * m)
        s = self.overlay.target_size(avail)
        self.overlay.setGeometry(self.width() - m - s.width(), m, s.width(), s.height())
        self.overlay.raise_()

    def resizeEvent(self, e):
        self.place_overlay()
        super().resizeEvent(e)


# ─────────────────────────────────────────────────────────────────────────────
# Main window: the stock one with the camera panel in the 3D card
# ─────────────────────────────────────────────────────────────────────────────
class CameraViewMixin:
    def _build_view_card(self):
        card, lay = make_card("Arm", "sim pose = command sent")
        self.view_host = ViewHost()
        self.scene = Scene(self.view_host)
        self.camera = CameraPanel(self.view_host, self.scene)
        self.view_host.set_children(self.scene.widget, self.camera)
        lay.addWidget(self.view_host, 1)
        self.pose_lbl = label("", "Mono")
        self.pose_lbl.setWordWrap(True)
        lay.addWidget(self.pose_lbl)
        return card

    # ── AprilTag capture in sequence steps ──
    def _build_sequence_tab(self):
        card = super()._build_sequence_tab()
        lay = card.layout()
        row = QHBoxLayout()
        self.tag_on_save = QCheckBox("Tag on save")
        self.tag_on_save.setChecked(self.camera.prefs.value("camera/tag_on_save", True, type=bool))
        self.tag_on_save.setToolTip("Add current pose / Update also store the AprilTags the camera "
                                    "sees (steps with no tag in view are saved without one)")
        self.tag_on_save.toggled.connect(lambda on: self.camera.prefs.setValue("camera/tag_on_save", on))
        row.addWidget(self.tag_on_save)
        cap = QPushButton("Capture tag")
        cap.setToolTip("Store the tags in view in the selected step (the arm must be at that step)")
        cap.clicked.connect(self._step_capture_tag)
        row.addWidget(cap)
        clr = QPushButton("Clear tag")
        clr.setToolTip("Remove the selected step's tag")
        clr.clicked.connect(self._step_clear_tag)
        row.addWidget(clr)
        row.addStretch()
        lay.insertLayout(lay.indexOf(self.step_pose) + 1, row)
        return card

    def _show_pose(self):
        super()._show_pose()
        self.camera.refresh_sim()

    def _tick(self):
        moving_before = getattr(self, "_was_moving", False)
        super()._tick()
        if self.motion.moving or moving_before:
            self._still_since = time.monotonic()

    def _capture(self, step):
        """Fill step.tag from the camera. Returns (ok, note for the status line)."""
        if self.motion.moving:
            return False, "the arm is still moving"
        servo = [float(v) for v in self.motion.cmd]
        if max(abs(servo[k] - step.servo[k]) for k in (AXES_IDX["Arm Base"], AXES_IDX["Arm A"])) > 0.5:
            return False, "the arm is not at this step (double-click it to go there first)"
        snap = self.camera.tag_snapshot(getattr(self, "_still_since", 0.0) + TAG_SETTLE_S)
        if isinstance(snap, str):
            return False, snap
        size, found = snap
        if not found:
            return False, "no AprilTag in view"
        step.tag = tag_record(found, size, servo)
        return True, describe_tag(step.tag)

    def _run_tick(self):
        """Before a step's hold starts, centre its saved tag with Arm Base."""
        r = self.run
        q = r["seq"]
        s = q.steps[r["idx"]] if r["idx"] < len(q.steps) else None
        if s is not None and s.tag and s.tag.get("tags") and not self._align_done(r, s):
            return
        super()._run_tick()

    def _align_done(self, r, s):
        """One tick of the centring loop. True when finished (or given up)."""
        st = r.get("align")
        if st is None or st["idx"] != r["idx"]:
            st = r["align"] = {"idx": r["idx"], "moves": 0, "deadline": time.monotonic() + ALIGN_WAIT_S}
        if st.get("done"):
            return True
        saved = s.tag["tags"][0]
        snap = self.camera.tag_snapshot(getattr(self, "_still_since", 0.0) + TAG_SETTLE_S)
        if isinstance(snap, str):
            if "not connected" in snap or "detection is off" in snap or time.monotonic() > st["deadline"]:
                return self._align_end(r, s, st, f"not centred: {snap}")
            return False
        (w, _h), found = snap
        match = next((t for t in found if t["id"] == saved["id"]), None)
        if match is None:
            if time.monotonic() > st["deadline"]:
                return self._align_end(r, s, st, f"not centred: tag {saved['id']} not in view")
            return False
        err = AZ_SIGN * (h_angle(match["center"][0], w) - saved_offset_deg(saved))   # + = tag too far right
        if abs(err) <= ALIGN_TOL_DEG:
            return self._align_end(r, s, st, f"tag {saved['id']} centred ({err:+.1f}°)")
        if st["moves"] >= ALIGN_MAX_MOVES:
            return self._align_end(r, s, st, f"gave up centring, {err:+.1f}° off")
        step = max(-ALIGN_MAX_STEP_DEG, min(ALIGN_MAX_STEP_DEG, ALIGN_GAIN * err))
        goal = self.motion.final_goal.copy()
        goal[AXES_IDX["Arm Base"]] += step                  # tag right -> turn right, left -> left
        why = self._request(goal)
        if why:
            return self._align_end(r, s, st, f"not centred: {why}")
        st["moves"] += 1
        st["deadline"] = time.monotonic() + ALIGN_WAIT_S
        self._goal_changed()
        self._run_msg(f"Step {r['idx'] + 1} “{s.name}”: centring tag {saved['id']}, "
                      f"{err:+.1f}° {'right' if err > 0 else 'left'} of its line")
        return False

    def _align_end(self, r, s, st, note):
        st["done"] = True
        self._run_msg(f"Step {r['idx'] + 1} “{s.name}”: {note}")
        return True

    def _step_add(self):
        super()._step_add()
        if self.tag_on_save.isChecked():
            self._tag_after_save("Added")

    def _step_update(self):
        q, i = self._sel_step()
        had = bool(q and q.steps[i].tag)
        super()._step_update()
        if q and self.tag_on_save.isChecked():
            q.steps[i].tag = None              # the old tag belongs to the old pose
            self._tag_after_save("Updated", dropped=had)

    def _tag_after_save(self, verb, dropped=False):
        q, i = self._sel_step()
        if not q:
            return
        s = q.steps[i]
        ok, note = self._capture(s)
        self._seq_edited()
        if ok:
            ids = ", ".join(str(t["id"]) for t in s.tag["tags"])
            self._seq_msg(f"{verb} “{s.name}” with tag {ids}.")
        else:
            self._seq_msg(f"{verb} “{s.name}” without a tag: {note}."
                          + (" Its old tag was removed." if dropped else ""))

    def _step_capture_tag(self):
        q, i = self._sel_step()
        if not q:
            self._seq_msg("Select a step first.", error=True)
            return
        s = q.steps[i]
        ok, note = self._capture(s)
        if not ok:
            self._seq_msg(f"No tag captured for “{s.name}”: {note}.", error=True)
            return
        self._seq_edited()
        self._seq_msg(f"Tag stored in “{s.name}”.")

    def _step_clear_tag(self):
        q, i = self._sel_step()
        if q and q.steps[i].tag:
            q.steps[i].tag = None
            self._seq_edited()
            self._seq_msg(f"Tag removed from “{q.steps[i].name}”.")

    def _show_step(self, *_):
        super()._show_step()
        q, i = self._sel_step()
        if q and q.steps[i].tag:
            self.step_pose.setText(self.step_pose.text() + "\n" + describe_tag(q.steps[i].tag))

    def closeEvent(self, e):
        self.camera.save_prefs()
        self.camera.shutdown()
        super().closeEvent(e)


AXES_IDX = {a.name: i for i, a in enumerate(config.AXES)}


def build_window():
    from hil.main_window import MainWindow      # after setup(): limits must be loaded first

    class CameraMainWindow(CameraViewMixin, MainWindow):
        pass

    w = CameraMainWindow()
    w.setWindowTitle("ELIOS HIL Toolkit · Camera")
    return w


def main():
    if sys.platform == "win32":
        import ctypes
        ctypes.windll.shell32.SetCurrentProcessExplicitAppUserModelID("ELIO.HILToolkit")

    app = QApplication(sys.argv)
    if os.path.exists(config.APP_ICON_FILE):
        app.setWindowIcon(QIcon(config.APP_ICON_FILE))
    if "Zen Dots" not in QFontDatabase().families() and os.path.exists(config.BRAND_FONT_FILE):
        QFontDatabase.addApplicationFont(config.BRAND_FONT_FILE)

    avail = app.primaryScreen().availableGeometry()
    theme.COMPACT = avail.width() <= theme.COMPACT_MAX_W or avail.height() <= theme.COMPACT_MAX_H

    setup()
    window = build_window()
    window.showFullScreen()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
