"""
Camera Manager for ELIOS HIL Robot Arm WebApp.
Ported from arm-hil-toolkit-2.py MaixCam2 SSH streaming + AprilTag detection,
with synthetic POV rendering for the simulated camera feed.
"""
import io
import math
import socket
import struct
import sys
import threading
import time
from typing import Dict, List, Optional, Tuple

import cv2
import numpy as np

from hil import config
from hil_toolkit.tags_toolkit.apriltag import AprilTagDetector

REMOTE_PATH = "/tmp/elios_cam_stream.py"
MAGIC = b"MXJP"
MAX_FRAME = 4 * 1024 * 1024
RESOLUTIONS = ["512x320", "320x240", "640x480", "1280x720"]
FIRST_FRAME_TIMEOUT_S = 12
STALL_TIMEOUT_S = 5
CMD_TIMEOUT_S = 6

CAM_HFOV_DEG = 60.0
AZ_SIGN = +1.0
EL_SIGN = +1.0
TAG_MAX_AGE_S = 1.0
TAG_SETTLE_S = 0.4

DEVICE_SCRIPT = r'''
import os, struct, sys

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
    pass
'''


def tag_center(corners):
    p = np.asarray(corners, dtype=float).reshape(4, 2)
    d1, d2 = p[2] - p[0], p[3] - p[1]
    den = d1[0] * d2[1] - d1[1] * d2[0]
    if abs(den) < 1e-9:
        return p.mean(axis=0)
    t = ((p[1] - p[0])[0] * d2[1] - (p[1] - p[0])[1] * d2[0]) / den
    return p[0] + t * d1


def annotate_tags(bgr: np.ndarray, detector: AprilTagDetector) -> Tuple[np.ndarray, List[dict]]:
    dets = detector.detect(bgr)
    r = max(3, bgr.shape[1] // 100)
    found = []
    for d in dets:
        cv2.polylines(bgr, [d.corners.astype(np.int32).reshape(-1, 1, 2)], True, (0, 255, 0), 2)
        c = tag_center(d.corners)
        found.append({
            "id": int(d.tag_id),
            "center": [float(c[0]), float(c[1])],
            "corners": d.corners.astype(float).tolist()
        })
        cx, cy = int(round(c[0])), int(round(c[1]))
        cv2.circle(bgr, (cx, cy), r + 2, (255, 255, 255), -1, cv2.LINE_AA)
        cv2.circle(bgr, (cx, cy), r, (0, 0, 255), -1, cv2.LINE_AA)
        cv2.putText(bgr, f"id {d.tag_id}", (cx + r + 4, cy - r - 2), cv2.FONT_HERSHEY_SIMPLEX,
                    0.5, (0, 255, 255), 1, cv2.LINE_AA)
    return bgr, found


def tag_record(found: List[dict], size: Tuple[int, int], servo: List[float]) -> dict:
    base = config.AXIS_BY_NAME["Arm Base"]
    arm_a = config.AXIS_BY_NAME["Arm A"]
    base_servo = float(servo[config.AXES.index(base)])
    arm_a_servo = float(servo[config.AXES.index(arm_a)])
    cam_az, cam_el = base.to_sim(base_servo), arm_a.to_sim(arm_a_servo)
    w, h = size
    f = (w / 2.0) / math.tan(math.radians(CAM_HFOV_DEG) / 2.0)
    tags = []
    for t in sorted(found, key=lambda t: t["id"]):
        u, v = t["center"]
        dx, dy = u - w / 2.0, h / 2.0 - v
        h_deg, v_deg = math.degrees(math.atan2(dx, f)), math.degrees(math.atan2(dy, f))
        cs = np.asarray(t["corners"])
        tags.append({
            "id": int(t["id"]),
            "px": [round(u, 1), round(v, 1)],
            "offset_px": [round(dx, 1), round(dy, 1)],
            "offset_norm": [round(dx / (w / 2.0), 4), round(dy / (h / 2.0), 4)],
            "offset_deg": [round(h_deg, 2), round(v_deg, 2)],
            "azimuth_deg": round(cam_az + AZ_SIGN * h_deg, 2),
            "elevation_deg": round(cam_el + EL_SIGN * v_deg, 2),
            "size_px": round(float(np.mean(np.linalg.norm(cs - np.roll(cs, 1, axis=0), axis=1))), 1),
        })
    return {
        "time": time.strftime("%Y-%m-%dT%H:%M:%S"),
        "image": [int(w), int(h)],
        "hfov_deg": CAM_HFOV_DEG,
        "camera": {
            "azimuth_deg": round(cam_az, 2),
            "elevation_deg": round(cam_el, 2),
            "base_servo": round(base_servo, 2),
            "arm_a_servo": round(arm_a_servo, 2)
        },
        "tags": tags,
    }


def describe_tag(tag: dict) -> str:
    cam = tag.get("camera", {})
    parts = [
        f"id{t['id']} at ({t['px'][0]:.0f}, {t['px'][1]:.0f}) px  "
        f"az {t['azimuth_deg']:.1f}°  el {t['elevation_deg']:.1f}°"
        for t in tag.get("tags", [])
    ]
    return (
        f"Tag  camera az {cam.get('azimuth_deg', 0):.1f}°  el {cam.get('elevation_deg', 0):.1f}°   "
        + ("   ".join(parts) or "no tags")
    )


class CameraManager:
    def __init__(self):
        self.host = "192.168.1.42"
        self.user = "root"
        self.password = "root"
        self.res = (512, 320)
        self.detect_tags = True
        self.connected = False
        self.status = "Offline"
        self.status_tone = "idle"
        self.fps = 0.0
        self.tag_count = 0
        self.last_tags = None  # (timestamp, (w, h), found)
        self.last_real_frame: Optional[bytes] = None
        self.last_annotated_frame: Optional[bytes] = None
        self.error_msg = ""
        self._thread: Optional[threading.Thread] = None
        self._stop_event = threading.Event()
        self._ssh = None
        self._pid: Optional[int] = None
        self._detector: Optional[AprilTagDetector] = None

        try:
            self._detector = AprilTagDetector()
        except Exception as e:
            print(f"[CameraManager] AprilTagDetector init error: {e}")

    def get_status(self) -> dict:
        return {
            "connected": self.connected,
            "status": self.status,
            "status_tone": self.status_tone,
            "fps": round(self.fps, 1),
            "tag_count": self.tag_count,
            "error_msg": self.error_msg,
            "host": self.host,
            "user": self.user,
            "res": f"{self.res[0]}x{self.res[1]}",
            "detect_tags": self.detect_tags
        }

    def connect(self, host: str, user: str = "root", password: str = "root",
                res_str: str = "512x320", detect_tags: bool = True):
        if self.connected:
            self.disconnect()

        self.host = host.strip()
        self.user = user.strip() or "root"
        self.password = password
        try:
            w, h = (int(v) for v in res_str.split("x"))
            self.res = (w, h)
        except Exception:
            self.res = (512, 320)
        self.detect_tags = detect_tags
        self._stop_event.clear()
        self.status = "Connecting…"
        self.status_tone = "info"
        self.error_msg = ""

        self._thread = threading.Thread(target=self._run_client, daemon=True)
        self._thread.start()

    def disconnect(self):
        self._stop_event.set()
        self.connected = False
        self.status = "Offline"
        self.status_tone = "idle"
        self.fps = 0.0
        self.tag_count = 0

    def _run_client(self):
        try:
            import paramiko
        except ImportError:
            self.status = "paramiko missing"
            self.status_tone = "bad"
            self.error_msg = "paramiko is not installed"
            return

        import logging
        logging.getLogger("paramiko").setLevel(logging.CRITICAL)
        ssh = paramiko.SSHClient()
        ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
        self._ssh = ssh
        last_err = ""

        try:
            ssh.connect(self.host, port=22, username=self.user, password=self.password,
                        timeout=6, banner_timeout=6, auth_timeout=6,
                        allow_agent=False, look_for_keys=False)
            if self._stop_event.is_set():
                return

            # Kill old stream script if left over from an earlier run
            self._exec(ssh, "pkill -f 'elios_cam_strea[m].py'; true")

            # Write device script via stdin piping (works without SFTP installed on camera)
            code, err = self._exec(ssh, f"cat > {REMOTE_PATH}", DEVICE_SCRIPT.encode())
            if code != 0:
                raise RuntimeError(f"could not write {REMOTE_PATH} on camera: {err or code}")

            if self._stop_event.is_set():
                return

            w, h = self.res
            chan = ssh.get_transport().open_session(timeout=CMD_TIMEOUT_S)
            chan.settimeout(0.3)
            cmd = (
                "P=python3; command -v python3 >/dev/null 2>&1 || P=python; "
                f"exec $P -u {REMOTE_PATH} {w} {h} 70 0"
            )
            chan.exec_command(cmd)
            self.status = "Starting camera…"
            self.status_tone = "info"

            buf = bytearray()
            err_buf = b""
            started = time.monotonic()
            last_frame = None
            fps_count = 0
            fps_t0 = time.monotonic()

            while not self._stop_event.is_set():
                while chan.recv_stderr_ready():
                    err_buf += chan.recv_stderr(4096)
                    *lines, err_buf = err_buf.split(b"\n")
                    for ln in lines:
                        text = ln.decode("utf-8", "replace").rstrip()
                        if text.startswith("PID:"):
                            try:
                                self._pid = int(text[4:])
                            except ValueError:
                                pass
                        elif text:
                            last_err = text

                try:
                    data = chan.recv(65536)
                except socket.timeout:
                    data = None

                if data == b"":
                    break
                if data:
                    buf += data
                    for jpeg in self._extract_frames(buf):
                        self.last_real_frame = jpeg
                        if last_frame is None:
                            self.connected = True
                            self.status = "Live"
                            self.status_tone = "ok"
                        last_frame = time.monotonic()
                        fps_count += 1

                        # Decode & annotate
                        bgr = cv2.imdecode(np.frombuffer(jpeg, np.uint8), cv2.IMREAD_COLOR)
                        if bgr is not None:
                            found = []
                            if self.detect_tags and self._detector is not None:
                                try:
                                    bgr, found = annotate_tags(bgr, self._detector)
                                except Exception as e:
                                    print(f"[camera] annotate_tags error: {e}")
                            self.tag_count = len(found)
                            self.last_tags = (time.monotonic(), (w, h), found)
                            _, enc = cv2.imencode(".jpg", bgr, [cv2.IMWRITE_JPEG_QUALITY, 80])
                            self.last_annotated_frame = enc.tobytes()

                now = time.monotonic()
                if now - fps_t0 >= 1.0:
                    self.fps = fps_count / (now - fps_t0)
                    fps_count = 0
                    fps_t0 = now

                if last_frame is None and now - started > FIRST_FRAME_TIMEOUT_S:
                    raise RuntimeError(last_err or "No frames received from the camera")
                if last_frame is not None and now - last_frame > STALL_TIMEOUT_S:
                    self.status = "Stalled"
                    self.status_tone = "warn"
                    last_frame = now

            self.connected = False
            self.status = "Offline"
            self.status_tone = "idle"
        except Exception as e:
            if not self._stop_event.is_set():
                err_text = str(e) or type(e).__name__
                if "Authentication" in err_text:
                    err_text = "SSH login refused (check user / password)"
                elif "timed out" in err_text.lower():
                    err_text = f"SSH to {self.host} timed out. Check IP and Wi-Fi."
                elif "Connection refused" in err_text:
                    err_text = f"{self.host} refused connection on port 22."
                self.error_msg = err_text
                self.status = "Offline"
                self.status_tone = "bad"
                self.connected = False
        finally:
            self._cleanup()

    def _cleanup(self):
        ssh, self._ssh = self._ssh, None
        if ssh is None:
            return
        try:
            t = ssh.get_transport()
            if self._pid and t is not None and t.is_active():
                self._exec(ssh, f"kill {self._pid}", timeout=3)
        except Exception:
            pass
        finally:
            try:
                ssh.close()
            except Exception:
                pass

    @staticmethod
    def _exec(ssh, cmd, stdin=None, timeout=CMD_TIMEOUT_S):
        """Run cmd on camera over SSH; never blocks longer than timeout."""
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

    @staticmethod
    def _extract_frames(buf: bytearray):
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

    def render_simulated_frame(self, servo_angles: List[float], width: int = 512, height: int = 320) -> bytes:
        """
        Renders synthetic perspective view from the hand camera looking out at the workspace.
        Camera is mounted on Arm A:
          cam azimuth = Arm Base joint angle
          cam elevation = Arm A joint angle
        """
        base_ax = config.AXIS_BY_NAME["Arm Base"]
        arm_a_ax = config.AXIS_BY_NAME["Arm A"]
        base_servo = servo_angles[config.AXES.index(base_ax)]
        arm_a_servo = servo_angles[config.AXES.index(arm_a_ax)]
        az = base_ax.to_sim(base_servo)
        el = arm_a_ax.to_sim(arm_a_servo)

        img = np.zeros((height, width, 3), dtype=np.uint8)
        # Background gradient: dark cockpit
        img[:, :] = (15, 15, 15)

        # Draw simulated ground perspective grid
        horizon_y = int(height * (0.45 - el * 0.003))
        horizon_y = max(20, min(height - 20, horizon_y))
        cv2.line(img, (0, horizon_y), (width, horizon_y), (50, 50, 50), 1)

        center_x = int(width / 2.0 - az * (width / 180.0))
        # Draw perspective lines vanishing towards (center_x, horizon_y)
        for vx in range(-width * 2, width * 3, 60):
            cv2.line(img, (center_x, horizon_y), (vx, height), (35, 35, 35), 1)
        for vy in range(horizon_y + 10, height, 25):
            cv2.line(img, (0, vy), (width, vy), (35, 35, 35), 1)

        # Draw simulated objects on the ground:
        # e.g. Waste items, conveyor bins, and an AprilTag (id 7)
        # Tag at world position in front of arm (near X=100, Y=-150)
        tag_world_az = 10.0
        tag_world_dist = 200.0  # mm
        rel_az = tag_world_az - az
        if abs(rel_az) < CAM_HFOV_DEG / 2.0 + 10:
            f = (width / 2.0) / math.tan(math.radians(CAM_HFOV_DEG) / 2.0)
            tag_px_x = int(width / 2.0 + f * math.tan(math.radians(rel_az)))
            tag_px_y = int(horizon_y + (height - horizon_y) * 0.55 + el * 1.5)
            tag_sz = 36

            x0, y0 = tag_px_x - tag_sz // 2, tag_px_y - tag_sz // 2
            x1, y1 = x0 + tag_sz, y0 + tag_sz
            if 0 <= x0 < width and 0 <= y0 < height:
                # White square border
                cv2.rectangle(img, (x0, y0), (x1, y1), (255, 255, 255), -1)
                # Black inner area
                cv2.rectangle(img, (x0 + 3, y0 + 3), (x1 - 3, y1 - 3), (0, 0, 0), -1)
                # White pixel bits resembling tag36h11 id 7
                cv2.rectangle(img, (x0 + 8, y0 + 8), (x0 + 16, y0 + 16), (255, 255, 255), -1)
                cv2.rectangle(img, (x1 - 16, y1 - 16), (x1 - 8, y1 - 8), (255, 255, 255), -1)

                # AprilTag overlay marker (green poly + red center dot + id text)
                cv2.rectangle(img, (x0 - 2, y0 - 2), (x1 + 2, y1 + 2), (0, 255, 0), 2)
                cv2.circle(img, (tag_px_x, tag_px_y), 4, (255, 255, 255), -1)
                cv2.circle(img, (tag_px_x, tag_px_y), 2, (0, 0, 255), -1)
                cv2.putText(img, "id 7", (x1 + 4, y0 + 12), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 0), 1)

        # Draw simulated conveyor blocks / sorting bins
        bin_x1, bin_y1 = int(width * 0.15), int(height * 0.65)
        cv2.rectangle(img, (bin_x1, bin_y1), (bin_x1 + 50, bin_y1 + 40), (60, 60, 60), -1)
        bin_x2 = int(width * 0.8)
        cv2.rectangle(img, (bin_x2, bin_y1), (bin_x2 + 50, bin_y1 + 40), (60, 60, 60), -1)

        # HUD text
        info_text = f"sim {width}x{height} - HFOV {CAM_HFOV_DEG:.0f} deg"
        cv2.putText(img, info_text, (10, height - 12), cv2.FONT_HERSHEY_SIMPLEX, 0.35, (160, 160, 160), 1)

        _, enc = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, 85])
        return enc.tobytes()

    def tag_snapshot(self, since: float) -> Tuple[Optional[Tuple[int, int]], Optional[List[dict]], str]:
        """Tags in the newest frame, if it was taken after `since` (monotonic).
        Returns ((w, h), found_tags, "ok") or (None, None, reason)."""
        if not self.connected or self.status not in ("Live", "Stalled"):
            return None, None, "camera not connected"
        if not self.detect_tags:
            return None, None, "tag detection is off (Tags button)"
        snap = self.last_tags
        if snap is None or time.monotonic() - snap[0] > TAG_MAX_AGE_S:
            return None, None, "no recent frame from the camera"
        if snap[0] < since:
            return None, None, "no frame since the arm stopped yet, try again"
        return snap[1], snap[2], "ok"
