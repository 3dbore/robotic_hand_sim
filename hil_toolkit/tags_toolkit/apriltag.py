"""
APRILTAG DETECTION — laptop webcam test tool
-----------------------------------------------
Live AprilTag detection using OpenCV's built-in `cv2.aruco` module (no extra
AprilTag library needed — the APRILTAG dictionaries ship with opencv-python
4.7+). Standalone: run this file directly to point your laptop's webcam at a
tag and see detections, and their X/Y/Z orientation axes, drawn in real time.

    python -m hil_toolkit.tags_toolkit.apriltag
    python -m hil_toolkit.tags_toolkit.apriltag --camera 1 --family tag25h9
    python -m hil_toolkit.tags_toolkit.apriltag --tag-size 0.032

Press 'q' or Esc to quit.

Orientation (pose) needs a camera matrix. Without --fx/--fy/--cx/--cy this
uses a rough guess from the frame size (fx=fy=width, centred principal point,
no distortion) — good enough to see the axes track the tag's rotation, but
not for metric accuracy. Pass real intrinsics from calibration.py's
CameraModel, or a fresh cv2.calibrateCamera() run, for accurate pose/distance.
"""
import argparse
import time
from dataclasses import dataclass

import cv2
import numpy as np

TAG_FAMILIES = {
    "tag16h5": cv2.aruco.DICT_APRILTAG_16h5,
    "tag25h9": cv2.aruco.DICT_APRILTAG_25h9,
    "tag36h10": cv2.aruco.DICT_APRILTAG_36h10,
    "tag36h11": cv2.aruco.DICT_APRILTAG_36h11,
}
DEFAULT_FAMILY = "tag36h11"
DEFAULT_TAG_SIZE_M = 0.05  # side length of the printed tag's black square

# Tag-frame corners matching the detector's TL, TR, BR, BL corner order:
# X right, Y up, Z out of the tag face (toward the camera when tag is facing it).
def tag_object_points(tag_size):
    s = tag_size / 2.0
    return np.array([
        [-s,  s, 0],
        [ s,  s, 0],
        [ s, -s, 0],
        [-s, -s, 0],
    ], dtype=np.float32)


@dataclass
class Detection:
    tag_id: int
    corners: np.ndarray  # (4, 2) float32, order: TL, TR, BR, BL
    center: np.ndarray   # (2,) float32
    rvec: np.ndarray = None  # (3, 1) float64, set by estimate_pose()
    tvec: np.ndarray = None  # (3, 1) float64, set by estimate_pose()


class AprilTagDetector:
    """Thin wrapper over cv2.aruco.ArucoDetector for one tag family."""

    def __init__(self, family=DEFAULT_FAMILY):
        if family not in TAG_FAMILIES:
            raise ValueError(f"Unknown tag family '{family}'. Options: {list(TAG_FAMILIES)}")
        self.family = family
        self._dictionary = cv2.aruco.getPredefinedDictionary(TAG_FAMILIES[family])
        self._params = cv2.aruco.DetectorParameters()
        # Sub-pixel corners: unrefined ones sit ~0.6 px inside the black square
        # (measured on the simulated camera: mean error 0.75 -> 0.20 px), which
        # shrinks the tag and biases PnP distance and calibrated focal length.
        self._params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        self._detector = cv2.aruco.ArucoDetector(self._dictionary, self._params)

    def detect(self, frame_bgr):
        gray = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2GRAY)
        corners, ids, _rejected = self._detector.detectMarkers(gray)
        if ids is None:
            return []
        detections = []
        for tag_corners, tag_id in zip(corners, ids.flatten()):
            pts = tag_corners.reshape(4, 2)
            detections.append(Detection(tag_id=int(tag_id), corners=pts, center=pts.mean(axis=0)))
        return detections


def size_for(tag_size, tag_id):
    """tag_size is one black-square side for every tag, or {tag_id: side};
    ids missing from the mapping get None (pose skipped)."""
    return tag_size.get(tag_id) if isinstance(tag_size, dict) else tag_size


def estimate_pose(detections, camera_matrix, dist_coeffs, tag_size=DEFAULT_TAG_SIZE_M):
    """Fill in rvec/tvec on each detection in place via solvePnP (planar-square
    method), using the object model from tag_object_points(). tvec comes out in
    tag_size's unit. Returns detections for convenience.
    """
    for det in detections:
        size = size_for(tag_size, det.tag_id)
        if size is None:
            continue
        ok, rvec, tvec = cv2.solvePnP(tag_object_points(size), det.corners, camera_matrix,
                                       dist_coeffs, flags=cv2.SOLVEPNP_IPPE_SQUARE)
        if ok:
            det.rvec, det.tvec = rvec, tvec
    return detections


def default_camera_matrix(width, height):
    """Rough pinhole guess when no real calibration is available: fx=fy=width,
    principal point at the frame centre, zero distortion.
    """
    camera_matrix = np.array([
        [width, 0, width / 2.0],
        [0, width, height / 2.0],
        [0, 0, 1.0],
    ], dtype=np.float64)
    dist_coeffs = np.zeros(5, dtype=np.float64)
    return camera_matrix, dist_coeffs


def draw_detections(frame_bgr, detections, camera_matrix=None, dist_coeffs=None,
                     axis_length=DEFAULT_TAG_SIZE_M):
    for det in detections:
        pts = det.corners.astype(int)
        cv2.polylines(frame_bgr, [pts], isClosed=True, color=(0, 255, 0), thickness=2)
        cx, cy = det.center.astype(int)
        cv2.circle(frame_bgr, (cx, cy), 4, (0, 0, 255), -1)
        cv2.putText(frame_bgr, f"id={det.tag_id}", (pts[0][0], pts[0][1] - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2, cv2.LINE_AA)
        if det.rvec is not None and camera_matrix is not None:
            cv2.drawFrameAxes(frame_bgr, camera_matrix, dist_coeffs, det.rvec, det.tvec,
                              size_for(axis_length, det.tag_id))
    return frame_bgr


def open_camera(index=0, width=None, height=None):
    backend = cv2.CAP_DSHOW if hasattr(cv2, "CAP_DSHOW") else cv2.CAP_ANY
    cap = cv2.VideoCapture(index, backend)
    if not cap.isOpened():
        cap = cv2.VideoCapture(index)
    if not cap.isOpened():
        raise RuntimeError(f"Could not open camera index {index}")
    if width:
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    if height:
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    return cap


def run(camera_index=0, family=DEFAULT_FAMILY, width=None, height=None,
        tag_size=DEFAULT_TAG_SIZE_M, fx=None, fy=None, cx=None, cy=None):
    detector = AprilTagDetector(family)
    cap = open_camera(camera_index, width, height)
    window = f"AprilTag Test — {family} (cam {camera_index})"

    frame_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    frame_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    camera_matrix, dist_coeffs = default_camera_matrix(frame_w, frame_h)
    if fx is not None:
        camera_matrix[0, 0] = fx
    if fy is not None:
        camera_matrix[1, 1] = fy
    if cx is not None:
        camera_matrix[0, 2] = cx
    if cy is not None:
        camera_matrix[1, 2] = cy

    prev_t = time.perf_counter()
    fps = 0.0
    try:
        while True:
            ok, frame = cap.read()
            if not ok:
                print("Frame grab failed — camera disconnected?")
                break

            detections = detector.detect(frame)
            estimate_pose(detections, camera_matrix, dist_coeffs, tag_size)
            draw_detections(frame, detections, camera_matrix, dist_coeffs, axis_length=tag_size)

            now = time.perf_counter()
            dt = now - prev_t
            prev_t = now
            if dt > 0:
                fps = 0.9 * fps + 0.1 * (1.0 / dt)
            cv2.putText(frame, f"FPS: {fps:4.1f}  tags: {len(detections)}", (10, 24),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2, cv2.LINE_AA)

            cv2.imshow(window, frame)
            key = cv2.waitKey(1) & 0xFF
            if key in (ord("q"), 27):  # 'q' or Esc
                break
    finally:
        cap.release()
        cv2.destroyAllWindows()


def main():
    parser = argparse.ArgumentParser(description="Live AprilTag detection over a webcam.")
    parser.add_argument("--camera", type=int, default=0, help="camera index (default: 0)")
    parser.add_argument("--family", choices=sorted(TAG_FAMILIES), default=DEFAULT_FAMILY,
                         help=f"AprilTag family (default: {DEFAULT_FAMILY})")
    parser.add_argument("--width", type=int, default=None, help="requested capture width")
    parser.add_argument("--height", type=int, default=None, help="requested capture height")
    parser.add_argument("--tag-size", type=float, default=DEFAULT_TAG_SIZE_M,
                         help=f"printed tag's black-square side length in metres (default: {DEFAULT_TAG_SIZE_M})")
    parser.add_argument("--fx", type=float, default=None, help="camera focal length x, px (default: guessed from frame width)")
    parser.add_argument("--fy", type=float, default=None, help="camera focal length y, px (default: guessed from frame width)")
    parser.add_argument("--cx", type=float, default=None, help="principal point x, px (default: frame centre)")
    parser.add_argument("--cy", type=float, default=None, help="principal point y, px (default: frame centre)")
    args = parser.parse_args()
    run(camera_index=args.camera, family=args.family, width=args.width, height=args.height,
        tag_size=args.tag_size, fx=args.fx, fy=args.fy, cx=args.cx, cy=args.cy)


if __name__ == "__main__":
    main()
