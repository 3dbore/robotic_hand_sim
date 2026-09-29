"""
Camera calibration (Spong, Hutchinson & Vidyasagar, Appendix E — docs/camera_calibration.md).

Camera-agnostic: nothing here knows about the simulator. The same code consumes
correspondences from the simulated Camera View or from a real camera.

Stored pixel convention: (row, col) — row increases downward, col rightward,
origin at the centre of the top-left pixel.

Book symbols: the solver uses the book's (r, c) with r = col, c = row. With the
book's signs (r - o_r = -f_x x^c/z^c, c - o_c = -f_y y^c/z^c) this is the
assignment that makes the camera frame right-handed (x^c left, y^c up, z^c
forward), which step 6 (r3 = r1 × r2) requires. f_x is therefore the horizontal
focal length (px) and (o_r, o_c) = (principal col, principal row).
"""
import csv
import datetime
import json
import numpy as np
from scipy import ndimage


class CorrespondenceSet:
    """N correspondences {(x, y, z) in the calibration world frame  <->  (row, col) pixels}.

    extra: optional per-point columns (name -> (N,) or (N,k) array), kept for
    diagnostics and later stages (e.g. joint angles, base-frame position).
    meta:  free-form string metadata (source, frame definition, marker, ...).
    """
    def __init__(self, world, pixels, image_size, meta=None, extra=None):
        self.world = np.asarray(world, dtype=float).reshape(-1, 3)
        self.pixels = np.asarray(pixels, dtype=float).reshape(-1, 2)
        self.image_size = (int(image_size[0]), int(image_size[1]))  # (W, H)
        self.meta = dict(meta or {})
        self.extra = {k: np.asarray(v, dtype=float) for k, v in (extra or {}).items()}

    def __len__(self):
        return len(self.world)

    def coplanarity(self):
        """Std-devs (mm) of the point cloud along its principal axes, largest first.
        The last value is the out-of-plane thickness; ~0 means a degenerate set."""
        return np.sqrt(np.linalg.eigvalsh(np.cov(self.world.T))[::-1])

    def save_csv(self, path):
        cols = ["x", "y", "z", "row", "col"]
        blocks = [self.world, self.pixels]
        for name, arr in self.extra.items():
            arr = arr.reshape(len(self), -1)
            cols += [name] if arr.shape[1] == 1 else [f"{name}_{i}" for i in range(arr.shape[1])]
            blocks.append(arr)
        data = np.hstack(blocks)
        with open(path, "w", newline="", encoding="utf-8") as f:
            f.write(f"# image_size={self.image_size[0]}x{self.image_size[1]}\n")
            f.write(f"# saved={datetime.datetime.now().isoformat(timespec='seconds')}\n")
            for k, v in self.meta.items():
                f.write(f"# {k}={v}\n")
            w = csv.writer(f)
            w.writerow(cols)
            w.writerows([[f"{v:.6f}" for v in row] for row in data])

    @classmethod
    def load_csv(cls, path):
        meta, size = {}, None
        with open(path, encoding="utf-8") as f:
            lines = f.read().splitlines()
        body = []
        for line in lines:
            if line.startswith("#"):
                k, _, v = line[1:].strip().partition("=")
                if k == "image_size":
                    size = tuple(int(s) for s in v.split("x"))
                elif k != "saved":
                    meta[k] = v
            elif line.strip():
                body.append(line)
        rows = list(csv.reader(body))
        cols, data = rows[0], np.array(rows[1:], dtype=float)
        idx = {c: i for i, c in enumerate(cols)}
        world = data[:, [idx["x"], idx["y"], idx["z"]]]
        pixels = data[:, [idx["row"], idx["col"]]]
        extra = {}
        for c in cols[5:]:
            base, _, i = c.rpartition("_")
            name = base if i.isdigit() else c
            extra.setdefault(name, []).append(data[:, idx[c]])
        extra = {k: np.stack(v, axis=1) if len(v) > 1 else v[0] for k, v in extra.items()}
        return cls(world, pixels, size, meta, extra)


def detect_color_blob(image, rgb, tol=60.0, min_area=12, min_fill=0.98, max_elong=1.7):
    """Locate one marker of a known colour in an RGB image (H, W, 3, uint8).

    Returns ((row, col) sub-pixel centroid, area_px) of the largest matching blob, or
    None when it fails the shape tests. A full disc/ellipse has area equal to
    4π·sqrt(det Σ) of its pixel covariance Σ; a blob cut by an occluder or the
    image border does not, so such blobs are rejected rather than returning a
    biased centroid — as a real detector would.
    """
    img = np.asarray(image, dtype=float)[..., :3]
    mask = np.linalg.norm(img - np.asarray(rgb, dtype=float), axis=2) < tol
    labels, n = ndimage.label(mask)
    if n == 0:
        return None
    areas = ndimage.sum(mask, labels, index=np.arange(1, n + 1))
    k = int(np.argmax(areas)) + 1
    area = float(areas[k - 1])
    if area < min_area:
        return None
    rows, cols = np.nonzero(labels == k)
    if rows.min() == 0 or cols.min() == 0 or rows.max() == mask.shape[0] - 1 or cols.max() == mask.shape[1] - 1:
        return None
    ev = np.linalg.eigvalsh(np.cov(np.vstack([rows, cols])) + np.eye(2) / 12.0)
    fill = area / (4.0 * np.pi * np.sqrt(ev[0] * ev[1]))
    if fill < min_fill or ev[1] / ev[0] > max_elong:
        return None
    return (rows.mean(), cols.mean()), area


class CameraModel:
    """Calibrated pinhole camera: x^c = R x^w + T, then the book's projection.
    World frame = whatever frame the correspondences were expressed in."""

    def __init__(self, R, T, fx, fy, o_r, o_c, image_size, meta=None):
        self.R = np.asarray(R, dtype=float).reshape(3, 3)
        self.T = np.asarray(T, dtype=float).reshape(3)
        self.fx, self.fy, self.o_r, self.o_c = float(fx), float(fy), float(o_r), float(o_c)
        self.image_size = (int(image_size[0]), int(image_size[1]))
        self.meta = dict(meta or {})

    @property
    def center(self):
        """Optical centre in world coordinates."""
        return -self.R.T @ self.T

    def project(self, pts):
        """World points (N,3) -> ((N,2) (row, col) pixels, (N,) z^c depth)."""
        xc = np.atleast_2d(pts) @ self.R.T + self.T
        r = -self.fx * xc[:, 0] / xc[:, 2] + self.o_r
        c = -self.fy * xc[:, 1] / xc[:, 2] + self.o_c
        return np.c_[c, r], xc[:, 2]

    def pixel_ray(self, row, col):
        """Viewing ray through a pixel: (origin, unit direction) in world coordinates."""
        d_cam = np.array([-(col - self.o_r) / self.fx, -(row - self.o_c) / self.fy, 1.0])
        d = self.R.T @ d_cam
        return self.center, d / np.linalg.norm(d)

    def to_dict(self):
        return {"R": self.R.tolist(), "T": self.T.tolist(), "fx": self.fx, "fy": self.fy,
                "o_r": self.o_r, "o_c": self.o_c, "image_size": list(self.image_size),
                "meta": self.meta}

    def save_json(self, path):
        with open(path, "w", encoding="utf-8") as f:
            json.dump(self.to_dict(), f, indent=2)

    @classmethod
    def load_json(cls, path):
        with open(path, encoding="utf-8") as f:
            d = json.load(f)
        return cls(d["R"], d["T"], d["fx"], d["fy"], d["o_r"], d["o_c"], d["image_size"], d.get("meta"))


def calibrate(cs, principal_point=None):
    """Appendix E closed-form calibration (docs/camera_calibration.md, steps 1–7).

    principal_point: (row, col). Step 1's vanishing-point method needs a cube in
    view, which the simulated scene lacks, so by default the principal point is
    taken as the image centre. Returns (CameraModel, diagnostics dict).
    """
    if len(cs) < 8:
        raise ValueError("need at least 8 correspondences")
    W, H = cs.image_size
    o_row, o_col = principal_point if principal_point is not None else ((H - 1) / 2.0, (W - 1) / 2.0)
    o_r, o_c = o_col, o_row
    x, y, z = cs.world.T
    # Step 1: shift to the principal point (book r = col, c = row)
    r = cs.pixels[:, 1] - o_r
    c = cs.pixels[:, 0] - o_c

    # Step 2: A x = 0,  x = [r21, r22, r23, T_y, α r11, α r12, α r13, α T_x]
    A = np.c_[r * x, r * y, r * z, r, -c * x, -c * y, -c * z, -c]

    # Step 3: x̄ = right singular vector of the smallest singular value
    _, s, Vt = np.linalg.svd(A, full_matrices=False)
    xb = Vt[-1]

    # Step 4: |k| and α from unit-norm rows of R
    k_abs = np.linalg.norm(xb[0:3])
    alpha = np.linalg.norm(xb[4:7]) / k_abs

    # Step 5: sign of k such that r · x^c < 0 (majority vote over all points)
    xc_scaled = xb[4:7] @ cs.world.T + xb[7]            # = k α x^c
    k = k_abs if np.sum(r * xc_scaled) < 0 else -k_abs
    r2, T_y = xb[0:3] / k, xb[3] / k
    r1, T_x = xb[4:7] / (k * alpha), xb[7] / (k * alpha)

    # Step 6: r3 = r1 × r2
    r3 = np.cross(r1, r2)
    R = np.vstack([r1, r2, r3])

    # Step 7: least squares for [T_z, f_x]:  r (r3·p + T_z) + f_x (r1·p + T_x) = 0
    xc1 = cs.world @ r1 + T_x
    M = np.c_[r, xc1]
    b = -r * (cs.world @ r3)
    (T_z, fx), *_ = np.linalg.lstsq(M, b, rcond=None)
    fy = fx / alpha

    model = CameraModel(R, [T_x, T_y, T_z], fx, fy, o_r, o_c, (W, H),
                        meta={"n_points": len(cs), "source": cs.meta.get("source", ""),
                              "principal_point": "image centre (assumed)" if principal_point is None
                              else "given",
                              "frame": cs.meta.get("frame", "")})
    pred, depth = model.project(cs.world)
    err = np.linalg.norm(pred - cs.pixels, axis=1)
    diag = {"reproj_rms": float(np.sqrt(np.mean(err ** 2))), "reproj_max": float(err.max()),
            "sv_ratio": float(s[-1] / s[-2]), "r1_dot_r2": float(r1 @ r2),
            "alpha": float(alpha), "behind_camera": int(np.sum(depth <= 0))}
    return model, diag
