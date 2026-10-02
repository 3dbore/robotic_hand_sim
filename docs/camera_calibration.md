# Camera Calibration Method

Source: *Robot Modeling and Control*, 2nd ed., Spong, Hutchinson & Vidyasagar (Wiley, 2020), Appendix E, pp. 556–559.

Goal: determine all camera parameters (intrinsic and extrinsic) that relate a 3D point's **world coordinates** `(x, y, z)` to its projected **pixel coordinates** `(r, c)` in an image, so that after calibration `(r, c)` can be predicted for any known `(x, y, z)`.

## 1. Parameters to determine

**Intrinsic parameters** (fixed for a given camera, independent of camera pose):
- `f_x = λ / s_x`, `f_y = λ / s_y` — focal length divided by pixel width/height (only the *ratios* λ/s_x, λ/s_y matter, not λ, s_x, s_y individually)
- `(o_r, o_c)` — pixel coordinates of the principal point

**Extrinsic parameters** (camera pose relative to the world frame):
- `R` (3×3 rotation matrix, rows `r1, r2, r3` i.e. `r_ij`) and `T = [T_x, T_y, T_z]^T`, defined by `x^c = R x^w + T` (world point mapped into camera frame)

## 2. Base projection equations

Image-plane to pixel-array relationship:

```
r - o_r = -f_x * x^c / z^c
c - o_c = -f_y * y^c / z^c
```

Substituting `x^c = r11 x + r12 y + r13 z + T_x`, `y^c = r21 x + r22 y + r23 z + T_y`, `z^c = r31 x + r32 y + r33 z + T_z`:

```
r - o_r = -f_x * (r11 x + r12 y + r13 z + T_x) / (r31 x + r32 y + r33 z + T_z)     (E.3)
c - o_c = -f_y * (r21 x + r22 y + r23 z + T_y) / (r31 x + r32 y + r33 z + T_z)     (E.4)
```

## 3. Data collection

Acquire N correspondences `{r_i, c_i, x_i, y_i, z_i}` for `i = 1...N`: place a bright light (or marker) at known world coordinates `(x, y, z)` and hand-select (or auto-detect) the corresponding pixel location `(r_i, c_i)` in the image. More points and a well-spread 3D configuration improve conditioning.

## 4. Step 1 — Determine the principal point `(o_r, o_c)`

Use the vanishing-point method:
1. Position a cube (or any object with 3 mutually orthogonal edge directions) in the workspace and find its edges in the image.
2. Each of the 3 sets of parallel 3D edges produces a vanishing point in the image (intersection of the corresponding image lines) — this gives a triangle with 3 vertices.
3. The **orthocenter** of that triangle (intersection of the triangle's 3 altitudes) is the principal point `(o_r, o_c)`.

Shift the data to origin at the principal point before the next step:

```
r ← r - o_r
c ← c - o_c
```

## 5. Step 2 — Linear system for the remaining parameters

Cross-multiplying (E.3) and (E.4) and setting `α = f_x / f_y`, each correspondence gives one linear equation in the unknowns:

```
r_i x_i + r_i y_i + r_i z_i + r_i * T_z - α*c_i*r11*x_i - α*c_i*r12*y_i - α*c_i*r13*z_i - α*c_i*T_x = 0
```

(as printed: `r21 x_i + r22 y_i + r23 z_i + T_y - α c_i r11 x_i - α c_i r12 y_i - α c_i r13 z_i - α c_i T_x = 0`, i.e. eq. E.4's numerator times `r`, minus eq. E.3's numerator times `α c`, equals 0 — this eliminates the common denominator `z^c`.)

Stack all N equations into matrix form:

```
A x = 0                                                                    (E.5)

        [ r1 x1  r1 y1  r1 z1  r1  -c1 x1  -c1 y1  -c1 z1  -c1 ]
        [ r2 x2  r2 y2  r2 z2  r2  -c2 x2  -c2 y2  -c2 z2  -c2 ]
A   =   [   :       :      :    :     :       :       :      : ]
        [ rN xN  rN yN  rN zN  rN  -cN xN  -cN yN  -cN zN  -cN ]

x = [ r21, r22, r23, T_y, α*r11, α*r12, α*r13, α*T_x ]^T
```

(A is N×8; need N ≥ 8, in practice use many more points and solve by least squares / SVD.)

## 6. Step 3 — Solve for x up to scale

`x` is a homogeneous unknown — any nonzero solution of `A x = 0` is a scalar multiple `x̄ = k * x` of the true solution. Solve for `x̄` as the right singular vector of `A` associated with the smallest singular value (i.e. the 1-D null space of `A`, or the least-squares solution under a unit-norm constraint).

## 7. Step 4 — Resolve the scale factor `k` and `α`

Because `R` is a rotation matrix, its rows have unit norm. Using `x̄ = [x̄1...x̄8]^T = k*[r21, r22, r23, T_y, α r11, α r12, α r13, α T_x]^T`:

```
|k|      = sqrt(x̄1² + x̄2² + x̄3²)              since [r21,r22,r23] is unit norm
α * |k|  = sqrt(x̄5² + x̄6² + x̄7²)              since [r11,r12,r13] is unit norm
⇒ α = sqrt(x̄5² + x̄6² + x̄7²) / |k|
```

`α > 0` by definition (`α = f_x/f_y`), so this gives the magnitude of `α`, but the **sign of k** is still undetermined (±).

## 8. Step 5 — Determine the sign of `k`

Pick any data point and recall the coordinate convention `r ← r - o_r`. From (E.2)/(E.3), `z^c > 0` (point in front of camera) and `f_x > 0` imply:

```
r * x^c < 0   i.e.   r * (r11 x + r12 y + r13 z + T_x) < 0
```

Choose the sign of `k` (i.e. `k` or `-k`) such that this inequality holds for the data:

```
r * ( (k*α*r11) x + (k*α*r12) y + (k*α*r13) z + (k*α*T_x) ) < 0
```

Once the sign of k is fixed, recover `r21, r22, r23, T_y, r11, r12, r13, T_x` (divide the `α*r1j`/`α*T_x` entries by α) — dividing the whole vector by the resolved `k` gives the true `r2 = [r21,r22,r23]`, `T_y`, `r1 = [r11,r12,r13]`, `T_x`.

## 9. Step 6 — Recover the third row of R

Use the rotation-matrix (orthonormality) property — the third row is the cross product of the first two:

```
r3 = r1 × r2
```

`R = [r1; r2; r3]` is now fully known, along with `T_x`, `T_y`.

## 10. Step 7 — Solve for the remaining unknowns `T_z`, `f_x`, `f_y`

At this point `k, α, r21, r22, r23, r11, r12, r13, T_x, T_y` are all known. Only `T_z`, `f_x` (equivalently `f_y = f_x/α`) remain. Substitute back into the original projection equation (E.3):

```
r = -f_x * (r11 x + r12 y + r13 z + T_x) / (r31 x + r32 y + r33 z + T_z)
```

which rearranges (per data point) to a linear equation in the 2 unknowns `[T_z, f_x]`:

```
r * (r31 x_i + r32 y_i + r33 z_i) + r_i * T_z + f_x * (r11 x_i + r12 y_i + r13 z_i + T_x) = 0
```

Stack over all points and solve the resulting 2-unknown linear least-squares system for `T_z` and `f_x`. Then:

```
f_y = f_x / α
```

## 11. Output

Full parameter set: intrinsics `f_x, f_y, o_r, o_c`; extrinsics `R (3x3), T = [T_x, T_y, T_z]`.

Forward projection of any new world point `(x, y, z)`:

```
x^c = R [x y z]^T + T
r   = -f_x * x^c_1 / x^c_3 + o_r
c   = -f_y * x^c_2 / x^c_3 + o_c
```

## 12. Implementation notes (verified numerically)

- Steps 2–3 (`Ax=0`) are naturally solved with SVD: `x̄ = Vᵀ`'s last row (smallest singular value) of `A`, or `np.linalg.svd(A)[2][-1]`.
- Step 7's 2-unknown system is solved with ordinary least squares (`np.linalg.lstsq`), stacking one row per data point.
- This pipeline (steps 2–7) was implemented and tested against synthetic ground-truth `R, T, f_x, f_y` with a known principal point:
  - Noise-free data: recovers `R`, `T`, `f_x`, `f_y` to numerical precision (errors ~1e-13).
  - With pixel noise (σ = 0.5 px): recovers parameters close to ground truth with sub-pixel reprojection RMS error, as expected for a linear (non-iterative) estimator.
- Caveats to flag to the calibration routine's users:
  - This linear method does not model lens distortion; for a real robotics camera, consider following this closed-form solve with a nonlinear refinement (e.g. Levenberg-Marquardt on reprojection error, optionally with radial/tangential distortion terms) if accuracy requirements are tight.
  - Points should not be coplanar (a planar point set makes `A` rank-deficient / the calibration ill-posed) — use points spread through a 3D volume.
  - Need enough points/well-conditioned geometry: 8 equations are the theoretical minimum for the 8 unknowns in `A x = 0`, but many more (and well-spread, non-degenerate points) are needed for a robust/noise-tolerant solution.
## 13. Startup procedure in this repo — the AprilTag on the hand

**Camera Calibration → "AprilTag on the hand — calibration G-code"** runs the procedure below in the HIL toolkit. It is the same procedure the real arm runs.

**Target** (`hil_toolkit/tags_toolkit/tag_model.py`, `CALIB_TAG = TAGS[0]`): the tag that is already on the hand. Nothing extra is mounted.

| | |
|---|---|
| Family / id | `tag36h11`, id 0 (`paper/apriltags/tag36h11_id00_base_endeffector.png`) |
| Mount | arm5 bottom face, so it moves with the last two axes (J4 pitch, J5 roll). Faces the floor at the initial pose; centre (0, −97, 80.5) mm, base frame |
| Printed image side | 30.0 mm |
| Black square (the size that matters) | **18.52 mm** (the PNG's black square is 410/664 of its side) |

Measure the printed black square with calipers. If it differs from 18.52 mm, change `print_size` of tag 0 in `TAGS` to match.

**Motion** (`hil_toolkit/gcode.py`, `CALIBRATION_GCODE`):

- 16 stations at four depths: wrist Y −100/−120/−140/−160, putting the tag 100–169 mm from the lens.
- Heights Z90–150.
- Wrist pitch A +20/+40° turns the downward-facing tag toward the camera. The camera sees it 51–75° off its face normal.
- Roll B 0/30/60° tilts the tag sideways, which moves the corners out of the arm's plane.
- J1 stays at 0°, because the camera rides on arm1.
- At each station the program waits `G4 P400` to settle, then sends `M240` (camera capture).

The program was checked in the runner:

- G1 moves only (no G0/G28 rapids), at most F600, which is 10 mm/s.
- Roll always moves together with a translation, so no joint exceeds 20°/s. The servo limit is 60°/s.
- Every joint stays at least 14° inside its effective limit. J3 starts at its 0° stop, which is the initial pose itself.
- The hand stays at least 29 mm above the floor.
- The run takes about 64 s.

**Correspondences:** each capture gives the 4 black-square corners. Their 3D positions come from forward kinematics plus the tag's mounting on arm5, in the arm1 frame. Their 2D positions are the detector's sub-pixel corners. 16 captures give 64 points.

**Simulated result** (exact camera known):

- Corner detection error: 0.24 px mean.
- Reprojection RMS: 0.20 px.
- f_x is 1.9 px (0.25 %) below the true value.
- Optical centre is within 0.19 mm, and R within 0.03°.

**Saved to `calibration/`:**

- `apriltag_groundtruth_<stamp>.json`, plus `apriltag_groundtruth.json` as a copy of the latest run. This is the ground truth for the real run. It contains:
  - the tag spec;
  - the true and solved camera models;
  - the G-code text and its SHA-1;
  - for every capture: G-code line, joint angles, servo commands, tag pose (base and camera frames), corner positions, ideal and detected pixels, and the PnP distance with its error.
- `correspondences_apriltag_<stamp>.csv`: the solver input.
- `camera_model.json`: the solved model, which also drives the distance guides.

**On the real arm:**

1. Run the same G-code.
2. Grab a frame at each `M240` and detect tag id 0.
3. Compare the frames with the ground-truth file capture by capture, matched on `gcode_line`.
4. Feed the measured corners into `calibration.calibrate()`.

The tag is seen at a steep angle, so check on the real camera that it is detected at every station. If a station is missed, raise A there by up to 10°, as long as J4 stays below its 90° stop.
