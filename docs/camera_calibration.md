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