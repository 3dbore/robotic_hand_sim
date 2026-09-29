# -*- coding: utf-8 -*-
"""
ELIOS 5-DOF Robotic Arm — End Effector IK Demo
============================================================
Controls the end effector via translational and rotational
buttons (game-like). Inverse kinematics solves for joint angles
using Jacobian pseudoinverse. Moving the end effector causes
all joints to move accordingly.

Coordinate system: mm, Right-handed
  X = lateral, Y = depth/forward, Z = up

Joints:
  J1: Z-rot   | Range: [-180, 0] deg (180° CW and back)
  J2: X-rot   | Range: [0, 50] deg (50° CCW and back)
  J3: X-rot   | Range: [-200, 0] deg (200° CW and back)
  J4: X-rot   | Range: [-25, 230] deg (25° CW, 230° CCW)
  J5: X-rot   | Range: [-180, 0] deg (180° CW and back)

Layout:
  Left panel  — Control buttons (Translation + Rotation)
  Right top   — End effector position (relative to initial)
  Right bottom — Joint rotation angles

Usage
-----
  python arm-sim-end.py
"""

import os
import math
import numpy as np
import pyvista as pv

# ─────────────────────────────────────────────────────────────────────────────
# PATHS & CONFIG
# ─────────────────────────────────────────────────────────────────────────────
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
ARMS_DIR   = os.path.join(SCRIPT_DIR, "arms")

ARM_CONFIG = {
    "arm1": {"file": "arm1.stl", "color": "#4A90D9"},
    "arm2": {"file": "arm2.stl", "color": "#5BBF72"},
    "arm3": {"file": "arm3.stl", "color": "#D97B4A"},
    "arm4": {"file": "arm4.stl", "color": "#A060C8"},
    "arm5": {"file": "arm5.stl", "color": "#D94A4A"},
}

CAM_FILE = os.path.join(ARMS_DIR, "cam.stl")
FOV_FILE = os.path.join(ARMS_DIR, "fov.stl")
CAM_PIVOT = np.array([0.0, -38.0, 35.5])

JOINTS = [
    # name           world position (mm)       rotation axis  colour
    ("J1 (Z-rot)",   np.array([ 0.0,   0.0,   0.0]),  "Z",  "red"    ),
    ("J2 (X-rot)",   np.array([ 0.0,   5.0,  35.5]),  "X",  "lime"   ),
    ("J3 (X-rot)",   np.array([ 0.0, -42.5, 230.0]),  "X",  "cyan"   ),
    ("J4 (X-rot/Pitch)", np.array([ 0.0, -42.5,  90.0]),  "X",  "magenta"),
    ("J5 (Y-rot/Roll)",  np.array([ 0.0, -67.0,  90.0]),  "Y",  "yellow" ),
]

JOINT_SPHERE_RADIUS = 5.0   # mm

# Homogeneous joint pivots (5,4), precomputed once for the vectorized
# per-joint transform in update_scene()'s kinematic-chain redraw.
_JOINT_PIVOTS_HOMO = np.array([[*j[1], 1.0] for j in JOINTS])

# Joint limits [min_deg, max_deg] from the user's spec:
#   J1: 90° CW (negative), 90° CCW (positive) → [-90, 90]
#   J2: 50° CW and 50° CCW from initial       → [-50, 50]
#   J3: 200° CW (negative for X-rot) and back → [-200, 0]
#   J4: 25° CW (negative), 230° CCW (positive)→ [-25, 230]
#   J5: 180° CW (negative) and back           → [-180, 0]
JOINT_LIMITS = [
    ( -90.0,  90.0),   # J1
    ( -50.0,  50.0),   # J2
    (-200.0,   0.0),   # J3
    ( -25.0, 230.0),   # J4
    (-180.0,   0.0),   # J5
]

# Link offset vectors (from calculation.md)
D_OFFSETS = [
    np.array([ 0.0,    5.0,   35.5]),   # d1: J1 → J2
    np.array([ 0.0,  -47.5,  194.5]),   # d2: J2 → J3
    np.array([ 0.0,    0.0, -140.0]),   # d3: J3 → J4
    np.array([ 0.0,  -24.5,    0.0]),   # d4: J4 → J5
]

# IK parameters
IK_STEP_SIZE  = 2.0     # mm per button press (translation)
IK_ROT_STEP   = 2.0     # degrees per button press (rotation)
IK_DAMPING    = 0.5     # Damped least squares lambda
IK_MAX_ITER   = 50      # Max iterations per step

# ─────────────────────────────────────────────────────────────────────────────
# ROTATION HELPERS
# ─────────────────────────────────────────────────────────────────────────────
def Rz(theta_deg):
    """Rotation matrix around Z axis."""
    t = math.radians(theta_deg)
    c, s = math.cos(t), math.sin(t)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])

def Rx(theta_deg):
    """Rotation matrix around X axis."""
    t = math.radians(theta_deg)
    c, s = math.cos(t), math.sin(t)
    return np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]])

def Ry(theta_deg):
    """Rotation matrix around Y axis."""
    t = math.radians(theta_deg)
    c, s = math.cos(t), math.sin(t)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])

# ─────────────────────────────────────────────────────────────────────────────
# FORWARD KINEMATICS
# ─────────────────────────────────────────────────────────────────────────────
def forward_kinematics(angles):
    """
    Compute FK for the 5-DOF arm.
    Returns:
        joint_positions: list of 5 positions [J1, J2, J3, J4, J5]
        cumulative_rotations: list of 5 cumulative rotation matrices (3x3)
        ee_position: end-effector position = J4 position (3,)
    """
    th = angles  # [θ1, θ2, θ3, θ4, θ5]

    # Cumulative rotation matrices
    R1 = Rz(th[0])
    R12 = R1 @ Rx(th[1])
    R123 = R12 @ Rx(th[2])
    R1234 = R123 @ Rx(th[3])
    R12345 = R1234 @ Ry(th[4])

    # Joint positions (from calculation.md Section 5.2)
    p_J1 = np.array([0.0, 0.0, 0.0])
    p_J2 = R1 @ D_OFFSETS[0]
    p_J3 = p_J2 + R12 @ D_OFFSETS[1]
    p_J4 = p_J3 + R123 @ D_OFFSETS[2]
    p_J5 = p_J4 + R1234 @ D_OFFSETS[3]

    # EE position is defined as J4 (the wrist/pitch joint)
    ee_pos = p_J4.copy()

    joint_positions = [p_J1, p_J2, p_J3, p_J4, p_J5]
    cumulative_rotations = [R1, R12, R123, R1234, R12345]

    return joint_positions, cumulative_rotations, ee_pos

_Z_AXIS = np.array([0.0, 0.0, 1.0])


def jacobian_from_fk(joint_positions, cumulative_rotations, ee_pos):
    """
    3x3 position Jacobian for J4 position w.r.t. J1, J2, J3, built from an
    forward_kinematics() result already computed for the same angles — avoids
    recomputing FK or re-deriving R1/R12 (ik_step's hot loop calls this once
    per iteration instead of running FK twice).
    J_col_i = z_i × (p_J4 - p_Ji)
    """
    R1, R12 = cumulative_rotations[0], cumulative_rotations[1]
    z1 = R1[:, 0]   # R1 @ [1,0,0]: J2 axis (X) transformed to base frame
    z2 = R12[:, 0]  # R12 @ [1,0,0]: J3 axis (X) transformed to base frame

    J = np.empty((3, 3))
    J[:, 0] = np.cross(_Z_AXIS, ee_pos - joint_positions[0])  # J1
    J[:, 1] = np.cross(z1, ee_pos - joint_positions[1])       # J2
    J[:, 2] = np.cross(z2, ee_pos - joint_positions[2])       # J3

    return J


def compute_jacobian_j4(angles):
    """Compute the 3x3 position Jacobian for J4 position w.r.t. J1, J2, J3.
    Convenience wrapper that runs FK itself; ik_step's inner loop calls
    jacobian_from_fk() directly to reuse an FK result it already has."""
    joint_pos, cum_rot, ee_pos = forward_kinematics(angles)
    return jacobian_from_fk(joint_pos, cum_rot, ee_pos)

def clamp_angles(angles):
    """Clamp joint angles to their limits."""
    for i in range(5):
        angles[i] = np.clip(angles[i], JOINT_LIMITS[i][0], JOINT_LIMITS[i][1])
    return angles

def ik_step(current_angles, delta_pos):
    """
    Perform IK to move the end-effector (J4 position) by delta_pos.

    Strategy:
      1. Solve for J1, J2, J3 using 3×3 Jacobian (position of J4).
      2. Preserve global pitch: since J2, J3, J4 all rotate about X
         (in the J1-rotated frame), the global pitch = θ2 + θ3 + θ4.
         After IK changes θ2, θ3, adjust θ4 to keep the sum constant.
      3. Keep θ5 unchanged (roll is preserved).

    Uses persistent desired_global_pitch / desired_global_roll so
    orientation never drifts across multiple calls.
    """
    angles = current_angles.copy()

    # Use persistent orientation targets (not per-step snapshot)
    pitch_target = desired_global_pitch
    roll_target  = desired_global_roll

    # Target EE position. Seed the loop's FK cache here too, so the first
    # iteration doesn't re-run FK for the same (still unchanged) angles.
    joint_pos, cum_rot, ee_current = forward_kinematics(angles)
    target_pos = ee_current + delta_pos

    for iteration in range(IK_MAX_ITER):
        residual = target_pos - ee_current

        if np.linalg.norm(residual) < 0.01:
            break

        # J from the FK already computed for the current `angles` — no
        # second forward_kinematics() call needed (was: one for the
        # jacobian, one for the residual, both on identical angles).
        J = jacobian_from_fk(joint_pos, cum_rot, ee_current)

        # Damped least-squares: dθ = J^T (J J^T + λ²I)^{-1} residual
        JJT = J @ J.T
        lam2 = IK_DAMPING ** 2
        dtheta = J.T @ np.linalg.solve(JJT + lam2 * np.eye(3), residual)

        # Convert to degrees and apply (only J1, J2, J3)
        dtheta_deg = np.degrees(dtheta)

        # Scale step to prevent huge jumps
        max_step = 5.0  # max degrees per iteration
        scale = min(1.0, max_step / (np.max(np.abs(dtheta_deg)) + 1e-8))
        dtheta_deg *= scale

        angles[0] += dtheta_deg[0]  # J1
        angles[1] += dtheta_deg[1]  # J2
        angles[2] += dtheta_deg[2]  # J3

        # Preserve global pitch: set J4 so θ2 + θ3 + θ4 = pitch_target
        angles[3] = pitch_target - angles[1] - angles[2]

        # Keep roll unchanged
        angles[4] = roll_target

        # Clamp J1, J2, J3 first (position joints)
        for idx in [0, 1, 2]:
            angles[idx] = np.clip(angles[idx], JOINT_LIMITS[idx][0], JOINT_LIMITS[idx][1])

        # Recompute J4 after J2/J3 may have been clamped
        angles[3] = pitch_target - angles[1] - angles[2]
        angles[3] = np.clip(angles[3], JOINT_LIMITS[3][0], JOINT_LIMITS[3][1])

        # Clamp roll
        angles[4] = np.clip(angles[4], JOINT_LIMITS[4][0], JOINT_LIMITS[4][1])

        # One FK call for the angles just produced — reused as both next
        # iteration's residual input and its jacobian input.
        joint_pos, cum_rot, ee_current = forward_kinematics(angles)

    # Final pitch/roll restoration after all iterations
    angles[3] = pitch_target - angles[1] - angles[2]
    angles[3] = np.clip(angles[3], JOINT_LIMITS[3][0], JOINT_LIMITS[3][1])
    angles[4] = roll_target
    angles[4] = np.clip(angles[4], JOINT_LIMITS[4][0], JOINT_LIMITS[4][1])

    return angles


# ─────────────────────────────────────────────────────────────────────────────
# VTK SCENE HELPERS (same as original)
# ─────────────────────────────────────────────────────────────────────────────
def get_transform_matrix(pivot, axis, angle_degrees):
    """Returns 4x4 homogeneous matrix for rotation around a pivot point.

    Single-axis rotation, so this is exactly Rx/Ry/Rz (already used by
    forward_kinematics) — cheaper than routing through scipy's general
    rotation-vector machinery for what is always a single-axis case.
    """
    if axis == "X":
        R = Rx(angle_degrees)
    elif axis == "Y":
        R = Ry(angle_degrees)
    else:
        R = Rz(angle_degrees)
    M = np.eye(4)
    M[:3, :3] = R
    M[:3, 3] = pivot - R @ pivot
    return M

def apply_transform_to_points(points, matrix):
    """Applies a 4x4 transform to an (N, 3) array of points."""
    ones = np.ones((points.shape[0], 1), dtype=np.float32)
    pts_homo = np.hstack([points, ones])
    return (matrix @ pts_homo.T).T[:, :3].astype(np.float32)


# ─────────────────────────────────────────────────────────────────────────────
# LOAD MESHES
# ─────────────────────────────────────────────────────────────────────────────
print("=" * 60)
print("ELIOS 5-DOF Arm — End Effector IK Demo")
print("=" * 60)

original_points = {}
loaded_meshes = {}
for arm_name, cfg in ARM_CONFIG.items():
    filepath = os.path.join(ARMS_DIR, cfg["file"])
    mesh = pv.read(filepath)
    mesh.points = mesh.points.astype(np.float32)
    original_points[arm_name] = mesh.points.copy()
    loaded_meshes[arm_name] = mesh

# Load Camera
cam_mesh = pv.read(CAM_FILE)
cam_mesh.points = cam_mesh.points.astype(np.float32)
original_cam_points = cam_mesh.points.copy()

# Load FOV
fov_mesh = pv.read(FOV_FILE)
fov_mesh.points = fov_mesh.points.astype(np.float32)
original_fov_points = fov_mesh.points.copy()

# ─────────────────────────────────────────────────────────────────────────────
# STATE
# ─────────────────────────────────────────────────────────────────────────────
current_angles = np.zeros(5)
cam_elevation = 0.0

# Persistent desired orientation state.
# These track the intended global pitch and roll, so they survive
# across multiple IK steps without drift from clamping.
desired_global_pitch = 0.0   # θ2 + θ3 + θ4 sum (degrees)
desired_global_roll  = 0.0   # θ5 (degrees)

# Compute initial EE position (= J4 position)
_, _, initial_ee_pos = forward_kinematics(current_angles)
print(f"Initial EE position (J4): {initial_ee_pos}")

# ─────────────────────────────────────────────────────────────────────────────
# PLOTTER SETUP — 2x2 Grid Layout
# ─────────────────────────────────────────────────────────────────────────────
# Shape: left column = full height (3D view), right column = 2 rows (info panels)
plotter = pv.Plotter(
    shape="1|2",  # 1 left panel, 2 right panels stacked
    window_size=(1600, 900),
    title="ELIOS 5-DOF — End Effector IK Control",
    border=True,
    border_color="#2A3A4A",
    border_width=2,
)

# ═══════════════════════════════════════════════════════════════════════════════
# LEFT PANEL (index 0) — 3D Viewport + Control Buttons
# ═══════════════════════════════════════════════════════════════════════════════
plotter.subplot(0)
plotter.set_background("#0D1B2A")

# Add a grid map / floor under the end effector and in front of it (-Y is front)
# Center the floor slightly forward (e.g. Y = -200)
floor_grid = pv.Plane(center=(0, -200, 0), direction=(0, 0, 1), i_size=800, j_size=800, i_resolution=16, j_resolution=16)
plotter.add_mesh(floor_grid, style="wireframe", color="#3A4A5A", line_width=1.5, name="floor_grid")
plotter.add_axes_at_origin(labels_off=True, line_width=3)

# Add arms
for arm_name in ARM_CONFIG:
    plotter.add_mesh(
        loaded_meshes[arm_name],
        color=ARM_CONFIG[arm_name]["color"],
        show_edges=False,
        opacity=1.0,
        smooth_shading=True,
        specular=0.5,
        specular_power=25,
        name=arm_name,
    )

plotter.add_mesh(
    cam_mesh,
    color="#A0A0A0",
    show_edges=False,
    opacity=1.0,
    smooth_shading=True,
    specular=0.5,
    specular_power=25,
    name="camera_mount",
)

plotter.add_mesh(
    fov_mesh,
    color="#00FFFF",  # Cyan for FOV
    show_edges=False,
    opacity=0.25,
    smooth_shading=True,
    name="camera_fov",
)

# Joint markers
original_markers_points = []
marker_meshes = []
marker_transform_indices = []

for i, (jname, jpos, jaxis, jcolor) in enumerate(JOINTS):
    glow = pv.Sphere(radius=JOINT_SPHERE_RADIUS * 1.8, center=jpos)
    sphere = pv.Sphere(radius=JOINT_SPHERE_RADIUS, center=jpos)

    if jaxis == "X":
        axis_vec = np.array([1.0, 0.0, 0.0])
    elif jaxis == "Y":
        axis_vec = np.array([0.0, 1.0, 0.0])
    else:
        axis_vec = np.array([0.0, 0.0, 1.0])

    arrow_line = pv.Line(jpos - axis_vec * 15.0, jpos + axis_vec * 15.0)

    transform_index = i

    for m in [glow, sphere]:
        original_markers_points.append(m.points.copy())
        marker_meshes.append(m)
        marker_transform_indices.append(transform_index)
        plotter.add_mesh(m, color=jcolor, opacity=0.15 if m is glow else 1.0, name=f"marker_{jname}_{id(m)}")

    original_markers_points.append(arrow_line.points.copy())
    marker_meshes.append(arrow_line)
    marker_transform_indices.append(transform_index)
    plotter.add_mesh(arrow_line, color=jcolor, line_width=3, opacity=0.75, name=f"arrow_{jname}")

# Kinematic chain line
chain_poly = pv.PolyData()
chain_poly.points = np.array([j[1] for j in JOINTS], dtype=np.float32)
chain_poly.lines = np.array([5, 0, 1, 2, 3, 4])
plotter.add_mesh(chain_poly, color="white", line_width=2.5, opacity=0.45, name="kinematic_chain")

# End effector marker at J4 position (larger, distinct)
ee_sphere = pv.Sphere(radius=JOINT_SPHERE_RADIUS * 2.0, center=initial_ee_pos)
original_ee_sphere_pts = ee_sphere.points.copy()
plotter.add_mesh(ee_sphere, color="#FF6B35", opacity=0.8, name="ee_marker")

ee_glow = pv.Sphere(radius=JOINT_SPHERE_RADIUS * 3.5, center=initial_ee_pos)
original_ee_glow_pts = ee_glow.points.copy()
plotter.add_mesh(ee_glow, color="#FF6B35", opacity=0.15, name="ee_glow")

# Axes
plotter.add_axes(line_width=3, labels_off=False, xlabel="X", ylabel="Y", zlabel="Z")

# ─────────────────────────────────────────────────────────────────────────────
# SCENE UPDATE FUNCTION
# ─────────────────────────────────────────────────────────────────────────────
def link_transforms(angles):
    """[M0..M5]: 4x4 base-frame transforms of each link from its home pose."""
    transforms = [np.eye(4)]
    for (_, pivot, axis, _), a in zip(JOINTS, angles):
        transforms.append(transforms[-1] @ get_transform_matrix(pivot, axis, a))
    return transforms

def update_scene(angles):
    """Computes FK and updates all mesh vertices in the scene."""
    plotter.subplot(0)

    transforms = link_transforms(angles)
    _, M1, M2, M3, M4, M5 = transforms

    # Update arm meshes
    loaded_meshes["arm1"].points[:] = apply_transform_to_points(original_points["arm1"], M1)
    loaded_meshes["arm2"].points[:] = apply_transform_to_points(original_points["arm2"], M2)
    loaded_meshes["arm3"].points[:] = apply_transform_to_points(original_points["arm3"], M3)
    loaded_meshes["arm4"].points[:] = apply_transform_to_points(original_points["arm4"], M4)
    loaded_meshes["arm5"].points[:] = apply_transform_to_points(original_points["arm5"], M5)

    # Update camera mesh
    T_elev = get_transform_matrix(CAM_PIVOT, "X", cam_elevation)
    M_cam = M1 @ T_elev
    cam_mesh.points[:] = apply_transform_to_points(original_cam_points, M_cam)
    fov_mesh.points[:] = apply_transform_to_points(original_fov_points, M_cam)

    # Update joint markers
    for pts_orig, mesh, t_idx in zip(original_markers_points, marker_meshes, marker_transform_indices):
        mesh.points[:] = apply_transform_to_points(pts_orig, transforms[t_idx])

    # Update kinematic chain: each joint's pivot transformed by a *different*
    # matrix (transforms[0..4]) — one batched einsum instead of 5 separate
    # apply_transform_to_points() calls (each with its own tiny allocation).
    chain_mats = np.array(transforms[:5])  # (5,4,4)
    chain_pts = np.einsum("nij,nj->ni", chain_mats, _JOINT_PIVOTS_HOMO)[:, :3]
    chain_poly.points[:] = chain_pts.astype(np.float32)

    # Update EE marker
    _, _, ee_pos = forward_kinematics(angles)
    ee_offset = ee_pos - initial_ee_pos
    ee_sphere.points[:] = original_ee_sphere_pts + ee_offset.astype(np.float32)
    ee_glow.points[:] = original_ee_glow_pts + ee_offset.astype(np.float32)

    # ── Update right panels ──
    update_info_panels(angles, ee_pos)


def update_info_panels(angles, ee_pos):
    """Update the text displays in the right panels."""
    delta_pos = ee_pos - initial_ee_pos
    global_pitch = angles[1] + angles[2] + angles[3]

    # ── Right Top: End Effector Position ──
    plotter.subplot(1)
    ee_text = (
        "  END EFFECTOR (J4) POSITION\n"
        "  ─────────────────────────\n"
        f"\n"
        f"  Absolute Position (mm):\n"
        f"    X: {ee_pos[0]:8.2f}\n"
        f"    Y: {ee_pos[1]:8.2f}\n"
        f"    Z: {ee_pos[2]:8.2f}\n"
        f"\n"
        f"  Δ from Initial (mm):\n"
        f"    ΔX: {delta_pos[0]:+8.2f}\n"
        f"    ΔY: {delta_pos[1]:+8.2f}\n"
        f"    ΔZ: {delta_pos[2]:+8.2f}\n"
        f"\n"
        f"  Distance: {np.linalg.norm(delta_pos):8.2f} mm\n"
        f"\n"
        f"  Global Pitch: {global_pitch:+7.1f}°\n"
        f"  Global Roll:  {angles[4]:+7.1f}°"
    )
    plotter.add_text(ee_text, position="upper_left", font_size=10,
                     color="#00FFAA", name="ee_position_display")

    # ── Right Bottom: Joint Angles ──
    plotter.subplot(2)
    # Build bar-like visual for each joint
    joint_text = (
        "  JOINT ROTATIONS\n"
        "  ─────────────────────────\n"
        f"\n"
    )
    joint_names = ["J1 (Z-rot)", "J2 (X-rot)", "J3 (X-rot)", "J4 (Pitch)", "J5 (Roll)"]
    for i in range(5):
        lo, hi = JOINT_LIMITS[i]
        rng = hi - lo
        pct = (angles[i] - lo) / rng * 100 if rng != 0 else 0
        joint_text += f"  {joint_names[i]}:\n"
        joint_text += f"    Angle: {angles[i]:+7.1f}°\n"
        joint_text += f"    Range: [{lo:+.0f}°, {hi:+.0f}°]  ({pct:.0f}%)\n"
        joint_text += f"\n"

    joint_text += "  CAMERA MOUNT\n"
    joint_text += "  ─────────────────────────\n"
    joint_text += f"  Elevation: {cam_elevation:+7.1f}°\n"

    plotter.add_text(joint_text, position="upper_left", font_size=9,
                     color="#FFDD57", name="joint_angle_display")


# ─────────────────────────────────────────────────────────────────────────────
# CONTROL CALLBACKS
# ─────────────────────────────────────────────────────────────────────────────
# Continuous movement state
movement_state = {
    'active': None,          # Which direction is active (or None)
    'timer_id': None,        # VTK timer ID for continuous movement
}

def move_ee(direction, amount):
    """Move end effector (J4) by delta in the given direction using IK.
    Preserves global pitch (θ2+θ3+θ4) and roll (θ5) using persistent state."""
    global current_angles

    delta = np.zeros(3)
    if direction == '+X':
        delta[0] = amount
    elif direction == '-X':
        delta[0] = -amount
    elif direction == '+Y':
        delta[1] = amount
    elif direction == '-Y':
        delta[1] = -amount
    elif direction == '+Z':
        delta[2] = amount
    elif direction == '-Z':
        delta[2] = -amount

    new_angles = ik_step(current_angles, delta)
    current_angles = new_angles
    update_scene(current_angles)
    plotter.subplot(0)
    plotter.render()

def rotate_cam(amount_deg):
    """Rotate the camera mount Elevation."""
    global cam_elevation
    
    cam_elevation += amount_deg
        
    update_scene(current_angles)
    plotter.subplot(0)
    plotter.render()

def rotate_ee(axis, amount_deg):
    """Rotate end effector orientation (Pitch or Roll).
    Updates the persistent desired orientation state."""
    global current_angles, desired_global_pitch, desired_global_roll

    new_angles = current_angles.copy()
    if axis == 'PITCH':
        new_angles[3] += amount_deg
        new_angles = clamp_angles(new_angles)
        desired_global_pitch = new_angles[1] + new_angles[2] + new_angles[3]
    elif axis == 'ROLL':
        new_angles[4] += amount_deg
        new_angles = clamp_angles(new_angles)
        desired_global_roll = new_angles[4]

    current_angles = new_angles
    update_scene(current_angles)
    plotter.subplot(0)
    plotter.render()

def rotate_base(amount_deg):
    """Rotate the arm base (J1) left/right, independent of the IK-driven hand pad."""
    global current_angles

    new_angles = current_angles.copy()
    new_angles[0] += amount_deg
    new_angles = clamp_angles(new_angles)

    current_angles = new_angles
    update_scene(current_angles)
    plotter.subplot(0)
    plotter.render()

def reset_arm():
    """Reset all joints and orientation targets to zero."""
    global current_angles, desired_global_pitch, desired_global_roll, cam_elevation
    current_angles = np.zeros(5)
    cam_elevation = 0.0
    desired_global_pitch = 0.0
    desired_global_roll = 0.0
    update_scene(current_angles)
    plotter.subplot(0)
    plotter.render()


# ─────────────────────────────────────────────────────────────────────────────
# STANDALONE DEMO UI (buttons, keyboard control, plotter.show())
# ─────────────────────────────────────────────────────────────────────────────
# Only built when this file is run directly. The HIL toolkit loads this module
# as a library (module name "arm_sim_end") purely for the kinematics/state
# above — it drives the arm through move_ee/rotate_ee/rotate_base/update_scene
# itself, via its own Qt widgets, so none of this demo-only VTK button/keyboard
# scaffolding needs to be built (or torn down) for that use case.
if __name__ == "__main__":
    # We'll use key events for continuous movement
    key_state = {}

    def on_key_press(obj, event):
        """Handle key press for game-like control."""
        key = plotter.iren.interactor.GetKeySym()
        if key in key_state and key_state[key]:
            return  # Already pressed
        key_state[key] = True
        process_key(key)

    def on_key_release(obj, event):
        """Handle key release."""
        key = plotter.iren.interactor.GetKeySym()
        key_state[key] = False

    def process_key(key):
        """Process a single key press for movement."""
        step = IK_STEP_SIZE
        rot_step = IK_ROT_STEP

        key_lower = key.lower() if key else ""

        if key_lower == 'd':
            move_ee('+X', step)
        elif key_lower == 'a':
            move_ee('-X', step)
        elif key_lower == 'w':
            move_ee('+Y', step)
        elif key_lower == 's':
            move_ee('-Y', step)
        elif key_lower == 'e':
            move_ee('+Z', step)
        elif key_lower == 'q':
            move_ee('-Z', step)
        elif key_lower == 'j':
            rotate_ee('PITCH', rot_step)
        elif key_lower == 'l':
            rotate_ee('PITCH', -rot_step)
        elif key_lower == 'i':
            rotate_ee('ROLL', rot_step)
        elif key_lower == 'k':
            rotate_ee('ROLL', -rot_step)
        elif key_lower == 'y':
            rotate_cam(rot_step)
        elif key_lower == 'h':
            rotate_cam(-rot_step)
        elif key_lower == 'r':
            reset_arm()

    # ── BUTTON WIDGETS (Left Panel) ──
    plotter.subplot(0)

    # Button layout config
    BTN_SIZE = 40
    BTN_GAP = 5
    START_X = 15
    START_Y = 20

    # Create button callbacks with closures
    def make_move_callback(direction, amount):
        def callback(state):
            move_ee(direction, amount)
        return callback

    def make_rotate_callback(axis, amount):
        def callback(state):
            rotate_ee(axis, amount)
        return callback

    def make_rotate_cam_callback(amount):
        def callback(state):
            rotate_cam(amount)
        return callback

    def make_reset_callback():
        def callback(state):
            reset_arm()
        return callback

    # ── TRANSLATION BUTTONS ──
    # Row labels and buttons arranged like a game pad
    # Layout:
    #          [+Z]
    #   [-X]   [+Y]   [+X]
    #          [-Y]
    #          [-Z]

    trans_buttons = [
        # (label, direction, amount, col, row)
        ("+Z",  "+Z",  IK_STEP_SIZE, 1, 4),   # Top
        ("-X",  "-X",  IK_STEP_SIZE, 0, 3),   # Left
        ("+Y",  "+Y",  IK_STEP_SIZE, 1, 3),   # Center
        ("+X",  "+X",  IK_STEP_SIZE, 2, 3),   # Right
        ("-Y",  "-Y",  IK_STEP_SIZE, 1, 2),   # Below center
        ("-Z",  "-Z",  IK_STEP_SIZE, 1, 1),   # Bottom
    ]

    # Add translation section label
    plotter.add_text(
        "TRANSLATION",
        position=(START_X, START_Y + 5 * (BTN_SIZE + BTN_GAP) + 10),
        font_size=9,
        color="#00CCFF",
        name="trans_label",
    )

    for label, direction, amount, col, row in trans_buttons:
        bx = START_X + col * (BTN_SIZE + BTN_GAP)
        by = START_Y + row * (BTN_SIZE + BTN_GAP)

        plotter.add_checkbox_button_widget(
            make_move_callback(direction, amount),
            value=False,
            position=(bx, by),
            size=BTN_SIZE,
            border_size=2,
            color_on="#00AAFF",
            color_off="#1A3050",
        )
        # Button label
        plotter.add_text(
            label,
            position=(bx + 5, by + 10),
            font_size=7,
            color="white",
            name=f"btn_label_{label}",
        )

    # ── ROTATION BUTTONS ──
    ROT_START_Y = START_Y + 6 * (BTN_SIZE + BTN_GAP) + 30

    plotter.add_text(
        "ROTATION",
        position=(START_X, ROT_START_Y + 2 * (BTN_SIZE + BTN_GAP) + 10),
        font_size=9,
        color="#FF8800",
        name="rot_label",
    )

    rot_buttons = [
        # (label, axis, amount, col, row)
        ("P+", "PITCH",  IK_ROT_STEP, 0, 1),
        ("P-", "PITCH", -IK_ROT_STEP, 2, 1),
        ("R+", "ROLL",   IK_ROT_STEP, 0, 0),
        ("R-", "ROLL",  -IK_ROT_STEP, 2, 0),
    ]

    for label, axis, amount, col, row in rot_buttons:
        bx = START_X + col * (BTN_SIZE + BTN_GAP)
        by = ROT_START_Y + row * (BTN_SIZE + BTN_GAP)

        plotter.add_checkbox_button_widget(
            make_rotate_callback(axis, amount),
            value=False,
            position=(bx, by),
            size=BTN_SIZE,
            border_size=2,
            color_on="#FF6600",
            color_off="#3A2010",
        )
        plotter.add_text(
            label,
            position=(bx + 3, by + 10),
            font_size=7,
            color="white",
            name=f"btn_label_{label}",
        )

    # ── CAMERA ROTATION BUTTONS ──
    CAM_ROT_START_Y = ROT_START_Y + 2 * (BTN_SIZE + BTN_GAP) + 20

    plotter.add_text(
        "CAMERA",
        position=(START_X, CAM_ROT_START_Y + 2 * (BTN_SIZE + BTN_GAP) + 10),
        font_size=9,
        color="#FF00FF",
        name="cam_rot_label",
    )

    cam_rot_buttons = [
        ("El+", IK_ROT_STEP, 0, 0),
        ("El-",-IK_ROT_STEP, 2, 0),
    ]

    for label, amount, col, row in cam_rot_buttons:
        bx = START_X + col * (BTN_SIZE + BTN_GAP)
        by = CAM_ROT_START_Y + row * (BTN_SIZE + BTN_GAP)

        plotter.add_checkbox_button_widget(
            make_rotate_cam_callback(amount),
            value=False,
            position=(bx, by),
            size=BTN_SIZE,
            border_size=2,
            color_on="#FF00FF",
            color_off="#3A103A",
        )
        plotter.add_text(
            label,
            position=(bx + 3, by + 10),
            font_size=7,
            color="white",
            name=f"btn_label_cam_{label}",
        )

    # ── RESET BUTTON ──
    RESET_Y = CAM_ROT_START_Y + 2 * (BTN_SIZE + BTN_GAP) + 10
    plotter.add_checkbox_button_widget(
        make_reset_callback(),
        value=False,
        position=(START_X, RESET_Y),
        size=BTN_SIZE,
        border_size=2,
        color_on="#FF3333",
        color_off="#550000",
    )
    plotter.add_text(
        "RST",
        position=(START_X + 5, RESET_Y + 10),
        font_size=8,
        color="white",
        name="btn_label_reset",
    )

    # ── KEYBOARD HELP ──
    KB_Y = RESET_Y + BTN_SIZE + 20
    kb_help = (
        "KEYBOARD CONTROLS\n"
        "─────────────────\n"
        "W/S  : +Y / -Y\n"
        "A/D  : -X / +X\n"
        "Q/E  : -Z / +Z\n"
        "J/L  : Pitch ±\n"
        "I/K  : Roll  ±\n"
        "Y/H  : Cam Elevation ±\n"
        "R    : Reset"
    )
    plotter.add_text(
        kb_help,
        position=(START_X, KB_Y),
        font_size=7,
        color="#888888",
        name="kb_help",
    )

    # ── RIGHT TOP PANEL (index 1) — End Effector Position ──
    plotter.subplot(1)
    plotter.set_background("#0A1628")
    plotter.add_text(
        "  END EFFECTOR POSITION\n"
        "  ─────────────────────────\n"
        "\n"
        "  Waiting for input...",
        position="upper_left",
        font_size=10,
        color="#00FFAA",
        name="ee_position_display",
    )

    # ── RIGHT BOTTOM PANEL (index 2) — Joint Angles ──
    plotter.subplot(2)
    plotter.set_background("#0A1220")
    plotter.add_text(
        "  JOINT ROTATIONS\n"
        "  ─────────────────────────\n"
        "\n"
        "  Waiting for input...",
        position="upper_left",
        font_size=9,
        color="#FFDD57",
        name="joint_angle_display",
    )

    # ── CAMERA & INIT ──
    plotter.subplot(0)
    plotter.camera_position = "iso"
    plotter.reset_camera()
    plotter.camera.zoom(0.8)

    # Add key observers
    plotter.iren.interactor.AddObserver('KeyPressEvent', on_key_press)

    # Initial display update
    update_scene(current_angles)

    print("\n" + "=" * 60)
    print("End Effector IK Control Ready!")
    print("=" * 60)
    print("\nUse the buttons on the left or keyboard to control the arm.")
    print("  Translation: W/A/S/D/Q/E  |  Pitch: J/L  |  Roll: I/K  |  Reset: R")
    print("  Cam Elev: Y/H")
    print("\nRight panels show EE position and joint angles in real-time.")

    plotter.show()
