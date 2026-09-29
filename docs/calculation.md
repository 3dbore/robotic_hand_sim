# ELIOS 5-DOF Robotic Arm — Kinematics Reference

## 1. Coordinate System Convention

The coordinate system is a right-handed Cartesian frame where:
- $X$ represents the lateral axis (pointing right).
- $Y$ represents the depth axis (pointing forward).
- $Z$ represents the vertical axis (pointing up).

At the home/zero position, all joint angles $\theta_1, \theta_2, \theta_3, \theta_4, \theta_5$ are $0^\circ$.
All vectors below are expressed in the world base frame $\mathcal{F}_0$ at this home position.

---

## 2. Joint Table

Let $\theta_i$ be the joint angle, $\mathbf{p}_{Ji}$ be the joint pivot position vector in the base frame $\mathcal{F}_0$, and $\hat{\mathbf{u}}_i$ be the unit vector along the axis of rotation for joint $i$.

| Joint | Symbol | Pivot Position $\mathbf{p}_{Ji}$ (mm) | Rotation Axis $\hat{\mathbf{u}}_i$ | Connection |
| :---: | :---:  | :---:                                | :---:                              | :---:      |
| J1    | $\theta_1$ | $\begin{bmatrix} 0.0 & 0.0 & 0.0 \end{bmatrix}^T$ | $\hat{\mathbf{z}} = \begin{bmatrix} 0 & 0 & 1 \end{bmatrix}^T$ | arm1 base |
| J2    | $\theta_2$ | $\begin{bmatrix} 0.0 & 5.0 & 35.5 \end{bmatrix}^T$ | $\hat{\mathbf{x}} = \begin{bmatrix} 1 & 0 & 0 \end{bmatrix}^T$ | arm1 $\leftrightarrow$ arm2 |
| J3    | $\theta_3$ | $\begin{bmatrix} 0.0 & -42.5 & 230.0 \end{bmatrix}^T$ | $\hat{\mathbf{x}} = \begin{bmatrix} 1 & 0 & 0 \end{bmatrix}^T$ | arm2 $\leftrightarrow$ arm3 |
| J4    | $\theta_4$ | $\begin{bmatrix} 0.0 & -42.5 & 90.0 \end{bmatrix}^T$ | $\hat{\mathbf{x}} = \begin{bmatrix} 1 & 0 & 0 \end{bmatrix}^T$ | arm3 $\leftrightarrow$ arm4 |
| J5    | $\theta_5$ | $\begin{bmatrix} 0.0 & -67.0 & 90.0 \end{bmatrix}^T$ | $\hat{\mathbf{y}} = \begin{bmatrix} 0 & 1 & 0 \end{bmatrix}^T$ | arm4 $\leftrightarrow$ arm5 / EE |

---

## 3. Link Offset Vectors (Home Position)

Let $\mathbf{d}_i$ be the offset/displacement vector from joint $i$ to joint $i+1$, expressed in the local coordinate frame of joint $i$ when the arm is in its home position (which aligns with the base frame $\mathcal{F}_0$).

$$
\begin{aligned}
\mathbf{d}_1 &= \mathbf{p}_{J2} - \mathbf{p}_{J1} = \begin{bmatrix} 0.0 \\ 5.0 \\ 35.5 \end{bmatrix} - \begin{bmatrix} 0.0 \\ 0.0 \\ 0.0 \end{bmatrix} = \begin{bmatrix} 0.0 \\ 5.0 \\ 35.5 \end{bmatrix} \text{ mm}, \quad \|\mathbf{d}_1\| \approx 35.85 \text{ mm} \\
\mathbf{d}_2 &= \mathbf{p}_{J3} - \mathbf{p}_{J2} = \begin{bmatrix} 0.0 \\ -42.5 \\ 230.0 \end{bmatrix} - \begin{bmatrix} 0.0 \\ 5.0 \\ 35.5 \end{bmatrix} = \begin{bmatrix} 0.0 \\ -47.5 \\ 194.5 \end{bmatrix} \text{ mm}, \quad \|\mathbf{d}_2\| \approx 200.22 \text{ mm} \\
\mathbf{d}_3 &= \mathbf{p}_{J4} - \mathbf{p}_{J3} = \begin{bmatrix} 0.0 \\ -42.5 \\ 90.0 \end{bmatrix} - \begin{bmatrix} 0.0 \\ -42.5 \\ 230.0 \end{bmatrix} = \begin{bmatrix} 0.0 \\ 0.0 \\ -140.0 \end{bmatrix} \text{ mm}, \quad \|\mathbf{d}_3\| \approx 140.00 \text{ mm} \\
\mathbf{d}_4 &= \mathbf{p}_{J5} - \mathbf{p}_{J4} = \begin{bmatrix} 0.0 \\ -67.0 \\ 90.0 \end{bmatrix} - \begin{bmatrix} 0.0 \\ -42.5 \\ 90.0 \end{bmatrix} = \begin{bmatrix} 0.0 \\ -24.5 \\ 0.0 \end{bmatrix} \text{ mm}, \quad \|\mathbf{d}_4\| \approx 24.50 \text{ mm}
\end{aligned}
$$

The scalar link lengths are denoted by $L_i = \|\mathbf{d}_i\|$:
- $L_1 = 35.85 \text{ mm}$
- $L_2 = 200.22 \text{ mm}$
- $L_3 = 140.00 \text{ mm}$
- $L_4 = 24.50 \text{ mm}$

---

## 4. Elementary Rotation Matrices

For joint rotations, we define rotation matrices around the $Z$ and $X$ axes:

$$
\mathbf{R}_z(\theta) = \begin{bmatrix}
\cos\theta & -\sin\theta & 0 \\
\sin\theta & \cos\theta & 0 \\
0 & 0 & 1
\end{bmatrix}
$$

$$
\mathbf{R}_x(\theta) = \begin{bmatrix}
1 & 0 & 0 \\
0 & \cos\theta & -\sin\theta \\
0 & \sin\theta & \cos\theta
\end{bmatrix}
$$

$$
\mathbf{R}_y(\theta) = \begin{bmatrix}
\cos\theta & 0 & \sin\theta \\
0 & 1 & 0 \\
-\sin\theta & 0 & \cos\theta
\end{bmatrix}
$$

---

## 5. Forward Kinematics (FK)

### 5.1 Homogeneous Transformation Matrices

The relative transform from frame $i-1$ to frame $i$ is defined by a rotation followed by a translation along the link offset vector:

$$
\begin{aligned}
\mathbf{T}_1(\theta_1) &= \begin{bmatrix} \mathbf{R}_z(\theta_1) & \mathbf{0}_{3\times 1} \\ \mathbf{0}_{1\times 3} & 1 \end{bmatrix} \\
\mathbf{T}_2(\theta_2) &= \begin{bmatrix} \mathbf{R}_x(\theta_2) & \mathbf{d}_1 \\ \mathbf{0}_{1\times 3} & 1 \end{bmatrix} \\
\mathbf{T}_3(\theta_3) &= \begin{bmatrix} \mathbf{R}_x(\theta_3) & \mathbf{d}_2 \\ \mathbf{0}_{1\times 3} & 1 \end{bmatrix} \\
\mathbf{T}_4(\theta_4) &= \begin{bmatrix} \mathbf{R}_x(\theta_4) & \mathbf{d}_3 \\ \mathbf{0}_{1\times 3} & 1 \end{bmatrix} \\
\mathbf{T}_5(\theta_5) &= \begin{bmatrix} \mathbf{R}_y(\theta_5) & \mathbf{d}_4 \\ \mathbf{0}_{1\times 3} & 1 \end{bmatrix}
\end{aligned}
$$

The cumulative transformation matrix from the base frame to the end-effector (EE) frame is given by:

$$
\mathbf{T}_{0,EE} = \mathbf{T}_1(\theta_1) \mathbf{T}_2(\theta_2) \mathbf{T}_3(\theta_3) \mathbf{T}_4(\theta_4) \mathbf{T}_5(\theta_5)
$$

### 5.2 Joint Positions in Base Frame

The spatial position of each joint $\mathbf{p}_{Ji}$ in the base frame $\mathcal{F}_0$ is computed recursively as:

$$
\begin{aligned}
\mathbf{p}_{J1} &= \begin{bmatrix} 0 \\ 0 \\ 0 \end{bmatrix} \\
\mathbf{p}_{J2} &= \mathbf{R}_z(\theta_1) \mathbf{d}_1 \\
\mathbf{p}_{J3} &= \mathbf{p}_{J2} + \mathbf{R}_z(\theta_1)\mathbf{R}_x(\theta_2) \mathbf{d}_2 \\
\mathbf{p}_{J4} &= \mathbf{p}_{J3} + \mathbf{R}_z(\theta_1)\mathbf{R}_x(\theta_2)\mathbf{R}_x(\theta_3) \mathbf{d}_3 \\
\mathbf{p}_{J5} &= \mathbf{p}_{J4} + \mathbf{R}_z(\theta_1)\mathbf{R}_x(\theta_2)\mathbf{R}_x(\theta_3)\mathbf{R}_x(\theta_4) \mathbf{d}_4
\end{aligned}
$$

---

## 6. Inverse Kinematics (IK) Strategy

### 6.1 Geometric Decoupling

The target end-effector position is given as $\mathbf{p}_{EE} = \begin{bmatrix} p_x & p_y & p_z \end{bmatrix}^T$.

#### Step 1: Solve Azimuth Angle ($\theta_1$)
The base joint $J_1$ rotates around the $Z$-axis, aligning the arm plane with the target point:

$$
\theta_1 = \operatorname{atan2}(p_y, p_x)
$$

#### Step 2: Projection to the 2D Planar Sub-problem
Rotate $\mathbf{p}_{EE}$ back by $-\theta_1$ into the local $YZ$-plane of the arm:

$$
\mathbf{p}_{\text{local}} = \mathbf{R}_z(-\theta_1) \mathbf{p}_{EE} = \begin{bmatrix} 0 \\ r \\ z \end{bmatrix}
$$

where:
- $r = \sqrt{p_x^2 + p_y^2}$
- $z = p_z$

#### Step 3: Solve Planar Chain ($\theta_2, \theta_3, \theta_4$) and Roll ($\theta_5$)
Because $J_5$ rotates about the local longitudinal $Y$-axis, it acts as a roll (or twist) joint for the end-effector, while $J_2, J_3, J_4$ form a 3-DOF planar positioning/pitch manipulator in the $(r, z)$ plane.

The position of $J_5$ (which is the tool attachment point) is $\mathbf{p}_{J5} = \begin{bmatrix} r_{J5} & z_{J5} \end{bmatrix}^T$ in the local YZ working plane. Since the offset $L_4$ from $J_4$ to $J_5$ is along the local $Y$-axis, the position of $J_5$ is controlled entirely by $\theta_2, \theta_3, \theta_4$:

$$
\begin{aligned}
r_{J5} &= r \\
z_{J5} &= z
\end{aligned}
$$

We define the desired pitch angle of the end-effector link ($L_4$) relative to the local horizon as $\phi_{EE} = \theta_2 + \theta_3 + \theta_4$. 
The position of joint $J_4$ is computed by:

$$
\begin{aligned}
r_{J4} &= r_{J5} - L_4 \cos(\phi_{EE}) \\
z_{J4} &= z_{J5} - L_4 \sin(\phi_{EE})
\end{aligned}
$$

Now we solve the standard 2-link positioning sub-problem for $\theta_2, \theta_3$ ($L_2$ and $L_3$ connecting $J_2$ to $J_4$). 

Since the base of this planar chain is $J_2$ located at $(r_{J2}, z_{J2}) = (5.0, 35.5) \text{ mm}$, we translate the target $J_4$ coordinates relative to $J_2$:

$$
\begin{aligned}
r'_{J4} &= r_{J4} - 5.0 \\
z'_{J4} &= z_{J4} - 35.5
\end{aligned}
$$

Using the Law of Cosines for the J3 angle $\theta_3$:

$$
\cos(\theta_3) = \frac{(r'_{J4})^2 + (z'_{J4})^2 - L_2^2 - L_3^2}{2 L_2 L_3}
$$

This yields two configurations (elbow-up and elbow-down):

$$
\theta_3 = \pm \arccos\left(\cos(\theta_3)\right)
$$

Then we solve for the $J_2$ angle $\theta_2$:

$$
\theta_2 = \operatorname{atan2}(z'_{J4}, r'_{J4}) - \operatorname{atan2}\left(L_3 \sin(\theta_3), L_2 + L_3 \cos(\theta_3)\right)
$$

The pitch constraint is satisfied by:

$$
\theta_4 = \phi_{EE} - \theta_2 - \theta_3
$$

Finally, the end-effector roll angle $\theta_5$ is set independently to align the orientation of the gripper/hand around its own longitudinal axis.

---

## 7. Jacobian Matrix (Velocity Kinematics)

The Jacobian matrix $\mathbf{J}(\boldsymbol{\theta})$ relates joint velocities $\dot{\boldsymbol{\theta}} = \begin{bmatrix} \dot{\theta}_1 & \dot{\theta}_2 & \dot{\theta}_3 & \dot{\theta}_4 & \dot{\theta}_5 \end{bmatrix}^T$ to end-effector linear velocity $\dot{\mathbf{p}}_{EE}$:

$$
\dot{\mathbf{p}}_{EE} = \mathbf{J}_p(\boldsymbol{\theta}) \dot{\boldsymbol{\theta}}
$$

where the $3 \times 5$ position Jacobian is composed of columns corresponding to each joint's rotation axis $\hat{\mathbf{z}}_{i-1}$ and position $\mathbf{p}_{Ji}$:

$$
\mathbf{J}_p(\boldsymbol{\theta}) = \begin{bmatrix}
\hat{\mathbf{z}}_0 \times (\mathbf{p}_{EE} - \mathbf{p}_{J1}) &
\hat{\mathbf{z}}_1 \times (\mathbf{p}_{EE} - \mathbf{p}_{J2}) &
\hat{\mathbf{z}}_2 \times (\mathbf{p}_{EE} - \mathbf{p}_{J3}) &
\hat{\mathbf{z}}_3 \times (\mathbf{p}_{EE} - \mathbf{p}_{J4}) &
\hat{\mathbf{z}}_4 \times (\mathbf{p}_{EE} - \mathbf{p}_{J5})
\end{bmatrix}
$$

The axis vectors $\hat{\mathbf{z}}_{i-1}$ in the base frame are computed via:

$$
\begin{aligned}
\hat{\mathbf{z}}_0 &= \begin{bmatrix} 0 \\ 0 \\ 1 \end{bmatrix} \\
\hat{\mathbf{z}}_1 &= \mathbf{R}_z(\theta_1) \begin{bmatrix} 1 \\ 0 \\ 0 \end{bmatrix} \\
\hat{\mathbf{z}}_2 &= \mathbf{R}_z(\theta_1)\mathbf{R}_x(\theta_2) \begin{bmatrix} 1 \\ 0 \\ 0 \end{bmatrix} \\
\hat{\mathbf{z}}_3 &= \mathbf{R}_z(\theta_1)\mathbf{R}_x(\theta_2)\mathbf{R}_x(\theta_3) \begin{bmatrix} 1 \\ 0 \\ 0 \end{bmatrix} \\
\hat{\mathbf{z}}_4 &= \mathbf{R}_z(\theta_1)\mathbf{R}_x(\theta_2)\mathbf{R}_x(\theta_3)\mathbf{R}_x(\theta_4) \begin{bmatrix} 0 \\ 1 \\ 0 \end{bmatrix}
\end{aligned}
$$

---

## 8. Summary Reference Card

$$
\begin{array}{c|c|c|c}
\mathbf{\text{Joint}} & \mathbf{\text{Pivot Position }} \mathbf{p}_{Ji} \text{ (mm)} & \mathbf{\text{Axis}} & \mathbf{\text{Link Offset Vector }} \mathbf{d} \text{ (mm)} \\
\hline
J_1 & [0.0, 0.0, 0.0]^T & Z & \mathbf{d}_1 = [0.0, 5.0, 35.5]^T \\
J_2 & [0.0, 5.0, 35.5]^T & X & \mathbf{d}_2 = [0.0, -47.5, 194.5]^T \\
J_3 & [0.0, -42.5, 230.0]^T & X & \mathbf{d}_3 = [0.0, 0.0, -140.0]^T \\
J_4 & [0.0, -42.5, 90.0]^T & X & \mathbf{d}_4 = [0.0, -24.5, 0.0]^T \\
J_5 & [0.0, -67.0, 90.0]^T & Y & - \\
\end{array}
$$

- **Link Lengths**: $L_1 = 35.85\text{ mm}$, $L_2 = 200.22\text{ mm}$, $L_3 = 140.00\text{ mm}$, $L_4 = 24.50\text{ mm}$.
- **Total Reach**: $R_{\text{max}} = L_1 + L_2 + L_3 + L_4 \approx 400.6\text{ mm}$.
- **Azimuth Range**: $\theta_1 \in [-\pi, \pi]$ (rotation about Z).
- **Pitch Range**: $\theta_2, \theta_3, \theta_4$ (rotation about X).
- **Roll Range**: $\theta_5$ (rotation about local longitudinal Y).
