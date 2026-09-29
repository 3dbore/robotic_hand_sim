# Lengan Robot ELIOS 5-DOF — Referensi Kinematika

## 1. Konvensi Sistem Koordinat

Sistem koordinat yang digunakan adalah kerangka Kartesius tangan kanan di mana:
- $X$ mewakili sumbu lateral (mengarah ke kanan).
- $Y$ mewakili sumbu kedalaman (mengarah ke depan).
- $Z$ mewakili sumbu vertikal (mengarah ke atas).

Pada posisi awal/nol (home position), semua sudut sendi $\theta_1, \theta_2, \theta_3, \theta_4, \theta_5$ adalah $0^\circ$.
Semua vektor di bawah ini dinyatakan dalam kerangka acuan dasar dunia (world base frame) $\mathcal{F}_0$ pada posisi awal ini.

---

## 2. Tabel Sendi (Joint Table)

Misalkan $\theta_i$ adalah sudut sendi, $\mathbf{p}_{Ji}$ adalah vektor posisi pivot sendi dalam kerangka acuan dasar $\mathcal{F}_0$, dan $\hat{\mathbf{u}}_i$ adalah vektor satuan di sepanjang sumbu rotasi untuk sendi $i$.

| Sendi | Simbol | Posisi Pivot $\mathbf{p}_{Ji}$ (mm) | Sumbu Rotasi $\hat{\mathbf{u}}_i$ | Koneksi |
| :---: | :---:  | :---:                                | :---:                              | :---:      |
| J1    | $\theta_1$ | $\begin{bmatrix} 0.0 & 0.0 & 0.0 \end{bmatrix}^T$ | $\hat{\mathbf{z}} = \begin{bmatrix} 0 & 0 & 1 \end{bmatrix}^T$ | basis lengan 1 (arm1 base) |
| J2    | $\theta_2$ | $\begin{bmatrix} 0.0 & 5.0 & 35.5 \end{bmatrix}^T$ | $\hat{\mathbf{x}} = \begin{bmatrix} 1 & 0 & 0 \end{bmatrix}^T$ | lengan 1 $\leftrightarrow$ lengan 2 |
| J3    | $\theta_3$ | $\begin{bmatrix} 0.0 & -42.5 & 230.0 \end{bmatrix}^T$ | $\hat{\mathbf{x}} = \begin{bmatrix} 1 & 0 & 0 \end{bmatrix}^T$ | lengan 2 $\leftrightarrow$ lengan 3 |
| J4    | $\theta_4$ | $\begin{bmatrix} 0.0 & -42.5 & 90.0 \end{bmatrix}^T$ | $\hat{\mathbf{x}} = \begin{bmatrix} 1 & 0 & 0 \end{bmatrix}^T$ | lengan 3 $\leftrightarrow$ lengan 4 |
| J5    | $\theta_5$ | $\begin{bmatrix} 0.0 & -67.0 & 90.0 \end{bmatrix}^T$ | $\hat{\mathbf{y}} = \begin{bmatrix} 0 & 1 & 0 \end{bmatrix}^T$ | lengan 4 $\leftrightarrow$ lengan 5 / EE |

---

## 3. Vektor Offset Penghubung (Link Offset) (Posisi Awal)

Misalkan $\mathbf{d}_i$ adalah vektor offset/perpindahan dari sendi $i$ ke sendi $i+1$, dinyatakan dalam kerangka koordinat lokal sendi $i$ ketika lengan berada pada posisi awal (yang sejajar dengan kerangka acuan dasar $\mathcal{F}_0$).

$$
\begin{aligned}
\mathbf{d}_1 &= \mathbf{p}_{J2} - \mathbf{p}_{J1} = \begin{bmatrix} 0.0 \\ 5.0 \\ 35.5 \end{bmatrix} - \begin{bmatrix} 0.0 \\ 0.0 \\ 0.0 \end{bmatrix} = \begin{bmatrix} 0.0 \\ 5.0 \\ 35.5 \end{bmatrix} \text{ mm}, \quad \|\mathbf{d}_1\| \approx 35.85 \text{ mm} \\
\mathbf{d}_2 &= \mathbf{p}_{J3} - \mathbf{p}_{J2} = \begin{bmatrix} 0.0 \\ -42.5 \\ 230.0 \end{bmatrix} - \begin{bmatrix} 0.0 \\ 5.0 \\ 35.5 \end{bmatrix} = \begin{bmatrix} 0.0 \\ -47.5 \\ 194.5 \end{bmatrix} \text{ mm}, \quad \|\mathbf{d}_2\| \approx 200.22 \text{ mm} \\
\mathbf{d}_3 &= \mathbf{p}_{J4} - \mathbf{p}_{J3} = \begin{bmatrix} 0.0 \\ -42.5 \\ 90.0 \end{bmatrix} - \begin{bmatrix} 0.0 \\ -42.5 \\ 230.0 \end{bmatrix} = \begin{bmatrix} 0.0 \\ 0.0 \\ -140.0 \end{bmatrix} \text{ mm}, \quad \|\mathbf{d}_3\| \approx 140.00 \text{ mm} \\
\mathbf{d}_4 &= \mathbf{p}_{J5} - \mathbf{p}_{J4} = \begin{bmatrix} 0.0 \\ -67.0 \\ 90.0 \end{bmatrix} - \begin{bmatrix} 0.0 \\ -42.5 \\ 90.0 \end{bmatrix} = \begin{bmatrix} 0.0 \\ -24.5 \\ 0.0 \end{bmatrix} \text{ mm}, \quad \|\mathbf{d}_4\| \approx 24.50 \text{ mm}
\end{aligned}
$$

Panjang penghubung (link lengths) skalar dinyatakan dengan $L_i = \|\mathbf{d}_i\|$:
- $L_1 = 35.85 \text{ mm}$
- $L_2 = 200.22 \text{ mm}$
- $L_3 = 140.00 \text{ mm}$
- $L_4 = 24.50 \text{ mm}$

---

## 4. Matriks Rotasi Dasar

Untuk rotasi sendi, kita mendefinisikan matriks rotasi di sekitar sumbu $Z$, $X$, dan $Y$:

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

## 5. Kinematika Maju (Forward Kinematics - FK)

### 5.1 Matriks Transformasi Homogen

Transformasi relatif dari kerangka $i-1$ ke kerangka $i$ didefinisikan oleh rotasi yang diikuti oleh translasi di sepanjang vektor offset penghubung (link offset):

$$
\begin{aligned}
\mathbf{T}_1(\theta_1) &= \begin{bmatrix} \mathbf{R}_z(\theta_1) & \mathbf{0}_{3\times 1} \\ \mathbf{0}_{1\times 3} & 1 \end{bmatrix} \\
\mathbf{T}_2(\theta_2) &= \begin{bmatrix} \mathbf{R}_x(\theta_2) & \mathbf{d}_1 \\ \mathbf{0}_{1\times 3} & 1 \end{bmatrix} \\
\mathbf{T}_3(\theta_3) &= \begin{bmatrix} \mathbf{R}_x(\theta_3) & \mathbf{d}_2 \\ \mathbf{0}_{1\times 3} & 1 \end{bmatrix} \\
\mathbf{T}_4(\theta_4) &= \begin{bmatrix} \mathbf{R}_x(\theta_4) & \mathbf{d}_3 \\ \mathbf{0}_{1\times 3} & 1 \end{bmatrix} \\
\mathbf{T}_5(\theta_5) &= \begin{bmatrix} \mathbf{R}_y(\theta_5) & \mathbf{d}_4 \\ \mathbf{0}_{1\times 3} & 1 \end{bmatrix}
\end{aligned}
$$

Matriks transformasi kumulatif dari kerangka acuan dasar ke kerangka efektor akhir (EE) diberikan oleh:

$$
\mathbf{T}_{0,EE} = \mathbf{T}_1(\theta_1) \mathbf{T}_2(\theta_2) \mathbf{T}_3(\theta_3) \mathbf{T}_4(\theta_4) \mathbf{T}_5(\theta_5)
$$

### 5.2 Posisi Sendi dalam Kerangka Acuan Dasar

Posisi spasial dari masing-masing sendi $\mathbf{p}_{Ji}$ dalam kerangka acuan dasar $\mathcal{F}_0$ dihitung secara rekursif sebagai:

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

## 6. Strategi Kinematika Balik (Inverse Kinematics - IK)

### 6.1 Dekopling Geometris

Posisi target efektor akhir diberikan sebagai $\mathbf{p}_{EE} = \begin{bmatrix} p_x & p_y & p_z \end{bmatrix}^T$.

#### Langkah 1: Selesaikan Sudut Azimut ($\theta_1$)
Sendi dasar $J_1$ berotasi di sekitar sumbu-$Z$, menyejajarkan bidang lengan dengan titik target:

$$
\theta_1 = \operatorname{atan2}(p_y, p_x)
$$

#### Langkah 2: Proyeksi ke Sub-masalah Bidang 2D (2D Planar Sub-problem)
Rotasikan $\mathbf{p}_{EE}$ kembali sebesar $-\theta_1$ ke dalam bidang-$YZ$ lokal dari lengan:

$$
\mathbf{p}_{\text{local}} = \mathbf{R}_z(-\theta_1) \mathbf{p}_{EE} = \begin{bmatrix} 0 \\ r \\ z \end{bmatrix}
$$

di mana:
- $r = \sqrt{p_x^2 + p_y^2}$
- $z = p_z$

#### Langkah 3: Selesaikan Rantai Bidang (Planar Chain) ($\theta_2, \theta_3, \theta_4$) dan Roll ($\theta_5$)
Karena $J_5$ berotasi di sekitar sumbu-$Y$ longitudinal lokal, sendi ini bertindak sebagai sendi roll (atau twist) untuk efektor akhir, sedangkan $J_2, J_3, J_4$ membentuk manipulator pemosisian/pitch bidang 3-DOF dalam bidang $(r, z)$.

Posisi $J_5$ (yang merupakan titik pemasangan alat / tool attachment point) adalah $\mathbf{p}_{J5} = \begin{bmatrix} r_{J5} & z_{J5} \end{bmatrix}^T$ pada bidang kerja YZ lokal. Karena offset $L_4$ dari $J_4$ ke $J_5$ berada di sepanjang sumbu-$Y$ lokal, posisi $J_5$ dikendalikan sepenuhnya oleh $\theta_2, \theta_3, \theta_4$:

$$
\begin{aligned}
r_{J5} &= r \\
z_{J5} &= z
\end{aligned}
$$

Kita mendefinisikan sudut pitch yang diinginkan dari penghubung efektor akhir ($L_4$) relatif terhadap cakrawala (horizon) lokal sebagai $\phi_{EE} = \theta_2 + \theta_3 + \theta_4$.
Posisi dari sendi $J_4$ dihitung dengan:

$$
\begin{aligned}
r_{J4} &= r_{J5} - L_4 \cos(\phi_{EE}) \\
z_{J4} &= z_{J5} - L_4 \sin(\phi_{EE})
\end{aligned}
$$

Sekarang kita menyelesaikan sub-masalah pemosisian 2-penghubung (2-link) standar untuk $\theta_2, \theta_3$ ($L_2$ dan $L_3$ yang menghubungkan $J_2$ ke $J_4$).

Karena basis dari rantai bidang ini adalah $J_2$ yang terletak pada $(r_{J2}, z_{J2}) = (5.0, 35.5) \text{ mm}$, kita mentranslasikan koordinat target $J_4$ relatif terhadap $J_2$:

$$
\begin{aligned}
r'_{J4} &= r_{J4} - 5.0 \\
z'_{J4} &= z_{J4} - 35.5
\end{aligned}
$$

Menggunakan Hukum Kosinus untuk sudut J3 ($\theta_3$):

$$
\cos(\theta_3) = \frac{(r'_{J4})^2 + (z'_{J4})^2 - L_2^2 - L_3^2}{2 L_2 L_3}
$$

Ini menghasilkan dua konfigurasi (siku-ke-atas / elbow-up dan siku-ke-bawah / elbow-down):

$$
\theta_3 = \pm \arccos\left(\cos(\theta_3)\right)
$$

Kemudian kita mencari sudut $J_2$ ($\theta_2$):

$$
\theta_2 = \operatorname{atan2}(z'_{J4}, r'_{J4}) - \operatorname{atan2}\left(L_3 \sin(\theta_3), L_2 + L_3 \cos(\theta_3)\right)
$$

Batasan pitch dipenuhi oleh:

$$
\theta_4 = \phi_{EE} - \theta_2 - \theta_3
$$

Terakhir, sudut roll efektor akhir $\theta_5$ diatur secara independen untuk menyejajarkan orientasi pencekam/tangan (gripper/hand) di sekitar sumbu longitudinalnya sendiri.

---

## 7. Matriks Jacobian (Kinematika Kecepatan - Velocity Kinematics)

Matriks Jacobian $\mathbf{J}(\boldsymbol{\theta})$ menghubungkan kecepatan sendi $\dot{\boldsymbol{\theta}} = \begin{bmatrix} \dot{\theta}_1 & \dot{\theta}_2 & \dot{\theta}_3 & \dot{\theta}_4 & \dot{\theta}_5 \end{bmatrix}^T$ ke kecepatan linear efektor akhir $\dot{\mathbf{p}}_{EE}$:

$$
\dot{\mathbf{p}}_{EE} = \mathbf{J}_p(\boldsymbol{\theta}) \dot{\boldsymbol{\theta}}
$$

di mana Jacobian posisi $3 \times 5$ terdiri dari kolom-kolom yang sesuai dengan sumbu rotasi $\hat{\mathbf{z}}_{i-1}$ dan posisi $\mathbf{p}_{Ji}$ dari masing-masing sendi:

$$
\mathbf{J}_p(\boldsymbol{\theta}) = \begin{bmatrix}
\hat{\mathbf{z}}_0 \times (\mathbf{p}_{EE} - \mathbf{p}_{J1}) &
\hat{\mathbf{z}}_1 \times (\mathbf{p}_{EE} - \mathbf{p}_{J2}) &
\hat{\mathbf{z}}_2 \times (\mathbf{p}_{EE} - \mathbf{p}_{J3}) &
\hat{\mathbf{z}}_3 \times (\mathbf{p}_{EE} - \mathbf{p}_{J4}) &
\hat{\mathbf{z}}_4 \times (\mathbf{p}_{EE} - \mathbf{p}_{J5})
\end{bmatrix}
$$

Vektor sumbu $\hat{\mathbf{z}}_{i-1}$ dalam kerangka acuan dasar dihitung melalui:

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

## 8. Kartu Referensi Ringkasan (Summary Reference Card)

$$
\begin{array}{c|c|c|c}
\mathbf{\text{Sendi}} & \mathbf{\text{Posisi Pivot }} \mathbf{p}_{Ji} \text{ (mm)} & \mathbf{\text{Sumbu}} & \mathbf{\text{Vektor Offset Penghubung }} \mathbf{d} \text{ (mm)} \\
\hline
J_1 & [0.0, 0.0, 0.0]^T & Z & \mathbf{d}_1 = [0.0, 5.0, 35.5]^T \\
J_2 & [0.0, 5.0, 35.5]^T & X & \mathbf{d}_2 = [0.0, -47.5, 194.5]^T \\
J_3 & [0.0, -42.5, 230.0]^T & X & \mathbf{d}_3 = [0.0, 0.0, -140.0]^T \\
J_4 & [0.0, -42.5, 90.0]^T & X & \mathbf{d}_4 = [0.0, -24.5, 0.0]^T \\
J_5 & [0.0, -67.0, 90.0]^T & Y & - \\
\end{array}
$$

- **Panjang Penghubung (Link Lengths)**: $L_1 = 35.85\text{ mm}$, $L_2 = 200.22\text{ mm}$, $L_3 = 140.00\text{ mm}$, $L_4 = 24.50\text{ mm}$.
- **Jangkauan Total**: $R_{\text{max}} = L_1 + L_2 + L_3 + L_4 \approx 400.6\text{ mm}$.
- **Rentang Azimut**: $\theta_1 \in [-\pi, \pi]$ (rotasi di sekitar Z).
- **Rentang Pitch**: $\theta_2, \theta_3, \theta_4$ (rotasi di sekitar X).
- **Rentang Roll**: $\theta_5$ (rotasi di sekitar Y longitudinal lokal).
