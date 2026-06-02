# UAV Terrain Segmentation & Safe Landing Zone Detector

This repository contains real-time point cloud reconstruction and plane-fitting algorithms designed to find flat, obstacle-free landing regions for Unmanned Aerial Vehicles (UAVs) using a single depth camera. It includes both a **high-performance, vectorized NumPy Python module** and a **speed-optimized OpenCV C++ implementation**.

---

## Math & Algorithmic Principles

```
  ┌──────────────────────┐
  │  2D Depth Map Input  │ (Grayscale intensity representation)
  └──────────┬───────────┘
             │
             ▼
  ┌──────────────────────┐
  │ 3D Point Cloud Grid  │ (Reconstructed coordinate vectors X_c, Y_c, Z_c)
  └──────────┬───────────┘
             │
             ▼
  ┌──────────────────────┐
  │   Surface Normals    │ (Cross-product calculations on tangent matrices)
  └──────────┬───────────┘
             │
             ▼
  ┌──────────────────────┐
  │  RANSAC Plane check  │ (Normal deviations & window threshold tests)
  └──────────┬───────────┘
             │
             ▼
  ┌──────────────────────┐
  │ Reconstructed Patch  │ (Optimal 3D landing zone candidate)
  └──────────────────────┘
```

### 1. Depth-to-3D Projection (Back-Projection Model)
Each grayscale pixel $(u, v)$ on a depth map has a normalized intensity representing its range $r$. We first compute the angular parameters (longitude $a$ and latitude $b$) across the camera's field of view (FOV):

$$a = \text{FOVs}_x \cdot \left( \frac{u - c_x}{W - 1} - 0.5 \right)$$
$$b = \text{FOVs}_y \cdot \left( \frac{v - c_y}{H - 1} - 0.5 \right)$$

where $W$ and $H$ are image dimensions, and $c_x, c_y$ represent camera optical offsets. 
Using spherical coordinates, the directional vector $\hat{\mathbf{p}}$ is defined as:

$$\hat{\mathbf{p}} = \begin{bmatrix} p_x \\ p_y \\ p_z \end{bmatrix} = \begin{bmatrix} \cos(a)\cos(b) \\ \sin(a)\cos(b) \\ \sin(b) \end{bmatrix}$$

We correct the distance measurement $r$ based on the perpendicular depth setting (cosine correction):
$$r_{\text{true}} = \frac{r}{p_x}$$

Finally, the 3D position $\mathbf{P}_c$ in the camera frame is reconstructed:
$$\mathbf{P}_c = \begin{bmatrix} X_c \\ Y_c \\ Z_c \end{bmatrix} = \begin{bmatrix} r_{\text{true}} \cdot p_y \\ r_{\text{true}} \cdot p_z \\ r_{\text{true}} \cdot p_x \end{bmatrix}$$

---

### 2. Vectorized Surface Normal Estimations
To identify the local slope and orientation of the terrain, surface normal vectors $\mathbf{n}$ are computed. For any point $\mathbf{P}(y, x)$ in the grid, we define two tangent vectors using neighboring points:

$$\mathbf{u} = \mathbf{P}(y, x) - \mathbf{P}(y, x-1)$$
$$\mathbf{v} = \mathbf{P}(y, x) - \mathbf{P}(y-1, x)$$

The surface normal vector $\mathbf{n}$ is the normalized cross product of these tangents:
$$\mathbf{n}(y, x) = \frac{\mathbf{v} \times \mathbf{u}}{\|\mathbf{v} \times \mathbf{u}\|}$$

---

### 3. Flat Rectangular Search & Plane Fitting
The landing zone selector searches for a rectangular patch of dimensions $w_x \times w_y$ that fits a plane with minimal deviation.
For a candidate region defined by four corners $\mathbf{p}_0, \mathbf{p}_1, \mathbf{p}_2, \mathbf{p}_3$, we determine the patch's normal vector:

$$\mathbf{n}_{\text{plane}} = \frac{(\mathbf{p}_3 - \mathbf{p}_0) \times (\mathbf{p}_1 - \mathbf{p}_0)}{\|(\mathbf{p}_3 - \mathbf{p}_0) \times (\mathbf{p}_1 - \mathbf{p}_0)\|}$$

For all points $\mathbf{p}_{i,j}$ located inside the rectangular window, we compute the perpendicular distance (residual) to this plane:
$$d = \left| (\mathbf{p}_{i,j} - \mathbf{p}_0) \cdot \mathbf{n}_{\text{plane}} \right|$$

A candidate patch is classified as a **safe landing zone** if and only if:
$$\max_{i,j} (d) \le d_{\max}$$
where $d_{\max}$ is the maximum allowable surface height deviation (e.g., $5\text{ mm}$ to prevent landing gear strikes).

---

## File Structure

```
uav-terrain-segmentation/
├── terrain_segmentation.py   # Vectorized, NumPy-optimized point-cloud builder
├── terrain_segmentation.cpp  # Real-time C++ point-cloud and normal estimator
├── math.cpp                  # C++ vector math utilities
├── LICENSE                   # MIT License
└── .gitignore                # System & build files ignore patterns
```

---

## Getting Started

### Python Execution
Install dependencies and run the vectorized segmenter:

```bash
pip install numpy opencv-python cameratransform

# Run the landing zone detector (generates detected_landing_zone.jpg)
python terrain_segmentation.py path/to/depth_image.jpg
```

### C++ Compilation
Ensure OpenCV is installed on your system. Compile using standard C++ compilers:

```bash
g++ -O3 -std=c++17 terrain_segmentation.cpp -o terrain_segmentation `pkg-config --cflags --libs opencv4`

# Run the C++ executable
./terrain_segmentation
```
