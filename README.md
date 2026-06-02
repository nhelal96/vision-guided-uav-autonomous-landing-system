# Vision-Guided UAV Autonomous Landing System

This repository implements a modular, vision-guided autonomous flight guidance and landing stack for Unmanned Aerial Vehicles (UAVs). It integrates pre-flight takeoff supervision, real-time depth map point cloud reconstruction, visual marker AprilTag tracking, and flight control state machines to enable precise landing on moving or stationary docking pads.

---

## System Flight Timeline

The flight supervisor coordinates transitions between four distinct structural components in a sequential mission timeline:

```
  [ Takeoff Signal Received ]
             │
             ▼
┌──────────────────────────┐
│   uav-safe-takeoff       │ (IMU validation & controlled vertical ascent)
└────────────┬─────────────┘
             │
             ▼
┌──────────────────────────┐
│ uav-terrain-segmentation │ (Depth-to-3D projection & plane estimations)
└────────────┬─────────────┘
             │
             ▼
┌──────────────────────────┐
│   uav-qr-docking         │ (Visual target PnP pose homography tracking)
└────────────┬─────────────┘
             │
             ▼
┌──────────────────────────┐
│   uav-safe-landing       │ (OpenCV density clearance & land command)
└──────────────────────────┘
```

1.  **`uav-safe-takeoff/`**: Pre-flight checks (IMU, GPS, altitude limits) and velocity-controlled takeoff sequences.
2.  **`uav-terrain-segmentation/`**: Vectorized point-cloud reconstruction and normal vector estimation. It filters noise and identifies flat, obstacle-free zones.
3.  **`uav-qr-docking/`**: Processes visual tag coordinates to solve the homography transformation, feeding lateral error metrics to flight trackers.
4.  **`uav-safe-landing/`**: Performs final terrain obstacle clearance checks and commands the landing phase.

---

## Subsystem Formulations & Control Mathematics

### 1. Controlled Ascent & Takeoff Supervision
The supervisor controls takeoff using a vertical velocity target loop. Given target altitude $z_{\text{target}}$ and current estimated altitude $z(t)$, vertical speed command $v_z(t)$ is modeled as:

$$e_z(t) = z_{\text{target}} - z(t)$$

$$v_z(t) = \text{clip}\left( K_p e_z(t) + K_d \frac{d e_z(t)}{dt}, \, v_{\min}, \, v_{\max} \right)$$

To confirm safe hover stability, it verifies that altitude and velocity variances drop below thresholds:
$$\sigma_z^2 = \frac{1}{T} \int_{t-T}^{t} (z(\tau) - \mu_z)^2 d\tau < \epsilon_z \quad \text{and} \quad \|\mathbf{v}\| < \epsilon_v$$

---

### 2. Depth-to-3D Back-Projection & Normal Extraction
The terrain analyzer maps 2D depth camera values $d(u,v)$ to 3D Cartesian coordinates in the camera coordinate frame:

$$\mathbf{P}_c = \begin{bmatrix} X_c \\ Y_c \\ Z_c \end{bmatrix} = \begin{bmatrix} r_{\text{true}} \cdot \sin(a)\cos(b) \\ r_{\text{true}} \cdot \sin(b) \\ r_{\text{true}} \cdot \cos(a)\cos(b) \end{bmatrix}$$

Surface normals are computed as:
$$\mathbf{n}(y, x) = \frac{\mathbf{v} \times \mathbf{u}}{\|\mathbf{v} \times \mathbf{u}\|} \quad \text{where } \mathbf{u} = \mathbf{P}(y, x) - \mathbf{P}(y, x-1) \text{ and } \mathbf{v} = \mathbf{P}(y, x) - \mathbf{P}(y-1, x)$$

A region is classified as flat if all internal points satisfy:
$$d = \left| (\mathbf{p}_{i,j} - \mathbf{p}_0) \cdot \mathbf{n}_{\text{plane}} \right| \le d_{\max}$$

---

### 3. Precision Docking (Perspective-n-Point)
The visual tracker solves the PnP problem to calculate the relative translation vector $\mathbf{t}_m^c = [x_r, y_r, z_r]^T$:

$$s \begin{bmatrix} u \\ v \\ 1 \end{bmatrix} = \mathbf{K} \left( \mathbf{R}_m^c \begin{bmatrix} X_w \\ Y_w \\ Z_w \end{bmatrix} + \mathbf{t}_m^c \right)$$

where:
*   $\mathbf{K}$ is the camera intrinsic matrix.
*   $\mathbf{R}_m^c$ is the rotation matrix mapping marker coordinates to camera coordinates.

Steering errors are projected into the vehicle's body coordinates and resolved using velocity tracking commands:
$$\mathbf{p}_{\text{body}} = \mathbf{R}_c^b \mathbf{t}_m^c + \mathbf{t}_c^b$$

$$\mathbf{v}_{\text{cmd}} = -K_p \mathbf{e} - K_d \frac{d\mathbf{e}}{dt} \quad \text{where } \mathbf{e} = \begin{bmatrix} x_{\text{body}} \\ y_{\text{body}} \end{bmatrix}$$

---

### 4. Safe Landing Clearance Check
Before touchdown, the landing supervisor performs an edge-texture density variance check to verify that no ground obstacles are present:

$$\sigma_g^2 = \frac{1}{N}\sum_{i=1}^{N} (\|\nabla I_i\| - \mu_g)^2$$

A surface patch is classified as **safe** if:
$$\sigma_g^2 < \tau_{\text{clear}}$$

---

## File Structure

```
vision-guided-uav-autonomous-landing-system/
├── README.md               # Main System Documentation
├── LICENSE                 # MIT License
├── .gitignore              # Global git ignore configurations
│
├── uav-safe-takeoff/        # Pre-Flight Launch Safety Supervisor
│   ├── include/            # C++ Headers
│   ├── src/
│   │   ├── algorithm.cpp   # Obstacle density assessment algorithms
│   │   ├── communicator.cpp# MAVROS state listeners and position publishers
│   │   ├── helper.cpp      # Flight control utilities
│   │   ├── image_processor.cpp# Visual validation functions
│   │   └── safetakeoff_node.cpp# Node runner state machine loop
│   ├── CMakeLists.txt      # Catkin CMake configurations
│   └── package.xml         # ROS package manifest
│
├── uav-terrain-segmentation/# Depth-to-3D Terrain Normal Segmenter
│   ├── terrain_segmentation.py# Vectorized, NumPy-optimized point-cloud builder
│   ├── terrain_segmentation.cpp# Real-time C++ point-cloud and normal estimator
│   └── math.cpp            # C++ vector math utilities
│
├── uav-qr-docking/          # Visual AprilTag Pose PnP Docking Subsystem
│   ├── include/            # C++ Headers
│   ├── src/
│   │   ├── algorithm.cpp   # QR detection and PnP solver wrappers
│   │   ├── communicator.cpp# MAVROS pose state publishers
│   │   ├── helper.cpp      # Matrix and transformation functions
│   │   ├── image_processor.cpp# Raw frame image processing
│   │   ├── image_reciever.cpp# Subscriber callbacks for camera feeds
│   │   └── qrlanding_node.cpp# ROS node runner and trajectory loop
│   ├── CMakeLists.txt      # Catkin CMake configurations
│   └── package.xml         # ROS package manifest
│
└── uav-safe-landing/        # ROS Navigation & Safe Landing Node
    ├── include/            # C++ Headers
    ├── src/
    │   ├── algorithm.cpp   # Computer vision landing zone checks
    │   ├── communicator.cpp# ROS publishers and subscribers (mavros/telemetry)
    │   ├── helper.cpp      # Vector math and coordinate translations
    │   ├── image_processor.cpp# OpenCV image conversions
    │   ├── image_reciever.cpp# Image subscriber handler
    │   └── safelanding_node.cpp# Node runner state machine loop
    ├── CMakeLists.txt      # Catkin CMake configurations
    └── package.xml         # ROS package manifest
```

---

## Getting Started

### Installation
Clone this repository into your Catkin workspace source folder:

```bash
cd ~/catkin_ws/src
git clone <repository-url> vision-guided-uav-landing-system
```

### Build
Build all Catkin packages:

```bash
cd ~/catkin_ws
catkin_make
```

### Running the System
Start the flight nodes in sequence:

```bash
# Launch takeoff supervisor
rosrun uav-safe-takeoff safetakeoff_node

# Launch safe landing supervisor
rosrun uav-safe-landing SafeLanding_node
```
