# UAV QR-Code Docking & Landing System (ROS Package)

This ROS package implements an autonomous precision docking and landing system for Unmanned Aerial Vehicles (UAVs) using visual markers (QR Codes / AprilTags). It processes real-time camera streams to estimate the relative 3D pose of the docking station, guiding the drone down to a precise landing.

---

## Technical Architecture & Coordinate Mapping

The docking loop continuously estimates the relative transformation between the camera sensor frame and the visual marker frame:

```
  ┌──────────────────────┐
  │ World ENU Map Frame  │
  └──────────┬───────────┘
             │
       [Odometry TF]
             │
             ▼
  ┌──────────────────────┐
  │   UAV Body Frame     │
  └──────────┬───────────┘
             │
       [Static Camera TF]
             │
             ▼
  ┌──────────────────────┐
  │  Camera Lens Frame   │
  └──────────┬───────────┘
             │
       [Visual Target Frame PnP]
             │
             ▼
  ┌──────────────────────┐
  │ QR Marker Target frame│
  └──────────────────────┘
```

---

## Mathematical Formulations

### 1. Relative Pose Estimation (Perspective-n-Point)
Given the known coordinates of the QR code corners in the marker frame $\mathbf{X}_i = [X_w, Y_w, Z_w]^T$ and their corresponding 2D projected pixel locations $\mathbf{x}_i = [u, v]^T$, the system solves the Perspective-n-Point (PnP) problem:

$$s \begin{bmatrix} u \\ v \\ 1 \end{bmatrix} = \mathbf{K} \left( \mathbf{R}_m^c \begin{bmatrix} X_w \\ Y_w \\ Z_w \end{bmatrix} + \mathbf{t}_m^c \right)$$

where:
*   $\mathbf{K}$ is the camera intrinsic matrix.
*   $\mathbf{R}_m^c$ is the rotation matrix mapping marker coordinates to camera coordinates.
*   $\mathbf{t}_m^c = [x_r, y_r, z_r]^T$ is the translation vector representing the relative 3D position of the marker center.
*   $s$ is an arbitrary scaling parameter.

---

### 2. Relative Error Transformations
The relative translation vector $\mathbf{t}_m^c$ is converted to vehicle steering commands. We transform the relative target coordinates into the UAV body frame:

$$\mathbf{p}_{\text{body}} = \mathbf{R}_c^b \mathbf{t}_m^c + \mathbf{t}_c^b$$

where $\mathbf{R}_c^b$ and $\mathbf{t}_c^b$ represent the extrinsic calibration matrices mapping the camera position relative to the UAV's center of gravity.
The steering error values $\mathbf{e} = [e_x, e_y]^T$ are used to generate flight velocity inputs:

$$\mathbf{e} = \begin{bmatrix} x_{\text{body}} \\ y_{\text{body}} \end{bmatrix}$$

$$\mathbf{v}_{\text{cmd}} = -K_p \mathbf{e} - K_d \frac{d\mathbf{e}}{dt}$$

---

## File Structure

```
uav-qr-docking/
├── include/           # C++ Headers (algorithm, communicator, helper, etc.)
├── src/
│   ├── algorithm.cpp       # QR detection and PnP solver wrappers
│   ├── communicator.cpp    # MAVROS pose state publishers
│   ├── helper.cpp          # Matrix and transformation functions
│   ├── image_processor.cpp # Raw frame image processing
│   ├── image_reciever.cpp  # Subscriber callbacks for camera feeds
│   └── qrlanding_node.cpp  # ROS node runner and trajectory loop
├── CMakeLists.txt     # CMake build configuration
├── package.xml        # ROS package manifest
├── LICENSE            # MIT License
└── .gitignore         # System file ignores
```

---

## Getting Started

### Installation
Clone this package into your Catkin workspace source folder:

```bash
cd ~/catkin_ws/src
git clone <repository-url> uav-qr-docking
```

### Build
Build the package:

```bash
cd ~/catkin_ws
catkin_make --only-pkg-with-deps uav-qr-docking
```

### Running the Node
Start the precision docking node:

```bash
rosrun uav-qr-docking qrlanding_node
```
