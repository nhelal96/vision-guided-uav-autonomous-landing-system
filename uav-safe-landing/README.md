# UAV Safe Landing System (ROS Package)

This ROS package implements an autonomous landing safety supervisor for Unmanned Aerial Vehicles (UAVs). It utilizes real-time image processing and computer vision algorithms to evaluate the terrain beneath the drone, identify flat landing zones, avoid obstacles, and supervise the descent phase dynamically.

---

## System Architecture & ROS Node Flow

The `SafeLanding_node` handles sensor subscriber readings, algorithmic decision updates, and flight controller command publishing:

```
  [ Landing Signal Received ]
             │
             ▼
  ┌────────────────────────┐
  │ Node Initialization    │
  └──────────┬─────────────┘
             │
             ▼
  ┌────────────────────────┐
  │ Search Mode (Scan)     │◄────────────────┐
  └──────────┬─────────────┘                 │
             │                               │
             ▼                               │
  ┌────────────────────────┐                 │
  │ Execute OpenCV Checker │                 │
  └──────────┬─────────────┘                 │
             │                               │
       [ Spot Found? ]                       │
       /             \                       │
     Yes              No                     │
     /                 \                     │
    ▼                   ▼                    │
  ┌──────────┐    ┌──────────┐               │
  │ Align    │    │Random    │               │
  │ Target   │    │Movement  │───────────────┘
  └────┬─────┘    └──────────┘
       │
       ▼
  ┌────────────────────────┐
  │ Clear Zone Check       │
  └──────────┬─────────────┘
             │
       [ Is Clear? ]
       /           \
     Yes            No
     /               \
    ▼                 ▼
  ┌──────────┐    ┌──────────┐
  │ Land     │    │ Resume   │───────────────┘
  │ Vehicle  │    │ Search   │
  └──────────┘    └──────────┘
```

---

## Mathematical and Technical Highlights

### 1. Coordinates Alignment & Trajectory Heuristics
When a candidate landing zone is detected at pixel coordinates $\mathbf{x}_{\text{pix}} = [u, v]^T$, the helper module converts the target into a spatial vector in the local reference frame. Given the camera intrinsic matrix $\mathbf{K}$ and drone altitude $z_{\text{alt}}$:

$$\mathbf{p}_{\text{cam}} = z_{\text{alt}} \cdot \mathbf{K}^{-1} \begin{bmatrix} u \\ v \\ 1 \end{bmatrix}$$

The displacement vector $\mathbf{d}$ in the world frame is obtained by applying the camera-to-world rotation matrix $\mathbf{R}_c^w$:
$$\mathbf{d} = \mathbf{R}_c^w \mathbf{p}_{\text{cam}}$$

The next target position $\mathbf{x}_{\text{goal}}$ is computed as:
$$\mathbf{x}_{\text{goal}} = \mathbf{x}_{\text{uav}} + \mathbf{d}$$

---

### 2. Decision Logic and Clearance Check
During the final alignment step, the safety algorithm performs an obstacle density clearance check on the landing patch. It calculates the variance of the edge gradients $\sigma_g^2$ across the landing area image patch:

$$\sigma_g^2 = \frac{1}{N}\sum_{i=1}^{N} (\|\nabla I_i\| - \mu_g)^2$$

A surface patch is classified as **occupied / unsafe** if the texture density exceeds a calibrated threshold:
$$\text{Status} = \begin{cases} \text{SAFE} & \text{if } \sigma_g^2 < \tau_{\text{clear}} \\ \text{UNSAFE} & \text{if } \sigma_g^2 \ge \tau_{\text{clear}} \end{cases}$$

---

## File Structure

```
uav-safe-landing/
├── include/           # C++ Headers (algorithm, communicator, helper, etc.)
├── src/
│   ├── algorithm.cpp       # Computer vision landing zone checks
│   ├── communicator.cpp    # ROS publishers and subscribers (mavros/telemetry)
│   ├── helper.cpp          # Vector math and coordinate translations
│   ├── image_processor.cpp # OpenCV image conversions
│   ├── image_reciever.cpp  # Image subscriber handler
│   └── safelanding_node.cpp# Node runner state machine loop
├── CMakeLists.txt     # CMake build definition file
├── package.xml        # ROS package manifest
├── LICENSE            # MIT License
└── .gitignore         # System file ignores
```

---

## Getting Started

### Installation
Clone this package into your Catkin workspace source directory:

```bash
cd ~/catkin_ws/src
git clone <repository-url> uav-safe-landing
```

### Build
Build the package using `catkin_make`:

```bash
cd ~/catkin_ws
catkin_make --only-pkg-with-deps uav-safe-landing
```

### Running the Node
Launch the safe landing supervisor node:

```bash
rosrun uav-safe-landing SafeLanding_node
```
