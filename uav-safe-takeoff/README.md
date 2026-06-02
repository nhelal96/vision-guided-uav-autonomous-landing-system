# UAV Safe Takeoff Supervisor (ROS Package)

This ROS package implements an autonomous takeoff controller and pre-flight safety supervisor for Unmanned Aerial Vehicles (UAVs). It ensures takeoff sequences are executed safely by validating sensor readings, verifying structural clearances, controlling ascent rates, and monitoring altitude hold stability before transferring flight control to local path planners.

---

## Takeoff Sequence Flow

The `safetakeoff_node` processes initialization parameters, checks vehicle status, and coordinates the takeoff sequence:

```
  ┌────────────────────────┐
  │  Node Initialization   │
  └──────────┬─────────────┘
             │
             ▼
  ┌────────────────────────┐
  │  Sensor Health Checks  │
  └──────────┬─────────────┘
             │
         [ Pass? ]
         /       \
       Yes        No ──► [ Abort Sequence ]
       /
      ▼
  ┌────────────────────────┐
  │ Arm Motors & Offboard  │
  └──────────┬─────────────┘
             │
             ▼
  ┌────────────────────────┐
  │ Ascent Phase (v_z target)│◄──────┐
  └──────────┬─────────────┘         │
             │                       │
      [ Altitude >= Target? ]        │
      /                     \        │
    Yes                      No ─────┘
    /
   ▼
  ┌────────────────────────┐
  │ Hover Hold Stability   │
  └──────────┬─────────────┘
             │
             ▼
  ┌────────────────────────┐
  │ Sequence Completed     │
  └────────────────────────┘
```

---

## Technical Highlights & Mathematical Modeling

### 1. Controlled Ascent & Altitude Tracking
The supervisor controls takeoff using a vertical velocity target loop. Given the target altitude $z_{\text{target}}$ and the current estimated altitude $z(t)$, the vertical speed command $v_z(t)$ is modeled as:

$$e_z(t) = z_{\text{target}} - z(t)$$

$$v_z(t) = \text{clip}\left( K_p e_z(t) + K_d \frac{d e_z(t)}{dt}, \, v_{\min}, \, v_{\max} \right)$$

where:
*   $K_p, K_d$ are gain constants.
*   $v_{\min}, v_{\max}$ constrain the ascent and descent rates for stability.

---

### 2. Takeoff Stability and Hover Verification
To confirm that the takeoff has successfully completed, the supervisor monitors altitude variance $\sigma_z^2$ and velocity magnitude $\|\mathbf{v}\|$ over a verification window of length $T$:

$$\sigma_z^2 = \frac{1}{T} \int_{t-T}^{t} (z(\tau) - \mu_z)^2 d\tau < \epsilon_z$$

$$\|\mathbf{v}\| = \sqrt{v_x^2 + v_y^2 + v_z^2} < \epsilon_v$$

If both stability conditions are met for a consecutive duration of $2\text{ seconds}$, the vehicle is classified as safely hovering, and the takeoff sequence completes.

---

## File Structure

```
uav-safe-takeoff/
├── include/           # C++ Headers (communicator, helper, image_processor)
├── src/
│   ├── algorithm.cpp       # Obstacle density assessment algorithms
│   ├── communicator.cpp    # MAVROS state listeners and position publishers
│   ├── helper.cpp          # Flight control utilities
│   ├── image_processor.cpp # Visual validation functions
│   └── safetakeoff_node.cpp# Node runner state machine loop
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
git clone <repository-url> uav-safe-takeoff
```

### Build
Build the package:

```bash
cd ~/catkin_ws
catkin_make --only-pkg-with-deps uav-safe-takeoff
```

### Running the Node
Start the safe takeoff controller:

```bash
rosrun uav-safe-takeoff safetakeoff_node
```
