# HAMR Workspace (ROS 2 Jazzy)

**HAMR (Holonomic Affordable Mobile Robot)** is a modular holonomic mobile robot developed at the  
**University of Pennsylvania – ModLab**, designed for **off-road omnidirectional locomotion** with a **stabilized torso / turret**.

This repository is the **top-level development workspace** that integrates:
- low-level motor control and state estimation,
- high-level holonomic planning and control in ROS 2,
- terrain mapping and traversability research.

---

## 📂 Repository Structure

hamr_ws/
├── HAMR_Controller/ # Low-level control, EKF, odometry, PID
├── src/
│ ├── hamr_holonomic_robot # ROS 2 Jazzy stack (planning, control, sim)
│ └── elevation_mapping_cupy # Elevation mapping (vendored, locally modified)
├── test_codes/ # Experiments, planners, plots
└── README.md


---

## 🔗 Related Repositories

- **High-level planner & ROS 2 stack**  
  https://github.com/cedrichld/hamr_holonomic_robot/tree/kartik  

This workspace **extends and integrates** the above stack with hardware-oriented control, probabilistic state estimation, and off-road experimentation.

---

## 🧠 Subsystems Overview

### 1️⃣ Low-Level Control & State Estimation  
📁 `HAMR_Controller/`

- Holonomic drive motor control (PID)
- Encoder-based probabilistic odometry motion model
- EKF-based sensor fusion (Encoders + IMU)
- Hardware-oriented control loops (ESP32, IMU, encoders)

➡️ See `HAMR_Controller/README.md` for:
- Motion model derivation
- Noise modeling
- EKF equations
- PID tuning details

---

### 2️⃣ Holonomic Planning & Control (ROS 2 Jazzy)  
📁 `src/hamr_holonomic_robot/`

- URDF / Xacro robot description
- Gazebo off-road simulation (heightmaps)
- Jacobian-based holonomic controller
- PRM / A* path planning
- Waypoint-based reference trajectories
- Turret + gimbal stabilization

➡️ See `src/hamr_holonomic_robot/README.md` for:
- Kinematics and mobility ellipsoid
- Simulation & hardware results
- ROS topics and interfaces

---

### 3️⃣ Terrain Mapping & Traversability  
📁 `src/elevation_mapping_cupy/`

- GPU-accelerated elevation mapping
- Terrain representation for rough environments
- Used for future **holonomic traversability optimization**

> This package is **vendored** (no upstream push access) and locally modified for HAMR.

---

## 🛠️ Technologies Used

- **ROS 2 Jazzy**
- **C++ / Python**
- **EKF / UKF**
- **Gazebo (gz sim)**
- **ESP32, IMU, Encoders**
- **Additive Manufacturing + CNC**
- **GPU Elevation Mapping**

---

## 📊 Project Slides & Documentation

📽️ **Project Slides (Design, Control, Results)**  
👉 Google Slides:  
PASTE_GOOGLE_SLIDES_LINK_HERE

(Optional badge)
```md
[![Slides](https://img.shields.io/badge/Slides-Google%20Slides-orange)](PASTE_GOOGLE_SLIDES_LINK_HERE)

🚀 Build & Run (ROS 2 Jazzy)
Build Workspace

cd ~/hamr_ws
colcon build --symlink-install
source install/setup.bash

Run Simulation

ros2 launch hamr_bringup compa.launch.xml

Run Reference Trajectory

ros2 run reference_trajectory waypoint_traj_simple

🧪 Experiments & Results

    Square / Triangle / Circle holonomic trajectories

    Maze navigation with PRM + A*

    Off-road heightmap traversal

    Stable turret orientation under base motion

(See sub-READMEs for plots, videos, and metrics.)
📌 Notes

    build/, install/, log/ are intentionally ignored

    elevation_mapping_cupy is tracked as plain source (not a submodule)

    hamr_holonomic_robot remains a submodule for upstream synchronization

👤 Author

Kartik Virmani
University of Pennsylvania – ModLab
Holonomic Robotics · Control · State Estimation · Autonomy


---

If you want, next I can:
- tighten this for **public GitHub / recruiters**
- add **figures/GIF embeds** from `hamr_holonomic_robot/img`
- split into **Simulation vs Hardware Quick Start**
- add **citation / paper section**

Just tell me 👍
::contentReference[oaicite:0]{index=0}