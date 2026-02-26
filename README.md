# HAMR Workspace (ROS 2 Jazzy)

**HAMR (Holonomic Affordable Mobile Robot)** is a modular holonomic
mobile robot developed at the\
**University of Pennsylvania -- ModLab**, designed for off-road
omnidirectional locomotion with active torso stabilization.

This repository is the top-level research and development workspace
integrating:

-   Low-level embedded motor control and probabilistic state estimation\
-   ROS 2 Jazzy holonomic planning and control\
-   Off-road simulation and terrain-aware navigation\
-   GPU-accelerated elevation mapping\
-   Traversability optimization research

------------------------------------------------------------------------

## Repository Structure

``` bash
hamr_ws/
├── HAMR_Controller/              # Embedded control + state estimation
├── src/
│   ├── hamr_holonomic_robot      # ROS 2 Jazzy stack (planning, control, sim)
│   └── elevation_mapping_cupy    # GPU elevation mapping (locally modified)
├── test_codes/                   # Experimental planners and analysis
└── README.md
```

------------------------------------------------------------------------

## Related Repository

High-level holonomic stack (forked & extended):

https://github.com/cedrichld/hamr_holonomic_robot/tree/kartik

This workspace extends that stack with:

-   Hardware-oriented control\
-   Probabilistic odometry\
-   EKF-based fusion\
-   Off-road terrain reasoning\
-   Embedded deployment support

------------------------------------------------------------------------

# System Overview

## 1. Low-Level Control & Probabilistic State Estimation

Directory: `HAMR_Controller/`

Implemented Features:

-   Holonomic drive motor PID control (per-wheel velocity loops)\
-   Probabilistic odometry motion model\
-   Covariance propagation for encoder-based motion\
-   EKF-based sensor fusion (Encoders + IMU)\
-   Tunable noise parameters (alpha-based motion uncertainty model)\
-   Real-time firmware for ESP32-based control\
-   Encoder interrupt handling and high-rate velocity estimation

Research Contributions:

-   Explicit covariance modeling of holonomic motion\
-   Encoder noise characterization and parameter tuning\
-   Fusion-ready state representation for ROS 2 integration\
-   Modular architecture for future UKF / factor-graph upgrades

See `HAMR_Controller/README.md` for detailed derivations and tuning
notes.

------------------------------------------------------------------------

## 2. Holonomic Planning & Control (ROS 2 Jazzy)

Directory: `src/hamr_holonomic_robot/`

Implemented Components:

-   URDF/Xacro robot description\
-   Holonomic kinematics and Jacobian-based controller\
-   Gazebo (gz sim) off-road simulation using heightmaps\
-   PRM and A\* planners\
-   Waypoint-based trajectory tracking\
-   Turret/gimbal stabilization under base motion\
-   Modular ROS 2 node architecture

Research Focus:

-   Mobility ellipsoid analysis\
-   Holonomic maneuverability in constrained environments\
-   Off-road heightmap traversal performance\
-   Planner performance benchmarking

See `src/hamr_holonomic_robot/README.md` for full details.

------------------------------------------------------------------------

## 3. Terrain Mapping & Traversability

Directory: `src/elevation_mapping_cupy/`

GPU-accelerated elevation mapping used for:

-   Heightmap construction\
-   Terrain representation\
-   Rough terrain navigation research\
-   Future traversability-aware holonomic optimization

Planned Extensions:

-   Traversability cost layers\
-   Friction-aware motion modeling\
-   Integration with sampling-based planners\
-   Gradient-based trajectory optimization over elevation maps

------------------------------------------------------------------------

# Current Capabilities

-   Holonomic trajectory tracking (square, triangle, circle)\
-   PRM + A\* maze navigation\
-   Heightmap-based simulation testing\
-   Stabilized turret orientation during aggressive base motion\
-   Probabilistic odometry with covariance propagation\
-   EKF fusion of IMU and encoder data\
-   Real-time ROS 2 Jazzy integration

------------------------------------------------------------------------

# Build & Run (ROS 2 Jazzy)

## Build Workspace

``` bash
cd ~/hamr_ws
colcon build --symlink-install
source install/setup.bash
```

## Launch Simulation

``` bash
ros2 launch hamr_bringup compa.launch.xml
```

## Run Reference Trajectory

``` bash
ros2 run reference_trajectory waypoint_traj_simple
```

------------------------------------------------------------------------

# Experimental Results

-   Square / Triangle / Circular holonomic tracking\
-   Maze navigation using PRM + A\*\
-   Off-road heightmap traversal\
-   Turret stabilization under base acceleration\
-   Covariance growth analysis under encoder noise

Quantitative metrics are provided in the respective sub-READMEs.

------------------------------------------------------------------------

# Technologies

-   ROS 2 Jazzy\
-   C++ and Python\
-   Extended Kalman Filter (EKF)\
-   Gazebo (gz sim)\
-   ESP32 embedded firmware\
-   IMU + Quadrature encoders\
-   GPU acceleration (CuPy)\
-   Additive manufacturing and CNC fabrication

------------------------------------------------------------------------

# Documentation & Slides

Project slides (Design, Control, Results):

PASTE_GOOGLE_SLIDES_LINK_HERE

Optional badge:

[![Slides](https://img.shields.io/badge/Slides-Google%20Slides-orange)](PASTE_GOOGLE_SLIDES_LINK_HERE)

------------------------------------------------------------------------

# Development Notes

-   build/, install/, and log/ are intentionally ignored\
-   elevation_mapping_cupy is tracked as source (not submodule)\
-   hamr_holonomic_robot remains a submodule for upstream
    synchronization\
-   Workspace structured for hardware + simulation dual testing

------------------------------------------------------------------------

# Author

Kartik Virmani\
University of Pennsylvania -- ModLab\
Mechanical Engineering & Applied Mechanics\
Robotics and Mechatronics Research