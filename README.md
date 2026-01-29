# ELEC70015 – Human-Centered Robotics  
## Group Crazy Thursday (Imperial College London, 2025–2026)

This repository contains the coursework materials for **ELEC70015 Human-Centered Robotics** (EEE, Imperial College London, 2025–2026), developed by **Group Crazy Thursday**.

The project focuses on the design and evaluation of an interactive mobile robotic system. This repository hosts code, simulation setups, documentation, and experimental results used throughout the coursework.

---

## Simulation Baseline: P3-AT Mobile Base (Gazebo)

The current implementation provides a **reproducible Gazebo-based simulation baseline** for the **Pioneer 3-AT (P3-AT)** mobile robot.  
This baseline establishes a stable and well-defined interface for downstream components such as navigation, perception, and human–robot interaction.

At this stage, the simulation provides a mobile base bringup with a forward-facing depth camera and a reproducible depth-to-LaserScan conversion pipeline. Navigation and higher-level autonomy are not yet enabled.

---

## Repository Structure

```text
ros_ws/
└── src/
    ├── amr-ros-config/      # Official MobileRobots robot descriptions (git submodule)
    ├── p3at_sim/            # Gazebo bringup for P3-AT (base + depth camera)
    │   ├── launch/
    │   ├── urdf/
    │   └── worlds/
    ├── p3at_nav/            # Perception and navigation-related nodes (depth → scan)
    │   ├── launch/
    │   └── scripts/
    └── CMakeLists.txt
```
## Prerequisites

- Ubuntu 20.04  
- ROS Noetic  
- Gazebo 11  

> Windows 11 + WSL2 (with WSLg) is supported.

## Setup

### 1. Clone the repository

```bash
git clone https://github.com/LYCMIA03/ELEC70015_Human-Centered-Robotics-2026_Imperial.git
cd ELEC70015_Human-Centered-Robotics-2026_Imperial
```
### 2. Initialise submodules (required)

This project depends on official MobileRobots descriptions, included as a git submodule.
```bash
git submodule update --init --recursive
```

## Build
```bash
source /opt/ros/noetic/setup.bash   # use setup.zsh if using zsh
cd ros_ws
catkin_make
source devel/setup.bash
```
## Run Gazebo Simulation
```bash
roslaunch p3at_sim bringup_depth.launch
```
This launches:

Gazebo with a predefined world
- The P3-AT mobile base
- A forward-facing depth camera
- TF publishing via `robot_state_publisher`
This launch file is also required when running perception pipelines described below.

## Depth to LaserScan Conversion

For compatibility with standard 2D navigation stacks, the depth image is converted into a planar `sensor_msgs/LaserScan` using `depthimage_to_laserscan`.
Due to timestamp inconsistencies in simulated depth camera `CameraInfo` messages, an intermediate synchronisation node is used to ensure reliable depth-to-scan conversion.
This pipeline is launched separately to ensure clear separation between simulation and perception components.

### Launch

Terminal 1:
```bash
roslaunch p3at_sim bringup_depth.launch
```
Terminal 2:
```bash
roslaunch p3at_nav depth_to_scan_fixed.launch
```
Output
```bash
/scan   (sensor_msgs/LaserScan, ~30 Hz)
```
The resulting scan is published in the `depth_camera_link` frame and is suitable for downstream navigation and obstacle avoidance modules.
## ROS Interfaces

The simulation exposes the following stable interfaces, which should be treated as a contract for downstream development.

### Velocity command
```text
/sim_p3at/cmd_vel   (geometry_msgs/Twist)
```
### Odometry
```text
/sim_p3at/odom      (nav_msgs/Odometry)
```
### TF (Transform Frames)
```text
odom → base_link
```
### LaserScan (from depth camera)
```text
/scan   (sensor_msgs/LaserScan)
```

## Notes

- Build artifacts (`build/`, `devel/`, `install/`) are intentionally excluded from version control.  
- The simulation baseline is designed to be reproducible and container-friendly.  
- Navigation, perception (e.g. depth camera), and higher-level behaviours will be added incrementally on top of this baseline.
