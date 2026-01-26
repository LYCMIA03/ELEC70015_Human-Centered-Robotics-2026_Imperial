# ELEC70015 – Human-Centered Robotics  
## Group Crazy Thursday (Imperial College London, 2025–2026)

This repository contains the coursework materials for **ELEC70015 Human-Centered Robotics** (EEE, Imperial College London, 2025–2026), developed by **Group Crazy Thursday**.

The project focuses on the design and evaluation of an interactive mobile robotic system. This repository hosts code, simulation setups, documentation, and experimental results used throughout the coursework.

---

## Simulation Baseline: P3-AT Mobile Base (Gazebo)

The current implementation provides a **reproducible Gazebo-based simulation baseline** for the **Pioneer 3-AT (P3-AT)** mobile robot.  
This baseline establishes a stable and well-defined interface for downstream components such as navigation, perception, and human–robot interaction.

At this stage, the scope is intentionally limited to a **mobile base bringup** (no navigation or sensors enabled yet).

---

## Repository Structure

```text
ros_ws/
└── src/
    ├── amr-ros-config/      # Official MobileRobots robot descriptions (git submodule)
    ├── p3at_sim/            # Project-specific Gazebo bringup for P3-AT
    │   ├── launch/
    │   └── worlds/
    └── CMakeLists.txt       # Catkin workspace entry point
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
roslaunch p3at_sim bringup.launch
```
This launches:

- Gazebo with an empty world  
- The P3-AT mobile base  
- TF publishing via `robot_state_publisher`  

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

## Notes

- Build artifacts (`build/`, `devel/`, `install/`) are intentionally excluded from version control.  
- The simulation baseline is designed to be reproducible and container-friendly.  
- Navigation, perception (e.g. depth camera), and higher-level behaviours will be added incrementally on top of this baseline.
