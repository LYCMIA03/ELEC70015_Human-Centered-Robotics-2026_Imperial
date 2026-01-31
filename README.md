# ELEC70015 Human-Centered Robotics - P3AT Mobile Robot Platform

ROS Noetic workspace for P3AT mobile robot simulation and navigation with depth camera integration.

---

## 🏗️ Project Structure

```
ELEC70015_Human-Centered-Robotics-2026_Imperial/
├── ros_ws/                    # ROS workspace
│   ├── src/
│   │   ├── p3at_sim/         # P3AT simulation package
│   │   ├── p3at_nav/         # Navigation and perception
│   │   ├── amr-ros-config/   # AMR configuration (git submodule)
│   │   └── gazebo_ros_pkgs/  # Gazebo-ROS interface (local, ignored by git)
│   ├── build/                # Build artifacts (ignored)
│   └── devel/                # Development space (ignored)
├── tools/                    # Utility scripts
└── README.md
```

---

## 🔧 Installation

### Prerequisites
- Ubuntu 20.04 LTS
- ROS Noetic
- Gazebo 11

### 1. Clone Repository

```bash
git clone <your-repo-url> ELEC70015_Human-Centered-Robotics-2026_Imperial
cd ELEC70015_Human-Centered-Robotics-2026_Imperial

# Initialize submodules
git submodule update --init --recursive
```

### 2. Install ROS Dependencies

```bash
# Core packages
sudo apt install ros-noetic-desktop-full \
                 ros-noetic-navigation \
                 ros-noetic-gmapping \
                 ros-noetic-amcl \
                 ros-noetic-depthimage-to-laserscan \
                 ros-noetic-urdf \
                 ros-noetic-xacro \
                 ros-noetic-robot-state-publisher \
                 ros-noetic-joint-state-publisher

# Install workspace dependencies
cd ros_ws
rosdep install --from-paths src --ignore-src -r -y
```

### 3. Setup gazebo_ros_pkgs (Source Build)

**⚠️ Important:** We use a source-built version of `gazebo_ros_pkgs` for compatibility with our URDF configuration.

```bash
cd ~/ELEC70015_Human-Centered-Robotics-2026_Imperial/ros_ws/src

# Clone gazebo_ros_pkgs
git clone https://github.com/ros-simulation/gazebo_ros_pkgs.git -b noetic-devel

# Return to workspace root
cd ..
```

**Note:** `gazebo_ros_pkgs/` is intentionally ignored by git (see `.gitignore`) - you must clone it manually after pulling the repository.

### 4. Build Workspace

```bash
cd ~/ELEC70015_Human-Centered-Robotics-2026_Imperial/ros_ws
catkin_make
source devel/setup.bash  # or setup.zsh for zsh users
```

---

## 🚀 Quick Start

### Launch Simulation with Depth Camera

#### Terminal 1: Gazebo Simulation
```bash
cd ~/ELEC70015_Human-Centered-Robotics-2026_Imperial/ros_ws
source devel/setup.bash
roslaunch p3at_sim bringup_depth.launch
```

#### Terminal 2: Perception Pipeline
```bash
cd ~/ELEC70015_Human-Centered-Robotics-2026_Imperial/ros_ws
source devel/setup.bash
roslaunch p3at_nav depth_to_scan.launch
```

#### Terminal 3: Visualization
```bash
cd ~/ELEC70015_Human-Centered-Robotics-2026_Imperial/ros_ws
source devel/setup.bash
rviz -d src/p3at_nav/rviz/depth_scan.rviz
```

---

## 📦 Packages Overview

### `p3at_sim` - Robot Simulation
P3AT mobile robot in Gazebo with depth camera.

**Key Files:**
- `urdf/p3at_with_depth_camera.urdf.xacro` - Robot model with RGB + Depth camera
- `launch/bringup_depth.launch` - Gazebo simulation launcher

**Robot Features:**
- 4-wheel skid-steer drive
- Intel RealSense-like depth camera (640×480 @ 30Hz)
- RGB camera (640×480 @ 30Hz)
- 10m detection range

### `p3at_nav` - Navigation & Perception
Depth-to-laserscan conversion for 2D navigation.

**Key Files:**
- `launch/depth_to_scan.launch` - Perception pipeline
- `scripts/depth_camera_info_from_image.py` - Camera info generator
- `rviz/depth_scan.rviz` - Visualization config

**Pipeline:**
```
Depth Image → Camera Info Generator → depthimage_to_laserscan → /scan
```

### `amr-ros-config` (Submodule)
MobileRobots AMR configuration files.

**Source:** https://github.com/MobileRobots/amr-ros-config.git

---

## 🎯 Key Topics

### Camera Topics
```bash
/sim_p3at/camera/color/image_raw          # RGB image (sensor_msgs/Image)
/sim_p3at/camera/color/camera_info        # RGB camera calibration
/sim_p3at/camera/depth/depth/image_raw    # Depth image (float32)
/sim_p3at/camera/depth/depth/camera_info  # Depth camera calibration
/sim_p3at/camera/depth/points             # Point cloud (sensor_msgs/PointCloud2)
```

### Navigation Topics
```bash
/scan                                     # Laser scan from depth (sensor_msgs/LaserScan)
/sim_p3at/cmd_vel                        # Velocity commands (geometry_msgs/Twist)
/sim_p3at/odom                           # Odometry (nav_msgs/Odometry)
```

---

## 🧪 Verification

### Check System Status

```bash
# List all camera topics
rostopic list | grep sim_p3at/camera

# Verify sensor frequencies
rostopic hz /sim_p3at/camera/depth/depth/image_raw  # Should show ~30 Hz
rostopic hz /scan                                    # Should show ~30 Hz

# Check node graph
rqt_graph
```

### Expected Output
```bash
$ rostopic hz /scan
subscribed to [/scan]
average rate: 29.980
        min: 0.024s max: 0.040s std dev: 0.00302s
```

### View Sensor Data

```bash
# RGB camera
rosrun image_view image_view image:=/sim_p3at/camera/color/image_raw

# Depth visualization (requires depth_image_proc)
rosrun image_view image_view image:=/sim_p3at/camera/depth/image_rect_raw
```

---

## 🛠️ Troubleshooting

### Problem: No sensor data (`rostopic hz` shows "no new messages")

**Diagnosis:**
```bash
rostopic list | grep camera  # Check if topics exist
rosnode list                 # Verify all nodes running
```

**Solution:**
1. Ensure `gazebo_ros_pkgs` is built from source
2. Verify URDF contains `<robotNamespace>/sim_p3at</robotNamespace>` in camera plugins
3. Restart simulation completely:
   ```bash
   killall -9 gzserver gzclient rosmaster
   roslaunch p3at_sim bringup_depth.launch
   ```

---

### Problem: Build errors with `gazebo_ros_pkgs`

**Solution:**
```bash
# Clean workspace
cd ~/ELEC70015_Human-Centered-Robotics-2026_Imperial/ros_ws
catkin_make clean
rm -rf build/ devel/

# Verify Gazebo version
gazebo --version  # Should be 11.x

# Rebuild
catkin_make
source devel/setup.bash
```

---

### Problem: TF errors or missing transforms

**Diagnosis:**
```bash
rosrun tf view_frames
evince frames.pdf  # View TF tree
```

**Common Issues:**
- Missing `robot_state_publisher` node
- URDF not loaded properly
- Camera frames not defined

**Solution:** Check that `bringup_depth.launch` includes `robot_state_publisher`.

---

## 📚 Technical Details

### Depth Camera Configuration

**URDF Plugins:**
- RGB: `libgazebo_ros_camera.so`
- Depth: `libgazebo_ros_depth_camera.so`

**Critical Settings:**
```xml
<plugin name="depth_camera_controller" filename="libgazebo_ros_depth_camera.so">
  <robotNamespace>/sim_p3at</robotNamespace>  <!-- Essential for namespacing -->
  <cameraName>camera/depth</cameraName>
  <!-- DO NOT use <format> tag - causes "Unsupported ImageFormat" error -->
</plugin>
```

### Depth-to-Scan Parameters

**Configuration** (`depth_to_scan.launch`):
- `output_frame_id`: `camera_depth_optical_frame`
- `scan_height`: 10 pixels (vertical sampling)
- `scan_time`: 0.033s (30 Hz)
- `range_min`: 0.2m
- `range_max`: 10.0m

**Topic Remapping:**
```xml
<remap from="image"       to="/sim_p3at/camera/depth/depth/image_raw"/>
<remap from="camera_info" to="/sim_p3at/camera/depth/depth/camera_info"/>
<remap from="scan"        to="/scan"/>
```

---

## 🔍 Development Tools

### Utility Scripts (`tools/`)

```bash
# Source ROS environment quickly
source tools/source_ros.zsh  # for zsh
source tools/source_ros.sh   # for bash

# Inspect depth image metadata once
python3 tools/inspect_depth_once.py

# Manual camera_info publisher (debugging)
python3 tools/camera_info_pub.py

# Relay camera_info (legacy)
python3 tools/relay_camera_info.py
```

---

## 📂 Git Workflow

### What's Tracked
- ✅ Source packages: `p3at_sim/`, `p3at_nav/`
- ✅ Launch files, URDF, RViz configs
- ✅ Utility scripts: `tools/`
- ✅ Documentation: `README.md`
- ✅ Submodule: `amr-ros-config/`

### What's Ignored (`.gitignore`)
- ❌ Build artifacts: `build/`, `devel/`
- ❌ Source build: `gazebo_ros_pkgs/` (must clone manually)
- ❌ Temporary files: `*.pyc`, `__pycache__/`
- ❌ IDE configs: `.vscode/`, `.idea/`
- ❌ ROS logs: `.ros/log/`

### Setup After Clone

```bash
# 1. Clone repository
git clone <repo-url> ELEC70015_Human-Centered-Robotics-2026_Imperial
cd ELEC70015_Human-Centered-Robotics-2026_Imperial

# 2. Initialize submodules
git submodule update --init --recursive

# 3. Clone gazebo_ros_pkgs (not a submodule, intentionally local)
cd ros_ws/src
git clone https://github.com/ros-simulation/gazebo_ros_pkgs.git -b noetic-devel

# 4. Build workspace
cd ..
catkin_make
source devel/setup.bash
```

---

## 📖 Resources

- [ROS Noetic Documentation](http://wiki.ros.org/noetic)
- [Gazebo Tutorials](http://gazebosim.org/tutorials)
- [depthimage_to_laserscan Package](http://wiki.ros.org/depthimage_to_laserscan)
- [URDF Tutorial](http://wiki.ros.org/urdf/Tutorials)
- [REP-103: Coordinate Frames](https://www.ros.org/reps/rep-0103.html)
- [REP-105: Coordinate Frames for Mobile Platforms](https://www.ros.org/reps/rep-0105.html)

---

## ✅ Post-Installation Checklist

Verify everything works:

- [ ] Workspace builds without errors: `catkin_make`
- [ ] Gazebo launches: `roslaunch p3at_sim bringup_depth.launch`
- [ ] Robot spawns in simulation (no errors in terminal)
- [ ] Camera topics exist: `rostopic list | grep camera`
- [ ] Depth image publishes at 30Hz: `rostopic hz /sim_p3at/camera/depth/depth/image_raw`
- [ ] Perception pipeline works: `roslaunch p3at_nav depth_to_scan.launch`
- [ ] Scan data publishes at 30Hz: `rostopic hz /scan`
- [ ] RViz displays correctly: `rviz -d src/p3at_nav/rviz/depth_scan.rviz`
- [ ] TF tree complete: `rosrun tf view_frames && evince frames.pdf`
