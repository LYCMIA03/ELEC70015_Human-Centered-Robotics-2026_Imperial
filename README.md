# ELEC70015 Human-Centered Robotics - P3AT Mobile Robot Platform

ROS Noetic workspace for P3AT mobile robot simulation, SLAM mapping, and navigation with depth camera integration.

**Important Note:** This project does not use a physical LiDAR sensor. The `/scan` topic is generated entirely from depth camera data using `depthimage_to_laserscan`.

---

## Project Structure

```
ELEC70015_Human-Centered-Robotics-2026_Imperial/
├── ros_ws/                    # ROS workspace
│   ├── src/
│   │   ├── p3at_sim/         # P3AT simulation package
│   │   ├── p3at_nav/         # Navigation, perception, and SLAM
│   │   │   ├── config/       # GMapping SLAM parameters
│   │   │   ├── launch/       # Launch files (depth_to_scan, gmapping_slam)
│   │   │   ├── maps/         # Saved SLAM maps
│   │   │   ├── rviz/         # RViz configurations (depth_scan, slam)
│   │   │   └── scripts/      # Utility scripts (save_map.sh)
│   │   ├── amr-ros-config/   # AMR configuration (git submodule)
│   │   └── gazebo_ros_pkgs/  # Gazebo-ROS interface (local, ignored by git)
│   ├── build/                # Build artifacts (ignored)
│   └── devel/                # Development space (ignored)
├── tools/                    # Utility scripts
├── data/                     # Data directory (optional)
│   └── maps/                 # Alternative map storage location
└── README.md
```

---

## Installation

### Prerequisites

#### System Requirements
- **Ubuntu 20.04 LTS**
- **Python 3.8+**
- **ROS Noetic** (path: `/opt/ros/noetic`)
- **Gazebo 11**

**Note for WSL2 Users:** If the robot model displays correctly in RViz but the P3AT visual mesh does not appear in Gazebo, this may be due to GPU rendering limitations in WSL2. For solutions, refer to [WSL2 GPU Acceleration Guide](https://zhuanlan.zhihu.com/p/19575977500).

#### Python Dependencies

The project requires additional Python packages for SLAM and image processing:

```bash
# Core dependencies
pip install numpy>=1.24.4
pip install matplotlib>=3.1.2
pip install opencv-python>=4.13.0
pip install opencv-contrib-python>=4.13.0
pip install defusedxml  # Required for xacro URDF processing

# SLAM and image processing
pip install scipy>=1.10.1
pip install scikit-image>=0.21.0
pip install pillow>=10.4.0
pip install networkx>=3.1
```

**Optional:** Add Python scripts to PATH:
```bash
echo 'export PATH="$HOME/.local/bin:$PATH"' >> ~/.bashrc
source ~/.bashrc
```

**For zsh users:**
```bash
echo 'export PATH="$HOME/.local/bin:$PATH"' >> ~/.zshrc
source ~/.zshrc
```

**Verify ROS installation:**
```bash
which roscore  # Should return: /opt/ros/noetic/bin/roscore
```

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
sudo apt update
sudo apt install ros-noetic-desktop-full \
                 ros-noetic-navigation \
                 ros-noetic-gmapping \
                 ros-noetic-amcl \
                 ros-noetic-depthimage-to-laserscan \
                 ros-noetic-urdf \
                 ros-noetic-xacro \
                 ros-noetic-robot-state-publisher \
                 ros-noetic-joint-state-publisher \
                 ros-noetic-map-server \
                 ros-noetic-teleop-twist-keyboard \
                 ros-noetic-realsense2-description

# If realsense2-description is not found, install the full RealSense ROS package
sudo apt install ros-noetic-realsense2-camera

# Install workspace dependencies
cd ros_ws
rosdep install --from-paths src --ignore-src -r -y
```

### 3. Setup gazebo_ros_pkgs (Source Build)

**Important:** We use a source-built version of `gazebo_ros_pkgs` for compatibility with our URDF configuration.

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
source devel/setup.bash  # for bash users
# OR
source devel/setup.zsh   # for zsh users
```

**Important for zsh users:** ROS setup scripts have both `.bash` and `.zsh` versions. Always use `.zsh` files if you're using zsh shell.

---

## Quick Start

### Option 1: Basic Simulation with Depth Camera

#### Terminal 1: Gazebo Simulation
```bash
cd ~/ELEC70015_Human-Centered-Robotics-2026_Imperial/ros_ws
source devel/setup.bash  # or setup.zsh
roslaunch p3at_sim bringup_depth.launch
```

#### Terminal 2: Perception Pipeline
```bash
cd ~/ELEC70015_Human-Centered-Robotics-2026_Imperial/ros_ws
source devel/setup.bash  # or setup.zsh
roslaunch p3at_nav depth_to_scan.launch
```

#### Terminal 3: Visualization
```bash
cd ~/ELEC70015_Human-Centered-Robotics-2026_Imperial/ros_ws
source devel/setup.bash  # or setup.zsh
rviz -d src/p3at_nav/rviz/depth_scan.rviz
```

### Option 2: SLAM Mapping

#### Terminal 1: Gazebo Simulation
```bash
source /opt/ros/noetic/setup.zsh  # or setup.bash
source ~/ELEC70015_Human-Centered-Robotics-2026_Imperial/ros_ws/devel/setup.zsh
roslaunch p3at_sim bringup_depth.launch
```

#### Terminal 2: Depth to Laser Scan Conversion
```bash
source /opt/ros/noetic/setup.zsh
source ~/ELEC70015_Human-Centered-Robotics-2026_Imperial/ros_ws/devel/setup.zsh
roslaunch p3at_nav depth_to_scan.launch
```

#### Terminal 3: GMapping SLAM
```bash
source /opt/ros/noetic/setup.zsh
source ~/ELEC70015_Human-Centered-Robotics-2026_Imperial/ros_ws/devel/setup.zsh
roslaunch p3at_nav gmapping_slam.launch
```

#### Terminal 4: Keyboard Teleoperation
```bash
source /opt/ros/noetic/setup.zsh
source ~/ELEC70015_Human-Centered-Robotics-2026_Imperial/ros_ws/devel/setup.zsh
rosrun teleop_twist_keyboard teleop_twist_keyboard.py cmd_vel:=/sim_p3at/cmd_vel
```

**Keyboard Controls:**
- `i` - Move forward
- `,` - Move backward
- `j` - Turn left
- `l` - Turn right
- `k` - Stop
- `q`/`z` - Increase/decrease max speeds
- `w`/`x` - Increase/decrease linear speed only
- `e`/`c` - Increase/decrease angular speed only

#### Save the Map

After exploring the environment and building a satisfactory map:

```bash
source /opt/ros/noetic/setup.zsh
source ~/ELEC70015_Human-Centered-Robotics-2026_Imperial/ros_ws/devel/setup.zsh

# Save map with custom name
rosrun map_server map_saver -f ~/ELEC70015_Human-Centered-Robotics-2026_Imperial/ros_ws/src/p3at_nav/maps/my_map_name

# Or use the provided script
./ros_ws/src/p3at_nav/scripts/save_map.sh my_map_name
```

**Output files:**
- `my_map_name.pgm` - Map image (grayscale)
- `my_map_name.yaml` - Map metadata (resolution, origin, thresholds)

---

## Packages Overview

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
- No physical LiDAR - `/scan` is generated from depth camera conversion

### `p3at_nav` - Navigation, Perception & SLAM
Depth-to-laserscan conversion and GMapping SLAM for 2D mapping and navigation.

**Key Files:**
- `launch/depth_to_scan.launch` - Depth camera to laser scan conversion
- `launch/gmapping_slam.launch` - GMapping SLAM with RViz visualization
- `config/gmapping_params.yaml` - SLAM algorithm parameters
- `scripts/depth_camera_info_from_image.py` - Camera info generator
- `scripts/save_map.sh` - Map saving utility script
- `rviz/depth_scan.rviz` - Depth visualization config
- `rviz/slam.rviz` - SLAM visualization config
- `maps/` - Directory for saved SLAM maps

**SLAM Pipeline:**
```
Gazebo Depth Camera
        ↓
/sim_p3at/camera/depth/image_raw
        ↓
depth_camera_info_generator
        ↓
depthimage_to_laserscan (output_frame_id: base_link)
        ↓
/scan (sensor_msgs/LaserScan)
        ↓
GMapping SLAM (slam_gmapping node)
        ↓
/map (nav_msgs/OccupancyGrid)
```

**GMapping Configuration:**
- Map resolution: 0.05m (5cm per pixel)
- Sensor range: 0-10m
- Particles: 30
- Update thresholds: 0.2m linear, 0.2 rad angular
- Output frame: `base_link` (corrected from `camera_depth_optical_frame`)

### `amr-ros-config` (Submodule)
MobileRobots AMR configuration files.

**Source:** https://github.com/MobileRobots/amr-ros-config.git

---

## Key Topics

### Camera Topics
```bash
/sim_p3at/camera/color/image_raw          # RGB image (sensor_msgs/Image)
/sim_p3at/camera/color/camera_info        # RGB camera calibration
/sim_p3at/camera/depth/depth/image_raw    # Depth image (32FC1, 640x480)
/sim_p3at/camera/depth/depth/camera_info  # Depth camera calibration (generated)
/sim_p3at/camera/depth/points             # Point cloud (sensor_msgs/PointCloud2)
```

### Navigation Topics
```bash
/scan                                     # Virtual laser scan (converted from depth, frame_id: base_link)
/sim_p3at/cmd_vel                        # Velocity commands (geometry_msgs/Twist)
/sim_p3at/odom                           # Odometry (nav_msgs/Odometry)
```

### SLAM Topics
```bash
/map                                      # Occupancy grid map (nav_msgs/OccupancyGrid)
/map_metadata                             # Map metadata (nav_msgs/MapMetaData)
/map_updates                              # Incremental map updates
/slam_gmapping/entropy                    # Particle filter entropy (std_msgs/Float64)
```

---

## System Architecture

### SLAM Mapping Data Flow

```
┌─────────────────────────────────────────────────────────────────────┐
│                         Gazebo Simulation                          │
│  ┌─────────────────┐                                                │
│  │   P3AT Robot    │                                                │
│  │  ┌───────────┐  │                                                │
│  │  │Depth Cam  │──┼──→ /sim_p3at/camera/depth/depth/image_raw     │
│  │  │(no lidar!)│  │                                                │
│  │  └───────────┘  │                                                │
│  │                 │                                                │
│  │  Skid-steer ────┼──→ /sim_p3at/odom                             │
│  │  Drive          │                                                │
│  └─────────────────┘                                                │
└─────────────────────────────────────────────────────────────────────┘
                                    │
                                    ▼
┌─────────────────────────────────────────────────────────────────────┐
│                      Perception Pipeline                           │
│                                                                     │
│  /sim_p3at/camera/depth/depth/image_raw                            │
│         │                                                           │
│         ▼                                                           │
│  ┌─────────────────────────────────┐                               │
│  │ depth_camera_info_generator     │                               │
│  └─────────────────────────────────┘                               │
│         │                                                           │
│         ▼                                                           │
│  /sim_p3at/camera/depth/depth/camera_info                          │
│         │                                                           │
│         ▼                                                           │
│  ┌─────────────────────────────┐                                   │
│  │  depthimage_to_laserscan    │                                   │
│  │  output_frame_id: base_link │                                   │
│  └─────────────────────────────┘                                   │
│         │                                                           │
│         ▼                                                           │
│      /scan (frame_id: base_link)                                   │
│                                                                     │
└─────────────────────────────────────────────────────────────────────┘
                                    │
                                    ▼
┌─────────────────────────────────────────────────────────────────────┐
│                         SLAM Module                                │
│                                                                     │
│  ┌──────────────────────────────┐                                  │
│  │  GMapping (slam_gmapping)    │                                  │
│  │                              │                                  │
│  │  Inputs:                     │                                  │
│  │  - /scan                     │                                  │
│  │  - /sim_p3at/odom           │                                  │
│  │  - /tf (odom → base_link)   │                                  │
│  │                              │                                  │
│  │  Outputs:                    │                                  │
│  │  - /map                      │                                  │
│  │  - /tf (map → odom)         │                                  │
│  └──────────────────────────────┘                                  │
│         │                                                           │
│         ▼                                                           │
│  Occupancy Grid Map (800×800 @ 0.05m)                             │
│                                                                     │
└─────────────────────────────────────────────────────────────────────┘
```

### Runtime Node List

#### Basic Simulation (7 nodes)
| Node Name | Function |
|-----------|----------|
| `/gazebo` | Gazebo physics simulation engine |
| `/gazebo_gui` | Gazebo graphical interface |
| `/robot_state_publisher` | Publishes robot TF transforms |
| `/joint_state_publisher` | Publishes joint states |
| `/depth_camera_info_generator` | Generates synchronized camera_info |
| `/depth_to_scan` | Converts depth image to laser scan |
| `/rosout` | ROS logging system |

#### SLAM Mapping (9 nodes)
All basic simulation nodes plus:
| Node Name | Function |
|-----------|----------|
| `/slam_gmapping` | GMapping SLAM algorithm |
| `/rviz_slam` | RViz visualization for SLAM |

---

## SLAM Mapping Workflow

### 1. Start SLAM System

Launch all four terminals as described in [Quick Start - Option 2: SLAM Mapping](#option-2-slam-mapping).

### 2. Verify System Status

```bash
# Check that slam_gmapping is running
rosnode list | grep slam

# Verify map is being published (~1Hz during exploration)
rostopic hz /map

# Check laser scan data (should be ~30Hz)
rostopic hz /scan

# Verify odometry (should be ~100Hz)
rostopic hz /sim_p3at/odom
```

### 3. Drive Robot to Build Map

Use keyboard teleoperation (Terminal 4) to explore the environment:

**Mapping Strategy:**
1. Start with slow, steady movements
2. Rotate in place occasionally to scan surroundings
3. Cover all areas of interest systematically
4. Avoid rapid movements that may cause odometry drift
5. Close loops by returning to previously visited areas

**Monitor RViz:**
- Red points: Current laser scan
- Gray cells: Occupied space (walls, obstacles)
- White cells: Free space
- Unknown cells: Unexplored areas

### 4. Save Map

When satisfied with the map coverage:

```bash
# Method 1: Direct map_saver
rosrun map_server map_saver -f ~/ELEC70015_Human-Centered-Robotics-2026_Imperial/ros_ws/src/p3at_nav/maps/my_map

# Method 2: Use provided script
cd ~/ELEC70015_Human-Centered-Robotics-2026_Imperial/ros_ws
./src/p3at_nav/scripts/save_map.sh my_map
```

### 5. Inspect Saved Map

```bash
# View map metadata
cat ~/ELEC70015_Human-Centered-Robotics-2026_Imperial/ros_ws/src/p3at_nav/maps/my_map.yaml

# View map image (if GUI available)
xdg-open ~/ELEC70015_Human-Centered-Robotics-2026_Imperial/ros_ws/src/p3at_nav/maps/my_map.pgm

# Or view in VS Code
code ~/ELEC70015_Human-Centered-Robotics-2026_Imperial/ros_ws/src/p3at_nav/maps/my_map.pgm
```

### 6. Stop SLAM System

Press `Ctrl+C` in all terminals to cleanly shut down:
1. Stop keyboard teleoperation (Terminal 4)
2. Stop GMapping SLAM (Terminal 3)
3. Stop depth_to_scan (Terminal 2)
4. Stop Gazebo simulation (Terminal 1)

**Clean shutdown (if needed):**
```bash
killall -9 gzserver gzclient roscore rosmaster
```

---

## Verification

### Check System Status

```bash
# List all running nodes (7 for basic, 9 with SLAM)
rosnode list

# Expected output (basic simulation):
# /depth_camera_info_generator
# /depth_to_scan
# /gazebo
# /gazebo_gui
# /joint_state_publisher
# /robot_state_publisher
# /rosout

# With SLAM active, also:
# /slam_gmapping
# /rviz_slam
```

### Verify Sensor Frequencies

```bash
# RGB camera (~30 Hz)
rostopic hz /sim_p3at/camera/color/image_raw

# Depth image (~30 Hz)
rostopic hz /sim_p3at/camera/depth/depth/image_raw

# Camera info (~30 Hz, synchronized)
rostopic hz /sim_p3at/camera/depth/depth/camera_info

# Laser scan (~30 Hz)
rostopic hz /scan

# Odometry (~100 Hz)
rostopic hz /sim_p3at/odom

# SLAM map (~1 Hz during exploration, less frequent when stationary)
rostopic hz /map
```

### Check /scan Data

```bash
# View scan message
rostopic echo /scan -n 1
```

**Expected characteristics (after frame_id correction):**
- `frame_id`: `base_link` (corrected from `camera_depth_optical_frame`)
- `angle_min/max`: Approximately ±0.52 rad (±30°)
- `range_min`: 0.2m
- `range_max`: 10.0m
- `ranges`: Contains valid range measurements and `nan` values

**Note:** `nan` values are normal and indicate sky, ground, or out-of-range areas.

### Check SLAM Status

```bash
# View GMapping node info
rosnode info /slam_gmapping

# Check map metadata
rostopic echo /map_metadata -n 1

# Monitor particle filter entropy
rostopic echo /slam_gmapping/entropy
```

**Expected map metadata:**
```yaml
resolution: 0.050000     # 5cm per pixel
width: 800               # 800 pixels
height: 800              # 800 pixels
origin:
  position:
    x: -26.200000        # Map origin in meters
    y: -15.000000
    z: 0.000000
  orientation:
    x: 0.0
    y: 0.0
    z: 0.0
    w: 1.0
```

### Verify Node Connections

```bash
# Check depth_to_scan node
rosnode info /depth_to_scan
```

**Important Note:** The `rosnode info /depth_to_scan` Subscriptions list may appear empty or show only `/clock`. This is normal behavior. The `depthimage_to_laserscan` package uses `image_transport` to subscribe to images, which does not appear in standard `rosnode info` output. As long as `/scan` publishes data at ~30Hz, the system is functioning correctly.

### Check TF Tree

```bash
# Generate TF tree PDF (26 frames basic, 27 with SLAM)
rosrun tf view_frames

# View the PDF
xdg-open frames.pdf  # Alternatives: evince, eog, okular

# Check specific transforms
rosrun tf tf_echo odom base_link
rosrun tf tf_echo base_link camera_depth_optical_frame

# With SLAM active, also check:
rosrun tf tf_echo map odom
```

**Expected TF tree with SLAM:**
```
map → odom → base_link → [all robot frames]
```

**Expected camera transform:**
```
At time 0.000
- Translation: [0.261, 0.018, 0.463]
- Rotation: in Quaternion [-0.500, 0.500, -0.500, 0.500]
            in RPY (radian) [-1.571, -0.000, -1.571]
            in RPY (degree) [-90.000, -0.000, -90.000]
```

### TF Frame List (27 frames with SLAM)

The complete TF tree with SLAM active contains:
- **SLAM frames:** `map` → `odom`
- **Base frames:** `odom` → `base_link`
- **Sensor frames:** `base_link` → `top_plate`, `front_sonar`, `back_sonar`
- **Camera frames:** `top_plate` → `camera_bottom_screw_frame` → `camera_link` → `camera_depth/color/infra_frame` → `*_optical_frame`
- **Wheel frames:** `base_link` → `p3at_front/back_left/right_axle` → `hub` → `wheel`

### View Sensor Data

```bash
# RGB camera
rosrun image_view image_view image:=/sim_p3at/camera/color/image_raw

# Node communication graph
rqt_graph

# 3D visualization
rviz
```

---

## Troubleshooting

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

### Problem: `/scan` topic exists but no data

**Diagnosis:**
```bash
# Check if depth_to_scan is publishing
rostopic info /scan

# Check node status
rosnode info /depth_to_scan

# Verify depth image is available
rostopic hz /sim_p3at/camera/depth/depth/image_raw
```

**Note:** Even if `rosnode info /depth_to_scan` shows empty subscriptions, check `/scan` frequency:
```bash
rostopic hz /scan  # Should be ~30 Hz if working
```

If `/scan` has no data:
1. Verify depth camera is publishing: `rostopic hz /sim_p3at/camera/depth/depth/image_raw`
2. Check camera_info synchronization: `rostopic hz /sim_p3at/camera/depth/depth/camera_info`
3. Restart the perception pipeline:
   ```bash
   roslaunch p3at_nav depth_to_scan.launch
   ```

---

### Problem: GMapping not publishing map

**Diagnosis:**
```bash
# Check if slam_gmapping node is running
rosnode list | grep slam

# Check map topic
rostopic hz /map

# Verify inputs are available
rostopic hz /scan
rostopic hz /sim_p3at/odom
```

**Common causes:**
1. **Robot not moving:** GMapping only updates when robot moves. Use keyboard teleoperation to drive the robot.
2. **Frame ID mismatch:** Ensure `/scan` frame_id is `base_link`, not `camera_depth_optical_frame`. This is configured in `depth_to_scan.launch`:
   ```xml
   <param name="output_frame_id" value="base_link"/>
   ```
3. **TF tree incomplete:** Check that `map → odom → base_link` transform chain exists:
   ```bash
   rosrun tf tf_echo map base_link
   ```

**Solution:**
```bash
# Restart GMapping
rosnode kill /slam_gmapping
roslaunch p3at_nav gmapping_slam.launch

# Drive robot to trigger map updates
rosrun teleop_twist_keyboard teleop_twist_keyboard.py cmd_vel:=/sim_p3at/cmd_vel
```

---

### Problem: Map saving fails

**Diagnosis:**
```bash
# Check if map topic is publishing
rostopic echo /map_metadata -n 1

# Verify output directory exists
ls -la ~/ELEC70015_Human-Centered-Robotics-2026_Imperial/ros_ws/src/p3at_nav/maps/
```

**Solution:**
```bash
# Create maps directory if missing
mkdir -p ~/ELEC70015_Human-Centered-Robotics-2026_Imperial/ros_ws/src/p3at_nav/maps

# Ensure you have write permissions
chmod -R u+w ~/ELEC70015_Human-Centered-Robotics-2026_Imperial/ros_ws/src/p3at_nav/maps

# Try saving again
rosrun map_server map_saver -f ~/ELEC70015_Human-Centered-Robotics-2026_Imperial/ros_ws/src/p3at_nav/maps/test_map
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
source devel/setup.bash  # or setup.zsh
```

---

### Problem: TF errors or missing transforms

**Diagnosis:**
```bash
rosrun tf view_frames
xdg-open frames.pdf  # View TF tree

# Monitor TF
rosrun tf tf_monitor
```

**Expected:** 27 coordinate frames (with SLAM), all delays less than 4ms

**Common Issues:**
- Missing `robot_state_publisher` node
- URDF not loaded properly
- Camera frames not defined
- GMapping not publishing `map → odom` transform

**Solution:** 
1. Check that `bringup_depth.launch` includes `robot_state_publisher`
2. Verify GMapping is running: `rosnode list | grep slam`
3. Restart the complete SLAM pipeline

---

### Problem: RViz shows robot model but P3AT mesh is invisible (WSL2)

**Diagnosis:**
This is a known GPU rendering limitation in WSL2 environments.

**Symptoms:**
- Robot TF frames display correctly in RViz
- `/scan` laser points visible
- P3AT visual mesh does not render in Gazebo or RViz

**Solution:**
Refer to [WSL2 GPU Acceleration and 3D Rendering Guide](https://zhuanlan.zhihu.com/p/19575977500) for detailed solutions including:
- Installing WSL2 GPU drivers
- Configuring X11 forwarding
- Using alternative rendering backends
- Hardware acceleration settings

---

### Problem: `evince` command not found

**Solution:** Use alternatives:
```bash
xdg-open frames.pdf    # System default PDF viewer
eog frames.pdf         # Eye of GNOME (if installed)
okular frames.pdf      # KDE PDF viewer (if installed)
```

---

### Problem: Processes not terminating cleanly

**Solution:**
```bash
# Kill all ROS and Gazebo processes
killall -9 gzserver gzclient roscore rosmaster rosout
pkill -9 -f ros
pkill -9 -f gazebo

# Wait a few seconds
sleep 3

# Verify cleanup
ps aux | grep -E "ros|gazebo" | grep -v grep
```

---

## Technical Details

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

### Camera Intrinsic Parameters

Camera intrinsics verified from camera_info:
```yaml
image_size: [640, 480]
distortion_model: "plumb_bob"
D: [0.0, 0.0, 0.0, 0.0, 0.0]  # No distortion (simulation)
K: [554.254691191187, 0.0, 320.0,    # fx, 0, cx
    0.0, 554.254691191187, 240.0,    # 0, fy, cy
    0.0, 0.0, 1.0]                   # 0, 0, 1
```

### Camera Position (relative to base_link)

```yaml
translation: [0.261, 0.018, 0.463]  # x, y, z in meters
rotation_rpy: [-90°, 0°, -90°]      # Optical frame convention
```

### Depth-to-Scan Parameters

**Configuration** (`depth_to_scan.launch`):
- `output_frame_id`: `base_link` (corrected for SLAM compatibility)
- `scan_height`: 10 pixels (vertical sampling)
- `scan_time`: 0.033s (30 Hz)
- `range_min`: 0.2m
- `range_max`: 10.0m

**Output Characteristics:**
- FOV: Approximately ±30° (horizontal)
- Valid ranges: ~640 points per scan
- `nan` values: Normal for sky/ground/out-of-range

**Topic Remapping:**
```xml
<remap from="image"       to="/sim_p3at/camera/depth/image_rect_raw"/>
<remap from="camera_info" to="/sim_p3at/camera/depth/camera_info"/>
<remap from="scan"        to="/scan"/>
```

### GMapping SLAM Parameters

**Configuration** (`gmapping_params.yaml`):

| Parameter | Value | Description |
|-----------|-------|-------------|
| `maxUrange` | 9.5m | Maximum usable range |
| `maxRange` | 10.0m | Maximum sensor range |
| `delta` | 0.05m | Map resolution (5cm) |
| `particles` | 30 | Number of particles in filter |
| `linearUpdate` | 0.2m | Process scan if robot moved 0.2m |
| `angularUpdate` | 0.2rad | Process scan if robot rotated 0.2rad |
| `temporalUpdate` | 3.0s | Process scan if 3s elapsed |
| `xmin/ymin` | -10.0m | Initial map bounds |
| `xmax/ymax` | 10.0m | Initial map bounds |
| `odom_frame` | odom | Odometry frame |
| `base_frame` | base_link | Robot base frame |
| `map_frame` | map | Map frame |

**Frame ID Correction:**

The original configuration had `/scan` with `frame_id: camera_depth_optical_frame`, which caused GMapping to fail because it couldn't find the transform between `odom` and `camera_depth_optical_frame`. 

**Solution:** Set `output_frame_id: base_link` in `depth_to_scan.launch` to ensure `/scan` is in the robot's base frame, allowing GMapping to properly integrate odometry and laser scan data.

---

## Development Tools

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

### Quick Diagnostic Commands

```bash
# Complete system check (basic simulation)
echo "=== Nodes ===" && rosnode list
echo "=== Scan Hz ===" && timeout 3 rostopic hz /scan
echo "=== Depth Hz ===" && timeout 3 rostopic hz /sim_p3at/camera/depth/depth/image_raw
echo "=== TF ===" && rosrun tf tf_echo base_link camera_depth_optical_frame

# SLAM system check
echo "=== SLAM Nodes ===" && rosnode list | grep slam
echo "=== Map Hz ===" && timeout 3 rostopic hz /map
echo "=== Odom Hz ===" && timeout 3 rostopic hz /sim_p3at/odom
echo "=== Map→Odom TF ===" && rosrun tf tf_echo map odom
```

### Map Visualization Script

```python
# Quick map viewer using matplotlib
python3 -c "
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
img = mpimg.imread('ros_ws/src/p3at_nav/maps/my_map.pgm')
plt.imshow(img, cmap='gray')
plt.title('SLAM Map')
plt.colorbar()
plt.show()
"
```

---

## Git Workflow

### What's Tracked
- Source packages: `p3at_sim/`, `p3at_nav/`
- Launch files, URDF, RViz configs
- SLAM configurations: `config/`, `rviz/`
- Map saving script: `scripts/save_map.sh`
- Utility scripts: `tools/`
- Documentation: `README.md`
- Submodule: `amr-ros-config/`

### What's Ignored (`.gitignore`)
- Build artifacts: `build/`, `devel/`
- Source build: `gazebo_ros_pkgs/` (must clone manually)
- Generated maps: `p3at_nav/maps/*.pgm`, `*.yaml` (optional: commit reference maps)
- Temporary files: `*.pyc`, `__pycache__/`
- IDE configs: `.vscode/`, `.idea/`
- ROS logs: `.ros/log/`
- TF frames PDF: `frames.pdf`

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
source devel/setup.bash  # or setup.zsh
```

---

## Resources

- [ROS Noetic Documentation](http://wiki.ros.org/noetic)
- [Gazebo Tutorials](http://gazebosim.org/tutorials)
- [GMapping SLAM](http://wiki.ros.org/gmapping)
- [depthimage_to_laserscan Package](http://wiki.ros.org/depthimage_to_laserscan)
- [map_server Package](http://wiki.ros.org/map_server)
- [URDF Tutorial](http://wiki.ros.org/urdf/Tutorials)
- [REP-103: Coordinate Frames](https://www.ros.org/reps/rep-0103.html)
- [REP-105: Coordinate Frames for Mobile Platforms](https://www.ros.org/reps/rep-0105.html)
- [WSL2 GPU Rendering Guide](https://zhuanlan.zhihu.com/p/19575977500)

---

## Post-Installation Checklist

Verify everything works:

### Basic Simulation
- [ ] Workspace builds without errors: `catkin_make`
- [ ] Gazebo launches: `roslaunch p3at_sim bringup_depth.launch`
- [ ] Robot spawns in simulation (no errors in terminal)
- [ ] All 7 core nodes running: `rosnode list`
- [ ] Camera topics exist: `rostopic list | grep camera`
- [ ] Depth image publishes at ~30Hz: `rostopic hz /sim_p3at/camera/depth/depth/image_raw`
- [ ] Camera info publishes at ~30Hz: `rostopic hz /sim_p3at/camera/depth/depth/camera_info`
- [ ] Perception pipeline works: `roslaunch p3at_nav depth_to_scan.launch`
- [ ] Scan data publishes at ~30Hz: `rostopic hz /scan`
- [ ] Scan frame_id is `base_link`: `rostopic echo /scan -n 1 | grep frame_id`
- [ ] Scan data contains valid ranges: `rostopic echo /scan -n 1`
- [ ] RViz displays correctly: `rviz -d src/p3at_nav/rviz/depth_scan.rviz`
- [ ] TF tree complete (26 frames): `rosrun tf view_frames && xdg-open frames.pdf`
- [ ] Camera transform correct: `rosrun tf tf_echo base_link camera_depth_optical_frame`

### SLAM Mapping
- [ ] GMapping launches: `roslaunch p3at_nav gmapping_slam.launch`
- [ ] SLAM node running: `rosnode list | grep slam`
- [ ] Map topic publishes: `rostopic hz /map`
- [ ] Map metadata available: `rostopic echo /map_metadata -n 1`
- [ ] TF tree includes map frame (27 frames): `rosrun tf tf_echo map odom`
- [ ] Keyboard teleoperation works: `rosrun teleop_twist_keyboard teleop_twist_keyboard.py cmd_vel:=/sim_p3at/cmd_vel`
- [ ] RViz SLAM view displays correctly: RViz window shows map, laser scan, robot model
- [ ] Map updates when robot moves (visible in RViz)
- [ ] Map saving works: `rosrun map_server map_saver -f /path/to/map`
- [ ] Saved map files exist: `my_map.pgm` and `my_map.yaml`
- [ ] Saved map has correct resolution: Check `resolution: 0.05` in YAML