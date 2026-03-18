# Navigation Simulation Experiments (Branch: `navigation_sim_experiments`)

This branch is dedicated to simulation only.
It is used to deploy, run, and reproduce Gazebo + RViz navigation experiments.

## Scope

- Simulation platform: ROS1 Noetic + Gazebo 11 + RViz on Ubuntu 20.04.
- Robot model used in experiments: dual-lidar + RGB-D simulation model.
- Covered workflow:
  - fixed-point navigation in simple and complex scenes
  - target following in simple scene
  - autonomous exploration mapping and AMCL verification
- Excluded from this branch README:
  - real-robot deployment
  - Docker operations
  - dialogue and detection integration details

## Simulation Sensor Policy (Mandatory)

All reported experiments in this branch follow the same role split:

- Unitree L1 is primary for:
  - `slam_gmapping`
  - `AMCL`
  - `global_costmap`
  - baseline `local_costmap`
- RPLIDAR is used only as short-range supplement on `local_costmap`.
- AMCL verification is Unitree-only and does not use RPLIDAR scan matching.

Sensor geometry update used in the reported runs:

- RPLIDAR is shifted by `-3 cm` along x.
- Effective value: `rplidar_tf_x = -0.232`.

## Prerequisites (Simulation Host)

```bash
sudo apt-get install -y \
  ros-noetic-desktop-full \
  ros-noetic-gmapping \
  ros-noetic-move-base \
  ros-noetic-dwa-local-planner \
  ros-noetic-navfn \
  ros-noetic-amcl \
  ros-noetic-map-server \
  ros-noetic-slam-gmapping \
  ros-noetic-robot-state-publisher \
  ros-noetic-joint-state-publisher \
  ros-noetic-tf2-ros \
  ros-noetic-actionlib \
  ros-noetic-teleop-twist-keyboard \
  python3-catkin-tools \
  python3-rospy \
  python3-numpy
```

## Build

```bash
cd catkin_ws
catkin_make
source devel/setup.bash
```

Verify:

```bash
rospack find p3at_lms_navigation
```

## Simulation Runbook

### 1) Launch Gazebo + Navigation Stack

Unitree stack (primary, recommended):

```bash
roslaunch p3at_lms_gazebo sim_unitree.launch \
  world:=$(rospack find p3at_lms_gazebo)/worlds/complex_maze.world
```

SICK stack (backup):

```bash
roslaunch p3at_lms_gazebo sim.launch \
  world:=$(rospack find p3at_lms_gazebo)/worlds/complex_maze.world
```

### 2) Mapping with Manual Control

Unitree:

```bash
roslaunch p3at_lms_navigation mapping_unitree.launch
```

SICK:

```bash
roslaunch p3at_lms_navigation mapping.launch
```

Save map:

```bash
rosrun map_server map_saver -f $(rospack find p3at_lms_navigation)/maps/my_map
```

### 3) Send Navigation Goals

In RViz use `2D Nav Goal`, or publish directly:

```bash
rostopic pub -1 /move_base_simple/goal geometry_msgs/PoseStamped \
  "{ header: {frame_id: 'map'},
     pose: { position: {x: 2.0, y: 1.0, z: 0.0},
             orientation: {w: 1.0} } }"
```

### 4) Target Following

Static target:

```bash
roslaunch p3at_lms_navigation mapping_unitree.launch use_gazebo_target:=true
```

Dynamic moving target:

```bash
roslaunch p3at_lms_navigation mapping_unitree.launch \
  move_target:=true \
  target_speed:=0.3 \
  target_pause:=2.0 \
  standoff_distance:=1.0 \
  face_target:=true
```

### 5) Autonomous Frontier Exploration

Unitree:

```bash
roslaunch p3at_lms_navigation auto_mapping_unitree.launch \
  exploration_timeout:=300 \
  gui:=false
```

SICK:

```bash
roslaunch p3at_lms_navigation auto_mapping.launch \
  exploration_timeout:=300 \
  gui:=false
```

### 6) AMCL Verification on Saved Map

Unitree:

```bash
roslaunch p3at_lms_navigation auto_amcl_verify_unitree.launch gui:=true
```

SICK:

```bash
roslaunch p3at_lms_navigation auto_amcl_verify.launch gui:=true
```

## One-Click Four Experiments

Main entry script:

- [`scripts/run_sim_four_experiments.sh`](scripts/run_sim_four_experiments.sh)

What it runs:

1. Exp1: simple-obstacle fixed-point navigation
2. Exp2: complex-obstacle fixed-point navigation
3. Exp3: simple-obstacle target following
4. Exp4: Task6/7-style autonomous mapping + AMCL verification

Run:

```bash
bash scripts/run_sim_four_experiments.sh
```

Useful options:

```bash
bash scripts/run_sim_four_experiments.sh --skip-build
bash scripts/run_sim_four_experiments.sh --gui
bash scripts/run_sim_four_experiments.sh --nav-goal-timeout 120
bash scripts/run_sim_four_experiments.sh --follow-duration 180
bash scripts/run_sim_four_experiments.sh --explore-timeout 300
```

Outputs:

```text
Log/sim_experiments/<timestamp>/
  logs/*.log
  metrics/*.json
  summary.json
  summary.md
```

## Experiment Design and Acceptance

| Experiment | Scene | Objective | Main metrics | Acceptance focus |
|---|---|---|---|---|
| Exp1 | Simple obstacles | Fixed-point navigation | waypoint success, final goal error, travel distance | stable end-to-end nav |
| Exp2 | Complex maze obstacles | Fixed-point navigation | waypoint success, timeout behavior, path efficiency | detour and clutter handling |
| Exp3 | Simple obstacles | Target following | standoff tolerance, tracking continuity, false terminations | robot must actually keep following |
| Exp4 | Complex maze | Autonomous mapping + AMCL | bounded-time coverage, AMCL convergence/error, nav success | 20-minute feasibility and localization reliability |

Acceptance criteria used in this branch:

- Exp1 and Exp2:
  - waypoint-level success from `move_base`
  - successful goals should usually end with final error near or below `0.2 m`
- Exp3:
  - robot motion must be non-trivial
  - tracking-state continuity and false-termination count must be reported
  - if target path is not physically followable, target policy must be redesigned
- Exp4 mapping:
  - full-maze closure is not required in a short run
  - focus on frontier discovery, partial map growth, and no permanent wall-lock
- Exp4 AMCL:
  - convergence within a few seconds
  - mean waypoint error below `0.30 m`
  - max waypoint error below `0.50 m`
  - waypoint success above `80%`

## Final Consolidated Results (2026-03-17)

Primary artifact directory:

```text
Log/sim_experiments/20260317_0215_manual/
```

### Exp1: Simple Obstacle Fixed-Point Navigation

Source: [`Log/sim_experiments/20260317_0215_manual/metrics/exp1_simple_nav.json`](Log/sim_experiments/20260317_0215_manual/metrics/exp1_simple_nav.json)

- success: `3/3` (`100%`)
- mean goal error: `0.1475 m`
- mean navigation time: `76.26 s`
- total distance: `16.80 m`

### Exp2: Complex Obstacle Fixed-Point Navigation

Source: [`Log/sim_experiments/20260317_0215_manual/metrics/exp2_complex_nav.json`](Log/sim_experiments/20260317_0215_manual/metrics/exp2_complex_nav.json)

- success: `4/5` (`80%`)
- mean goal error: `0.1530 m`
- mean navigation time: `71.49 s`
- total distance: `17.91 m`

### Exp3: Target Following

Slow-speed baseline source:
[`Log/sim_experiments/20260317_0215_manual/metrics/exp3_target_follow_tuned.json`](Log/sim_experiments/20260317_0215_manual/metrics/exp3_target_follow_tuned.json)

- tracking run-level success: `100%` (no false termination)
- mean robot-target distance: `1.8037 m`
- standoff target: `0.8 m`
- within-tolerance rate: `6.65%`

High-speed default source:
[`Log/sim_experiments/20260317_0215_manual/metrics/exp3_target_follow_hs_fix_attempt5.json`](Log/sim_experiments/20260317_0215_manual/metrics/exp3_target_follow_hs_fix_attempt5.json)

High-speed launch log:
[`Log/sim_experiments/20260317_0215_manual/logs/exp3_hs_fix_attempt5_launch.log`](Log/sim_experiments/20260317_0215_manual/logs/exp3_hs_fix_attempt5_launch.log)

- controller tracking-state occupancy: `100% TRACKING`
- false terminations: `0`
- within-tolerance success rate (`1.2 m ± 0.35 m`): `68.8%`
- commanded moving-target profile: `0.36 m/s`

### Exp4: Task6/7-Style Autonomous Mapping + AMCL

Mapping artifacts:

- [`Log/sim_experiments/20260317_0215_manual/metrics/exp4_task67_tuned_run2.yaml`](Log/sim_experiments/20260317_0215_manual/metrics/exp4_task67_tuned_run2.yaml)
- [`Log/sim_experiments/20260317_0215_manual/metrics/exp4_task67_tuned_run2.pgm`](Log/sim_experiments/20260317_0215_manual/metrics/exp4_task67_tuned_run2.pgm)

AMCL reports:

- [`Log/sim_experiments/20260317_0215_manual/metrics/amcl_report_unitree_tuned_run3.txt`](Log/sim_experiments/20260317_0215_manual/metrics/amcl_report_unitree_tuned_run3.txt)
- [`Log/sim_experiments/20260317_0215_manual/metrics/amcl_report_unitree_tuned_run3.json`](Log/sim_experiments/20260317_0215_manual/metrics/amcl_report_unitree_tuned_run3.json)

Final AMCL result:

- convergence time: `1.1 s`
- tracking mean error: `0.0610 m`
- tracking max error: `0.3340 m`
- waypoint navigation success: `6/6`
- mean waypoint error: `0.0479 m`
- max waypoint error: `0.0964 m`

## 20-Minute Auto-Explore Protocol (Wall-Stuck Aware)

This branch uses a bounded-time feasibility protocol rather than full-maze closure.

Recommended settings and behavior:

- run autonomous explorer with timeout in the same order of 10-20 minutes
- monitor repeated `stuck_no_progress` events
- retune blacklist/progress thresholds if repeated wall-lock occurs
- validate:
  - frontier discovery continues
  - map known-space keeps growing
  - map can be saved and used by AMCL

Tuned run reference that resolved severe wall-stuck behavior:

- `frontier_blacklist_radius = 0.25`
- `stuck_min_progress = 0.06`
- `stuck_progress_timeout = 10.0`
- achieved `14.4%` known-space coverage in `749.9 s`

## Local Costmap Dual Participation Check

Final logs confirm all four experiments used both scans in local costmap:

- Exp1: `unitree_scan_sensor rplidar_scan_sensor`
- Exp2: `unitree_scan_sensor rplidar_scan_sensor`
- Exp3: `unitree_scan_sensor rplidar_scan_sensor`
- Exp4: `unitree_scan_sensor rplidar_scan_sensor`

At the same time, `gmapping`, `AMCL`, and `global_costmap` remained Unitree-led.

## Quick Reproduction Checklist

```bash
git fetch origin
git checkout navigation_sim_experiments
cd catkin_ws
catkin_make
source devel/setup.bash
cd ..
bash scripts/run_sim_four_experiments.sh --gui
```

For headless CI-style runs:

```bash
bash scripts/run_sim_four_experiments.sh --skip-build
```

