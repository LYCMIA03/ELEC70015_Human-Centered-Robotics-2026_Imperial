#!/usr/bin/env bash
set -e

# Make workspace sourcing robust against polluted _CATKIN_SETUP_DIR
unset _CATKIN_SETUP_DIR

# System ROS
source /opt/ros/noetic/setup.bash

# Project workspace
WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)/ros_ws"
source "${WS_DIR}/devel/local_setup.bash"
