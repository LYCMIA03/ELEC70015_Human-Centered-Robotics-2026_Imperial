#!/usr/bin/env bash
set -e
cd "$(dirname "$0")/catkin_ws"
catkin_make
source devel/setup.bash
echo "Built workspace."
echo "Try: roslaunch p3at_lms_navigation mapping.launch"
