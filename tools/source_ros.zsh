export ROS_MASTER_URI=http://localhost:11311

export ROS_HOSTNAME=localhost


# Robust ROS sourcing for zsh terminals (WSL/VSCode)

# Prevent polluted catkin variables breaking setup scripts
unset _CATKIN_SETUP_DIR
unset CMAKE_PREFIX_PATH
unset ROS_PACKAGE_PATH

# System ROS (zsh)
source /opt/ros/noetic/setup.zsh

# Project workspace (zsh)
WS_DIR=~/work/ELEC70015_Human-Centered-Robotics-2026_Imperial/ros_ws
source ${WS_DIR}/devel/setup.zsh

echo "[source_ros.zsh] ROS_DISTRO=$ROS_DISTRO"
echo "[source_ros.zsh] ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH"
