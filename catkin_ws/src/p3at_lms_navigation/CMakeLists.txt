cmake_minimum_required(VERSION 3.0.2)
project(p3at_lms_navigation)

find_package(catkin REQUIRED)

catkin_package()

install(DIRECTORY launch param rviz maps
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS
)

catkin_install_python(PROGRAMS
  scripts/autonomous_explorer.py
  scripts/amcl_verifier.py
  scripts/continuous_map_manager.py
  scripts/waypoint_test.py
  scripts/nav_experiment_runner.py
  scripts/target_follow_metrics.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
