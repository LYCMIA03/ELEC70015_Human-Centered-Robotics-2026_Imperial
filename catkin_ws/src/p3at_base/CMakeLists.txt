cmake_minimum_required(VERSION 3.0.2)
project(p3at_base)

find_package(catkin REQUIRED)

catkin_package()

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS
)

install(PROGRAMS
  scripts/odom_republisher.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
