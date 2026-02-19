cmake_minimum_required(VERSION 3.0.2)
project(p3at_lms_navigation)

find_package(catkin REQUIRED)

catkin_package()

install(DIRECTORY launch param rviz maps
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS
)
