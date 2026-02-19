cmake_minimum_required(VERSION 3.0.2)
project(p3at_lms_description)

find_package(catkin REQUIRED COMPONENTS xacro)

catkin_package()

install(DIRECTORY launch urdf rviz meshes
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS
)
