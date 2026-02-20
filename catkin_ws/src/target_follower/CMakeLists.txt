cmake_minimum_required(VERSION 3.0.2)
project(target_follower)

find_package(catkin REQUIRED)

catkin_package()

install(PROGRAMS
  scripts/*.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
