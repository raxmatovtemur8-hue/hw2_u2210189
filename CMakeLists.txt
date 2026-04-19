cmake_minimum_required(VERSION 3.0.2)
project(hw2_u2210189)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  geometry_msgs
  nav_msgs
)

catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
  src/move_robots.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)