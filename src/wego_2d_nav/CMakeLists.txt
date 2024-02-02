cmake_minimum_required(VERSION 3.0.2)
project(wego_2d_nav)

find_package(catkin REQUIRED COMPONENTS
  rospy
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES wego_2d_nav
#  CATKIN_DEPENDS rospy
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
  scripts/cmd_vel_to_ackermann.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)