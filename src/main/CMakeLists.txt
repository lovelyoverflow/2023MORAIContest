cmake_minimum_required(VERSION 3.0.2)
project(main)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES wego
#  CATKIN_DEPENDS roscpp rospy
#  DEPENDS system_lib
)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  rospy
  std_msgs
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
  scripts/main.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)